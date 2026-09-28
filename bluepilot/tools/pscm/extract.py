#!/usr/bin/env python3
"""
BluePilot: extract per-cycle Ford lateral signals from rlogs for PSCM behaviour studies.

One compressed .npz per segment, sampled at every carState (100 Hz), each row carrying the most
recent value of every other source (sample-and-hold). Covers what the PSCM study needs: what we
commanded on the wire (LateralMotionControl2), what the PSCM reported, what the car did, and what
BluePilot's controller was doing (controllerStateBP).

Sign convention: lmcPA / lmcOff are the wire values negated back to the controller's internal
sign (carcontroller sends -lat.path_angle / -lat.path_offset), so they share a sign with
carControl.actuators.curvature and with -carState.yawRate.

Usage:
  extract.py [--cache DIR] ROUTE_DIR [ROUTE_DIR ...]
  (ROUTE_DIR holds <route>--<n>/rlog.zst segment folders, the CommaRoutes layout)
"""
import argparse
import glob
import os
from multiprocessing import Pool

import numpy as np

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader

DEFAULT_CACHE = os.path.expanduser('~/.cache/bp_pscm')

COLS = ['t', 'v', 'ang', 'tq', 'pressed', 'lblink', 'rblink', 'yaw', 'angrate', 'latActive', 'acurv',
        'htPaused', 'blip', 'blipSrc', 'devLim', 'rocLim', 'latMode', 'blipCnt',
        'lmcMode', 'lmcPA', 'lmcOff', 'lmcRamp',
        'steStat', 'limStat', 'handsOff', 'actDeny', 'cpblty',
        'motorI', 'drvTq', 'drvActv', 'colTq',
        'llL', 'llR', 'lpL', 'lpR', 'lcState', 'desireLeft']

_dbc = DBC('ford_lincoln_base_pt')


def _sigs(addr, names):
  m = _dbc.addr_to_msg[addr]
  return [m.sigs[n] for n in names]


_LMC2 = _sigs(982, ['LatCtl_D2_Rq', 'LatCtlPath_An_Actl', 'LatCtlPathOffst_L_Actl', 'LatCtlRampType_D_Rq'])
_LAD3 = _sigs(972, ['LatCtlSte_D_Stat', 'LatCtlLim_D_Stat', 'LaHandsOff_B_Actl', 'LaActDeny_B_Actl', 'LatCtlCpblty_D_Stat'])
_EPAS = _sigs(130, ['SteMdule_I_Est', 'DrvSte_Tq_Actl', 'DrvSteActv_B_Stat', 'SteeringColumnTorque'])


def _decode(dat, sigs):
  out = []
  for s in sigs:
    r = get_raw_value(dat, s)
    if s.is_signed:
      r -= ((r >> (s.size - 1)) & 1) * (1 << s.size)
    out.append(r * s.factor + s.offset)
  return out


def extract_segment(args):
  seg_dir, cache = args
  name = os.path.basename(seg_dir.rstrip('/'))
  out = os.path.join(cache, f'{name}.npz')
  if os.path.exists(out):
    return out
  rows = []
  cc, bp, lmc, lad, epas, md = [0, 0.0], [0] * 7, [0] * 4, [0] * 5, [0] * 4, [0] * 6
  pscm_bus = None
  try:
    for m in LogReader(os.path.join(seg_dir, 'rlog.zst')):
      w = m.which()
      if w == 'carState':
        c = m.carState
        rows.append([m.logMonoTime * 1e-9, c.vEgo, c.steeringAngleDeg, c.steeringTorque, c.steeringPressed,
                     c.leftBlinker, c.rightBlinker, c.yawRate, c.steeringRateDeg,
                     *cc, *bp, *lmc, *lad, *epas, *md])
      elif w == 'carControl':
        cc = [m.carControl.latActive, m.carControl.actuators.curvature]
      elif w == 'controllerStateBP':
        b = m.controllerStateBP
        bp = [b.humanTurnLateralPaused, b.stallBlipActive, b.stallBlipSource, b.curvatureDeviationLimited,
              b.angleRateLimited, b.activeLateralMode.raw, b.stallBlipEpisodeCount]
      elif w == 'sendcan':
        for f in m.sendcan:
          if f.address == 982 and f.src < 128:
            v = _decode(f.dat, _LMC2)
            lmc = [v[0], -v[1], -v[2], v[3]]
      elif w == 'can':
        for f in m.can:
          if f.src >= 128:
            continue
          # Latch the PSCM bus on the first 972, but let bus 0 (the main bus) take over: at route start
          # a short-lived copy on bus 2 can arrive first and then go stale, freezing LatCtlSte_D_Stat.
          if f.address == 972:
            if pscm_bus is None or (f.src == 0 and pscm_bus != 0):
              pscm_bus = f.src
            if f.src == pscm_bus:
              lad = _decode(f.dat, _LAD3)
          elif f.address == 130 and (pscm_bus is None or f.src == pscm_bus):
            epas = _decode(f.dat, _EPAS)
      elif w == 'modelV2':
        mv = m.modelV2
        if len(mv.laneLines) == 4 and len(mv.laneLines[1].y):
          ds = mv.meta.desireState
          md = [mv.laneLines[1].y[0], mv.laneLines[2].y[0], mv.laneLineProbs[1], mv.laneLineProbs[2],
                mv.meta.laneChangeState.raw, ds[1] if len(ds) > 2 else 0]
  except Exception as e:  # a corrupt segment shouldn't kill a whole batch
    print('ERR', seg_dir, e)
  np.savez_compressed(out, a=np.array(rows, dtype=np.float64))
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('routes', nargs='+')
  ap.add_argument('--cache', default=DEFAULT_CACHE)
  ap.add_argument('-j', type=int, default=max(1, (os.cpu_count() or 2) - 2))
  a = ap.parse_args()
  os.makedirs(a.cache, exist_ok=True)
  segs = []
  for r in a.routes:
    segs += glob.glob(os.path.join(r, '*--*/'))
  with Pool(a.j) as p:
    for _ in p.imap_unordered(extract_segment, [(s, a.cache) for s in segs]):
      pass
  print(f'extracted {len(segs)} segments -> {a.cache}')


if __name__ == '__main__':
  main()

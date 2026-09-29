#!/usr/bin/env python3
"""
BluePilot: extract the commaCarSegments Ford fleet dump (PLATFORM/DONGLE/ROUTE/SEG/rlog.zst).

Those rlogs hold only raw CAN (+ pandaStates, carParams), so everything is decoded from CAN: our
own LateralMotionControl(2) frames come back as TX echoes (src 128+bus), the car's response from
its own messages. One row per CAN batch (~100 Hz), sample-and-hold, one .npz per segment named
PLATFORM--DONGLE--ROUTE--SEG.

Columns (FLEET_COLS). Wire fields are raw wire values (NOT negated): stock openpilot sends
-apply_curvature, so wire C2 has carState sign (left +), like the wire C1 in emulator.py.

Usage: extract_fleet.py ROOT [--cache DIR] [-j N]
"""
import argparse
import glob
import os
from multiprocessing import Pool

import numpy as np

from opendbc.can.dbc import DBC
from openpilot.tools.lib.logreader import LogReader
from bluepilot.tools.pscm.extract import _decode

DEFAULT_FLEET_CACHE = os.path.expanduser('~/.cache/bp_pscm_fleet')
FLEET_COLS = ['t', 'v', 'ang', 'yaw', 'tq', 'mode', 'c0', 'c1', 'c2', 'c3', 'steStat', 'tx']

_dbc = DBC('ford_lincoln_base_pt')


def _s(name, sigs):
  m = _dbc.name_to_msg[name]
  return m.address, [m.sigs[n] for n in sigs]


_SPEED = _s('BrakeSysFeatures', ['Veh_V_ActlBrk'])
_ANGLE = _s('SteeringPinion_Data', ['StePinComp_An_Est'])
_YAW = _s('Yaw_Data_FD1', ['VehYaw_W_Actl'])
_EPAS = _s('EPAS_INFO', ['SteeringColumnTorque'])
_LAD3 = _s('Lane_Assist_Data3_FD1', ['LatCtlSte_D_Stat'])
_LMC2 = _s('LateralMotionControl2', ['LatCtl_D2_Rq', 'LatCtlPathOffst_L_Actl', 'LatCtlPath_An_Actl',
                                     'LatCtlCurv_No_Actl', 'LatCtlCrv_NoRate2_Actl'])
_LMC1 = _s('LateralMotionControl', ['LatCtl_D_Rq', 'LatCtlPathOffst_L_Actl', 'LatCtlPath_An_Actl',
                                    'LatCtlCurv_No_Actl', 'LatCtlCurv_NoRate_Actl'])


def extract_segment(args):
  seg_dir, cache, name = args
  out = os.path.join(cache, f'{name}.npz')
  if os.path.exists(out):
    return out
  rows = []
  st = {'v': 0.0, 'ang': 0.0, 'yaw': 0.0, 'tq': 0.0, 'wire': [0, 0.0, 0.0, 0.0, 0.0], 'ste': 0, 'tx': 0}
  main_bus = None
  try:
    for m in LogReader(os.path.join(seg_dir, 'rlog.zst')):
      if m.which() != 'can':
        continue
      for f in m.can:
        a = f.address
        if f.src >= 128:
          if a in (_LMC2[0], _LMC1[0]) and (main_bus is None or f.src - 128 == main_bus):
            st['wire'] = _decode(f.dat, _LMC2[1] if a == _LMC2[0] else _LMC1[1])
            st['tx'] = 1
          continue
        if a == _YAW[0] and main_bus is None:
          main_bus = f.src
        if f.src != main_bus:
          continue
        if a == _SPEED[0]:
          st['v'] = _decode(f.dat, _SPEED[1])[0] / 3.6
        elif a == _ANGLE[0]:
          st['ang'] = _decode(f.dat, _ANGLE[1])[0]
        elif a == _YAW[0]:
          st['yaw'] = _decode(f.dat, _YAW[1])[0]
        elif a == _EPAS[0]:
          st['tq'] = _decode(f.dat, _EPAS[1])[0]
        elif a == _LAD3[0]:
          st['ste'] = _decode(f.dat, _LAD3[1])[0]
      rows.append([m.logMonoTime * 1e-9, st['v'], st['ang'], st['yaw'], st['tq'], *st['wire'], st['ste'], st['tx']])
  except Exception as e:  # a corrupt segment shouldn't kill a whole batch
    print('ERR', seg_dir, e)
  np.savez_compressed(out, a=np.array(rows, dtype=np.float64))
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('root')
  ap.add_argument('--cache', default=DEFAULT_FLEET_CACHE)
  ap.add_argument('-j', type=int, default=30)
  a = ap.parse_args()
  os.makedirs(a.cache, exist_ok=True)
  jobs = []
  for f in glob.glob(os.path.join(a.root, 'FORD_*', '*', '*', '*', 'rlog.zst')):
    plat, dongle, route, seg = f.split(os.sep)[-5:-1]
    jobs.append((os.path.dirname(f), a.cache, f'{plat}--{dongle}--{route.replace("--", "_")}--{int(seg)}'))
  with Pool(a.j) as p:
    for _ in p.imap_unordered(extract_segment, jobs, chunksize=4):
      pass
  print(f'extracted {len(jobs)} segments -> {a.cache}')


if __name__ == '__main__':
  main()

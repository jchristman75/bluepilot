#!/usr/bin/env python3
"""
BluePilot: offline radar replay. Runs openpilot's real RadarD over a logged route and saves leadOne for
every modelV2 frame, next to the logged radarState.leadOne.

  --mode logged  radar tracks = the logged radarTracks (what stock radard saw): parity check against
                 the logged radarState
  --mode bp      what bluepilot/selfdrive/controls/radar/radard_bp.py does on the car: MrrTracker on the
                 logged radar-bus CAN + the logged radarTracks (Steer_Assist_Data), lead_match installed

usage: python -m bluepilot.tools.radar.radar_replay ROUTE_DIR OUT.npz --mode {logged,bp} [--bus 1]
ROUTE_DIR holds <route>--<n>/rlog.zst segment folders (or numbered segment folders).
Compare runs with bluepilot/tools/radar/compare.py.
"""
import argparse
import glob
import importlib.util
from collections import defaultdict
from types import SimpleNamespace

import numpy as np

from openpilot.tools.lib.logreader import LogReader
from openpilot.bluepilot.selfdrive.controls.radar import lead_match
from openpilot.bluepilot.selfdrive.controls.radar.mrr_tracker import MrrTracker
from openpilot.bluepilot.selfdrive.controls.radar.radard_bp import RAW_RADAR_DELAY, merge_points

COLS = ('t', 'v', 'p', 'r', 'd', 'vl', 'al', 'y', 'lp', 'lr', 'ld', 'lvl', 'lal', 'ntr')


class FakeSM:
  def __init__(self):
    self.data = {}
    self.seen = defaultdict(bool)
    self.recv_frame = defaultdict(int)
    self.logMonoTime = defaultdict(int)

  def put(self, w, msg, t):
    self.data[w] = getattr(msg, w)
    self.seen[w] = True
    self.recv_frame[w] += 1
    self.logMonoTime[w] = t

  def __getitem__(self, k):
    return self.data[k]

  def all_checks(self):
    return True


def segments(route):
  segs = glob.glob(f'{route}/*--*/rlog.zst')
  if segs:
    return sorted(segs, key=lambda p: int(p.split('--')[-1].split('/')[0]))
  return sorted(glob.glob(f'{route}/*/rlog.zst'), key=lambda p: int(p.rstrip('/').split('/')[-2]))


def private_radard():
  spec = importlib.util.find_spec("openpilot.selfdrive.controls.radard")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def replay(route, mode, bus=1):
  R = private_radard()
  tracker = None
  if mode == 'bp':
    lead_match.install(R)
    tracker = MrrTracker(bus)
  sm = FakeSM()
  radard = None
  radar_tracks = None  # wait for the first logged radarTracks (RadarD copies its errors struct)
  logged_lead = None
  rows = []
  t0 = None
  for p in segments(route):
    for m in LogReader(p):
      try:
        w = m.which()
      except Exception:
        continue
      t = m.logMonoTime
      if t0 is None:
        t0 = t
      if w == 'carParams' and radard is None:
        delay = RAW_RADAR_DELAY if mode == 'bp' else m.carParams.radarDelay
        radard = R.RadarD(SimpleNamespace(brand='ford'), SimpleNamespace(flags=0), delay)
      elif w == 'carState':
        sm.put('carState', m, t)
      elif w == 'radarState':
        logged_lead = m.radarState.leadOne
      elif w == 'radarTracks':
        radar_tracks = m.radarTracks
      elif w == 'can' and tracker is not None:
        tracker.update([(t, [(c.address, bytes(c.dat), c.src) for c in m.can if c.src == bus])])
      elif w == 'modelV2':
        sm.put('modelV2', m, t)
        if radard is None or radar_tracks is None or 'carState' not in sm.data:
          continue
        rr = merge_points(tracker.tracks if tracker is not None else [], radar_tracks)
        radard.update(sm, rr)
        L, lg = radard.radar_state.leadOne, logged_lead
        rows.append(((t - t0) / 1e9, sm['carState'].vEgo, float(L.present), float(L.radar), L.dRel, L.vLead, L.aLeadK, L.yRel,
                     float(lg.present) if lg is not None else 0.0, float(lg.radar) if lg is not None else 0.0,
                     lg.dRel if lg is not None else 0.0, lg.vLead if lg is not None else 0.0, lg.aLeadK if lg is not None else 0.0,
                     float(len(rr.points))))
  arr = np.array(rows, dtype=float).reshape(-1, len(COLS))
  return {c: arr[:, i] for i, c in enumerate(COLS)}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('route')
  ap.add_argument('out')
  ap.add_argument('--mode', choices=('logged', 'bp'), required=True)
  ap.add_argument('--bus', type=int, default=1)
  a = ap.parse_args()
  res = replay(a.route, a.mode, a.bus)
  np.savez(a.out, **res)
  print(f'saved {a.out}: {len(res["t"])} model frames, mean radar points {np.mean(res["ntr"]) if len(res["t"]) else 0:.1f}')


if __name__ == '__main__':
  main()

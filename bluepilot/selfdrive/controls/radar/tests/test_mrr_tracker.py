"""BluePilot: raw Delphi MRR (CAN-FD 'FORD_CADS_64') tracker - parsing round trip and track behavior."""
import math

import numpy as np
from opendbc.can.packer import CANPacker

from openpilot.bluepilot.selfdrive.controls.radar.mrr_tracker import MrrTracker, N_MSGS, POINTS_PER_MSG, LAST_MSG_POINTS, RAW_ID_BASE

BUS = 1
packer = CANPacker('FORD_CADS_64')
SLOTS = [(i, j) for i in range(1, N_MSGS + 1) for j in range(1, (POINTS_PER_MSG if i < N_MSGS else LAST_MSG_POINTS) + 1)]


def scan_frames(dets, scan_index, nanos):
  """dets: [(d, y, range_rate)] -> one scan's 22 MRR_Detection frames (unused slots invalid)."""
  vals = {i: {} for i in range(1, N_MSGS + 1)}
  for (i, j) in SLOTS:
    vals[i][f"CAN_SCAN_INDEX_2LSB_{i:02d}_{j:02d}"] = scan_index
    vals[i][f"CAN_DET_VALID_LEVEL_{i:02d}_{j:02d}"] = 0
  for (i, j), (d, y, rr) in zip(SLOTS, dets, strict=False):
    k = f"{i:02d}_{j:02d}"
    vals[i].update({f"CAN_DET_VALID_LEVEL_{k}": 1, f"CAN_DET_RANGE_{k}": math.hypot(d, y),
                    f"CAN_DET_AZIMUTH_{k}": math.atan2(y, d), f"CAN_DET_RANGE_RATE_{k}": rr})
  frames = [packer.make_can_msg(f"MRR_Detection_{i:03d}", BUS, vals[i]) for i in range(1, N_MSGS + 1)]
  return [(nanos, [(a, d, b) for a, d, b in frames])]


def run(tr, scans):
  out = []
  for n, dets in enumerate(scans):
    assert tr.update(scan_frames(dets, n % 4, int(n * 5e7)))
    out.append(list(tr.tracks))
  return out


def test_parse_round_trip():
  tr = MrrTracker(BUS)
  tr.update(scan_frames([(60.0, 1.2, -5.0), (30.0, -2.0, 0.5)], 2, 0))
  dets = tr.detections()
  got = sorted(map(tuple, np.round(dets, 1)))
  assert got == [(30.0, -2.0, 0.5), (60.0, 1.2, -5.0)]


def test_stopped_car_is_confirmed_and_keeps_its_id_through_dropouts():
  # closing on a stopped car at 15 m/s from 130 m; every 3rd scan misses it
  rng = np.random.default_rng(0)
  scans = []
  for n in range(60):
    d = 130.0 - 15.0 * 0.05 * n
    car = [] if n % 3 == 2 else [(d + rng.normal(0, 0.2), 0.3 + rng.normal(0, 0.1), -15.0)]
    scans.append(car)
  out = run(MrrTracker(BUS), scans)
  assert not out[3]                     # not reported before it is confirmed
  ids = {t.trackId for frame in out[10:] for t in frame}
  assert len(ids) == 1 and min(ids) >= RAW_ID_BASE
  last = out[-1][0]
  assert abs(last.vRel + 15.0) < 0.3 and abs(last.dRel - (130.0 - 15.0 * 0.05 * 59)) < 1.5


def test_one_scan_clutter_is_never_reported():
  rng = np.random.default_rng(1)
  scans = [[(rng.uniform(5, 150), rng.uniform(-10, 10), rng.uniform(-20, 20)) for _ in range(40)] for _ in range(40)]
  out = run(MrrTracker(BUS), scans)
  assert sum(len(f) for f in out) == 0


def test_two_cars_side_by_side_are_two_tracks():
  scans = [[(50.0 - 0.1 * n, 0.0, -2.0), (50.0 - 0.1 * n, 3.6, -2.0)] for n in range(20)]
  out = run(MrrTracker(BUS), scans)
  assert len(out[-1]) == 2
  assert sorted(round(t.yRel) for t in out[-1]) == [0, 4]


def test_detections_of_one_car_merge_into_one_track():
  # a car returns 3 detections a scan (bumper, wheels): one object, one track
  scans = [[(40.0, 0.0, -1.0), (41.5, 0.6, -1.1), (40.8, -0.7, -0.9)] for _ in range(15)]
  out = run(MrrTracker(BUS), scans)
  assert len(out[-1]) == 1

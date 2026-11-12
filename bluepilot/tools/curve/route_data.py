"""
BluePilot: per-model-frame data from saved routes, cached as .npz, for replaying the curve limiter and the
lane-change follow gap (bluepilot/tools/curve/replay.py).

Routes live in a data dir (default /mnt/j/CommaRoutes/c4, override with BP_ROUTES_DIR) as
<date>_..._<route>--<dongle>/<route>--<dongle>--<segment>/rlog.zst. Time is seconds since the first message of
segment 0 of the route, the same base the road-test notes use. Only the requested segments are parsed.
"""
import glob
import hashlib
import os
from multiprocessing import Pool

import numpy as np

ROUTES_DIR = os.environ.get("BP_ROUTES_DIR", "/mnt/j/CommaRoutes/c4")
CACHE_DIR = os.environ.get("BP_REPLAY_CACHE", os.path.expanduser("~/.cache/bp_replay"))
CACHE_VERSION = 3
PLAN_N = 33


def route_dir(route: str) -> str | None:
  hits = glob.glob(f"{ROUTES_DIR}/*_{route}--*")
  return hits[0] if hits else None


def segment_path(route: str, seg: int) -> str | None:
  d = route_dir(route)
  if d is None:
    return None
  hits = glob.glob(f"{d}/*--{seg}/rlog.zst")
  return hits[0] if hits else None


def available(route: str, segs) -> bool:
  return all(segment_path(route, s) for s in segs)


def _parse(path):
  from openpilot.tools.lib.logreader import LogReader
  rows, st = [], {}
  for m in LogReader(path):
    try:
      w = m.which()
    except Exception:
      continue
    t = m.logMonoTime / 1e9
    if w == 'carState':
      c = m.carState
      st['cs'] = (c.vEgo, c.aEgo, c.yawRate, float(c.gasPressed), float(c.brakePressed), float(c.leftBlinker), float(c.rightBlinker),
                  float(c.leftBlindspot), float(c.rightBlindspot), float(c.standstill), float(c.vCruise))
    elif w == 'carControl':
      st['cc'] = (float(m.carControl.longActive), float(m.carControl.enabled))
    elif w == 'controlsState':
      st['ct'] = (m.controlsState.curvature, m.controlsState.desiredCurvature)
    elif w == 'radarState':
      L = m.radarState.leadOne
      st['rs'] = (float(L.present), float(L.radar), L.dRel, L.vLead, L.aLeadK)
    elif w == 'modelV2' and len(st) == 4:
      mv = m.modelV2
      rows.append((t, st['cs'], st['cc'], st['ct'], st['rs'],
                   (float(int(mv.meta.laneChangeState.raw)), float(int(mv.meta.laneChangeDirection.raw))),
                   np.array(mv.position.x, dtype=np.float32), np.array(mv.position.y, dtype=np.float32), float(mv.frameId)))
  return rows


def _first_time(path):
  from openpilot.tools.lib.logreader import LogReader
  for m in LogReader(path):
    return m.logMonoTime / 1e9
  return 0.0


def load(route: str, segs, use_cache: bool = True) -> dict[str, np.ndarray] | None:
  """Frames (one per modelV2 message) of the given segments, or None when the route data isn't available."""
  segs = sorted(segs)
  if not available(route, segs):
    return None
  key = hashlib.md5(f"{route}-{segs}-{CACHE_VERSION}".encode()).hexdigest()[:12]
  cache = os.path.join(CACHE_DIR, f"{route}_{key}.npz")
  if use_cache and os.path.isfile(cache):
    with np.load(cache) as z:
      return {k: z[k] for k in z.files}
  paths = [segment_path(route, s) for s in segs]
  t0 = _first_time(segment_path(route, 0) or paths[0])
  with Pool(min(8, len(paths))) as pool:
    parsed = pool.map(_parse, paths)
  rows = [r for p in parsed for r in p]
  if not rows:
    return None
  g = lambda i, j: np.array([r[i][j] for r in rows], dtype=float)  # noqa: E731
  data = {
    "t": np.array([r[0] - t0 for r in rows]),
    "v": g(1, 0), "aego": g(1, 1), "yaw": g(1, 2), "gas": g(1, 3), "brake": g(1, 4), "lblink": g(1, 5), "rblink": g(1, 6), "lbs": g(1, 7), "rbs": g(1, 8),
    "standstill": g(1, 9), "vcruise": g(1, 10), "longact": g(2, 0), "enabled": g(2, 1), "kcur": g(3, 0), "kdes": g(3, 1),
    "lead": g(4, 0), "lead_radar": g(4, 1), "lead_d": g(4, 2), "lead_v": g(4, 3), "lead_a": g(4, 4), "lcs": g(5, 0), "lcd": g(5, 1),
    "px": np.stack([r[6] for r in rows]), "py": np.stack([r[7] for r in rows]), "frame": np.array([r[8] for r in rows])}
  os.makedirs(CACHE_DIR, exist_ok=True)
  np.savez_compressed(cache, **data)
  return data

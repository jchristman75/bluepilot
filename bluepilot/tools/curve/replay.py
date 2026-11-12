"""
BluePilot: model-in-the-loop kinematic replay of saved routes through the real curve limiter and lane-change gap.

Curve replay: the logged model plans are fed to the real SmartCruiseControlVisionBP while a kinematic car
follows the logged speed unless the limiter asks for less. The simulated car lags the logged one once it slows, so
the plan distances are shifted by the logged-minus-simulated distance (the curve is farther away for the slower
car). The model's later plans would differ in a real closed loop (it replans around the car's actual state), so
this is an estimate of what the limiter would have asked for, not a re-drive. Where the driver or op long was
not in control the simulated car snaps back to the logged one.

  python -m openpilot.bluepilot.tools.curve.replay curve 539 --t0 120 --t1 180
  python -m openpilot.bluepilot.tools.curve.replay cases
"""
import argparse
from dataclasses import dataclass

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.bluepilot.selfdrive.controls.curve_limit_bp import CurveLimiter
from openpilot.bluepilot.selfdrive.controls.lane_change_gap_bp import LaneChangeGap
from openpilot.bluepilot.tools.curve import route_data

FREE_GAIN = 1.0       # 1/s: how fast the simulated car returns to the logged speed when nothing limits it
FREE_ACCEL_MAX = 1.0  # m/s^2
FREE_DECEL_MAX = 3.0


@dataclass
class CurveReplay:
  t: np.ndarray
  v_log: np.ndarray
  v_sim: np.ndarray
  a_sim: np.ndarray
  a_lim: np.ndarray
  active: np.ndarray
  braking: np.ndarray
  lat_log: np.ndarray
  lat_sim: np.ndarray
  longact: np.ndarray

  def removed_mph(self, t0: float, t1: float) -> float:
    m = (self.t >= t0) & (self.t <= t1)
    return float((self.v_log[m] - self.v_sim[m]).max() / CV.MPH_TO_MS) if m.any() else 0.0

  def v_at(self, t: float):
    i = int(np.searchsorted(self.t, t))
    i = min(i, len(self.t) - 1)
    return self.v_log[i] / CV.MPH_TO_MS, self.v_sim[i] / CV.MPH_TO_MS

  def active_fraction(self) -> float:
    m = self.longact > 0
    return float(self.active[m].mean()) if m.any() else 0.0

  def min_accel(self) -> float:
    return float(self.a_lim.min())   # the limiter's own brake request, not the sim chasing the logged speed


def curve_replay(d: dict, t0: float, t1: float, comfort=None) -> CurveReplay:
  idx = np.where((d['t'] >= t0) & (d['t'] <= t1))[0]
  lim = CurveLimiter()
  n = len(idx)
  out = {k: np.zeros(n) for k in ('t', 'v_log', 'v_sim', 'a_sim', 'a_lim', 'active', 'braking', 'lat_log', 'lat_sim', 'longact')}
  v_sim, shift = float(d['v'][idx[0]]), 0.0
  for j, i in enumerate(idx):
    v_log = float(d['v'][i])
    in_control = d['longact'][i] > 0 and not d['gas'][i] and not d['brake'][i]
    blinker = bool(d['lblink'][i] or d['rblink'][i])
    if not in_control:
      v_sim, shift = v_log, 0.0
    cruise = max(float(d['vcruise'][i]) * CV.KPH_TO_MS, v_log)           # the set speed, never below what the car was doing
    r = lim.update(d['px'][i], d['py'][i], v_sim, cruise, enabled=bool(in_control), blinker=blinker, comfort=comfort, s_shift=shift)
    a_free = float(np.clip(FREE_GAIN * (v_log - v_sim), -FREE_DECEL_MAX, FREE_ACCEL_MAX))
    a = min(a_free, r.a_cmd) if r.braking else a_free
    if r.active and v_sim + a * DT_MDL > r.v_target:        # the ceiling holds while a curve binds
      a = min(a, (r.v_target - v_sim) / DT_MDL)
    a = max(a, -FREE_DECEL_MAX)
    v_new = max(v_sim + a * DT_MDL, 0.0)
    shift += (v_log - 0.5 * (v_sim + v_new)) * DT_MDL      # logged car's distance minus the simulated car's
    k = abs(float(d['kcur'][i]))
    for key, val in (('t', d['t'][i]), ('v_log', v_log), ('v_sim', v_sim), ('a_sim', a), ('a_lim', r.a_cmd),
                     ('active', float(r.active)), ('braking', float(r.braking)),
                     ('lat_log', v_log ** 2 * k), ('lat_sim', v_sim ** 2 * k), ('longact', float(in_control))):
      out[key][j] = val
    v_sim = v_new
  return CurveReplay(**out)


@dataclass
class LaneChangeReplay:
  t: np.ndarray
  t_follow: np.ndarray
  lc: np.ndarray

  def min_t_follow(self) -> float:
    return float(self.t_follow.min())

  def seconds_reduced(self, stock: float) -> float:
    return float((self.t_follow < stock - 0.02).sum() * DT_MDL)


def lane_change_replay(d: dict, t0: float, t1: float, stock: float = 1.45) -> LaneChangeReplay:
  """Run the real LaneChangeGap over the logged lane-change state, lead and blind spots."""
  idx = np.where((d['t'] >= t0) & (d['t'] <= t1))[0]
  g = LaneChangeGap()
  tf = np.zeros(len(idx))
  for j, i in enumerate(idx):
    tf[j] = g.update(stock, float(d['v'][i]), int(d['lcs'][i]), int(d['lcd'][i]), bool(d['lbs'][i]), bool(d['rbs'][i]),
                     bool(d['lead'][i]), bool(d['lead_radar'][i]), max(0.0, -float(d['lead_a'][i])), bool(d['standstill'][i]), True)
  return LaneChangeReplay(d['t'][idx], tf, d['lcs'][idx])


def segs_for(t0: float, t1: float):
  return list(range(max(int(t0 // 60), 0), int(t1 // 60) + 1))


# (route, event time, window) from the road-test notes
CURVE_CASES = {
  '539 t=173': ('00000539', 173.0, 140.0, 185.0),
  '537 t=274': ('00000537', 274.0, 240.0, 290.0),
  '53f t=232': ('0000053f', 232.0, 200.0, 245.0),
  '532 t=306': ('00000532', 306.0, 275.0, 320.0),
}
HIGHWAY_CASES = {
  '52d seg6-7': ('0000052d', 360.0, 480.0),
  '405 seg35': ('00000405', 2100.0, 2220.0),
}
LANE_CHANGE_CASES = {'532 t=531': ('00000532', 520.0, 545.0)}


def run_cases():
  for name, (route, te, t0, t1) in CURVE_CASES.items():
    d = route_data.load(route, segs_for(t0, t1))
    if d is None:
      print(f'{name}: route data not available')
      continue
    r = curve_replay(d, t0, t1)
    vl, vs = r.v_at(te)
    print(f'{name}: logged {vl:.1f} mph -> replay {vs:.1f} mph at the event, max removed {r.removed_mph(t0, t1):.1f} mph, ' +
          f'peak lat {r.lat_log.max():.2f} -> {r.lat_sim.max():.2f} m/s^2, min accel {r.min_accel():.2f}')
  for name, (route, t0, t1) in HIGHWAY_CASES.items():
    d = route_data.load(route, segs_for(t0, t1))
    if d is None:
      print(f'{name}: route data not available')
      continue
    r = curve_replay(d, t0, t1)
    print(f'{name}: max removed {r.removed_mph(t0, t1):.2f} mph, active {100 * r.active_fraction():.1f}% of op-long time, min accel {r.min_accel():.2f}')
  for name, (route, t0, t1) in LANE_CHANGE_CASES.items():
    d = route_data.load(route, segs_for(t0, t1))
    if d is None:
      print(f'{name}: route data not available')
      continue
    r = lane_change_replay(d, t0, t1)
    print(f'{name}: min follow time {r.min_t_follow():.2f} s, reduced for {r.seconds_reduced(1.45):.1f} s')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('mode', choices=['curve', 'lanechange', 'cases'])
  ap.add_argument('route', nargs='?')
  ap.add_argument('--t0', type=float, default=0.0)
  ap.add_argument('--t1', type=float, default=60.0)
  a = ap.parse_args()
  if a.mode == 'cases':
    return run_cases()
  d = route_data.load(a.route.zfill(8), segs_for(a.t0, a.t1))
  if d is None:
    raise SystemExit('route data not available')
  if a.mode == 'curve':
    r = curve_replay(d, a.t0, a.t1)
    print(f'max removed {r.removed_mph(a.t0, a.t1):.1f} mph, active {100 * r.active_fraction():.1f}%, min accel {r.min_accel():.2f}')
  else:
    r = lane_change_replay(d, a.t0, a.t1)
    print(f'min follow time {r.min_t_follow():.2f} s, reduced {r.seconds_reduced(1.45):.1f} s')


if __name__ == '__main__':
  main()

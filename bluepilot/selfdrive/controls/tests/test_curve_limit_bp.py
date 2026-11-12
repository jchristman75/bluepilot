"""Curve speed limit from the car's steering ability (choice A2): geometry, envelope, accel request, gating."""
import math

import numpy as np
import pytest

from openpilot.common.constants import CV
from openpilot.bluepilot.selfdrive.controls import curve_limit_bp as cl

MPH = CV.MPH_TO_MS
UNSET = 255 * CV.KPH_TO_MS


def arc(radius, start, length=140.0, step=2.0, sign=1.0):
  """Plan polyline: straight to `start` m, then a circular arc of `radius` (sign + = turning right, y positive)."""
  xs, ys = [0.0], [0.0]
  s = 0.0
  heading = 0.0
  x = y = 0.0
  while s < length:
    k = 0.0 if s < start else sign / radius
    heading += k * step
    x += step * math.cos(heading)
    y += step * math.sin(heading)
    s += step
    xs.append(x)
    ys.append(y)
  return np.array(xs), np.array(ys)


def straight(length=140.0, step=2.0):
  n = int(length / step) + 1
  return np.arange(n) * step, np.zeros(n)


def run(x, y, v_ego, v_cruise=UNSET, **kw):
  lim = kw.pop('limiter', None) or cl.CurveLimiter()
  res = None
  for _ in range(kw.pop('steps', 1)):
    res = lim.update(x, y, v_ego, v_cruise, **{'enabled': True, 'blinker': False, **kw})
  return lim, res


# ---------------- curvature profile ----------------

def test_straight_plan_has_no_curvature():
  s, k = cl.plan_curvature_profile(*straight())
  assert len(s) > 20 and np.all(k < 1e-6)


def test_circular_plan_reads_its_radius():
  s, k = cl.plan_curvature_profile(*arc(50.0, start=0.0))
  inner = k[(s > 10) & (s < 100)]
  assert np.allclose(inner, 1 / 50.0, rtol=0.05)


def test_profile_is_zero_before_the_curve_and_rises_at_it():
  s, k = cl.plan_curvature_profile(*arc(60.0, start=40.0))
  assert np.all(k[s < 36] < 0.002)
  assert np.all(k[s > 46] > 0.012)


def test_left_and_right_curves_give_the_same_magnitude():
  s1, k1 = cl.plan_curvature_profile(*arc(60.0, 30.0, sign=1.0))
  s2, k2 = cl.plan_curvature_profile(*arc(60.0, 30.0, sign=-1.0))
  assert np.allclose(k1, k2, atol=1e-6)


def test_standstill_or_tiny_plan_is_empty_not_garbage():
  for x, y in ((np.zeros(33), np.zeros(33)), (np.array([0.0, 1.0]), np.array([0.0, 0.0])), (np.array([]), np.array([]))):
    s, k = cl.plan_curvature_profile(x, y)
    assert len(s) == 0 and len(k) == 0


def test_nonfinite_plan_is_rejected():
  x, y = straight()
  y[10] = np.nan
  s, k = cl.plan_curvature_profile(x, y)
  assert len(s) == 0


# ---------------- steering ceiling and far-field ----------------

def test_ceiling_is_the_measured_one_and_blends_between_city_and_highway_speed():
  assert cl.ceiling_lat_accel(10.0) == pytest.approx(2.1)
  assert cl.ceiling_lat_accel(30.0) == pytest.approx(2.2)
  assert 2.1 < cl.ceiling_lat_accel(19.0) < 2.2
  assert cl.ceiling_lat_accel(30.0) < 2.4            # tracking falls off above ~2.4 m/s^2


def test_far_field_gain_only_on_firm_far_curves():
  s = np.array([10.0, 30.0, 45.0, 45.0])
  k = np.array([0.01, 0.01, 0.01, 0.002])
  g = cl.far_field_gain(s, k)
  assert g[0] == pytest.approx(1.0)
  assert 1.0 < g[1] < cl.FAR_GAIN_MAX
  assert g[2] == pytest.approx(cl.FAR_GAIN_MAX)
  assert g[3] == pytest.approx(1.0)   # gentle curves are not scaled


# ---------------- target and accel request ----------------

def test_straight_road_asks_for_nothing():
  lim, r = run(*straight(), v_ego=30.0)
  assert not r.active and not r.braking and r.a_req == 0.0 and r.v_target == UNSET


def test_sharp_curve_ahead_at_speed_requests_the_brakes():
  # R 47 m starting 60 m ahead at 30 mph: curve speed ~22 mph, needs ~0.7 m/s^2
  x, y = arc(47.0, start=60.0)
  lim, r = run(x, y, v_ego=30 * MPH)
  assert r.active and r.v_raw < 30 * MPH
  assert -1.0 <= r.a_req < -0.4
  assert 40.0 < r.bind_distance < 130.0


def test_curve_speed_matches_the_ceiling():
  # inside 20 m there is no far-field gain: curve speed = sqrt(a_max / k) with k = 1/47
  x, y = arc(47.0, start=4.0)
  _, r = run(x, y, v_ego=20 * MPH)
  assert r.bind_speed == pytest.approx(math.sqrt(cl.ceiling_lat_accel(10.0) * 47.0), rel=0.06)
  # a curve 34 m away reads 1.2x tighter: the model under-reads far curves
  x, y = arc(47.0, start=30.0)
  _, far = run(x, y, v_ego=20 * MPH)
  assert far.bind_speed == pytest.approx(r.bind_speed / math.sqrt(cl.FAR_GAIN_MAX), rel=0.04)


def test_no_braking_when_already_slow_enough():
  x, y = arc(47.0, start=60.0)
  _, r = run(x, y, v_ego=18 * MPH)
  assert not r.braking and r.a_req == 0.0 and r.a_cmd == 0.0
  assert r.active and r.v_raw >= 18 * MPH           # the ceiling is on (the curve binds) but the car is under it


def test_target_never_drops_below_the_floor():
  x, y = arc(8.0, start=20.0)
  _, r = run(x, y, v_ego=35 * MPH)
  assert r.v_raw >= cl.V_FLOOR - 1e-6


def test_farther_curves_allow_more_speed_now():
  _, near = run(*arc(47.0, start=20.0), v_ego=40 * MPH)
  _, far = run(*arc(47.0, start=100.0), v_ego=40 * MPH)
  assert far.v_raw > near.v_raw


def test_request_is_capped_even_for_a_curve_right_ahead():
  _, r = run(*arc(30.0, start=8.0), v_ego=45 * MPH)
  assert r.a_req >= -cl.DECEL_CAP - 1e-9


def test_target_is_never_above_cruise_and_accel_never_positive():
  rng = np.random.default_rng(0)
  for _ in range(300):
    radius = rng.uniform(15, 600)
    start = rng.uniform(5, 120)
    v = rng.uniform(0, 35)
    cruise = rng.uniform(5, 40)
    lim, r = run(*arc(radius, start), v_ego=v, v_cruise=cruise)
    assert r.a_req <= 0.0 and r.v_target <= cruise + 1e-9
    assert np.isfinite(r.v_target) and np.isfinite(r.a_req)


# ---------------- far-field behaviour on a realistic highway sweeper ----------------

def test_gentle_highway_sweeper_that_the_car_tracks_is_left_alone():
  # R 500 m at 70 mph: 1.9 m/s^2, below the highway ceiling 2.2 -> no braking
  _, r = run(*arc(500.0, start=60.0), v_ego=70 * MPH)
  assert not r.braking and r.a_req == 0.0


def test_tighter_highway_curve_is_trimmed():
  # R 400 m at 70 mph needs 2.45 m/s^2, over the ceiling
  _, r = run(*arc(400.0, start=60.0), v_ego=70 * MPH)
  assert r.v_raw < 70 * MPH


# ---------------- gating ----------------

def test_off_switch_blinker_and_low_speed_disable_it():
  x, y = arc(47.0, start=60.0)
  for kw in ({'enabled': False}, {'blinker': True}):
    _, r = run(x, y, v_ego=30 * MPH, **kw)
    assert not r.active and not r.braking and r.a_req == 0.0 and r.v_target == UNSET
  _, r = run(x, y, v_ego=cl.V_ACTIVE_MIN - 0.5)
  assert not r.active and not r.braking


def test_gates_reset_the_state():
  x, y = arc(47.0, start=60.0)
  lim, r = run(x, y, v_ego=30 * MPH, steps=40)
  assert lim.a_cmd < 0
  _, r2 = run(x, y, v_ego=30 * MPH, limiter=lim, blinker=True)
  assert lim.a_cmd == 0.0 and not r2.active and not r2.braking


# ---------------- smoothing ----------------

def test_brake_request_builds_at_a_limited_jerk_and_releases_after_a_hold():
  x, y = arc(47.0, start=60.0)
  lim = cl.CurveLimiter()
  cmds = []
  for _ in range(60):
    _, r = run(x, y, v_ego=30 * MPH, limiter=lim)
    cmds.append(r.a_cmd)
  steps = np.diff(cmds)
  assert cmds[0] > -0.1                             # no step at onset
  assert steps.min() >= -cl.JERK_DOWN * cl.DT - 1e-9
  assert cmds[-1] < -0.4
  # curve gone (straight plan): request releases to zero, after the hold
  held = []
  for _ in range(int(2.0 / cl.DT)):
    _, r = run(*straight(), v_ego=30 * MPH, limiter=lim)
    held.append(r.a_cmd)
  assert held[0] < 0.0 and held[-1] == 0.0


def test_reported_target_moves_at_a_limited_rate():
  x, y = arc(47.0, start=60.0)
  lim = cl.CurveLimiter()
  seen = []
  run(*straight(), v_ego=30 * MPH, limiter=lim, v_cruise=35 * MPH)
  for _ in range(30):
    _, r = run(x, y, v_ego=30 * MPH, limiter=lim, v_cruise=35 * MPH)
    assert r.active
    seen.append(r.v_target)
  assert len(seen) == 30 and np.diff(seen).min() >= -cl.TARGET_DOWN_RATE * cl.DT - 1e-9
  assert seen[-1] < seen[0]


# ---------------- comfort layer hook (A3) ----------------

def test_a_gentler_learned_comfort_lowers_the_target_but_never_raises_it():
  x, y = arc(47.0, start=60.0)
  _, base = run(x, y, v_ego=30 * MPH)
  _, gentle = run(x, y, v_ego=30 * MPH, comfort=lambda k: np.full_like(np.asarray(k, float), 1.5))
  _, generous = run(x, y, v_ego=30 * MPH, comfort=lambda k: np.full_like(np.asarray(k, float), 3.5))
  assert gentle.v_raw < base.v_raw
  assert generous.v_raw == pytest.approx(base.v_raw)       # the car ceiling still caps it


def test_distance_shift_moves_the_curve_farther_away():
  x, y = arc(47.0, start=40.0)
  _, near = run(x, y, v_ego=35 * MPH)
  _, shifted = run(x, y, v_ego=35 * MPH, s_shift=25.0)
  assert shifted.a_req >= near.a_req

"""Replay of saved routes through the real curve limiter and lane-change gap (skipped when the route data is absent).
Set BP_ROUTES_DIR to the folder holding the dated route directories."""
import pytest

from openpilot.bluepilot.tools.curve import replay as rp
from openpilot.bluepilot.tools.curve import route_data as rd

def load(route, t0, t1):
  d = rd.load(route, rp.segs_for(t0, t1))
  if d is None:
    pytest.skip(f'route {route} not available in {rd.ROUTES_DIR}')
  return d

@pytest.mark.parametrize('case', ['537 t=274', '53f t=232'])
def test_the_curves_that_cost_a_grab_get_at_least_3_mph_off_before_the_event(case):
  route, te, t0, t1 = rp.CURVE_CASES[case]
  r = rp.curve_replay(load(route, t0, t1), t0, t1)
  v_log, v_sim = r.v_at(te)
  assert v_log - v_sim >= 3.0
  assert r.min_accel() >= -1.0               # gentle: never beyond the limiter's cap

def test_539_curve_is_slowed_but_less_than_3_mph_at_the_event():
  # Honest result: the curve speed (~20 mph floor) is close to what the car drove (23.5 mph), so the plan's 3 mph
  # goal is not reachable here. Pin what the replay shows so a regression is visible.
  route, te, t0, t1 = rp.CURVE_CASES['539 t=173']
  r = rp.curve_replay(load(route, t0, t1), t0, t1)
  v_log, v_sim = r.v_at(te)
  assert 0.0 < v_log - v_sim < 3.0
  assert r.removed_mph(t0, t1) >= 2.0

def test_tracked_highway_sweepers_are_left_alone():
  route, t0, t1 = rp.HIGHWAY_CASES['52d seg6-7']
  r = rp.curve_replay(load(route, t0, t1), t0, t1)
  assert r.removed_mph(t0, t1) < 1.0
  assert r.active_fraction() < 0.10

def test_replay_is_a_noop_when_op_long_is_off():
  route, t0, t1 = rp.HIGHWAY_CASES['405 seg35']
  r = rp.curve_replay(load(route, t0, t1), t0, t1)
  assert r.removed_mph(t0, t1) == 0.0 and r.active_fraction() == 0.0

def test_lane_change_gap_shrinks_on_the_logged_lane_change_and_never_exceeds_stock():
  route, t0, t1 = rp.LANE_CHANGE_CASES['532 t=531']
  r = rp.lane_change_replay(load(route, t0, t1), t0, t1)
  assert r.min_t_follow() >= 0.9
  assert r.t_follow.max() <= 1.45 + 1e-9
  assert r.seconds_reduced(1.45) > 0.0

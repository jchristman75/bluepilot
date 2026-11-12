"""Lane-change close gap (choice C): the follow time shrinks, never grows, only when a lane change is safe."""
import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.bluepilot.selfdrive.controls import lane_change_gap_bp as lg

LCS = log.LaneChangeState
LCD = log.LaneChangeDirection
MPH = CV.MPH_TO_MS
STOCK = 1.45


def call(g, **kw):
  args = {"t_follow": STOCK, "v_ego": 30 * MPH, "lane_change_state": LCS.laneChangeStarting, "direction": LCD.left,
              "left_blindspot": False, "right_blindspot": False, "lead_present": True, "lead_radar": True, "lead_brake": 0.0,
              "standstill": False, "enabled": True}
  args.update(kw)
  return g.update(**args)


def settle(g, n=100, **kw):
  t = None
  for _ in range(n):
    t = call(g, **kw)
  return t


def test_off_or_no_lane_change_changes_nothing():
  g = lg.LaneChangeGap()
  assert settle(g, enabled=False) == STOCK
  assert settle(g, lane_change_state=LCS.off) == STOCK


def test_the_gap_shrinks_toward_the_request_during_a_lane_change():
  g = lg.LaneChangeGap()
  t = settle(g)
  assert t == pytest.approx(lg.REQUEST_S)


def test_the_states_that_count():
  for state, expected in ((LCS.preLaneChange, True), (LCS.laneChangeStarting, True), (LCS.laneChangeFinishing, True), (LCS.off, False)):
    t = settle(lg.LaneChangeGap(), lane_change_state=state)
    assert (t < STOCK) == expected, state


def test_it_ramps_in_slowly_and_out_fast():
  g = lg.LaneChangeGap()
  seq = [call(g) for _ in range(10)]
  steps_in = np.diff([STOCK] + seq)
  assert steps_in.min() >= -lg.RAMP_IN * lg.DT - 1e-9 and seq[-1] < STOCK
  settle(g)
  out = [call(g, lane_change_state=LCS.off) for _ in range(5)]
  assert np.diff([lg.REQUEST_S] + out).max() <= lg.RAMP_OUT * lg.DT + 1e-9
  assert settle(g, n=40, lane_change_state=LCS.off) == STOCK
  # in is slower than out
  assert lg.RAMP_IN < lg.RAMP_OUT


def test_never_longer_than_stock_and_never_below_the_floor():
  rng = np.random.default_rng(1)
  g = lg.LaneChangeGap()
  for _ in range(2000):
    stock = float(rng.choice([1.25, 1.45, 1.75]))
    t = call(g, t_follow=stock, lane_change_state=int(rng.choice([0, 1, 2, 3])), v_ego=float(rng.uniform(0, 40)),
             left_blindspot=bool(rng.random() < 0.3), lead_brake=float(rng.uniform(0, 2)), lead_radar=bool(rng.random() < 0.7))
    assert t <= stock + 1e-12
    assert t >= min(stock, lg.MIN_T_FOLLOW) - 1e-12


def test_a_stock_gap_already_shorter_than_the_request_is_left_alone():
  assert settle(lg.LaneChangeGap(), t_follow=0.95) == pytest.approx(0.95)


def test_blind_spot_on_the_lane_change_side_blocks_it_but_the_other_side_does_not():
  assert settle(lg.LaneChangeGap(), direction=LCD.left, left_blindspot=True) == STOCK
  assert settle(lg.LaneChangeGap(), direction=LCD.right, right_blindspot=True) == STOCK
  assert settle(lg.LaneChangeGap(), direction=LCD.left, right_blindspot=True) < STOCK
  # direction unknown: either side blocks
  assert settle(lg.LaneChangeGap(), direction=LCD.none, right_blindspot=True) == STOCK


def test_other_safety_gates():
  assert settle(lg.LaneChangeGap(), v_ego=lg.MIN_SPEED - 0.5) == STOCK
  assert settle(lg.LaneChangeGap(), standstill=True) == STOCK
  assert settle(lg.LaneChangeGap(), lead_brake=lg.ABORT_LEAD_BRAKE + 0.01) == STOCK
  assert settle(lg.LaneChangeGap(), lead_radar=False) == STOCK       # a vision-only distance is too noisy to close in on
  assert settle(lg.LaneChangeGap(), lead_present=False) == STOCK


def test_a_safety_trip_snaps_back_quickly():
  g = lg.LaneChangeGap()
  settle(g)
  t = [call(g, left_blindspot=True) for _ in range(12)]
  assert t[-1] == STOCK


def test_personalities_all_shrink_to_the_same_request_but_not_below_it():
  for stock in (1.25, 1.45, 1.75):
    t = settle(lg.LaneChangeGap(), t_follow=stock)
    assert t == pytest.approx(min(stock, lg.REQUEST_S))

"""The lane-change close gap inside the real LongitudinalPlanner and MPC, closed loop behind a lead (choice C)."""
import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.bluepilot.selfdrive.controls import lane_change_gap_bp as lg
from openpilot.bluepilot.selfdrive.controls.tests.planner_sim import LaneChangeDirection as LCD
from openpilot.bluepilot.selfdrive.controls.tests.planner_sim import LaneChangeState as LCS
from openpilot.bluepilot.selfdrive.controls.tests.planner_sim import PlannerSim, Road

STANDARD = log.LongitudinalPersonality.standard
ON = {'BPLaneChangeCloseGap': True, 'SmartCruiseControlVision': False}
OFF = {'SmartCruiseControlVision': False}
V = 29.0       # 65 mph


def scenario(bp, lc_s=10.0, state=LCS.laneChangeStarting, direction=LCD.left, blind=(False, False), lead_brake=0.0, personality=STANDARD):
  """Follow a lead at 65 mph to steady state, change lanes for lc_s seconds, then back to normal for 25 s."""
  sim = PlannerSim(Road([(4000.0, 0.0)]), V, 35.0, bp, lead_gap=70.0, lead_speed=V, personality=personality)
  sim.step(int(40 / 0.05))
  out = {'g0': sim.lead_gap}
  sim.lane_change, sim.blinker, sim.blindspot = (state, direction), (direction == LCD.left, direction == LCD.right), blind
  n0 = len(sim.log)
  for _i in range(int(lc_s / 0.05)):
    if lead_brake:
      sim.lead_speed = max(sim.lead_speed - lead_brake * 0.05, 0.0)
    sim.step()
  out.update(gap_lc=sim.lead_gap, v_lc=sim.speed, tf_lc=sim.log[-1]['t_follow'], amin_lc=min(r['a'] for r in sim.log[n0:]))
  sim.lane_change, sim.blinker = (LCS.off, LCD.none), (False, False)
  n1 = len(sim.log)
  sim.step(int(25 / 0.05))
  out.update(gap_after=sim.lead_gap, amin_after=min(r['a'] for r in sim.log[n1:]), fcw=sim.planner.fcw, log=sim.log)
  return out


def test_flag_off_is_the_stock_planner():
  a, b = scenario(OFF), scenario({})
  assert a['tf_lc'] == b['tf_lc']
  assert a['gap_lc'] == pytest.approx(b['gap_lc'], abs=0.01)


def test_the_follow_time_drops_to_the_request_only_while_changing_lanes():
  r = scenario(ON)
  assert r['tf_lc'] == pytest.approx(lg.REQUEST_S, abs=0.02)
  tail = [x['t_follow'] for x in r['log'][-40:]]
  assert tail[-1] == pytest.approx(1.45, abs=0.02)                   # standard personality again afterwards


def test_the_car_closes_up_behind_the_lead_during_the_lane_change():
  off, on = scenario(OFF), scenario(ON)
  assert on['gap_lc'] < off['gap_lc'] - 3.0
  assert on['v_lc'] <= V + 2.0                                       # no rush: closing speed stays small
  assert not on['fcw']


def test_the_gap_reopens_gently_afterward():
  on = scenario(ON)
  assert on['amin_after'] > -1.0
  assert on['gap_after'] > on['gap_lc']
  assert not on['fcw']


def test_a_car_in_the_blind_spot_on_that_side_cancels_it_completely():
  off = scenario(OFF)
  on = scenario(ON, blind=(True, False))
  assert on['tf_lc'] == pytest.approx(off['tf_lc'], abs=0.01)
  assert on['gap_lc'] == pytest.approx(off['gap_lc'], abs=0.3)


def test_the_other_sides_blind_spot_does_not():
  on = scenario(ON, blind=(False, True))
  assert on['tf_lc'] == pytest.approx(lg.REQUEST_S, abs=0.02)


def test_a_braking_lead_aborts_it_and_the_car_brakes_normally():
  off = scenario(OFF, lc_s=6.0, lead_brake=1.0)
  on = scenario(ON, lc_s=6.0, lead_brake=1.0)
  assert on['tf_lc'] >= 1.35                                         # the reduction was cancelled when the lead braked
  assert on['amin_lc'] >= off['amin_lc'] - 0.3                       # and braking is no weaker than stock
  assert not on['fcw']


def test_every_personality_is_never_longer_than_its_own_stock_time():
  for p in (log.LongitudinalPersonality.aggressive, log.LongitudinalPersonality.standard, log.LongitudinalPersonality.relaxed):
    r = scenario(ON, personality=p, lc_s=6.0)
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
    assert max(x['t_follow'] for x in r['log']) <= get_T_FOLLOW(p) + 1e-9
    assert np.isfinite([x['a'] for x in r['log']]).all()

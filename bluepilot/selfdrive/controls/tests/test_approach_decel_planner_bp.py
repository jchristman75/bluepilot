"""Soft approach inside the real LongitudinalPlanner and MPC, closed loop behind a lead on the highway."""
import pytest

from openpilot.bluepilot.selfdrive.controls.tests.planner_sim import PlannerSim, Road

V = 29.0       # 65 mph
OFF = {'SmartCruiseControlVision': False, 'BPSoftApproach': False}
ON = {'SmartCruiseControlVision': False, 'BPSoftApproach': True}


def run(bp, d0, v_lead, secs=40.0, lead_brake=0.0, brake_at=0.0):
  sim = PlannerSim(Road([(6000.0, 0.0)]), V, 31.0, bp, lead_gap=d0, lead_speed=v_lead)
  for _ in range(int(secs / 0.05)):
    if lead_brake and sim.t >= brake_at:
      sim.lead_speed = max(sim.lead_speed - lead_brake * 0.05, 0.0)
    sim.step()
  log = sim.log
  return {'amin': min(r['a'] for r in log), 'gmin': min(r['lead_gap'] for r in log), 'gend': log[-1]['lead_gap'], 'fcw': sim.planner.fcw,
          'soft_cap_used': sim.planner.soft_approach.last}


def test_a_far_slower_lead_is_approached_with_less_decel_than_stock_and_a_safe_gap():
  stock, soft = run(OFF, 110.0, 22.0), run(ON, 110.0, 22.0)
  assert soft['amin'] > stock['amin'] + 0.15                      # gentler
  assert soft['gmin'] > 20.0 and not soft['fcw']                  # and still a safe gap, never close to impact
  assert soft['amin'] > -0.6


def test_a_faster_closing_lead_is_still_approached_gently_and_safely():
  stock, soft = run(OFF, 90.0, 18.0), run(ON, 90.0, 18.0)         # closing 11 m/s from 90 m
  assert soft['amin'] > stock['amin'] + 0.4
  assert soft['amin'] > -1.4                                      # inside what the regen floor covers: no friction brakes
  assert soft['gmin'] > 15.0 and not soft['fcw']


def test_a_cut_in_is_left_entirely_to_the_planner():
  stock, soft = run(OFF, 30.0, 22.0), run(ON, 30.0, 22.0)
  assert soft['amin'] == pytest.approx(stock['amin'], abs=0.01)
  assert soft['gmin'] == pytest.approx(stock['gmin'], abs=0.1)


def test_a_lead_that_brakes_hard_is_left_entirely_to_the_planner():
  stock, soft = run(OFF, 120.0, 28.0, 50.0, lead_brake=3.0, brake_at=8.0), run(ON, 120.0, 28.0, 50.0, lead_brake=3.0, brake_at=8.0)
  assert soft['amin'] == pytest.approx(stock['amin'], abs=0.01)
  assert soft['gmin'] == pytest.approx(stock['gmin'], abs=0.1) and soft['gmin'] > 0.0


def test_the_flag_off_is_the_stock_planner():
  a, b = run(OFF, 90.0, 18.0), run({'SmartCruiseControlVision': False}, 90.0, 18.0)
  assert a['amin'] == pytest.approx(b['amin'], abs=1e-6) and a['soft_cap_used'] is None


def test_a_slower_lead_at_a_comfortable_gap_is_followed_not_braked_for():
  soft = run(ON, 60.0, 28.0)                                      # already pacing close to the planner's gap
  assert soft['amin'] > -0.8 and not soft['fcw']

"""Smooth following inside the real LongitudinalPlanner and MPC, closed loop behind a lead."""
import numpy as np
import pytest

from openpilot.bluepilot.selfdrive.controls.tests.planner_sim import PlannerSim, Road

OFF = {'SmartCruiseControlVision': False, 'BPSmoothFollow': False}
ON = {'SmartCruiseControlVision': False, 'BPSmoothFollow': True, 'BPFollowGapScale': 0.75}


def run(bp, v0, d0, lead_speed, secs=60.0, lead_fn=None):
  sim = PlannerSim(Road([(8000.0, 0.0)]), v0, 30.0, bp, lead_gap=d0, lead_speed=lead_speed(0.0) if lead_fn is None else lead_fn(0.0), personality=1)
  for _ in range(int(secs / 0.05)):
    if lead_fn is not None:
      sim.lead_speed = lead_fn(sim.t)
    sim.step()
  L = sim.log
  v, a, g = (np.array([r[k] for r in L]) for k in ('v', 'a', 'lead_gap'))
  return {'a': a, 'v': v, 'gap': g, 'tgap': g / np.maximum(v, 1.0), 'amin': a.min(), 'gmin': g.min(), 'fcw': sim.planner.fcw}


def bus(t):
  """A stop-and-go bus at ~35 mph: +-4 mph wobble every 8 s."""
  return 15.6 + 1.8 * np.sin(2 * np.pi * t / 8.0)


def test_following_a_wobbling_bus_is_calmer_and_a_little_closer():
  off = run(OFF, 15.6, 30.0, None, secs=120.0, lead_fn=bus)
  on = run(ON, 15.6, 30.0, None, secs=120.0, lead_fn=bus)
  s = slice(int(40 / 0.05), None)                                  # after the start-up transient
  assert on['a'][s].std() < off['a'][s].std()                      # calmer
  assert np.median(on['tgap'][s]) < np.median(off['tgap'][s]) - 0.05  # and a little closer in time
  assert on['a'][s].std() < 0.8 * off['a'][s].std()                # a real reduction, not noise
  assert on['gmin'] > 15.0 and not on['fcw']


def test_a_lead_that_brakes_hard_is_still_safe():
  def lead(t):
    return 15.6 if t < 10.0 else max(15.6 - 3.0 * (t - 10.0), 0.0)
  off = run(OFF, 15.6, 30.0, None, secs=40.0, lead_fn=lead)
  on = run(ON, 15.6, 30.0, None, secs=40.0, lead_fn=lead)
  assert on['gmin'] > 1.0 and on['gmin'] > off['gmin'] - 2.0       # never much closer than stock, never close to impact
  assert on['amin'] > off['amin'] - 0.6 and not on['fcw']


def test_a_cut_in_is_handled_like_stock():
  off = run(OFF, 20.0, 12.0, lambda t: 17.0, secs=25.0)
  on = run(ON, 20.0, 12.0, lambda t: 17.0, secs=25.0)
  assert on['gmin'] > 4.0 and on['gmin'] > off['gmin'] - 2.0 and not on['fcw']
  assert on['amin'] > off['amin'] - 0.5


def test_the_flag_off_is_the_stock_planner():
  a = run(OFF, 15.6, 38.0, None, secs=30.0, lead_fn=bus)
  b = run({'SmartCruiseControlVision': False}, 15.6, 38.0, None, secs=30.0, lead_fn=bus)
  assert np.allclose(a['a'], b['a'])


def test_the_follow_gap_scale_param_is_honored_and_clamped():
  mild = run({**ON, 'BPFollowGapScale': 1.0}, 15.6, 38.0, None, secs=40.0, lead_fn=lambda t: 15.6)
  tight = run({**ON, 'BPFollowGapScale': 0.7}, 15.6, 38.0, None, secs=40.0, lead_fn=lambda t: 15.6)
  too_tight = run({**ON, 'BPFollowGapScale': 0.3}, 15.6, 38.0, None, secs=40.0, lead_fn=lambda t: 15.6)   # clamped to 0.6 (the floor then gives the same 1.0 s)
  assert tight['gap'][-1] < mild['gap'][-1] - 3.0
  assert too_tight['gap'][-1] == pytest.approx(tight['gap'][-1], abs=0.5)

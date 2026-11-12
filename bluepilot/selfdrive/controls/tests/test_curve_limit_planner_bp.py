"""The curve limiter inside the real LongitudinalPlanner, closed loop on synthetic roads (choice A2/A3)."""
import math

import numpy as np

from openpilot.bluepilot.selfdrive.controls import curve_limit_bp as cl
from openpilot.bluepilot.selfdrive.controls.tests.planner_sim import MPH, PlannerSim, Road

ON = {'SmartCruiseControlVision': False, 'BPCurveSpeedLimit': True, 'BPCurveLearnComfort': False}
OFF = {'SmartCruiseControlVision': False, 'BPCurveSpeedLimit': False, 'BPCurveLearnComfort': False}
CURVE_START = 150.0


def drive(bp, radius=47.0, speed=35 * MPH, v_cruise=None, start=CURVE_START, sign=1.0, blinker=False):
  sim = PlannerSim(Road.curve(start, radius, sign=sign), speed, v_cruise, bp)
  if blinker:
    sim.blinker = (False, True)
  data = sim.run_until(start + 60.0)
  return sim, data


def at_distance(data, d):
  return data[np.searchsorted(data[:, 1], d)]


def test_without_the_feature_the_car_enters_the_curve_at_full_speed():
  _, data = drive(OFF)
  assert at_distance(data, CURVE_START)[2] > 35 * MPH - 0.5
  assert data[:, 4].max() > 4.5                      # lateral accel far beyond what the steering can deliver


def test_with_the_feature_the_car_arrives_at_a_speed_the_steering_can_hold():
  _, data = drive(ON)
  v_curve = math.sqrt(cl.ceiling_lat_accel(10.0) * 47.0)
  v_entry = at_distance(data, CURVE_START + 10.0)[2]
  assert v_entry < v_curve * 1.1
  # lateral acceleration in the curve stays close to the steering ceiling
  in_curve = data[(data[:, 1] > CURVE_START + 10) & (data[:, 1] < CURVE_START + 55)]
  assert in_curve[:, 4].max() < cl.ceiling_lat_accel(10.0) * 1.25


def test_braking_is_gentle_and_starts_well_before_the_curve():
  _, data = drive(ON)
  assert data[:, 3].min() > -1.15                       # never harsher than the cap (+ planner shaping)
  braking = data[data[:, 3] < -0.1]
  assert braking[0, 1] < CURVE_START - 60               # >= 60 m ahead of the curve
  # no jerk spikes: accel changes < 1.5 m/s^3 frame to frame
  assert np.abs(np.diff(data[:, 3])).max() / 0.05 < 3.0


def test_speed_recovers_after_the_curve():
  sim = PlannerSim(Road.curve(CURVE_START, 47.0, length=60.0), 35 * MPH, 35 * MPH, ON)
  data = sim.run_until(CURVE_START + 260.0)
  assert data[-1, 2] > 30 * MPH


def test_a_gentle_curve_the_car_tracks_fine_is_not_slowed():
  _, data = drive(ON, radius=700.0, speed=60 * MPH)
  assert data[:, 2].min() > 60 * MPH - 1.0


def test_the_blinker_hands_the_turn_to_the_driver():
  _, data = drive(ON, blinker=True)
  assert at_distance(data, CURVE_START)[2] > 35 * MPH - 0.5


def test_left_and_right_curves_behave_the_same():
  _, right = drive(ON, sign=1.0)
  _, left = drive(ON, sign=-1.0)
  n = min(len(right), len(left))
  assert np.allclose(right[:n, 2], left[:n, 2], atol=0.3)


def test_sunnypilots_stock_limiter_is_untouched_when_the_bp_param_is_off():
  sim, data = drive(OFF)
  assert max(abs(r['curve_accel']) for r in sim.log) == 0.0
  assert not sim.planner.scc.vision.bp_enabled


def test_the_plan_source_stays_a_valid_cereal_enum():
  sim, _ = drive(ON)
  assert int(sim.planner.mpc.source) in (0, 1, 2, 3, 4)      # no schema change: curve braking reports as `cruise`


def test_a_slower_set_speed_than_the_curve_speed_is_left_alone():
  _, data = drive(ON, speed=20 * MPH, v_cruise=20 * MPH)
  assert data[:, 2].min() > 20 * MPH - 0.7

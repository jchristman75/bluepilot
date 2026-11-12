"""SmartCruiseControlVisionBP: the curve limiter in sunnypilot's SCC-V slot, params, gating, learning, persistence."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.bluepilot.selfdrive.controls import curve_scc_bp as scc_bp
from openpilot.bluepilot.selfdrive.controls.curve_comfort_bp import SAVE_PERIOD_S, load_comfort
from openpilot.bluepilot.selfdrive.controls.tests.test_curve_limit_bp import arc, straight

MPH = CV.MPH_TO_MS
VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
CRUISE = 70 * MPH


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)
    self.writes = {}

  def get_bool(self, key, block=False):
    if key not in self.values:
      raise KeyError(key)
    return bool(self.values[key])

  def get(self, key, block=False, return_default=False):
    return self.values.get(key)

  def put(self, key, val, block=False):
    self.writes[key] = val
    self.values[key] = val


class UnknownKey(Exception):
  """openpilot.common.params.UnknownKeyName is a plain Exception subclass."""


class StaleParams(FakeParams):
  def get_bool(self, key, block=False):
    if key.startswith("BP"):
      raise UnknownKey(key)
    return super().get_bool(key)


def make(params=None, **flags):
  s = scc_bp.SmartCruiseControlVisionBP()
  s.params = params or FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=True, BPCurveLearnComfort=False, **flags)
  s.frame = -1            # force a param read on the first update
  s._read_bp_params()
  return s


def sm_for(plan, curvature=0.0, blinker=False, brake=False, lead=False):
  x, y = plan
  return {
    'modelV2': NS(position=NS(x=list(x), y=list(y)), orientationRate=NS(z=[0.0] * 33), velocity=NS(x=[20.0] * 33)),
    'controlsState': NS(curvature=curvature),
    'carState': NS(leftBlinker=blinker, rightBlinker=False, brakePressed=brake, gasPressed=False),
    'radarState': NS(leadOne=NS(present=lead, dRel=30.0, vLead=20.0)),
  }


def step(s, plan, v_ego, long_enabled=True, long_override=False, v_cruise=CRUISE, a_ego=0.0, **kw):
  s.update(sm_for(plan, **kw), long_enabled, long_override, v_ego, a_ego, v_cruise)
  return s


# ---------------- params ----------------

def test_off_means_stock_behaviour_and_no_curve_request():
  s = make(FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=False, BPCurveLearnComfort=False))
  step(s, arc(47.0, 60.0), 30 * MPH)
  assert s.curve_accel == 0.0 and not s.is_active
  assert s.output_v_target == V_CRUISE_UNSET


def test_an_unknown_bp_key_is_treated_as_off_instead_of_crashing():
  s = scc_bp.SmartCruiseControlVisionBP()
  s.params = StaleParams(SmartCruiseControlVision=False)
  s.frame = -1
  step(s, arc(47.0, 60.0), 30 * MPH)          # must not raise
  assert s.curve_accel == 0.0 and not s.bp_enabled


def test_params_are_reread_about_every_few_seconds_not_every_frame():
  p = FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=False, BPCurveLearnComfort=False)
  s = make(p)
  s.frame = 0
  step(s, straight(), 30 * MPH)
  p.values['BPCurveSpeedLimit'] = True
  step(s, straight(), 30 * MPH)
  assert not s.bp_enabled
  s.frame = 3 * 20                             # a refresh frame (every 3 s at 20 Hz)
  step(s, straight(), 30 * MPH)
  assert s.bp_enabled


# ---------------- behaviour ----------------

def test_sharp_curve_ahead_sets_the_speed_target_and_the_brake_request():
  s = make()
  step(s, arc(47.0, 60.0), 30 * MPH)
  assert s.is_active and s.is_enabled
  assert s.curve_accel < 0.0
  assert s.output_v_target < V_CRUISE_UNSET
  assert s.output_v_target < 40 * MPH
  assert s.state in (VisionState.entering, VisionState.turning)
  assert s.max_pred_lat_acc > 2.0


def test_the_planner_state_is_not_perturbed():
  s = make()
  step(s, arc(47.0, 60.0), 30 * MPH, a_ego=-0.2)
  assert s.output_a_target == pytest.approx(-0.2)       # a_target passes through; the brake goes via curve_accel


def test_straight_road_is_inactive():
  s = make()
  step(s, straight(), 30 * MPH)
  assert not s.is_active and s.curve_accel == 0.0
  assert s.output_v_target == V_CRUISE_UNSET and s.state == VisionState.enabled


def test_long_off_override_and_blinker_gate_it():
  plan = arc(47.0, 60.0)
  s = make()
  step(s, plan, 30 * MPH, long_enabled=False)
  assert s.state == VisionState.disabled and s.curve_accel == 0.0 and not s.is_active
  s = make()
  step(s, plan, 30 * MPH, long_override=True)
  assert s.state == VisionState.overriding and s.curve_accel == 0.0
  s = make()
  step(s, plan, 30 * MPH, blinker=True)
  assert s.curve_accel == 0.0 and not s.is_active


def test_stock_toggle_still_works_when_the_bp_param_is_off():
  s = make(FakeParams(SmartCruiseControlVision=True, BPCurveSpeedLimit=False, BPCurveLearnComfort=False))
  s.frame = 60                                    # a refresh frame: the stock class reads its own toggle
  step(s, straight(), 30 * MPH)
  assert s.state != VisionState.disabled          # stock SCC-V state machine ran (enabled)
  assert s.curve_accel == 0.0


# ---------------- A3: learning and persistence ----------------

def drive_curve_manually(s, v=14.0, k=0.008, frames=400, **kw):
  for _ in range(frames):
    step(s, straight(), v, long_enabled=False, curvature=k, **kw)


def test_learning_only_when_enabled_and_the_driver_is_in_charge():
  k, v = 0.008, 14.0
  off = make()
  drive_curve_manually(off, v, k)
  assert not off.comfort.dirty
  on = make(FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=True, BPCurveLearnComfort=True))
  drive_curve_manually(on, v, k)
  assert on.comfort.dirty
  # op long engaged and nobody overriding: not the driver's choice, no samples
  eng = make(FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=True, BPCurveLearnComfort=True))
  for _ in range(400):
    step(eng, straight(), v, long_enabled=True, curvature=k)
  assert not eng.comfort.dirty


def test_no_learning_with_a_lead_a_blinker_or_a_gentle_road():
  p = lambda: FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=True, BPCurveLearnComfort=True)  # noqa: E731
  for kw in ({'lead': True}, {'blinker': True}):
    s = make(p())
    drive_curve_manually(s, **kw)
    assert not s.comfort.dirty
  s = make(p())
  drive_curve_manually(s, 14.0, 0.0005)                # 0.1 m/s^2: not a curve
  assert not s.comfort.dirty


def test_learned_comfort_is_saved_as_json_and_loaded_back():
  p = FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=True, BPCurveLearnComfort=True)
  s = make(p)
  drive_curve_manually(s, 14.0, 0.008, frames=int(SAVE_PERIOD_S / 0.05) + 600)
  assert "BPCurveComfortData" in p.writes
  data = p.writes["BPCurveComfortData"]
  assert isinstance(data, dict) and any(v["count"] > 0 for v in data.values())
  loaded = scc_bp.SmartCruiseControlVisionBP()
  loaded.params = FakeParams(BPCurveComfortData=json.loads(json.dumps(data)))
  loaded.comfort = load_comfort(loaded.params)
  assert np.allclose(loaded.comfort.lat_accel_for_curvature(np.array([0.008])),
                     s.comfort.lat_accel_for_curvature(np.array([0.008])))


def test_brake_or_gas_while_the_limiter_controls_nudges_the_table_once():
  p = FakeParams(SmartCruiseControlVision=False, BPCurveSpeedLimit=True, BPCurveLearnComfort=True)
  s = make(p)
  for _ in range(40):
    step(s, arc(47.0, 20.0), 30 * MPH)                  # limiter active
  assert s.is_active
  before = s.comfort.lat_accel_for_curvature(np.array([1 / 47.0]))[0]
  for _ in range(10):
    step(s, arc(47.0, 20.0), 30 * MPH, brake=True, long_enabled=True)
  after = s.comfort.lat_accel_for_curvature(np.array([1 / 47.0]))[0]
  assert after < before
  total = sum(d["count"] for d in s.comfort.data.values())
  for _ in range(10):
    step(s, arc(47.0, 20.0), 30 * MPH, brake=True, long_enabled=True)
  assert sum(d["count"] for d in s.comfort.data.values()) == total        # one nudge per episode, not per frame


"""
BluePilot: the curve limiter (curve_limit_bp.py) plugged into sunnypilot's Smart Cruise Control - Vision slot.

sunnypilot's planner already takes the lowest speed target of several sources (cruise, SCC-V, SCC-Map, speed
limit assist). This class takes the SCC-V slot, so the speed target, the source switch and the UI/log fields
(longitudinalPlanSP.smartCruiseControl.vision) need no message changes.

It is selected with the `BPCurveSpeedLimit` param. With it off, the stock SCC-V runs unchanged (its own
`SmartCruiseControlVision` toggle decides). Optional learning of the driver's comfort (curve_comfort_bp.py) is
the `BPCurveLearnComfort` param; its table is persisted in `BPCurveComfortData`.

The planner cruise path can only ask for -0.5 m/s^2 (A_CRUISE_MIN), too little for a curve the car is
carrying too much speed into, so the limiter's brake request is exposed as `curve_accel` and applied by the
planner as one more accel candidate (the planner takes the lowest). a_target stays a pass-through so the MPC's
starting state is not disturbed.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import (
  SmartCruiseControlVision,
  VisionState,
)
from openpilot.bluepilot.selfdrive.controls.curve_comfort_bp import ComfortLearner, load_comfort
from openpilot.bluepilot.selfdrive.controls.param_flag_bp import read_flag
from openpilot.bluepilot.selfdrive.controls.curve_limit_bp import CurveLimiter

LEAD_HEADWAY_S = 3.0         # a lead closer than this is setting the speed, not the curve
TURNING_DISTANCE = 20.0      # m: a binding point this close is "turning" for the UI state


class SmartCruiseControlVisionBP(SmartCruiseControlVision):
  def __init__(self):
    super().__init__()
    self.bp_enabled = False
    self.bp_learn = False
    self.limiter = CurveLimiter()
    self.learner = ComfortLearner(load_comfort(self.params))
    self.curve_accel = 0.0            # brake request for the planner's accel candidates (<= 0)
    self.bind_curvature = 0.0

  @property
  def comfort(self):
    return self.learner.comfort

  @comfort.setter
  def comfort(self, value):
    self.learner.comfort = value

  def _read_bp_params(self) -> None:
    self.bp_enabled = read_flag(self.params, "BPCurveSpeedLimit")
    self.bp_learn = read_flag(self.params, "BPCurveLearnComfort")

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise_setpoint: float) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self._read_bp_params()
    if not self.bp_enabled:
      self.curve_accel = 0.0
      self.limiter.reset()
      super().update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise_setpoint)
      return

    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    cs = sm['carState']
    blinker = bool(cs.leftBlinker or cs.rightBlinker)
    plan = sm['modelV2'].position
    res = self.limiter.update(plan.x, plan.y, v_ego, v_cruise_setpoint, enabled=bool(long_enabled and not long_override),
                              blinker=blinker, comfort=self.comfort.lat_accel_for_curvature if self.bp_learn else None)

    self.current_lat_acc = v_ego ** 2 * abs(sm['controlsState'].curvature)
    self.max_pred_lat_acc = res.pred_lat_acc
    self.is_enabled = bool(long_enabled)
    self.is_active = bool(res.active)
    self.curve_accel = float(res.a_cmd) if res.braking else 0.0
    self.bind_curvature = res.bind_curvature
    self.v_target = res.v_target
    self.a_target = res.a_cmd
    self.output_v_target = res.v_target if res.active else V_CRUISE_UNSET
    self.output_a_target = a_ego            # pass-through: the brake request goes in as curve_accel
    if not long_enabled:
      self.state = VisionState.disabled
    elif long_override:
      self.state = VisionState.overriding
    elif res.active:
      self.state = VisionState.turning if res.bind_distance <= TURNING_DISTANCE else VisionState.entering
    else:
      self.state = VisionState.enabled

    if self.bp_learn:
      cs = sm['carState']
      lead = sm['radarState'].leadOne
      lead_near = bool(lead.present) and float(lead.dRel) < max(v_ego * LEAD_HEADWAY_S, 15.0)
      self.learner.step(DT_MDL, self.is_active, long_enabled, long_override, bool(cs.brakePressed), bool(cs.gasPressed), v_ego,
                        blinker, lead_near, abs(float(sm['controlsState'].curvature)), self.bind_curvature)
      self.learner.maybe_save(self.params, DT_MDL)
    self.frame += 1

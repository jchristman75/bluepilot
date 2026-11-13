"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap
# BluePilot: steering-ability aware curve limiter in the SCC-V slot
from openpilot.common.bluepilot import is_bluepilot
# End BluePilot


class SmartCruiseControl:
  def __init__(self):
    self.vision = SmartCruiseControlVision()
    # BluePilot: the BP class runs stock SCC-V unchanged unless BPCurveSpeedLimit is on
    self.bp_vision = None
    if is_bluepilot():
      from openpilot.bluepilot.selfdrive.controls.curve_scc_bp import SmartCruiseControlVisionBP
      self.vision = self.bp_vision = SmartCruiseControlVisionBP()
    # End BluePilot
    self.map = SmartCruiseControlMap()

  # BluePilot: the curve limiter's brake request (m/s^2, <= 0) for the planner's accel candidates
  @property
  def curve_accel(self) -> float:
    return 0.0 if self.bp_vision is None else self.bp_vision.curve_accel

  # BluePilot: what longitudinalPlanSP logs as vision.aTarget. The BP limiter passes the car's own accel through as
  # its output_a_target (so the MPC start state isn't disturbed), which hides its request from the logs; log the
  # request itself. Logging only: control reads output_a_target and curve_accel.
  @property
  def logged_vision_a_target(self) -> float:
    return self.vision.output_a_target if self.bp_vision is None else self.bp_vision.curve_accel
  # End BluePilot

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise: float) -> None:
    self.map.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)
    self.vision.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

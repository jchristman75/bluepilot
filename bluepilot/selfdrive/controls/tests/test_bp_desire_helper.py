"""BluePilot: lane change state machine (desire_helper.py) -- pre-sync finishing hold and initiation-only
blinker pause (route 000004d6 t=505: a right lane change abandoned while straddling the line)."""
from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LaneChangeDirection, log
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

MPH = 0.44704


def _cs(v_mph, right_blinker=False, pressed=False, torque=0.0):
  return SimpleNamespace(vEgo=v_mph * MPH, leftBlinker=False, rightBlinker=right_blinker,
                         leftBlindspot=False, rightBlindspot=False, brakePressed=False,
                         steeringPressed=pressed, steeringTorque=torque)


class TestBPDesireHelper(OpenpilotTestCase):
  def setup_method(self):
    params = Params()
    params.put_bool("BlinkerPauseLaneChange", True)
    params.put("BlinkerMinLateralControlSpeed", 43)
    params.put("BlinkerLateralReengageDelay", 0)
    self.DH = DesireHelper()
    self.DH.alc.read_params = lambda: None  # keep the mode set below
    self.DH.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE

  def _start_right(self, v_mph=67):
    self.DH.update(_cs(v_mph), True, 0.0)
    self.DH.update(_cs(v_mph, right_blinker=True), True, 0.0)
    assert self.DH.lane_change_state == LaneChangeState.preLaneChange
    self.DH.update(_cs(v_mph, right_blinker=True, pressed=True, torque=-1.0), True, 0.0)
    assert self.DH.lane_change_state == LaneChangeState.laneChangeStarting

  def test_probability_dip_keeps_the_desire_through_finishing(self):
    self._start_right()
    for _ in range(int(0.6 / DT_MDL)):  # model confident, lane lines fading out
      self.DH.update(_cs(67), True, 0.95)
    # the model's lane change probability collapses mid-change: the desire must not drop at once
    self.DH.update(_cs(67), True, 0.01)
    assert self.DH.lane_change_state == LaneChangeState.laneChangeFinishing
    assert self.DH.desire == log.Desire.laneChangeRight
    for _ in range(int(0.9 / DT_MDL)):
      self.DH.update(_cs(67), True, 0.01)
      assert self.DH.desire == log.Desire.laneChangeRight
    for _ in range(int(0.3 / DT_MDL)):
      self.DH.update(_cs(67), True, 0.01)
    assert self.DH.lane_change_state == LaneChangeState.off

  def test_slowing_below_the_blinker_pause_speed_does_not_abort(self):
    self._start_right(v_mph=45)
    for _ in range(int(1.0 / DT_MDL)):
      self.DH.update(_cs(40, right_blinker=True), True, 0.95)  # below the 43 mph pause, blinker still on
      assert self.DH.lane_change_state == LaneChangeState.laneChangeStarting
      assert self.DH.lane_change_direction == LaneChangeDirection.right

  def test_blinker_pause_still_blocks_initiation(self):
    self.DH.update(_cs(40), True, 0.0)
    for _ in range(5):
      self.DH.update(_cs(40, right_blinker=True, pressed=True, torque=-1.0), True, 0.0)
      assert self.DH.lane_change_state == LaneChangeState.off

  def test_disengage_still_aborts(self):
    self._start_right()
    self.DH.update(_cs(67), False, 0.95)
    assert self.DH.lane_change_state == LaneChangeState.off

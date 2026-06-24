"""BluePilot MICI: Lateral tuning panel — control variable, factors, lane change, offset, mode display."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import (
  BigParamControlBP,
  BigMultiParamToggleBP,
)
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller


class LateralLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    self.primary_lateral_control = BigMultiParamToggleBP(
      "Primary Control Variable", "FordPrefLateralControl", ["curvature", "angle"],
    )
    self.low_speed_factor = BigParamFloatControl(
      "Low Speed Adjustment Factor", "FordAngleLowSpeedFactor", min=0.5, max=1.5, step=0.01,
    )
    self.high_speed_factor = BigParamFloatControl(
      "High Speed Adjustment Factor", "FordAngleHighSpeedFactor", min=0.5, max=1.5, step=0.01,
    )
    self.angle_auto_tune_enable = BigParamControlBP("Angle Auto-Tune", "FordAngleAutoTuneEnable")
    self.disable_BP_lat = BigParamControlBP("Disable BP Lateral Control", "disable_BP_lat_UI")
    self.disable_lane_change_under_speed = BigParamControlBP(
      "Disable Auto Lane Change Under Speed", "BlinkerPauseLaneChange",
    )
    self.lane_change_factor_high = BigParamFloatControl(
      "Lane Change Factor High", "lane_change_factor_high", min=0.5, max=2.0,
    )
    self.custom_path_offset = BigParamFloatControl(
      "In-Lane Offset", "custom_path_offset", min=-0.5, max=0.5,
    )
    self.show_lateral_control = BigParamControlBP("Show Lateral Control Mode", "BpShowLateralControl")

    self._scroller.add_widgets([
      self.primary_lateral_control,
      self.low_speed_factor,
      self.high_speed_factor,
      self.angle_auto_tune_enable,
      self.disable_BP_lat,
      self.disable_lane_change_under_speed,
      self.lane_change_factor_high,
      self.custom_path_offset,
      self.show_lateral_control,
    ])

    self._refresh_toggles = (
      ("FordAngleAutoTuneEnable", self.angle_auto_tune_enable),
      ("disable_BP_lat_UI", self.disable_BP_lat),
      ("BlinkerPauseLaneChange", self.disable_lane_change_under_speed),
      ("BpShowLateralControl", self.show_lateral_control),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()
    self.primary_lateral_control._load_value()

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

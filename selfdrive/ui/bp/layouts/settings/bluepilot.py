import pyray as rl

from openpilot.common.params import Params
from openpilot.common.params_pyx import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item, button_item, ButtonAction, ListItem
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.bp.widgets.float_control_item import float_control_item, int_control_item
from openpilot.selfdrive.ui.bp.widgets.section_header import CollapsibleSectionHeader
from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl
from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import GaugeStyle


class BluePilotLayout(Widget):
  """BluePilot settings layout for TICI UI."""

  @staticmethod
  def _safe_get_bool(params: Params, key: str, default: bool = False) -> bool:
    """Get bool param; return default if key is unknown (e.g. dev environment with reduced params)."""
    try:
      return params.get_bool(key)
    except UnknownKeyName:
      return default

  @staticmethod
  def _safe_get(params: Params, key: str, default=None):
    """Get param; return default if key is unknown (e.g. dev environment with reduced params)."""
    try:
      val = params.get(key, return_default=True)
      return val if val not in (None, b"", "") else default
    except UnknownKeyName:
      return default

  def __init__(self):
    super().__init__()
    self._params = Params()

    # Create WifiManager instance for preferred network selector
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)  # Don't scan unless needed
    self._saved_networks: list[Network] = []
    self._preferred_network_dialog: MultiOptionDialog | None = None

    # Register callback to update saved networks list
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    # Initialize items
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

    # Toggle refresh list
    self._refresh_toggles = (
      ("send_hands_free_cluster_msg", self._show_hands_free_ui),
      ("BPDisableLaneLineStatusColor", self._disable_lane_line_status_color),
      ("ShowBlindspotOverlay", self._show_blindspot),
      ("ShowBrakeStatus", self._show_brake_status),
      ("BPHideOnroadBorder", self._hide_onroad_border),
      ("BPShowConfidenceBall", self._show_confidence_ball),
      ("BPAnimateSteeringWheel", self._animate_steering_wheel),
      ("FordPrefShowRadarLeadOverlay", self._show_ford_radar_overlay),
      ("FordPrefHybridBatteryStatus", self._show_hybrid_battery_status),
      ("FordPrefHybridPowerFlow", self._show_hybrid_power_flow),
      ("enable_human_turn_detection", self._enable_human_turn_detection),
      ("BlinkerPauseLaneChange", self._disable_lane_change_under_speed),
      ("enable_lane_positioning", self._enable_lane_positioning),
      ("enable_lane_full_mode", self._enable_lane_full_mode),
      ("custom_profile", self._custom_profile),
      ("disable_BP_lat_UI", self._disable_BP_lat),
      ("disable_BP_long_UI", self._disable_BP_long),
      ("disable_downhill_comp_UI", self._disable_dowhill_comp),
      ("disable_ford_radar_UI", self._disable_ford_radar),
      ("BpShowLateralControl", self._show_lateral_control),
      ("BPUIDebugLog", self._ui_debug_log),
      ("FordAngleAutoTuneEnable", self._angle_auto_tune),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _initialize_items(self):
    """Initialize all BluePilot menu items."""

    # BlueCruise icon on dash toggle
    self._show_hands_free_ui = toggle_item(
      lambda: tr("Show BlueCruise UI on Cluster"),
      lambda: tr("Display BlueCruise UI on the cluster for supported vehicles."),
      initial_state=self._safe_get_bool(self._params, "send_hands_free_cluster_msg"),
      callback=lambda state: self._toggle_callback(state, "send_hands_free_cluster_msg"),
      icon="monitoring.png"
    )

    # Lane line status color toggle (issue #109: option to keep lane lines grey instead of green when engaged)
    self._disable_lane_line_status_color = toggle_item(
      lambda: tr("Disable Lane Line Status Color"),
      lambda: tr("Keep lane lines grey instead of changing to green when engaged."),
      initial_state=self._safe_get_bool(self._params, "BPDisableLaneLineStatusColor"),
      callback=lambda state: self._toggle_callback(state, "BPDisableLaneLineStatusColor"),
      icon="monitoring.png"
    )

    # Blindspot overlay toggle (BluePilot red edge overlay; SunnyPilot BSM is controlled by Visuals → BlindSpot)
    self._show_blindspot = toggle_item(
      lambda: tr("Show Blindspot Overlay"),
      lambda: tr("Display red overlay when vehicle is detected in blindspot."),
      initial_state=self._safe_get_bool(self._params, "ShowBlindspotOverlay"),
      callback=lambda state: self._toggle_callback(state, "ShowBlindspotOverlay"),
      icon="warning.png"
    )

    # Brake status toggle
    self._show_brake_status = toggle_item(
      lambda: tr("Show Brake Status"),
      lambda: tr("Display speed setpoint in red when vehicle is braking."),
      initial_state=self._safe_get_bool(self._params, "ShowBrakeStatus"),
      callback=lambda state: self._toggle_callback(state, "ShowBrakeStatus"),
      icon="warning.png"
    )

    # Hide onroad border toggle
    self._hide_onroad_border = toggle_item(
      lambda: tr("Hide Onroad Border"),
      lambda: tr("Hide the colored status border around the driving view."),
      initial_state=self._safe_get_bool(self._params, "BPHideOnroadBorder"),
      callback=lambda state: self._toggle_callback(state, "BPHideOnroadBorder"),
      icon="warning.png"
    )

    # Show confidence ball toggle
    self._show_confidence_ball = toggle_item(
      lambda: tr("Show Confidence Ball"),
      lambda: tr("Display the confidence ball on the left side of the driving view."),
      initial_state=self._safe_get_bool(self._params, "BPShowConfidenceBall"),
      callback=lambda state: self._toggle_callback(state, "BPShowConfidenceBall"),
      icon="warning.png"
    )

    # Animate steering wheel toggle
    self._animate_steering_wheel = toggle_item(
      lambda: tr("Animate Steering Wheel"),
      lambda: tr("Rotate the steering wheel icon to match the current steering angle."),
      initial_state=self._safe_get_bool(self._params, "BPAnimateSteeringWheel"),
      callback=lambda state: self._toggle_callback(state, "BPAnimateSteeringWheel"),
      icon="chffr_wheel.png"
    )

    # Ford radar lead overlay toggle
    self._show_ford_radar_overlay = toggle_item(
      lambda: tr("Show Radar Lead Overlay (Ford ACC)"),
      lambda: tr("Display chevron with lead vehicle info when using Ford stock ACC."),
      initial_state=self._safe_get_bool(self._params, "FordPrefShowRadarLeadOverlay"),
      callback=lambda state: self._toggle_callback(state, "FordPrefShowRadarLeadOverlay"),
      icon="speed_limit.png"
    )

    # Ford radar overlay size selector (inline buttons like Driving Personality)
    try:
      overlay_size_idx = int(self._safe_get(self._params, "FordPrefRadarOverlaySize") or 1)
    except (TypeError, ValueError):
      overlay_size_idx = 1
    # Ensure default is persisted so consumers read the correct value on first load
    try:
      if self._safe_get(self._params, "FordPrefRadarOverlaySize") is None:
        self._params.put("FordPrefRadarOverlaySize", str(overlay_size_idx))
    except UnknownKeyName:
      pass
    self._radar_overlay_size_btn = multiple_button_item(
      lambda: tr("Radar Overlay Size"),
      lambda: tr("Set the size of the radar lead overlay chevron and info boxes."),
      buttons=[lambda: tr("Small"), lambda: tr("Medium"), lambda: tr("Large")],
      button_width=225,
      callback=self._set_overlay_size,
      selected_index=overlay_size_idx,
      icon="speed_limit.png"
    )

    # Hybrid battery status toggle
    self._show_hybrid_battery_status = toggle_item(
      lambda: tr("Show Hybrid/EV Battery Status"),
      lambda: tr("Display hybrid battery gauge with SOC, voltage, and amps."),
      initial_state=self._safe_get_bool(self._params, "FordPrefHybridBatteryStatus"),
      callback=lambda state: self._toggle_callback(state, "FordPrefHybridBatteryStatus"),
      icon="warning.png"
    )

    # Hybrid power flow toggle
    self._show_hybrid_power_flow = toggle_item(
      lambda: tr("Show Hybrid/EV Power Flow"),
      lambda: tr("Display power flow gauge showing throttle demand and regenerative braking."),
      initial_state=self._safe_get_bool(self._params, "FordPrefHybridPowerFlow"),
      callback=lambda state: self._toggle_callback(state, "FordPrefHybridPowerFlow"),
      icon="warning.png"
    )

    # Hybrid drive gauge size selector (inline buttons: Small=1, Large=2)
    try:
      gauge_size_idx = int(self._safe_get(self._params, "FordPrefHybridDriveGaugeSize") or 1)
    except (TypeError, ValueError):
      gauge_size_idx = 1
    # Clamp old 3-tier values to new 2-tier range
    gauge_size_idx = min(gauge_size_idx, 2)
    # Ensure default is persisted so consumers read the correct value on first load
    try:
      if self._safe_get(self._params, "FordPrefHybridDriveGaugeSize") is None:
        self._params.put("FordPrefHybridDriveGaugeSize", str(gauge_size_idx))
    except UnknownKeyName:
      pass
    # Map 1/2 to button index 0/1
    self._hybrid_gauge_size_btn = multiple_button_item(
      lambda: tr("Hybrid/EV Gauge Size"),
      lambda: tr("Set the size of the battery and power flow gauges."),
      buttons=[lambda: tr("Small"), lambda: tr("Large")],
      button_width=225,
      callback=self._set_hybrid_gauge_size,
      selected_index=gauge_size_idx - 1,
      icon="warning.png"
    )

    # Hybrid gauge style: Flat (horizontal bar + container) vs Arched (arch above torque bar)
    gauge_style_idx = GaugeStyle(self._params.get("FordPrefGaugeStyle", return_default=True) or 0)
    self._hybrid_gauge_style_btn = multiple_button_item(
      lambda: tr("Hybrid Gauge Style"),
      lambda: tr("Flat: horizontal bar in shared container. Arched: arch above torque bar (older style)."),
      buttons=[lambda: tr("Flat"), lambda: tr("Arched")],
      button_width=225,
      callback=self._set_hybrid_gauge_style,
      selected_index=gauge_style_idx,
      icon="warning.png"
    )

    # Human turn detection toggle
    self._enable_human_turn_detection = toggle_item(
      lambda: tr("Enable Human Turn Detection"),
      lambda: tr("Enable detection of human-initiated turns."),
      initial_state=self._safe_get_bool(self._params, "enable_human_turn_detection"),
      callback=lambda state: self._toggle_callback(state, "enable_human_turn_detection"),
      icon="warning.png"
    )

    # Lane change factor high (float)
    self._lane_change_factor_high = float_control_item(
      lambda: tr("Lane Change Factor High"),
      lambda: tr("Scales steering during a lane change. <1.0 reduces (curvature control); >1.0 boosts (angle control)."),
      param="lane_change_factor_high",
      min_value=0.5,
      max_value=2.0,
      step=0.05,
      icon="speed_limit.png"
    )

    self._disable_lane_change_under_speed = toggle_item(
      lambda: tr("Disable Lane Change Under Speed"),
      lambda: tr("Pause lateral control when blinker is on and below minimum speed."),
      initial_state=self._safe_get_bool(self._params, "BlinkerPauseLaneChange"),
      callback=self._on_blinker_pause_changed,
      icon="chffr_wheel.png"
    )

    self._blinker_min_speed = int_control_item(
      lambda: tr("Minimum Speed to Pause Lane Change"),
      lambda: tr("Below this speed, lateral control is paused when the blinker is active."),
      param="BlinkerMinLateralControlSpeed",
      min_value=5,
      max_value=50,
      step=1,
      icon="chffr_wheel.png"
    )

    # Enable lane positioning toggle
    self._enable_lane_positioning = toggle_item(
      lambda: tr("Enable Lane Positioning"),
      lambda: tr("Enable custom lane positioning controls."),
      initial_state=self._safe_get_bool(self._params, "enable_lane_positioning"),
      callback=lambda state: self._toggle_callback(state, "enable_lane_positioning"),
      icon="chffr_wheel.png"
    )

    # Custom path offset (float, conditional on lane positioning)
    self._custom_path_offset = float_control_item(
      lambda: tr("In-Lane Offset"),
      lambda: tr("Adjust the in-lane offset (-0.5 to 0.5)."),
      param="custom_path_offset",
      min_value=-0.5,
      max_value=0.5,
      step=0.05,
      enabled=lambda: self._safe_get_bool(self._params, "enable_lane_positioning"),
      icon="chffr_wheel.png"
    )

    # Enable lanefull mode toggle (conditional on lane positioning)
    self._enable_lane_full_mode = toggle_item(
      lambda: tr("Enable Lanefull Mode"),
      lambda: tr("Enable lanefull mode for lane positioning."),
      initial_state=self._safe_get_bool(self._params, "enable_lane_full_mode"),
      callback=lambda state: self._toggle_callback(state, "enable_lane_full_mode"),
      enabled=lambda: self._safe_get_bool(self._params, "enable_lane_positioning"),
      icon="chffr_wheel.png"
    )

    # Custom profile toggle
    self._custom_profile = toggle_item(
      lambda: tr("Use Custom Tuning Profile"),
      lambda: tr("Enable custom tuning profile settings."),
      initial_state=self._safe_get_bool(self._params, "custom_profile"),
      callback=lambda state: self._toggle_callback(state, "custom_profile"),
      icon="chffr_wheel.png"
    )

    # Predicted curvature blend ratio high (float, conditional on custom profile)
    self._pc_blend_ratio_high_C = float_control_item(
      lambda: tr("Predicted Curvature Blend Ratio High"),
      lambda: tr("Adjust the high curvature blend ratio (0.0-1.0)."),
      param="pc_blend_ratio_high_C_UI",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      enabled=lambda: self._safe_get_bool(self._params, "custom_profile"),
      icon="chffr_wheel.png"
    )

    # Predicted curvature blend ratio low (float, conditional on custom profile)
    self._pc_blend_ratio_low_C = float_control_item(
      lambda: tr("Predicted Curvature Blend Ratio Low"),
      lambda: tr("Adjust the low curvature blend ratio (0.0-1.0)."),
      param="pc_blend_ratio_low_C_UI",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      enabled=lambda: self._safe_get_bool(self._params, "custom_profile"),
      icon="chffr_wheel.png"
    )

    # Centering PID gain — used by angle-mode trim (always) and curv-mode lane positioning (when custom_profile is on).
    self._lc_pid_gain = float_control_item(
      lambda: tr("Centering PID gain"),
      lambda: tr("PID gain for the centering controller. In angle mode, drives the path_angle trim. In curvature mode, drives the path_angle PID (only effective when 'custom profile' is enabled)."),
      param="LC_PID_gain_UI",
      min_value=0.0,
      max_value=50.0,
      step=0.5,
      icon="chffr_wheel.png"
    )

    # 12V battery limit (float)
    self._vbatt_pause_charging = float_control_item(
      lambda: tr("12V Battery Limit"),
      lambda: tr("Set the 12V battery charging pause limit (11.0-14.0V)."),
      param="vbatt_pause_charging",
      min_value=11.0,
      max_value=14.0,
      step=0.1,
      suffix="V",
      icon="warning.png"
    )

    # UI Debug Logging toggle
    self._ui_debug_log = toggle_item(
      lambda: tr("UI Debug Logging"),
      lambda: tr("Log UI state transitions for diagnosing rendering issues on device."),
      initial_state=self._safe_get_bool(self._params, "BPUIDebugLog"),
      callback=lambda state: self._toggle_callback(state, "BPUIDebugLog"),
      icon="warning.png"
    )

    # Lane line feedback trim — toggle + tuning floats
    # Primary lateral actuator: curvature-primary (historical) vs angle-primary (experimental)
    primary_lat_idx = PrimaryLateralControl(self._params.get("FordPrefLateralControl", return_default=True) or 0)
    self._primary_lateral_control_btn = multiple_button_item(
      lambda: tr("Primary Control Variable"),
      lambda: tr("Curvature matches the existing strategy. Angle uses path_angle as the main actuator (in development)."),
      buttons=[lambda: tr("Curvature"), lambda: tr("Angle")],
      button_width=225,
      callback=self._set_primary_lateral_control,
      selected_index=primary_lat_idx,
      icon="chffr_wheel.png"
    )
    self._low_speed_curv_factor = float_control_item(
      lambda: tr("Low Speed Adjustment Factor"),
      lambda: tr("Scales the low-speed steering response in angle mode. Adjust for personal feel. Default 1.0."),
      param="FordAngleLowSpeedFactor",
      min_value=0.5,
      max_value=1.5,
      step=0.01,
      icon="chffr_wheel.png"
    )
    self._high_speed_curv_factor = float_control_item(
      lambda: tr("High Speed Adjustment Factor"),
      lambda: tr("Scales the high-speed steering response in angle mode. Adjust for personal feel. Default 1.0."),
      param="FordAngleHighSpeedFactor",
      min_value=0.5,
      max_value=1.5,
      step=0.01,
      icon="chffr_wheel.png"
    )

    # Auto-tune toggle — automatically adjusts low/high speed factors while driving
    self._angle_auto_tune = toggle_item(
      lambda: tr("Auto-Tune Angle Factors"),
      lambda: tr("Automatically adjusts low and high speed factors while driving to minimize steering tracking error. "
                 "Disable before manual tuning. Requires active lateral control and steady curves to engage."),
      initial_state=self._safe_get_bool(self._params, "FordAngleAutoTuneEnable"),
      callback=lambda state: self._toggle_callback(state, "FordAngleAutoTuneEnable"),
      icon="chffr_wheel.png"
    )

    # Disable BP lateral control toggle
    self._disable_BP_lat = toggle_item(
      lambda: tr("Disable BP Lateral Control"),
      lambda: tr("Disable BluePilot lateral control."),
      initial_state=self._safe_get_bool(self._params, "disable_BP_lat_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_BP_lat_UI"),
      icon="chffr_wheel.png"
    )

    # Bypass BP longitudinal control toggle (use stock long logic)
    self._disable_BP_long = toggle_item(
      lambda: tr("Bypass BP Longitudinal Control"),
      lambda: tr("Use stock longitudinal logic instead of BluePilot TTC/coasting tuning."),
      initial_state=self._safe_get_bool(self._params, "disable_BP_long_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_BP_long_UI"),
      icon="chffr_wheel.png"
    )

    # Disable downhill compensation toggle
    self._disable_dowhill_comp = toggle_item(
      lambda: tr("Disable Downhill Compensation"),
      lambda: tr("Disable pitch-based brake/gas compensation when going downhill."),
      initial_state=self._safe_get_bool(self._params, "disable_downhill_comp_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_downhill_comp_UI"),
      icon="chffr_wheel.png"
    )

    # Disable Ford radar — vision-only lead detection (requires reboot)
    self._disable_ford_radar = toggle_item(
      lambda: tr("Disable Ford Radar (Vision-Only Leads)"),
      lambda: tr("Ignore the vehicle radar and drive leads exclusively from the vision model. Requires reboot."),
      initial_state=self._safe_get_bool(self._params, "disable_ford_radar_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_ford_radar_UI"),
      icon="chffr_wheel.png"
    )

    # Show lateral control mode overlay toggle
    self._show_lateral_control = toggle_item(
      lambda: tr("Show Lateral Control Mode"),
      lambda: tr("Display the lateral control mode overlay on the steering wheel icon."),
      initial_state=self._safe_get_bool(self._params, "BpShowLateralControl"),
      callback=lambda state: self._toggle_callback(state, "BpShowLateralControl"),
      icon="chffr_wheel.png"
    )

    # Preferred WiFi Network selector
    self._preferred_network_action = ButtonAction(lambda: tr("SELECT"))
    self._preferred_network_action.set_value(lambda: self._get_preferred_network_display())
    self._preferred_network_btn = ListItem(
      lambda: tr("Preferred WiFi Network"),
      description=lambda: tr("Automatically connect to this network when available"),
      action_item=self._preferred_network_action,
      callback=self._select_preferred_network
    )

    # Clear model runner cache (ModelRunnerTypeCache + ModelManager_ActiveBundle) and reboot
    self._clear_model_cache_btn = button_item(
      lambda: tr("Clear Crashed Model"),
      lambda: tr("CLEAR"),
      lambda: tr("Clear crashed model runner cache and reboot. Fixes 'Communication Issue' if modeld fails to start."),
      callback=self._clear_model_cache
    )

    # BluePilot: reset menu layout — collapses all sections, fixing overlap glitch without reboot.
    self._reset_menu_btn = button_item(
      lambda: tr("Reset Menu Layout"),
      lambda: tr("RESET"),
      lambda: tr("Collapse all sections to fix overlapping items. Use this if the menu looks broken."),
      callback=lambda: self._scroller.show_event()
    )
    # End BluePilot

    # BluePilot: collapsible section groups — all start collapsed when the menu opens.
    def _section(title: str, items: list) -> list:
      header = CollapsibleSectionHeader(title)
      header.set_items(items)
      return [header] + items

    lateral_items = [
      self._low_speed_curv_factor,
      self._high_speed_curv_factor,
      self._angle_auto_tune,
      self._enable_human_turn_detection,
      self._disable_lane_change_under_speed,
      self._blinker_min_speed,
      self._lane_change_factor_high,
      self._enable_lane_positioning,
      self._custom_path_offset,
      self._enable_lane_full_mode,
      self._custom_profile,
      self._pc_blend_ratio_high_C,
      self._pc_blend_ratio_low_C,
      self._lc_pid_gain,
      self._show_lateral_control,
      self._disable_BP_lat,
    ]
    lateral_section = _section(tr("Lateral Tuning"), lateral_items)
    self._lateral_header = lateral_section[0]

    return (
      _section(tr("System"), [
        self._preferred_network_btn,
        self._clear_model_cache_btn,
        self._ui_debug_log,
        self._reset_menu_btn,
      ]) +
      _section(tr("Vehicle"), [
        self._show_hands_free_ui,
        self._vbatt_pause_charging,
      ]) +
      _section(tr("Visuals"), [
        self._hide_onroad_border,
        self._disable_lane_line_status_color,
        self._show_blindspot,
        self._show_brake_status,
        self._show_confidence_ball,
        self._animate_steering_wheel,
        self._show_ford_radar_overlay,
        self._radar_overlay_size_btn,
        self._show_hybrid_battery_status,
        self._show_hybrid_power_flow,
        self._hybrid_gauge_size_btn,
        self._hybrid_gauge_style_btn,
      ]) +
      _section(tr("Longitudinal Tuning"), [
        self._disable_BP_long,
        self._disable_dowhill_comp,
        self._disable_ford_radar,
      ]) +
      [self._primary_lateral_control_btn] +
      lateral_section
    )
    # End BluePilot

  def _get_float_param(self, param: str, default: float) -> float:
    """Get float parameter value."""
    try:
      return float(self._params.get(param, return_default=True))
    except (TypeError, ValueError):
      return default

  def _toggle_callback(self, state: bool, param: str):
    """Handle toggle state changes."""
    try:
      self._params.put_bool(param, state)
    except UnknownKeyName:
      pass  # Param not available in dev environment
    self._update_toggles(just_toggled={param: state})

  def _on_blinker_pause_changed(self, state: bool) -> None:
    self._toggle_callback(state, "BlinkerPauseLaneChange")
    self._blinker_min_speed.action_item.set_enabled(state)

  def _update_toggles(self, just_toggled: dict | None = None):
    """Update toggle states from params. just_toggled: {param: value} for params we just wrote (avoids refresh race)."""
    ui_state.update_params()
    fresh = just_toggled or {}

    # Refresh toggles from params to mirror external changes (use fresh for params we just wrote)
    for key, item in self._refresh_toggles:
      state = fresh[key] if key in fresh else self._safe_get_bool(ui_state.params, key)
      item.action_item.set_state(state)

    # Update button enabled states
    self._radar_overlay_size_btn.action_item.set_enabled(self._safe_get_bool(ui_state.params, "FordPrefShowRadarLeadOverlay"))
    try:
      overlay_idx = int(self._safe_get(ui_state.params, "FordPrefRadarOverlaySize") or 1)
    except (TypeError, ValueError):
      overlay_idx = 1
    self._radar_overlay_size_btn.action_item.set_selected_button(overlay_idx)
    # Hybrid gauge size and style: enable only when power flow gauge is enabled (NOT battery status)
    self._hybrid_gauge_size_btn.action_item.set_enabled(
      lambda: self._safe_get_bool(ui_state.params, "FordPrefHybridPowerFlow")
    )
    self._hybrid_gauge_style_btn.action_item.set_enabled(
      lambda: self._safe_get_bool(ui_state.params, "FordPrefHybridPowerFlow")
    )
    try:
      gauge_size = int(self._safe_get(ui_state.params, "FordPrefHybridDriveGaugeSize") or 1)
    except (TypeError, ValueError):
      gauge_size = 1
    gauge_size = min(gauge_size, 2)  # Clamp old 3-tier values
    self._hybrid_gauge_size_btn.action_item.set_selected_button(gauge_size - 1)
    style_idx = GaugeStyle(ui_state.params.get("FordPrefGaugeStyle", return_default=True) or 0)
    self._hybrid_gauge_style_btn.action_item.set_selected_button(style_idx)
    plat_idx = PrimaryLateralControl(ui_state.params.get("FordPrefLateralControl", return_default=True) or 0)
    self._primary_lateral_control_btn.action_item.set_selected_button(plat_idx)
    custom_prof = fresh.get("custom_profile") if "custom_profile" in fresh else self._safe_get_bool(ui_state.params, "custom_profile")
    lane_pos = fresh.get("enable_lane_positioning") if "enable_lane_positioning" in fresh else self._safe_get_bool(ui_state.params, "enable_lane_positioning")
    pause_lc = fresh.get("BlinkerPauseLaneChange") if "BlinkerPauseLaneChange" in fresh else self._safe_get_bool(ui_state.params, "BlinkerPauseLaneChange")
    is_angle = (plat_idx == PrimaryLateralControl.angle)
    is_curv = not is_angle
    # Angle-mode-only items
    self._lateral_header.set_item_visible(self._low_speed_curv_factor, is_angle)
    self._lateral_header.set_item_visible(self._high_speed_curv_factor, is_angle)
    self._lateral_header.set_item_visible(self._angle_auto_tune, is_angle)
    # Conditional on BlinkerPauseLaneChange
    self._blinker_min_speed.action_item.set_enabled(pause_lc)
    # Curvature-mode-only items
    self._lateral_header.set_item_visible(self._enable_human_turn_detection, is_curv)
    self._lateral_header.set_item_visible(self._enable_lane_positioning, is_curv)
    self._lateral_header.set_item_visible(self._enable_lane_full_mode, is_curv)
    self._lateral_header.set_item_visible(self._custom_profile, is_curv)
    self._lateral_header.set_item_visible(self._pc_blend_ratio_high_C, is_curv)
    self._lateral_header.set_item_visible(self._pc_blend_ratio_low_C, is_curv)
    self._lateral_header.set_item_visible(self._lc_pid_gain, is_curv)
    # Enabled states depend on mode
    if is_curv:
      self._custom_path_offset.action_item.set_enabled(lane_pos)
      self._enable_lane_full_mode.action_item.set_enabled(lane_pos)
      self._pc_blend_ratio_high_C.action_item.set_enabled(custom_prof)
      self._pc_blend_ratio_low_C.action_item.set_enabled(custom_prof)
      self._lc_pid_gain.action_item.set_enabled(lane_pos and custom_prof)
    else:
      self._custom_path_offset.action_item.set_enabled(True)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()
    # Enable WiFi scanning when BluePilot menu is shown
    self._wifi_manager.set_active(True)

  def hide_event(self):
    super().hide_event()
    # Disable WiFi scanning when BluePilot menu is hidden
    self._wifi_manager.set_active(False)

  def _on_network_updated(self, networks: list[Network]):
    """Update saved networks list when WiFi networks are updated"""
    self._saved_networks = [n for n in networks if self._wifi_manager.is_connection_saved(n.ssid)]
    self._preferred_network_action.set_enabled(len(self._saved_networks) > 0)

    # Check if preferred network is still saved in NetworkManager
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      current_favorite = ""
      if favorite_value:
        if isinstance(favorite_value, bytes):
          current_favorite = favorite_value.decode('utf-8', errors='replace').strip('\x00')
        else:
          current_favorite = str(favorite_value).strip('\x00')
      if current_favorite:
        # Check NetworkManager's saved connections directly
        saved_connections = self._wifi_manager._connections
        if current_favorite not in saved_connections:
          # Network is no longer saved, clear preferred setting
          self._params.put("WifiFavoriteSSID", "")
          cloudlog.info(f"Cleared preferred network '{current_favorite}' - network no longer saved in NetworkManager")
    except Exception as e:
      cloudlog.debug(f"Error checking preferred network: {e}")

  def _get_preferred_network_display(self) -> str:
    """Get the display text for preferred network"""
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          favorite_ssid = favorite_value.decode('utf-8', errors='replace').strip('\x00')
        else:
          favorite_ssid = str(favorite_value).strip('\x00')
        if favorite_ssid:
          # Truncate if too long
          if len(favorite_ssid) > 20:
            return favorite_ssid[:17] + "..."
          return favorite_ssid
    except Exception:
      pass
    return tr("None")

  def _select_preferred_network(self):
    """Open dialog to select preferred network from saved networks"""
    if len(self._saved_networks) == 0:
      return

    # Get current favorite
    current_favorite = ""
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          current_favorite = favorite_value.decode('utf-8', errors='replace').strip('\x00')
        else:
          current_favorite = str(favorite_value).strip('\x00')
    except Exception:
      pass

    # Build list of network names (add "None" option first)
    network_options = [tr("None")]
    network_options.extend([n.ssid for n in self._saved_networks])

    def handle_selection(result):
      """Handle selection from dialog"""
      if result == DialogResult.CONFIRM and self._preferred_network_dialog is not None:
        selection = self._preferred_network_dialog.selection
        # Convert "None" back to empty string
        if selection == tr("None"):
          selection = ""

        # Save the selection
        self._params.put("WifiFavoriteSSID", selection)
        if selection:
          cloudlog.info(f"Set preferred network: {selection}")
        else:
          cloudlog.info("Cleared preferred network")

        # Update button value display
        self._preferred_network_action.set_value(self._get_preferred_network_display())

      self._preferred_network_dialog = None

    # Create dialog with callback; MultiOptionDialog calls pop_widget and callback internally
    self._preferred_network_dialog = MultiOptionDialog(
      tr("Select Preferred Network"),
      network_options,
      current_favorite if current_favorite else tr("None"),
      callback=handle_selection
    )
    gui_app.push_widget(self._preferred_network_dialog)

  def _clear_model_cache(self):
    """Clear ModelRunnerTypeCache and ModelManager_ActiveBundle, then reboot."""

    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        try:
          self._params.remove("ModelRunnerTypeCache")
        except Exception:
          pass
        try:
          self._params.remove("ModelManager_ActiveBundle")
        except Exception:
          pass
        self._params.put_bool("DoReboot", True, block=False)
        cloudlog.info("BluePilot: Cleared model cache (ModelRunnerTypeCache, ModelManager_ActiveBundle), triggered reboot")

    dialog = ConfirmDialog(
      tr("Clear crashed model runner cache and reboot? This fixes 'Communication Issue' when modeld fails to start."),
      tr("Clear & Reboot"),
      callback=handle_confirm
    )
    gui_app.push_widget(dialog)

  def _set_overlay_size(self, button_index: int):
    """Handle overlay size button selection."""
    self._params.put("FordPrefRadarOverlaySize", button_index)

  def _set_hybrid_gauge_size(self, button_index: int):
    """Handle hybrid gauge size button selection. Buttons are 0/1/2, param stores 1/2/3."""
    self._params.put("FordPrefHybridDriveGaugeSize", button_index + 1)

  def _set_hybrid_gauge_style(self, button_index: int):
    """Handle hybrid gauge style: 0 = Flat, 1 = Arched."""
    try:
      self._params.put("FordPrefGaugeStyle", int(GaugeStyle(button_index)))
    except UnknownKeyName:
      pass

  def _set_primary_lateral_control(self, button_index: int):
    try:
      self._params.put("FordPrefLateralControl", int(PrimaryLateralControl(button_index)))
    except UnknownKeyName:
      pass
    self._update_toggles()

  def _render(self, rect):
    # Process WiFi manager callbacks
    self._wifi_manager.process_callbacks()
    self._scroller.render(rect)

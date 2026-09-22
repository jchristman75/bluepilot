"""
BluePilot Charging Overlay Panel
Slide-in overlay (same animation/close pattern as ControlsDebugPanel) showing the
full charge-session curve, live kW/SOC (plus Amps where the car reports pack current),
and a time-to-80% estimate.
"""

import pyray as rl

from openpilot.selfdrive.ui.bp.charging.session import charge_session_history, is_in_drive
from openpilot.selfdrive.ui.bp.widgets.charge_curve_chart import ChargeCurveChart
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from bluepilot.ui.lib.colors import BPColors
from bluepilot.ui.widgets.debug.debug_colors import DebugColors

TARGET_SOC_PCT = 80.0
HERO_VALUE_FONT = 96  # kW: the one charging figure the car reports usefully
VALUE_FONT = 48
LABEL_FONT = 24
VALUE_LINE = 1.15  # line height as a multiple of the font size
LABEL_LINE = 1.25
READOUT_H = HERO_VALUE_FONT * VALUE_LINE + LABEL_FONT * LABEL_LINE
# No true "never timeout" sentinel exists in Device; reuse the large-number convention
# from selfdrive/ui/tests/diff/replay.py to keep the screen on indefinitely while open.
STAY_AWAKE_TIMEOUT_S = 99999


class ChargingOverlayPanel(Widget):
  """Onroad overlay panel showing charge session detail, toggled from the sidebar icon."""

  CLOSE_BUTTON_SIZE = 60
  CLOSE_BUTTON_MARGIN = 15
  ANIMATION_SPEED = 5.0  # Progress units per second (~200ms to full open)

  def __init__(self):
    super().__init__()
    self._visible_state = False
    self._animation_progress = 0.0  # 0 = fully hidden, 1 = fully visible
    self._chart = self._child(ChargeCurveChart(compact=False))

    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)

    self._consumed_click = False
    self._prev_charging_active = None
    self._prev_in_drive = None

  def toggle_visibility(self):
    self._set_visible(not self._visible_state)

  def show_panel(self, reason: str = "auto"):
    """Open the panel from outside (charge-session auto-open); no-op if already open."""
    self._set_visible(True, reason=reason)

  def _set_visible(self, visible: bool, reason: str = "toggle"):
    if visible == self._visible_state:
      return
    print(f"[ChargingPanel] _set_visible({visible}) reason={reason}")
    self._visible_state = visible
    if visible:
      self._prev_charging_active = None
      self._prev_in_drive = None
      device.set_override_interactive_timeout(STAY_AWAKE_TIMEOUT_S)
    else:
      device.set_override_interactive_timeout(None)

  @property
  def is_panel_visible(self) -> bool:
    return self._visible_state or self._animation_progress > 0.01

  def _update_state(self):
    target = 1.0 if self._visible_state else 0.0
    dt = rl.get_frame_time()
    if dt <= 0:
      dt = 1.0 / 60.0

    if self._animation_progress < target:
      self._animation_progress = min(target, self._animation_progress + self.ANIMATION_SPEED * dt)
    elif self._animation_progress > target:
      self._animation_progress = max(target, self._animation_progress - self.ANIMATION_SPEED * dt)

    if self._animation_progress > 0.5:
      charge_session_history.update()

    if self._visible_state:
      # Auto-dismiss once Drive is seen on two consecutive polls, not a single one:
      # a lone transient/stale gearShifter==drive sample (e.g. while parked and
      # charging) would otherwise re-close the panel the instant it's opened,
      # making the sidebar icon look like it does nothing when clicked.
      in_drive = is_in_drive()
      if self._prev_in_drive and in_drive:
        self._set_visible(False, reason="drive_engaged")
        return
      self._prev_in_drive = in_drive

      # Auto-dismiss when the charging session ends (active -> inactive transition)
      charging_active = charge_session_history.is_active
      if self._prev_charging_active and not charging_active:
        self._set_visible(False, reason="charging_session_ended")
        return
      self._prev_charging_active = charging_active

  def _get_data(self):
    try:
      car_state_bp = ui_state.sm['carStateBP']
      charging = car_state_bp.charging
      hybrid_battery = car_state_bp.hybridBattery
      return {
        'available': charging.dataAvailable,
        'active': charging.chargingActive,
        'status': charging.statusText,
        'kw': charging.powerKw,
        'amps': hybrid_battery.ampsActual if hybrid_battery.dataAvailable else 0.0,
        'soc': hybrid_battery.socActual if hybrid_battery.dataAvailable else 0.0,
      }
    except (KeyError, AttributeError, TypeError):
      return {'available': False, 'active': False, 'status': "", 'kw': 0.0, 'amps': 0.0, 'soc': 0.0}

  def _render(self, rect: rl.Rectangle):
    if self._animation_progress < 0.01:
      return

    self._consumed_click = False

    panel_w = rect.width
    x_offset = panel_w * (1.0 - self._animation_progress)
    panel_rect = rl.Rectangle(rect.x + x_offset, rect.y, panel_w, rect.height)

    rl.begin_scissor_mode(int(rect.x), int(rect.y), int(rect.width), int(rect.height))
    rl.draw_rectangle_rec(panel_rect, DebugColors.PANEL_BG)

    if self._animation_progress > 0.5:
      self._render_content(panel_rect)

    self._render_close_button(panel_rect)

    rl.end_scissor_mode()

    self._consume_mouse_events(panel_rect)

  def _render_content(self, rect: rl.Rectangle):
    data = self._get_data()
    pad = 30

    rl.draw_text_ex(self._font_bold, "Charging", rl.Vector2(rect.x + pad, rect.y + pad), 48, 0, BPColors.WHITE)

    if not data['available']:
      msg = "No charging data available"
      msg_size = measure_text_cached(self._font_semi, msg, 36)
      rl.draw_text_ex(self._font_semi, msg,
                       rl.Vector2(rect.x + (rect.width - msg_size.x) / 2, rect.y + rect.height / 2 - 18),
                       36, 0, BPColors.TEXT_SECONDARY)
      return

    status_color = BPColors.GOOD if data['active'] else BPColors.TEXT_SECONDARY
    status_text = data['status'] or ("Charging" if data['active'] else "Not Charging")
    rl.draw_text_ex(self._font_semi, status_text, rl.Vector2(rect.x + pad, rect.y + pad + 60), 32, 0, status_color)

    # kW leads, in the big font. Amps only earns a column on a car that actually reports pack
    # current: on the Mach-E it is motor current, flat 0.0 A for a whole parked charge, so the
    # tile was only ever a zero (see charge_session_history.amps_reported).
    readout_y = rect.y + pad + 130
    readout_h = READOUT_H
    columns = [(f"{data['kw']:.1f}", "kW", HERO_VALUE_FONT), (f"{data['soc']:.0f}%", "SOC", VALUE_FONT)]
    if charge_session_history.amps_reported:
      columns.insert(1, (f"{data['amps']:.0f}", "Amps", VALUE_FONT))
    col_w = (rect.width - 2 * pad) / len(columns)
    for i, (value, label, font_size) in enumerate(columns):
      self._draw_stat(rl.Rectangle(rect.x + pad + i * col_w, readout_y, col_w, readout_h),
                      value, label, font_size)

    # Time-to-80% estimate: fixed position right below the readout row (small
    # margin), not chart-relative, so it can never float below the visible panel.
    time_y = readout_y + readout_h + 15
    minutes = charge_session_history.estimate_minutes_to(TARGET_SOC_PCT)
    if minutes is not None:
      time_text = f"~{minutes:.0f} min to {TARGET_SOC_PCT:.0f}%"
    elif data['soc'] >= TARGET_SOC_PCT:
      time_text = f"Already at/above {TARGET_SOC_PCT:.0f}%"
    else:
      time_text = "Estimating time to 80%..."
    time_size = measure_text_cached(self._font_semi, time_text, 32)
    rl.draw_text_ex(self._font_semi, time_text,
                     rl.Vector2(rect.x + (rect.width - time_size.x) / 2, time_y), 32, 0, BPColors.WHITE)

    chart_y = time_y + 45
    close_area_h = self.CLOSE_BUTTON_SIZE + self.CLOSE_BUTTON_MARGIN * 2
    chart_h = max(100, rect.height - (chart_y - rect.y) - pad - close_area_h)
    self._chart.render(rl.Rectangle(rect.x + pad, chart_y, rect.width - 2 * pad, chart_h))

  def _draw_stat(self, rect: rl.Rectangle, value: str, label: str, value_font: int = VALUE_FONT) -> None:
    # Values are centered in a shared row height rather than drawn from its top, so a small
    # number (SOC) lines up with the big one (kW) instead of floating above it.
    value_h = rect.height - LABEL_FONT * LABEL_LINE
    value_size = measure_text_cached(self._font_bold, value, value_font)
    rl.draw_text_ex(self._font_bold, value,
                     rl.Vector2(rect.x + (rect.width - value_size.x) / 2,
                                rect.y + (value_h - value_size.y) / 2), value_font, 0, BPColors.WHITE)
    label_size = measure_text_cached(self._font_semi, label, LABEL_FONT)
    rl.draw_text_ex(self._font_semi, label,
                     rl.Vector2(rect.x + (rect.width - label_size.x) / 2, rect.y + value_h),
                     LABEL_FONT, 0, BPColors.TEXT_SECONDARY)

  def _render_close_button(self, panel_rect: rl.Rectangle):
    close_w = self.CLOSE_BUTTON_SIZE
    close_x = panel_rect.x + panel_rect.width - close_w - self.CLOSE_BUTTON_MARGIN
    close_y = panel_rect.y + self.CLOSE_BUTTON_MARGIN
    close_rect = rl.Rectangle(close_x, close_y, close_w, close_w)

    rl.draw_rectangle_rounded(close_rect, 0.3, 8, DebugColors.CLOSE_BG)
    rl.draw_rectangle_rounded_lines_ex(close_rect, 0.3, 8, 1.5, DebugColors.CLOSE_BORDER)

    close_text = "X"
    close_text_size = measure_text_cached(self._font_bold, close_text, 46)
    rl.draw_text_ex(self._font_bold, close_text,
                     rl.Vector2(close_x + (close_w - close_text_size.x) / 2,
                                close_y + (close_w - close_text_size.y) / 2),
                     46, 0, DebugColors.LEGEND_TEXT)

    for mouse_event in gui_app.mouse_events:
      if mouse_event.left_released:
        if rl.check_collision_point_rec(mouse_event.pos, close_rect):
          self._set_visible(False, reason="close_button")
          self._consumed_click = True

  def _consume_mouse_events(self, panel_rect: rl.Rectangle):
    if self._animation_progress > 0.5:
      self._rect = panel_rect

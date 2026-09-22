"""BluePilot MICI: Charging screen — the charge curve fills the whole background,
with live status/kW/SOC/time-to-80% overlaid on top.

kW carries the display: pack current is not shown because ampsActual is motor current on the
Mach-E (MtrTrac2_I_Actl), flat 0.0 A for an entire parked charge (route 0000041c: 5569 charging
samples, none non-zero), and that platform's own BattTrac_I_Actl is dead -- see carstate_ext.py.
"""

from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.bp.charging.session import charge_session_history, is_in_drive
from openpilot.selfdrive.ui.bp.widgets.charge_curve_chart import ChargeCurveChart
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.nav_widget import NavWidget
from bluepilot.ui.lib.colors import BPColors

TARGET_SOC_PCT = 80.0
SCRIM_COLOR = rl.Color(0, 0, 0, 150)
HERO_VALUE_FONT = 92  # kW, the one charging number the car reports usefully
VALUE_FONT = 48
LABEL_FONT = 24
VALUE_LINE = 1.15  # line height as a multiple of the font size
LABEL_LINE = 1.25
# Row height follows the fonts rather than a guessed constant: the mici screen is only 240px
# tall, so the whole block gets scaled down and a hard-coded row would clip the hero number.
READOUT_H = HERO_VALUE_FONT * VALUE_LINE + LABEL_FONT * LABEL_LINE
# No true "never timeout" sentinel exists in Device; reuse the large-number convention
# from selfdrive/ui/tests/diff/replay.py to keep the screen on indefinitely while open.
STAY_AWAKE_TIMEOUT_S = 99999


_shared_layout: 'ChargingLayoutMici | None' = None


def get_charging_layout() -> 'ChargingLayoutMici':
  """The one charging screen instance.

  The settings button and the charge-session auto-open (charging/auto_open.py) push the
  same widget: gui_app refuses a widget that is already on the nav stack, and two
  instances would fight over the interactive-timeout override on show/hide.
  """
  global _shared_layout
  if _shared_layout is None:
    _shared_layout = ChargingLayoutMici(back_callback=gui_app.pop_widget)
  return _shared_layout


class ChargingLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._chart = self._child(ChargeCurveChart(full_bleed=True))
    self._prev_charging_active = None
    self._prev_in_drive = None

  def show_event(self):
    super().show_event()
    self._prev_charging_active = None
    self._prev_in_drive = None
    device.set_override_interactive_timeout(STAY_AWAKE_TIMEOUT_S)

  def hide_event(self):
    super().hide_event()
    device.set_override_interactive_timeout(None)

  def _update_state(self):
    super()._update_state()
    charge_session_history.update()

    # Auto-dismiss once Drive is seen on two consecutive polls, not a single one:
    # a lone transient/stale gearShifter==drive sample (e.g. while parked and
    # charging) would otherwise re-close the screen the instant it's opened.
    in_drive = is_in_drive()
    if self._prev_in_drive and in_drive:
      gui_app.pop_widget()
      return
    self._prev_in_drive = in_drive

    # Auto-dismiss when the charging session ends (active -> inactive transition)
    charging_active = charge_session_history.is_active
    if self._prev_charging_active and not charging_active:
      gui_app.pop_widget()
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
        'soc': hybrid_battery.socActual if hybrid_battery.dataAvailable else 0.0,
      }
    except (KeyError, AttributeError, TypeError):
      return {'available': False, 'active': False, 'status': "", 'kw': 0.0, 'soc': 0.0}

  def _render(self, rect: rl.Rectangle) -> None:
    # Charge curve fills the entire background
    self._chart.render(rect)

    data = self._get_data()
    pad = 30

    if not data['available']:
      rl.draw_rectangle_rounded(rl.Rectangle(rect.x + pad, rect.y + rect.height / 2 - 50,
                                              rect.width - 2 * pad, 100), 0.15, 8, SCRIM_COLOR)
      gui_label(rl.Rectangle(rect.x + pad, rect.y + rect.height / 2 - 40, rect.width - 2 * pad, 80),
                "No charging data available", font_size=36, font_weight=FontWeight.MEDIUM,
                color=BPColors.TEXT_SECONDARY, alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)
      return

    # Top scrim: title, status, time-to-80% estimate, then the kW/SOC readout row.
    # The whole block is scaled to fit within a fraction of the actual screen height,
    # so on a short mici display everything shrinks together instead of the bottom
    # rows getting pushed off-screen.
    base_title_h, base_status_h, base_time_h, base_readout_h, base_row_gap, base_pad = 50, 40, 44, READOUT_H, 8, 30
    base_total_h = base_pad * 2 + base_title_h + base_status_h + base_time_h + base_readout_h + 3 * base_row_gap

    scale = min(1.0, (rect.height * 0.9) / base_total_h)
    pad = base_pad * scale
    title_h = base_title_h * scale
    status_h = base_status_h * scale
    time_h = base_time_h * scale
    readout_h = base_readout_h * scale
    row_gap = base_row_gap * scale

    title_y = rect.y + pad
    status_y = title_y + title_h
    time_y = status_y + status_h + row_gap
    readout_y = time_y + time_h + row_gap

    solid_h = min((readout_y - rect.y) + readout_h + pad, rect.height)
    fade_h = 40 * scale
    rl.draw_rectangle_rec(rl.Rectangle(rect.x, rect.y, rect.width, solid_h), rl.Color(0, 0, 0, 160))
    rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y + solid_h), int(rect.width), int(fade_h),
                                  rl.Color(0, 0, 0, 160), rl.Color(0, 0, 0, 0))

    gui_label(rl.Rectangle(rect.x + pad, title_y, rect.width - 2 * pad, title_h),
              "Charging", font_size=int(48 * scale), font_weight=FontWeight.BOLD, color=BPColors.WHITE)

    status_color = BPColors.GOOD if data['active'] else BPColors.TEXT_SECONDARY
    status_text = data['status'] or ("Charging" if data['active'] else "Not Charging")
    gui_label(rl.Rectangle(rect.x + pad, status_y, rect.width - 2 * pad, status_h),
              status_text, font_size=int(32 * scale), font_weight=FontWeight.MEDIUM, color=status_color)

    # Time-to-80% estimate
    minutes = charge_session_history.estimate_minutes_to(TARGET_SOC_PCT)
    if minutes is not None:
      time_text = f"~{minutes:.0f} min to {TARGET_SOC_PCT:.0f}%"
    elif data['soc'] >= TARGET_SOC_PCT:
      time_text = f"Already at/above {TARGET_SOC_PCT:.0f}%"
    else:
      time_text = "Estimating time to 80%..."
    gui_label(rl.Rectangle(rect.x + pad, time_y, rect.width - 2 * pad, time_h),
              time_text, font_size=int(32 * scale), font_weight=FontWeight.MEDIUM, color=BPColors.WHITE,
              alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)

    # kW in the hero font, SOC alongside it (no Amps column -- see the module docstring).
    col_w = (rect.width - 2 * pad) / 2
    self._draw_stat(rl.Rectangle(rect.x + pad, readout_y, col_w, readout_h),
                    f"{data['kw']:.1f}", "kW", scale, value_font=HERO_VALUE_FONT)
    self._draw_stat(rl.Rectangle(rect.x + pad + col_w, readout_y, col_w, readout_h),
                    f"{data['soc']:.0f}%", "SOC", scale)

  def _draw_stat(self, rect: rl.Rectangle, value: str, label: str, scale: float = 1.0,
                 value_font: int = VALUE_FONT) -> None:
    # Every column reserves the hero row height, whatever its font size, so a small value
    # (SOC) sits centered against the big one (kW) instead of riding above it.
    value_h = rect.height - LABEL_FONT * LABEL_LINE * scale
    gui_label(rl.Rectangle(rect.x, rect.y, rect.width, value_h), value, font_size=int(value_font * scale),
              font_weight=FontWeight.BOLD, color=BPColors.WHITE, alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)
    gui_label(rl.Rectangle(rect.x, rect.y + value_h, rect.width, rect.height - value_h), label,
              font_size=int(LABEL_FONT * scale), font_weight=FontWeight.MEDIUM, color=BPColors.LIGHT_GRAY,
              alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)

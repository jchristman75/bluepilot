"""BluePilot: HV battery temperature icon for the MICI onroad view, on the left edge between the driver-monitoring
ring and the steering wheel, behind BPShowBatteryTemp.

A small vertical battery whose fill and number show the pack temperature in degrees Celsius, colored cold-blue through green to hot-red.
Beside it an arrow shows the coolant loop's effect: up (orange) while the coolant is heating the pack, down (cyan) while
it is cooling it, none when they are close. The logic (hysteresis, units, colors) lives in lib/battery_thermal.py.
"""
import math

import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib import battery_thermal as bt
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PARAM = "BPShowBatteryTemp"
PARAM_REFRESH_FRAMES = 60

# Center of the icon as fractions of the content rect (the spot picked on the comma four's 476 x 255 road view).
CENTER_X = 0.099
CENTER_Y = 0.478

BODY_W, BODY_H = 38, 54
NUB_W, NUB_H = 14, 5
ARROW_W, ARROW_H = 15, 12
TEXT_CENTER = 0.36      # the number sits in the upper part of the body, as a fraction of its height
ARROW_CENTER = 0.76     # the arrow sits below it, inside the body, so showing it never moves the icon
FONT_SIZE = 26
DEGREE_R = 2.6          # the degree mark is drawn, so the number can use the full font size
OUTLINE_THICKNESS = 2.5
MAX_ALPHA = 0.92

HEATING_COLOR = (255, 150, 50)
COOLING_COLOR = (80, 210, 255)
ARROW_PULSE_RATE = 5.0       # rad/s
ARROW_PULSE_MIN = 0.55
SHADOW = rl.Color(0, 0, 0, 120)


def _rl(color: tuple[int, int, int], alpha: float) -> rl.Color:
  return rl.Color(color[0], color[1], color[2], int(255 * min(max(alpha, 0.0), 1.0)))


class BatteryTempIcon(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._enabled = self._params.get_bool(PARAM)
    self._param_counter = 0
    self._thermal = bt.BatteryThermal()
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._telemetry: tuple[float, float] | None = None
    self._hud_busy = False

  def set_hud_busy(self, busy: bool) -> None:
    """True while the HUD draws the set-speed block over this corner; the icon fades out until it is gone."""
    self._hud_busy = busy

  def _update_state(self) -> None:
    self._param_counter += 1
    if self._param_counter >= PARAM_REFRESH_FRAMES:
      self._param_counter = 0
      self._enabled = self._params.get_bool(PARAM)
    self._telemetry = bt.telemetry_from(ui_state.sm, ui_state.started_frame) if self._enabled else None
    pack, coolant = self._telemetry if self._telemetry is not None else (None, None)
    self._thermal.update(pack, coolant, 1 / gui_app.target_fps)

  def _draw_battery(self, x: float, y: float, pack_c: float, optimal: bool, alpha: float) -> None:
    color = bt.temp_color(pack_c)
    body = rl.Rectangle(x, y + NUB_H, BODY_W, BODY_H)
    radius = 0.28
    # soft backing so the icon reads over the camera image
    rl.draw_rectangle_rounded(rl.Rectangle(body.x - 2, body.y - 2, body.width + 4, body.height + 4), radius, 8,
                              rl.Color(0, 0, 0, int(110 * alpha)))
    # terminal nub
    rl.draw_rectangle_rounded(rl.Rectangle(x + (BODY_W - NUB_W) / 2, y, NUB_W, NUB_H + 3), 0.5, 4, _rl((235, 235, 235), 0.85 * alpha))
    # temperature fill, rising from the bottom
    fill_h = max(BODY_H * bt.fill_fraction(pack_c) - 4, 3)
    fill = rl.Rectangle(body.x + 3, body.y + body.height - 3 - fill_h, BODY_W - 6, fill_h)
    rl.draw_rectangle_rounded(fill, 0.3, 6, _rl(color, 0.72 * alpha))
    rl.draw_rectangle_rounded_lines_ex(body, radius, 8, OUTLINE_THICKNESS, _rl(bt.outline_color(optimal), 0.95 * alpha))

    text = bt.temp_text(pack_c)
    room = BODY_W - 4 - 2 * DEGREE_R
    size = measure_text_cached(self._font, text, FONT_SIZE)
    font_size = FONT_SIZE if size.x <= room else int(FONT_SIZE * room / size.x)
    size = measure_text_cached(self._font, text, font_size)
    # the number is centered with its degree mark counted in, so the pair sits in the middle of the body
    pos = rl.Vector2(body.x + (BODY_W - size.x - 2 * DEGREE_R - 1) / 2, body.y + BODY_H * TEXT_CENTER - size.y / 2)
    rl.draw_text_ex(self._font, text, rl.Vector2(pos.x + 1, pos.y + 1), font_size, 0, SHADOW)
    rl.draw_text_ex(self._font, text, pos, font_size, 0, rl.Color(255, 255, 255, int(255 * alpha)))
    ring = rl.Vector2(pos.x + size.x + DEGREE_R + 1, pos.y + size.y * 0.24)
    rl.draw_circle_lines(int(ring.x), int(ring.y), DEGREE_R, rl.Color(255, 255, 255, int(255 * alpha)))

  def _draw_arrow(self, center_x: float, center_y: float, trend: bt.Trend, alpha: float) -> None:
    pulse = ARROW_PULSE_MIN + (1.0 - ARROW_PULSE_MIN) * (0.5 - 0.5 * math.cos(rl.get_time() * ARROW_PULSE_RATE))
    color = _rl(HEATING_COLOR if trend == bt.Trend.HEATING else COOLING_COLOR, pulse * alpha)
    up = trend == bt.Trend.HEATING

    def tri(grow: float):
      w, h = ARROW_W / 2 + grow, ARROW_H / 2 + grow
      if up:      # counter-clockwise vertices for raylib
        return rl.Vector2(center_x, center_y - h), rl.Vector2(center_x - w, center_y + h), rl.Vector2(center_x + w, center_y + h)
      return rl.Vector2(center_x - w, center_y - h), rl.Vector2(center_x, center_y + h), rl.Vector2(center_x + w, center_y - h)

    rl.draw_triangle(*tri(1.6), rl.Color(0, 0, 0, int(170 * alpha)))    # dark rim so the arrow reads over any fill color
    rl.draw_triangle(*tri(0.0), color)

  def _render(self, rect: rl.Rectangle) -> None:
    visible = bt.should_show(self._enabled, self._telemetry, self._hud_busy)
    alpha = self._alpha_filter.update(MAX_ALPHA if visible else 0.0)
    if alpha < 1e-2 or self._telemetry is None:
      return
    pack_c, _ = self._telemetry
    trend = self._thermal.trend

    cx = rect.x + rect.width * CENTER_X
    cy = rect.y + rect.height * CENTER_Y
    x = cx - BODY_W / 2
    y = cy - (NUB_H + BODY_H) / 2
    self._draw_battery(x, y, pack_c, self._thermal.optimal, alpha)
    if trend != bt.Trend.STEADY:
      self._draw_arrow(cx, y + NUB_H + BODY_H * ARROW_CENTER, trend, alpha)

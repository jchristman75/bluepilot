"""
BluePilot charge-curve chart widget.

Draws a kW-over-time area chart for the current charge session using raw pyray
primitives (no charting library exists elsewhere in this codebase). Used as:
- a small sparkline in the tici sidebar's expanded charging card (compact=True)
- a large card-style chart on the tici charging overlay panel (default)
- a full-bleed background chart filling the entire mici Charging screen (full_bleed=True)
"""

import pyray as rl

from openpilot.selfdrive.ui.bp.charging.session import charge_session_history
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from bluepilot.ui.lib.colors import BPColors

LINE_COLOR = BPColors.ACCENT
FILL_COLOR = rl.Color(BPColors.ACCENT.r, BPColors.ACCENT.g, BPColors.ACCENT.b, 60)
GRID_COLOR = rl.Color(255, 255, 255, 25)
AXIS_TEXT_COLOR = BPColors.TEXT_SECONDARY

# Full-bleed (mici background) styling: darker base + a stronger vertical gradient fill
BACKGROUND_BASE_COLOR = BPColors.BACKGROUND
GRADIENT_TOP_COLOR = rl.Color(BPColors.ACCENT.r, BPColors.ACCENT.g, BPColors.ACCENT.b, 50)
GRADIENT_BOTTOM_COLOR = rl.Color(BPColors.ACCENT.r, BPColors.ACCENT.g, BPColors.ACCENT.b, 190)
FULL_BLEED_GRID_COLOR = rl.Color(255, 255, 255, 18)
FULL_BLEED_TOP_MARGIN = 40  # keeps the curve's peak from touching the very top edge


class ChargeCurveChart(Widget):
  """Renders the shared ChargeSessionHistory as a kW-over-time area chart."""

  def __init__(self, compact: bool = False, full_bleed: bool = False):
    super().__init__()
    self._compact = compact
    self._full_bleed = full_bleed
    self._font = gui_app.font(FontWeight.MEDIUM)

  def _update_state(self):
    charge_session_history.update()

  def _render(self, rect: rl.Rectangle) -> None:
    samples = charge_session_history.samples

    if self._full_bleed:
      rl.draw_rectangle_rec(rect, BACKGROUND_BASE_COLOR)
    elif not self._compact:
      rl.draw_rectangle_rounded(rect, 0.05, 8, BPColors.CARD_BACKGROUND)

    if len(samples) < 2:
      if not self._compact:
        text = "Waiting for charge data..."
        font_size = 32
        size = measure_text_cached(self._font, text, font_size)
        rl.draw_text_ex(self._font, text,
                         rl.Vector2(rect.x + (rect.width - size.x) / 2, rect.y + (rect.height - size.y) / 2),
                         font_size, 0, AXIS_TEXT_COLOR)
      return

    max_kw = max(0.1, max(s[2] for s in samples))
    max_elapsed = max(1.0, samples[-1][0])

    pad = 0 if self._full_bleed else (4 if self._compact else 16)
    top_pad = FULL_BLEED_TOP_MARGIN if self._full_bleed else pad
    plot_x = rect.x + pad
    plot_y = rect.y + top_pad
    plot_w = max(1.0, rect.width - 2 * pad)
    plot_h = max(1.0, rect.height - top_pad - pad)

    grid_color = FULL_BLEED_GRID_COLOR if self._full_bleed else GRID_COLOR
    if not self._compact:
      # Horizontal gridlines at 25/50/75/100% of max_kw
      for frac in (0.25, 0.5, 0.75, 1.0):
        y = plot_y + plot_h * (1.0 - frac)
        rl.draw_line_ex(rl.Vector2(plot_x, y), rl.Vector2(plot_x + plot_w, y), 1, grid_color)

    def to_point(elapsed: float, kw: float) -> rl.Vector2:
      x = plot_x + (elapsed / max_elapsed) * plot_w
      y = plot_y + plot_h * (1.0 - kw / max_kw)
      return rl.Vector2(x, y)

    points = [to_point(s[0], s[2]) for s in samples]
    base_y = plot_y + plot_h

    if self._full_bleed:
      # Area chart look: draw a strong vertical gradient across the whole plot,
      # then mask back out the area above the curve with the base background color.
      rl.draw_rectangle_gradient_v(int(plot_x), int(plot_y), int(plot_w), int(plot_h),
                                    GRADIENT_TOP_COLOR, GRADIENT_BOTTOM_COLOR)
      for i in range(len(points) - 1):
        p0, p1 = points[i], points[i + 1]
        rl.draw_triangle(rl.Vector2(p0.x, plot_y), p0, rl.Vector2(p1.x, plot_y), BACKGROUND_BASE_COLOR)
        rl.draw_triangle(rl.Vector2(p1.x, plot_y), p0, p1, BACKGROUND_BASE_COLOR)
    else:
      # Filled area under the curve
      for i in range(len(points) - 1):
        p0, p1 = points[i], points[i + 1]
        rl.draw_triangle(rl.Vector2(p0.x, base_y), p0, p1, FILL_COLOR)
        rl.draw_triangle(rl.Vector2(p0.x, base_y), p1, rl.Vector2(p1.x, base_y), FILL_COLOR)

    # Line
    line_thick = 2 if self._compact else (5 if self._full_bleed else 4)
    for i in range(len(points) - 1):
      rl.draw_line_ex(points[i], points[i + 1], line_thick, LINE_COLOR)

    # Current-value marker
    marker_radius = 4 if self._compact else 7
    rl.draw_circle_v(points[-1], marker_radius, BPColors.WHITE)
    rl.draw_circle_v(points[-1], marker_radius - 2, LINE_COLOR)

    if not self._compact and not self._full_bleed:
      label = f"{max_kw:.1f} kW"
      font_size = 24
      rl.draw_text_ex(self._font, label, rl.Vector2(plot_x, rect.y + 4), font_size, 0, AXIS_TEXT_COLOR)

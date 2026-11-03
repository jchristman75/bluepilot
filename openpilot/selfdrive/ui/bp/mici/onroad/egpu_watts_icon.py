"""BluePilot: eGPU (Chestnut) badge for the MICI onroad view, upper right above the complication.

The eGPU logo with the GPU's current power draw printed where the "GPU" lettering normally is.
Shown only while the big model is running on the eGPU (ChestnutState.ACTIVE), behind BPShowEgpuWatts.

Power comes from chestnutState.powerDrawW (hardwared, 10 Hz), which carries the SMU's
AverageSocketPower that modeld samples. modeld only re-reads the SMU every 100 sends (~10 s),
so the number steps rather than tracks.
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PARAM = "BPShowEgpuWatts"

# Source art is 144x107 (icons_mici/chestnut_green.png with the letters filled in).
ICON_W = 80
ICON_H = round(ICON_W * 107 / 144)
# Screen-body area of the logo (where "GPU" sits), as fractions of the icon.
BODY_CENTER_Y = 0.385
TEXT_MAX_W = 0.80
TEXT_MAX_H = 0.48
MAX_FONT_SIZE = 40

# Lines up with MiciComplication's column (WIDTH 80, 5 px from the right edge).
COMPLICATION_W = 80
RIGHT_MARGIN = 5
TOP_MARGIN = 14
TEXT_CLEARANCE = 10  # gap kept between alert text and the badge

TEXT_COLOR = rl.Color(12, 20, 16, 235)
# Partly see-through so the road shows behind it; the text keeps a little more weight than the logo.
ICON_OPACITY = 0.6
TEXT_OPACITY = 0.85
PARAM_REFRESH_FRAMES = 60


class EgpuWattsIcon(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._enabled = self._params.get_bool(PARAM)
    self._param_counter = 0
    self._texture: rl.Texture = gui_app.texture("icons_mici/bp_egpu_blank.png", ICON_W, ICON_H)
    self._font: rl.Font = gui_app.font(FontWeight.BOLD)
    self._alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._watts: float | None = None

  def _update_state(self) -> None:
    self._param_counter += 1
    if self._param_counter >= PARAM_REFRESH_FRAMES:
      self._param_counter = 0
      self._enabled = self._params.get_bool(PARAM)

    sm = ui_state.sm
    if sm.alive["chestnutState"] and sm.valid["chestnutState"] and sm["chestnutState"].powerDrawW > 0:
      self._watts = sm["chestnutState"].powerDrawW
    elif not sm.alive["chestnutState"]:
      self._watts = None

  def reserved_right(self) -> float:
    """Width from the content rect's right edge that other top-of-screen text should avoid while the badge shows."""
    if self._alpha_filter.x < 1e-2:
      return 0.0
    return RIGHT_MARGIN + COMPLICATION_W / 2 + ICON_W / 2 + TEXT_CLEARANCE

  def watts_text(self) -> str:
    return f"{round(self._watts)}W" if self._watts is not None else "--W"

  def _render(self, rect: rl.Rectangle) -> None:
    visible = self._enabled and ui_state.chestnut_state == ChestnutState.ACTIVE
    alpha = self._alpha_filter.update(1.0 if visible else 0.0)
    if alpha < 1e-2:
      return

    column_center_x = rect.x + rect.width - RIGHT_MARGIN - COMPLICATION_W / 2
    x = column_center_x - ICON_W / 2
    y = rect.y + TOP_MARGIN
    rl.draw_texture_ex(self._texture, rl.Vector2(x, y), 0.0, 1.0, rl.Color(255, 255, 255, int(255 * ICON_OPACITY * alpha)))

    text = self.watts_text()
    font_size = MAX_FONT_SIZE
    size = measure_text_cached(self._font, text, font_size)
    scale = min(1.0, ICON_W * TEXT_MAX_W / max(size.x, 1), ICON_H * TEXT_MAX_H / max(size.y, 1))
    if scale < 1.0:
      font_size = int(font_size * scale)
      size = measure_text_cached(self._font, text, font_size)

    pos = rl.Vector2(column_center_x - size.x / 2, y + ICON_H * BODY_CENTER_Y - size.y / 2)
    color = rl.Color(TEXT_COLOR.r, TEXT_COLOR.g, TEXT_COLOR.b, int(TEXT_COLOR.a * TEXT_OPACITY * alpha))
    rl.draw_text_ex(self._font, text, pos, font_size, 0, color)

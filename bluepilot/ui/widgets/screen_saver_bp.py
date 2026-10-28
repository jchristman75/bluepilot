"""
BluePilot: bouncing BluePilot logo screensaver.

Same behaviour as sunnypilot's ScreenSaverSP (bounce, recolor on every edge hit, tap to dismiss,
timeout) but draws the BluePilot logo -- the white-on-transparent spinner artwork, tinted with the
current hue -- instead of the "sunnypilot" text. Falls back to the text "bluepilot" if the logo
can't be loaded.
"""
import time

import pyray as rl

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.sunnypilot.widgets.screen_saver import ScreenSaverSP
from openpilot.system.ui.widgets import Widget

LOGO_PATH = "images/spinner_bluepilot.png"
LOGO_SRC_SIZE = 1024                  # the artwork is 1024 x 1024 with a transparent background
LOGO_CROP = (41, 256, 1006, 727)      # bounding box of the drawn logo (x0, y0, x1, y1) in source pixels


class ScreenSaverBP(ScreenSaverSP):
  def __init__(self, params: Params | None = None):
    super().__init__(params=params)
    self.text = "bluepilot"
    crop_w = LOGO_CROP[2] - LOGO_CROP[0]
    crop_h = LOGO_CROP[3] - LOGO_CROP[1]
    self._logo_draw_w = 240.0 if self._is_mici else 760.0
    self._logo_draw_h = self._logo_draw_w * crop_h / crop_w
    self._logo: rl.Texture | None = None
    self._logo_src: rl.Rectangle | None = None
    self._logo_tried = False

  def _ensure_logo(self) -> None:
    # loaded on first use: UIState builds the screensaver before the window (GL context) exists
    if self._logo_tried:
      return
    self._logo_tried = True
    crop_w = LOGO_CROP[2] - LOGO_CROP[0]
    crop_h = LOGO_CROP[3] - LOGO_CROP[1]
    try:
      tex_size = round(self._logo_draw_w * LOGO_SRC_SIZE / crop_w)  # load just big enough for the draw size
      self._logo = gui_app.texture(LOGO_PATH, tex_size, tex_size)
      s = self._logo.width / LOGO_SRC_SIZE
      self._logo_src = rl.Rectangle(LOGO_CROP[0] * s, LOGO_CROP[1] * s, crop_w * s, crop_h * s)
    except Exception:
      cloudlog.exception("ScreenSaverBP: logo failed to load, using text")
      self._logo = None

  def _update_state(self):
    self._ensure_logo()
    if self._logo is None:
      return super()._update_state()
    # ScreenSaverSP._update_state with the bounce box sized to the logo instead of the text
    Widget._update_state(self)
    self.logo_width, self.logo_height = self._logo_draw_w, self._logo_draw_h

    if self._start_time and time.monotonic() - self._start_time > self._screensaver_timeout:
      self._dismiss = True
      self._start_time = None

    dt = rl.get_frame_time()
    self.x += self.vx * dt
    self.y += self.vy * dt

    hit_x = hit_y = False
    if self.x + self.logo_width > self.rect.width:
      self.vx *= -1
      self.x = self.rect.width - self.logo_width
      hit_x = True
    elif self.x < 0:
      self.vx *= -1
      self.x = 0
      hit_x = True

    if self.y + self.logo_height > self.rect.height:
      self.vy *= -1
      self.y = self.rect.height - self.logo_height
      hit_y = True
    elif self.y < 0:
      self.vy *= -1
      self.y = 0
      hit_y = True

    hit = hit_x or hit_y
    if hit and not self._hit_last_frame:
      while self._hue_dist((new_hue := rl.get_random_value(0, 360)), self._hue) < 120:
        pass
      self._hue = new_hue
      self.color = rl.color_from_hsv(self._hue, 1, 1)
    self._hit_last_frame = hit

  def _render(self, rect: rl.Rectangle):
    if self._logo is None:
      return super()._render(rect)
    self.set_rect(rect)
    rl.clear_background(rl.BLACK)
    dst = rl.Rectangle(int(self.x), int(self.y), self._logo_draw_w, self._logo_draw_h)
    rl.draw_texture_pro(self._logo, self._logo_src, dst, rl.Vector2(0, 0), 0.0, self.color)
    return -1

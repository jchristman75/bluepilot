"""
BluePilot: hatched blind spot lane, shared by the TICI and MICI model renderers.

When the car reports a blind spot detection, the lane on that side is marked with diagonal stripes in the
road, like CarrotPilot's. Beyond CarrotPilot: signaling toward an occupied lane (the lane-change case)
escalates the stripes to red and a faster pulse, and the lane is placed off the planned path when the
model can't see the ego lane line (faded paint). Shown with the existing "Show Blindspot Overlay" setting.

The mixin only needs what both ModelRenderer families have: _lane_lines, _lane_line_probs, _path,
_car_space_transform and _clip_region (set by the model renderer's own _render).
"""
import math
import time

import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.lib import blindspot_geometry as geo
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app

CAUTION_RGB = (255, 215, 0)     # CarrotPilot's gold
WARNING_RGB = (255, 56, 40)     # detection on the side you are signaling toward
FILL_ALPHA = 190
OUTLINE_ALPHA = 215
OUTLINE_WIDTH = 2.0             # px, near stripes; far stripes get a thinner, fainter outline
CAUTION_PULSE_S, WARNING_PULSE_S = 2.0, 0.8
CAUTION_PULSE_MIN, WARNING_PULSE_MIN = 0.8, 0.55   # lowest brightness of the pulse
PARAM_REFRESH_FRAMES = 60


def pulse(t: float, period_s: float, lowest: float) -> float:
  """Brightness factor from `lowest` to 1.0, a smooth sine over `period_s`."""
  return lowest + (1.0 - lowest) * (0.5 + 0.5 * math.sin(2 * math.pi * t / period_s))


class BlindspotPathMixin:
  """Draws the hatched lane for left/right blind spot detections (see module docstring)."""

  def _init_blindspot_path(self):
    self._bsp_params = Params()
    fps = gui_app.target_fps
    self._bsp_alpha = [FirstOrderFilter(0.0, 0.15, 1 / fps), FirstOrderFilter(0.0, 0.15, 1 / fps)]  # left, right
    self._bsp_counter = PARAM_REFRESH_FRAMES
    self._bsp_enabled = False
    self._bsp_failed = False

  def _bsp_param_enabled(self) -> bool:
    self._bsp_counter += 1
    if self._bsp_counter >= PARAM_REFRESH_FRAMES:
      self._bsp_counter = 0
      try:
        self._bsp_enabled = self._bsp_params.get_bool("ShowBlindspotOverlay")
      except UnknownKeyName:
        self._bsp_enabled = False
    return self._bsp_enabled

  def _bsp_ready(self) -> bool:
    sm = ui_state.sm
    return (self._clip_region is not None and sm.valid['carState'] and
            sm.recv_frame["extrinsicsCalibration"] >= ui_state.started_frame and
            sm.recv_frame["modelV2"] >= ui_state.started_frame)

  def _bsp_project(self, quad3: np.ndarray):
    """(4, 3) car-space corners -> list of 4 rl.Vector2 in screen space, or None if any is off the clip region."""
    proj = self._car_space_transform @ quad3.T            # 3 x 4
    if np.any(np.abs(proj[2]) < 1e-6):
      return None
    xy = (proj[:2] / proj[2]).T                            # 4 x 2
    clip = self._clip_region
    if not (np.all(xy[:, 0] >= clip.x) and np.all(xy[:, 0] <= clip.x + clip.width) and
            np.all(xy[:, 1] >= clip.y) and np.all(xy[:, 1] <= clip.y + clip.height)):
      return None
    return [rl.Vector2(float(x), float(y)) for x, y in xy]

  def _draw_blindspot_paths(self):
    """Call once per frame after the road is drawn (cheap when there is no detection).

    Purely cosmetic, so a failure here must never take the UI down: log once and stay off until restart."""
    if self._bsp_failed:
      return
    try:
      self._draw_blindspot_paths_impl()
    except Exception:
      self._bsp_failed = True
      cloudlog.exception("bp blindspot path failed, disabled until the UI restarts")

  def _draw_blindspot_paths_impl(self):
    enabled = self._bsp_param_enabled()
    if not (enabled and self._bsp_ready()):
      for f in self._bsp_alpha:
        f.update(0.0)
      return

    cs = ui_state.sm['carState']
    now = time.monotonic()
    # side -1 = left (negative y), +1 = right; lane lines 0..3 are far-left, ego-left, ego-right, far-right
    for k, (side, detected, signaling, inner_i, outer_i) in enumerate((
      (-1, cs.leftBlindspot, cs.leftBlinker, 1, 0),
      (+1, cs.rightBlindspot, cs.rightBlinker, 2, 3),
    )):
      f = self._bsp_alpha[k]
      f.update(1.0 if detected else 0.0)
      if f.x <= 0.01:
        continue

      lane = geo.adjacent_lane(
        self._lane_lines[inner_i].raw_points, float(self._lane_line_probs[inner_i]),
        self._lane_lines[outer_i].raw_points, float(self._lane_line_probs[outer_i]),
        self._path.raw_points, side, path_z_offset=float(getattr(self, '_path_offset_z', 0.0)))
      if lane is None:
        continue
      xs, inner3, outer3 = lane

      rgb, period, lowest = (WARNING_RGB, WARNING_PULSE_S, WARNING_PULSE_MIN) if signaling else \
                            (CAUTION_RGB, CAUTION_PULSE_S, CAUTION_PULSE_MIN)
      brightness = f.x * pulse(now, period, lowest)
      fill = rl.Color(rgb[0], rgb[1], rgb[2], int(FILL_ALPHA * brightness))
      for quad3, fade in geo.stripes(xs, inner3, outer3):
        pts = self._bsp_project(quad3)
        if pts is None:
          continue
        area2 = sum(pts[i].x * pts[(i + 1) % 4].y - pts[(i + 1) % 4].x * pts[i].y for i in range(4))
        if area2 > 0:
          pts.reverse()
        scaled = rl.Color(fill.r, fill.g, fill.b, int(fill.a * fade))
        rl.draw_triangle_fan(pts, len(pts), scaled)
        # the outline fades faster than the fill, or the small far stripes turn into pale smudges
        edge = rl.Color(255, 255, 255, int(OUTLINE_ALPHA * f.x * fade * fade))
        width = 1.0 + (OUTLINE_WIDTH - 1.0) * fade
        for i in range(4):
          rl.draw_line_ex(pts[i], pts[(i + 1) % 4], width, edge)

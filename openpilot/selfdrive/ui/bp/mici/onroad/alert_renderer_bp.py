"""BluePilot: MICI alert renderer that keeps alert text clear of the eGPU badge.

While the badge is up (upper right, see egpu_watts_icon.py), the alert text area stops short of the
badge column and the text shrinks so the alert is no taller than it would be at full width
("turning right", "big model ready", ...). With no badge this is the stock renderer.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.alert_renderer import AlertRenderer, AlertLayout
from openpilot.system.ui.widgets.label import UnifiedLabel

MIN_FONT_SIZE = 32
FONT_STEP = 2


class _FitLabel(UnifiedLabel):
  """UnifiedLabel that, given a narrowed width, picks the largest font up to the requested size whose
  wrapped text is no taller than the requested size at the full width."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.fit_width: float | None = None
    self.full_width: float | None = None

  def set_font_size(self, size: int):
    super().set_font_size(size)
    if self.fit_width is None or self.full_width is None or self.fit_width >= self.full_width or not self.text:
      return

    target_h = self.get_content_height(int(self.full_width))
    font_size = size
    while font_size > MIN_FONT_SIZE:
      super().set_font_size(font_size)
      if self.get_content_height(int(self.fit_width)) <= target_h + 0.5 and self.text_width <= self.fit_width:
        return
      font_size -= FONT_STEP
    super().set_font_size(MIN_FONT_SIZE)


def _fit_label_from(label: UnifiedLabel) -> _FitLabel:
  return _FitLabel(text="", font_size=label.font_size, font_weight=label._font_weight,
                   line_height=label._line_height / 0.9, letter_spacing=label._letter_spacing)


class MiciAlertRendererBP(AlertRenderer):
  def __init__(self):
    super().__init__()
    self._alert_text1_label = _fit_label_from(self._alert_text1_label)
    self._alert_text2_label = _fit_label_from(self._alert_text2_label)
    self._reserved_right = 0.0

  def set_reserved_right(self, px: float) -> None:
    """Width at the right edge of the alert rect the text must stay out of (0 = none)."""
    self._reserved_right = px

  def _icon_helper(self, alert) -> AlertLayout:
    layout = super()._icon_helper(alert)
    full_width = layout.text_rect.width
    text_rect = layout.text_rect
    if self._reserved_right > 0:
      limit = self._rect.x + self._rect.width - self._reserved_right
      text_rect = rl.Rectangle(text_rect.x, text_rect.y, max(0.0, min(full_width, limit - text_rect.x)), text_rect.height)

    fit_width = text_rect.width if text_rect.width < full_width else None
    for label in (self._alert_text1_label, self._alert_text2_label):
      label.fit_width = fit_width
      label.full_width = full_width
    return AlertLayout(text_rect, layout.icon)

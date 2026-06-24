class AutotuneDisplayMixin:
  """Mixin for HUD renderers that display autotune factor values.

  Requires self._bp_params to be a Params instance (set by the concrete class).
  """

  def _init_autotune_display(self) -> None:
    self._autotune_enable: bool = self._bp_params.get_bool("FordAngleAutoTuneEnable")
    self._at_low_factor: float = 1.0
    self._at_high_factor: float = 1.0

  def _refresh_autotune_display(self) -> None:
    self._autotune_enable = self._bp_params.get_bool("FordAngleAutoTuneEnable")
    try:
      self._at_low_factor = float(self._bp_params.get("FordAngleLowSpeedFactor", return_default=True))
      self._at_high_factor = float(self._bp_params.get("FordAngleHighSpeedFactor", return_default=True))
    except (TypeError, ValueError):
      pass

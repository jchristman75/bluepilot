"""Ford angle auto-tune: integrating controller that adjusts low/high speed curvature factors
based on the tracking error between commanded and actual curvature (yawRate / vEgo).
"""
import time

from numpy import clip


class _AutoTuner:
  __slots__ = ("error_smooth_curve", "integral_curve",
               "error_smooth_straight", "integral_straight",
               "frames_since_adj", "dirty",
               "factor_name", "ts_name", "count_name")

  def __init__(self, factor_name: str, ts_name: str, count_name: str):
    self.error_smooth_curve = 0.0
    self.integral_curve = 0.0
    self.error_smooth_straight = 0.0
    self.integral_straight = 0.0
    self.frames_since_adj = 0
    self.dirty = False
    self.factor_name = factor_name
    self.ts_name = ts_name
    self.count_name = count_name

  def reset(self):
    self.error_smooth_curve = 0.0
    self.integral_curve = 0.0
    self.error_smooth_straight = 0.0
    self.integral_straight = 0.0
    self.frames_since_adj = 0

  def _post_adjust(self):
    self.dirty = True
    self.reset()

  def flush(self, params) -> None:
    if not self.dirty:
      return
    try:
      params.put(self.ts_name, int(time.time()))
      try:
        cnt_raw = params.get(self.count_name, return_default=True)
        cnt = cnt_raw if isinstance(cnt_raw, int) else 0
        params.put(self.count_name, cnt + 1)
      except Exception:
        pass
    except Exception:
      pass
    self.dirty = False


class LateralAutoTuner:
  """Integrating auto-tuner for Ford angle-mode low/high speed curvature factors.

  Curve and straight tracking are integrated separately:
  - Curve integral drives the factor both UP and DOWN (primary signal).
  - Straight integral only drives DOWN, and only at a higher threshold so that
    ordinary road camber / sensor noise cannot pull the factor down on its own;
    only sustained, noticeable oscillation on a commanded-straight path triggers it.
  """

  _AT_ALPHA = 0.1
  # Curve threshold — normal sensitivity.
  _AT_INT_THRESH = 0.0008
  # Straight threshold — requires much more sustained oscillation to trigger.
  _AT_INT_THRESH_STRAIGHT = 0.005
  _AT_INT_CLAMP = 0.0002
  _AT_MIN_FRAMES = 150
  _AT_STEP = 0.01
  _AT_SPEED_BOUNDARY = 20.0
  _AT_KAPPA_MIN = 0.003
  _AT_KAPPA_MAX = 0.040
  _AT_KAPPA_STRAIGHT = 0.001
  _AT_ACTUAL_KAPPA_MIN = 0.0001
  _AT_V_MIN = 5.0

  def __init__(self):
    self._at_regime_low = _AutoTuner(
      "FordAngleLowSpeedFactor",
      "FordAngleAutoTuneLastAdjustedLow",
      "FordAngleAutoTuneAdjustmentsLow",
    )
    self._at_regime_high = _AutoTuner(
      "FordAngleHighSpeedFactor",
      "FordAngleAutoTuneLastAdjustedHigh",
      "FordAngleAutoTuneAdjustmentsHigh",
    )
    self._params = None
    self.enabled = False
    self.low_factor = 1.0
    self.high_factor = 1.0
    self._at_last_flush_ts = 0.0
    self._at_flush_interval = 30.0
    self.actual_curvature = 0.0
    self.curvature_error = 0.0
    self.integral_low = 0.0
    self.integral_high = 0.0

  def configure(self, params, enabled: bool, param_low: float, param_high: float) -> None:
    self._params = params
    self.enabled = enabled
    if not self._at_regime_low.dirty:
      self.low_factor = param_low
    if not self._at_regime_high.dirty:
      self.high_factor = param_high

  def reset(self) -> None:
    self._at_regime_low.reset()
    self._at_regime_high.reset()
    self.actual_curvature = 0.0
    self.curvature_error = 0.0

  def flush(self) -> None:
    if self._params is None:
      return
    try:
      self._at_regime_low.flush(self._params)
      self._at_regime_high.flush(self._params)
    except Exception:
      pass
    self._at_last_flush_ts = time.monotonic()

  def update(self, kappa_cmd: float, v_ego: float, CS) -> None:
    if not self.enabled:
      return

    actual_kappa = 0.0
    try:
      actual_yaw = float(CS.out.yawRate)
      actual_v = float(CS.out.vEgoRaw)
      if actual_v <= self._AT_V_MIN:
        return
      actual_kappa = actual_yaw / actual_v
    except Exception:
      return

    abs_kappa = abs(kappa_cmd)
    active = self._at_regime_low if v_ego < self._AT_SPEED_BOUNDARY else self._at_regime_high

    is_straight = abs_kappa < self._AT_KAPPA_STRAIGHT

    if is_straight:
      if abs(actual_kappa) < self._AT_ACTUAL_KAPPA_MIN:
        self._at_regime_low.frames_since_adj += 1
        self._at_regime_high.frames_since_adj += 1
        return
    else:
      if abs_kappa < self._AT_KAPPA_MIN or abs_kappa > self._AT_KAPPA_MAX:
        self._at_regime_low.frames_since_adj += 1
        self._at_regime_high.frames_since_adj += 1
        return

    self._at_regime_low.frames_since_adj += 1
    self._at_regime_high.frames_since_adj += 1

    raw_error = -abs(actual_kappa) if is_straight else kappa_cmd - actual_kappa

    if is_straight:
      active.error_smooth_straight = (self._AT_ALPHA * raw_error
                                      + (1 - self._AT_ALPHA) * active.error_smooth_straight)
      active.integral_straight += clip(active.error_smooth_straight, -self._AT_INT_CLAMP, self._AT_INT_CLAMP)
    else:
      active.error_smooth_curve = (self._AT_ALPHA * raw_error
                                   + (1 - self._AT_ALPHA) * active.error_smooth_curve)
      active.integral_curve += clip(active.error_smooth_curve, -self._AT_INT_CLAMP, self._AT_INT_CLAMP)

    if active.frames_since_adj >= self._AT_MIN_FRAMES:
      if active.integral_curve > self._AT_INT_THRESH:
        self._adjust_factor(active.factor_name, self._AT_STEP)
        active._post_adjust()
      elif active.integral_curve < -self._AT_INT_THRESH:
        self._adjust_factor(active.factor_name, -self._AT_STEP)
        active._post_adjust()
      elif active.integral_straight < -self._AT_INT_THRESH_STRAIGHT:
        self._adjust_factor(active.factor_name, -self._AT_STEP)
        active._post_adjust()

    self.actual_curvature = actual_kappa
    self.curvature_error = raw_error
    if v_ego < self._AT_SPEED_BOUNDARY:
      self.integral_low = active.integral_curve
    else:
      self.integral_high = active.integral_curve

    now = time.monotonic()
    if self._at_regime_low.dirty or self._at_regime_high.dirty:
      if now - self._at_last_flush_ts >= self._at_flush_interval:
        self.flush()

  def _adjust_factor(self, param_name: str, step: float) -> None:
    current = self.low_factor if param_name == "FordAngleLowSpeedFactor" else self.high_factor
    new_val = float(clip(current + step, 0.5, 1.5))
    try:
      self._params.put(param_name, new_val)
    except Exception:
      return
    if param_name == "FordAngleLowSpeedFactor":
      self.low_factor = new_val
    else:
      self.high_factor = new_val

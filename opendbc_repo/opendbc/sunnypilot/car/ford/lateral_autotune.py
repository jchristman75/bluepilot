"""Ford angle auto-tune: integrating controller that adjusts low/high speed curvature factors
based on the tracking error between commanded and actual curvature (yawRate / vEgo).

Convergence strategy:
  1. Curve-entry counting — require N distinct curve entries before each adjustment,
     so the integral reflects diverse real-world turns rather than one long curve.
  2. Blended attribution — at any speed, error is distributed between both factors
     proportionally to how much each contributes to the gain at that speed.  This
     mirrors the gain interpolation in lateral_angle_ext.py and lets both factors
     converge regardless of which speed range the driver favours.
  3. Oscillation detection — when recent adjustments alternate direction the factor
     is near optimal; switch to a finer alpha to zoom in rather than bounce.
  4. Decaying learning rate — alpha shrinks as lifetime adjustment count grows,
     locking the value in over many drives while keeping a floor for gradual drift.
"""
import time

from numpy import clip, interp


class AutoTuner:
  __slots__ = ("integral",
                "curve_count",
                "adj_history", "adj_count",
                "dirty", "factor_dirty",
                "factor",
                "factor_name", "ts_name", "count_name")

  def __init__(self, factor_name: str, ts_name: str, count_name: str, factor: float = 1.0):
    self.integral = 0.0
    self.curve_count = 0
    self.adj_history = []   # recent adjustment directions: +1 or -1
    self.adj_count = 0      # lifetime total, loaded from params on first configure
    self.dirty = False
    self.factor_dirty = False
    self.factor = factor
    self.factor_name = factor_name
    self.ts_name = ts_name
    self.count_name = count_name

  def reset(self):
    self.integral = 0.0
    self.curve_count = 0
    # adj_history and adj_count persist across adjustments intentionally

  def _post_adjust(self):
    self.dirty = True
    self.factor_dirty = True
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
  _AT_ALPHA = 0.1
  _AT_INT_THRESH = 0.0008
  _AT_INT_CLAMP = 0.0002
  # Require this many distinct curve entries before each adjustment.
  _AT_MIN_CURVES = 3
  _AT_STEP = 0.01
  _AT_MAX_RATIO = 3.0
  # Coarse alpha: factor is far from optimum, adjustments are consistent direction.
  _AT_BLEND_ALPHA = 0.4
  # Fine alpha: adjustments are oscillating — zoom in on the midpoint.
  _AT_BLEND_ALPHA_FINE = 0.08
  # Floor: never go below this so the system stays responsive to genuine drift.
  _AT_BLEND_ALPHA_MIN = 0.05
  # How fast alpha decays per lifetime adjustment (applied after coarse/fine pick).
  _AT_DECAY_RATE = 0.15
  # How many recent adjustment directions to inspect for oscillation.
  _AT_ADJ_HISTORY_LEN = 4
  # Speed breakpoints matching the high_gain_calc interpolation in lateral_angle_ext.py.
  # low_factor has full weight at or below _AT_V_LOW; high_factor at or above _AT_V_HIGH.
  # Between them, each factor's share of the error is proportional to its gain contribution.
  _AT_V_LOW  = 13.5    # m/s (~30 mph)
  _AT_V_HIGH = 26.82   # m/s (~60 mph)
  # Skip firing an adjustment for a factor whose weight is negligible at current speed.
  _AT_MIN_WEIGHT = 0.05
  _AT_KAPPA_MIN = 0.003
  _AT_KAPPA_MAX = 0.040
  _AT_V_MIN = 5.0
  # Debounce factor writes to params — max once per second.
  _AT_FACTOR_WRITE_INTERVAL = 1.0

  def __init__(self):
    self._at_regime_low = AutoTuner(
      "FordAngleLowSpeedFactor",
      "FordAngleAutoTuneLastAdjustedLow",
      "FordAngleAutoTuneAdjustmentsLow",
    )
    self._at_regime_high = AutoTuner(
      "FordAngleHighSpeedFactor",
      "FordAngleAutoTuneLastAdjustedHigh",
      "FordAngleAutoTuneAdjustmentsHigh",
    )
    self._params = None
    self.enabled = False
    self._at_last_flush_ts = 0.0
    self._at_flush_interval = 30.0
    self._at_last_factor_write_ts = 0.0
    self.actual_curvature = 0.0
    self.curvature_error = 0.0
    self.integral_low = 0.0
    self.integral_high = 0.0
    # Shared error smoothing and curve-entry detection (both factors see the same road).
    self._error_smooth = 0.0
    self._in_curve = False

  @property
  def low_factor(self) -> float:
    return self._at_regime_low.factor

  @property
  def high_factor(self) -> float:
    return self._at_regime_high.factor

  def configure(self, params, enabled: bool, param_low: float, param_high: float) -> None:
    was_enabled = self.enabled
    self._params = params
    self.enabled = enabled
    if not self._at_regime_low.factor_dirty:
      self._at_regime_low.factor = param_low
    if not self._at_regime_high.factor_dirty:
      self._at_regime_high.factor = param_high
    # Clear stale integral/curve state when re-enabling so we start fresh.
    # adj_count is NOT reset here — it persists across sessions via params.
    if enabled and not was_enabled:
      self.reset()
    # Reload lifetime adjustment counts from params on every configure call.
    # This ensures decay persists across sessions and survives disengages.
    for tuner in (self._at_regime_low, self._at_regime_high):
      try:
        cnt = params.get(tuner.count_name, return_default=True)
        tuner.adj_count = cnt if isinstance(cnt, int) else 0
      except Exception:
        tuner.adj_count = 0

  def reset(self) -> None:
    for tuner in (self._at_regime_low, self._at_regime_high):
      tuner.reset()
      tuner.adj_history = []
    self._error_smooth = 0.0
    self._in_curve = False
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

  def _flush_factors(self) -> None:
    if self._params is None:
      return
    now = time.monotonic()
    if now - self._at_last_factor_write_ts < self._AT_FACTOR_WRITE_INTERVAL:
      return
    for tuner in (self._at_regime_low, self._at_regime_high):
      if tuner.factor_dirty:
        try:
          self._params.put(tuner.factor_name, tuner.factor)
        except Exception:
          pass
        tuner.factor_dirty = False
    self._at_last_factor_write_ts = now

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
    is_curve = self._AT_KAPPA_MIN <= abs_kappa <= self._AT_KAPPA_MAX

    # Blended weights: how much each factor contributes to the gain at current speed.
    # Mirrors the high_gain_calc interp breakpoints in lateral_angle_ext.py so attribution
    # of the tracking error matches the actual gain structure at every speed.
    t = float(clip(interp(v_ego, [self._AT_V_LOW, self._AT_V_HIGH], [0.0, 1.0]), 0.0, 1.0))
    w_low  = 1.0 - t
    w_high = t

    # Detect rising edge of curve entry; both tuners see the same road.
    if is_curve and not self._in_curve:
      self._at_regime_low.curve_count  += 1
      self._at_regime_high.curve_count += 1
    self._in_curve = is_curve

    if not is_curve:
      return

    raw_error = kappa_cmd - actual_kappa
    self._error_smooth = (self._AT_ALPHA * raw_error
                          + (1 - self._AT_ALPHA) * self._error_smooth)
    clipped = float(clip(self._error_smooth, -self._AT_INT_CLAMP, self._AT_INT_CLAMP))

    # Each factor's integral accumulates only the fraction of error attributable to it.
    self._at_regime_low.integral  += clipped * w_low
    self._at_regime_high.integral += clipped * w_high

    for tuner, weight in ((self._at_regime_low, w_low), (self._at_regime_high, w_high)):
      if weight < self._AT_MIN_WEIGHT:
        continue
      if tuner.curve_count >= self._AT_MIN_CURVES:
        if tuner.integral > self._AT_INT_THRESH:
          ratio = min(tuner.integral / self._AT_INT_THRESH, self._AT_MAX_RATIO)
          self._adjust_factor(tuner, self._AT_STEP * ratio)
          tuner._post_adjust()
        elif tuner.integral < -self._AT_INT_THRESH:
          ratio = min(abs(tuner.integral) / self._AT_INT_THRESH, self._AT_MAX_RATIO)
          self._adjust_factor(tuner, -self._AT_STEP * ratio)
          tuner._post_adjust()

    self.actual_curvature = actual_kappa
    self.curvature_error = raw_error
    self.integral_low  = self._at_regime_low.integral
    self.integral_high = self._at_regime_high.integral

    now = time.monotonic()
    any_dirty = (self._at_regime_low.dirty or self._at_regime_high.dirty
                 or self._at_regime_low.factor_dirty or self._at_regime_high.factor_dirty)
    if any_dirty:
      if now - self._at_last_flush_ts >= self._at_flush_interval:
        self.flush()
      self._flush_factors()

  def _adjust_factor(self, tuner: AutoTuner, step: float) -> None:
    direction = 1 if step > 0 else -1

    # Oscillation: last adjustment was the opposite direction — we're near optimal.
    oscillating = len(tuner.adj_history) >= 2 and tuner.adj_history[-1] != direction

    # Pick coarse or fine base alpha, then decay by lifetime adjustment count.
    base_alpha = self._AT_BLEND_ALPHA_FINE if oscillating else self._AT_BLEND_ALPHA
    effective_alpha = max(self._AT_BLEND_ALPHA_MIN,
                          base_alpha / (1.0 + tuner.adj_count * self._AT_DECAY_RATE))

    new_val = float(clip(tuner.factor + effective_alpha * step, 0.5, 1.5))
    tuner.factor = new_val

    tuner.adj_history.append(direction)
    if len(tuner.adj_history) > self._AT_ADJ_HISTORY_LEN:
      tuner.adj_history.pop(0)
    tuner.adj_count += 1

"""Ford angle auto-tune: integrating controller that adjusts low/high speed curvature factors
based on the tracking error between commanded and actual curvature (yawRate / vEgo).

Convergence strategy:
  1. Curve-entry counting — require N distinct curve entries before each adjustment,
     so the integral reflects diverse real-world turns rather than one long curve.
  2. Blended attribution — at any speed, error is distributed between both factors
     proportionally to how much each contributes to the gain at that speed.  This
     mirrors the gain interpolation in lateral_angle_ext.py and lets both factors
     converge regardless of which speed range the driver favours.
  3. Convergence-oscillation detection — when recent *factor adjustments* alternate
     direction the factor is near optimal; switch to a finer alpha to zoom in rather
     than bounce.
  4. Limit-cycle detection — a too-high factor makes the vehicle overshoot the
     commanded curvature; the planner/model reacts to the resulting path error by
     commanding less curvature, which then undershoots, and the cycle repeats. That
     ringing has near-zero net area per cycle, so it can partially or fully cancel
     out of the smoothed/clamped integral above and go undetected. Zero-crossings
     and peak amplitude of the *raw*, unsmoothed error within a single curve entry
     catch this directly and force an immediate factor decrease, bypassing the
     curve-count/integral gating used for steady-state tuning.
"""
import time
from collections import deque

from numpy import clip, interp
from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams
from openpilot.common.swaglog import cloudlog

# Autotune update runs at STEER_STEP rate (20 Hz), not the 100 Hz carcontroller loop.
_DT_AT = DT_CTRL * CarControllerParams.STEER_STEP  # 0.05 s per sample
_MAX_HISTORY_LEN = 30  # ~1.5 s at 20 Hz — covers any valid lateral_delay
_DEFAULT_LATERAL_DELAY = 0.27


class AutoTuner:
  __slots__ = ("integral",
                "curve_count",
                "adj_history",
                "dirty", "factor_dirty",
                "factor",
                "factor_name", "ts_name")

  def __init__(self, factor_name: str, ts_name: str, factor: float = 1.0):
    self.integral = 0.0
    self.curve_count = 0
    self.adj_history = []   # recent adjustment directions: +1 or -1
    self.dirty = False
    self.factor_dirty = False
    self.factor = factor
    self.factor_name = factor_name
    self.ts_name = ts_name

  def reset(self):
    self.integral = 0.0
    self.curve_count = 0
    # adj_history persists across adjustments intentionally

  def _post_adjust(self):
    self.dirty = True
    self.factor_dirty = True
    self.reset()

  def flush(self, params) -> None:
    if not self.dirty:
      return
    try:
      params.put(self.ts_name, int(time.time()))
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
  # Limit-cycle detection on raw (unsmoothed) error within a single curve entry.
  # Sign flips smaller than the noise floor don't count as crossings; the peak-amplitude
  # gate then confirms the ringing is a real overshoot/undershoot and not sensor noise.
  _AT_OSC_MIN_CROSSINGS = 2
  _AT_OSC_NOISE_FLOOR = 0.0015   # 1/m
  _AT_OSC_MIN_AMP = 0.0025       # 1/m — peak |raw_error| required to trust the crossings
  # Debounce factor writes to params — max once per second.
  _AT_FACTOR_WRITE_INTERVAL = 1.0
  # Log debug state at ~2 Hz (every 10 frames at 20 Hz).
  _AT_LOG_INTERVAL_FRAMES = 10

  def __init__(self):
    self._at_regime_low = AutoTuner(
      "FordAngleLowSpeedFactor",
      "FordAngleAutoTuneLastAdjustedLow",
    )
    self._at_regime_high = AutoTuner(
      "FordAngleHighSpeedFactor",
      "FordAngleAutoTuneLastAdjustedHigh",
    )
    self._params = None
    self.enabled = False
    self._at_last_flush_ts = 0.0
    self._at_flush_interval = 30.0
    self._at_last_factor_write_ts = 0.0
    self._log_frame_counter = 0
    self.actual_curvature = 0.0
    self.curvature_error = 0.0
    self.integral_low = 0.0
    self.integral_high = 0.0
    # Shared error smoothing and curve-entry detection (both factors see the same road).
    self._error_smooth = 0.0
    self._in_curve = False
    self._kappa_history: deque = deque(maxlen=_MAX_HISTORY_LEN)
    # Limit-cycle detector state, reset on every curve entry (see update()).
    self._osc_prev_sign = 0
    self._osc_crossings = 0
    self._osc_peak_abs = 0.0
    self._osc_fired = False

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
    if enabled and not was_enabled:
      self.reset()

  def reset(self) -> None:
    for tuner in (self._at_regime_low, self._at_regime_high):
      tuner.reset()
      tuner.adj_history = []
    self._error_smooth = 0.0
    self._in_curve = False
    self._kappa_history.clear()
    self._osc_prev_sign = 0
    self._osc_crossings = 0
    self._osc_peak_abs = 0.0
    self._osc_fired = False
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

  def update(self, kappa_cmd: float, v_ego: float, CS, lateral_delay: float = _DEFAULT_LATERAL_DELAY) -> None:
    if not self.enabled:
      return

    # Push the current command into history before anything else so the buffer
    # is always populated even when we exit early below.
    self._kappa_history.append(kappa_cmd)

    # Look up the command that was issued `lateral_delay` seconds ago — that is
    # the request the vehicle is physically responding to right now.
    delay_samples = int(round(lateral_delay / _DT_AT))
    if len(self._kappa_history) <= delay_samples:
      return  # not enough history yet
    delayed_kappa_cmd = self._kappa_history[-(delay_samples + 1)]

    actual_kappa = 0.0
    try:
      actual_yaw = float(CS.out.yawRate)
      actual_v = float(CS.out.vEgoRaw)
      if actual_v <= self._AT_V_MIN:
        return
      actual_kappa = actual_yaw / actual_v
    except Exception:
      return

    abs_kappa = abs(delayed_kappa_cmd)
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
      self._osc_prev_sign = 0
      self._osc_crossings = 0
      self._osc_peak_abs = 0.0
      self._osc_fired = False
    self._in_curve = is_curve

    if not is_curve:
      return

    raw_error = delayed_kappa_cmd - actual_kappa

    # Limit-cycle detection on the raw, unsmoothed error (see module docstring, strategy 4).
    # Overshoot always means the factor commanded too much curvature, so unlike the integral
    # path below, the correction direction here is unconditionally "decrease" — a ringing
    # error alternates sign by definition, so its own sign can't tell us which way to go.
    abs_raw = abs(raw_error)
    if abs_raw >= self._AT_OSC_NOISE_FLOOR:
      sign = 1 if raw_error > 0 else -1
      if self._osc_prev_sign != 0 and sign != self._osc_prev_sign:
        self._osc_crossings += 1
      self._osc_prev_sign = sign
    self._osc_peak_abs = max(self._osc_peak_abs, abs_raw)

    if (not self._osc_fired and self._osc_crossings >= self._AT_OSC_MIN_CROSSINGS
        and self._osc_peak_abs >= self._AT_OSC_MIN_AMP):
      self._osc_fired = True
      for tuner, weight in ((self._at_regime_low, w_low), (self._at_regime_high, w_high)):
        if weight < self._AT_MIN_WEIGHT:
          continue
        self._adjust_factor(tuner, -self._AT_STEP)
        tuner._post_adjust()
      cloudlog.event("autotune_oscillation",
        v_ego=round(v_ego, 3),
        crossings=self._osc_crossings,
        peak_abs=round(self._osc_peak_abs, 6),
        w_low=round(w_low, 3),
        w_high=round(w_high, 3),
      )

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

    self._log_frame_counter += 1
    if self._log_frame_counter >= self._AT_LOG_INTERVAL_FRAMES:
      self._log_frame_counter = 0
      cloudlog.event("autotune_state", debug=True,
        v_ego=round(v_ego, 3),
        kappa_cmd=round(kappa_cmd, 6),
        delayed_kappa_cmd=round(delayed_kappa_cmd, 6),
        actual_kappa=round(actual_kappa, 6),
        raw_error=round(raw_error, 6),
        error_smooth=round(self._error_smooth, 6),
        w_low=round(w_low, 3),
        w_high=round(w_high, 3),
        integral_low=round(self._at_regime_low.integral, 6),
        integral_high=round(self._at_regime_high.integral, 6),
        curve_count_low=self._at_regime_low.curve_count,
        curve_count_high=self._at_regime_high.curve_count,
        low_factor=round(self._at_regime_low.factor, 4),
        high_factor=round(self._at_regime_high.factor, 4),
        lateral_delay=round(lateral_delay, 3),
      )

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

    alpha = self._AT_BLEND_ALPHA_FINE if oscillating else self._AT_BLEND_ALPHA
    old_val = tuner.factor
    new_val = float(clip(tuner.factor + alpha * step, 0.5, 1.5))
    tuner.factor = new_val

    cloudlog.event("autotune_adjust",
      factor=tuner.factor_name,
      old_factor=round(old_val, 4),
      new_factor=round(new_val, 4),
      step=round(step, 4),
      alpha=alpha,
      direction=direction,
      oscillating=oscillating,
      integral=round(tuner.integral, 6),
      curve_count=tuner.curve_count,
    )

    tuner.adj_history.append(direction)
    if len(tuner.adj_history) > self._AT_ADJ_HISTORY_LEN:
      tuner.adj_history.pop(0)

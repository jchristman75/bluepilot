"""
BluePilot: highway smoothing (BPHighwaySmoothing), a filter on the angle-mode curvature command.

On a straight highway the model and the car trade small corrections back and forth (the Oct 3 weave, the 4a1
straight-road oscillation): fast, low-amplitude changes in the commanded curvature that the steering then follows.
This pulls those small changes toward their own slow average and leaves everything else alone:

    out = baseline + k * (kappa - baseline),   k = 1 - weight * (1 - gain)

`baseline` is a ~1.5 s low-pass of the command, so a sustained change (a real curve, a deliberate correction) is
the baseline; only what is faster than that is attenuated, by up to (1 - gain). `weight` (0..1) says how much of
that to apply, and goes to 0, so the command passes through untouched, whenever it could be a real maneuver:

  * below SPEED_OFF the filter is off, full from SPEED_FULL up (the weave is a highway problem);
  * the command's distance from its own slow trend, as lateral acceleration, is held at its peak for
    ENVELOPE_HOLD_S: a large recent deviation (a corner, a lane change, an evasive correction) switches the
    filter off at once and keeps it off, so a weave cannot ride on a maneuver and a maneuver is never delayed;
  * if the model's own plan 1.5 s and 2.0 s ahead differs from the current trend by more than a weave would,
    a curve or a correction is coming, so the filter is off before it arrives;
  * above TIGHT_CURVE_ON of steady lateral acceleration (a tight curve) it is off;
  * the caller's `bypass` (blinker, hands on the wheel, lane change) switches it off and fades it back over
    BYPASS_FADE_S, so the output never steps.

It adds lag to small corrections by design, so it ships off by default and is re-checked against the measured
lateral delay (see the lateral-delay notes) before any default change. Written from the behavior of StarPilot's
highway correction gain, as a separate implementation with its own structure; its constants are starting points
validated on BluePilot drives with the PSCM emulator, not copied tuning.

Pure logic: no params, no messaging. Every time constant takes the caller's `dt`, so it is rate independent.
"""
import math

MPH = 0.44704
SPEED_OFF = 30.0 * MPH           # m/s: no smoothing at or below this
SPEED_FULL = 40.0 * MPH          # m/s: full strength at or above this

BASELINE_TAU_S = 1.5             # slow trend of the command
CURVE_TAU_S = 3.0                # slower trend, used to tell a held curve from a weave
CURVE_BLEND = (0.15, 0.35)       # m/s^2 of held lateral acceleration: how much of the trend counts as "in a curve"

DEVIATION_ON = 0.25              # m/s^2 of deviation from the trend where smoothing starts to back off...
DEVIATION_OFF = 0.60             # ...and where it is completely off (straight road)
DEVIATION_ON_CURVE = 0.12        # the same in a curve, where less deviation already means a correction
DEVIATION_OFF_CURVE = 0.30
PREDICTED_ON = 0.20              # m/s^2: plan-vs-trend difference that starts to back off (straight)
PREDICTED_OFF = 0.45
PREDICTED_ON_CURVE = 0.20
PREDICTED_OFF_CURVE = 0.45
TIGHT_CURVE_ON = 1.5             # m/s^2 of held lateral acceleration
TIGHT_CURVE_OFF = 2.0

ENVELOPE_HOLD_S = 2.0            # peak hold, longer than the weave's own peak spacing
ENVELOPE_RELEASE_PER_S = 0.3     # m/s^2 per second after the hold
BYPASS_FADE_S = 0.5

GAIN_MIN, GAIN_MAX = 0.1, 1.0
PREDICTION_HORIZONS_S = (1.5, 2.0)   # for the caller: seconds ahead on the model's plan


def _smoothstep(x: float, lo: float, hi: float) -> float:
  t = min(max((x - lo) / (hi - lo), 0.0), 1.0)
  return t * t * (3.0 - 2.0 * t)


class _PeakHold:
  """Running peak of a non-negative signal: holds the maximum for `hold_s`, then releases linearly."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.value = 0.0
    self._age = 0.0

  def update(self, x: float, dt: float) -> float:
    if x >= self.value:
      self.value, self._age = x, 0.0
    else:
      self._age += dt
      if self._age > ENVELOPE_HOLD_S:
        self.value = max(x, self.value - ENVELOPE_RELEASE_PER_S * dt)
    return self.value


class HighwaySmoothing:
  def __init__(self):
    self.reset()

  def reset(self, kappa: float = 0.0) -> None:
    self.baseline = kappa
    self.curve_level = kappa
    self.envelope = _PeakHold()
    self.prediction_envelope = _PeakHold()
    self.bypass_weight = 0.0
    self.weight = 0.0

  def update(self, kappa: float, v_ego: float, active: bool, gain: float, bypass: bool, dt: float,
             predicted: tuple[float, ...] | None = None) -> float:
    """Smoothed curvature for this tick.

    kappa: commanded curvature (1/m). gain: 0.1 (strongest) .. 1.0 (off). dt: seconds since the last call.
    predicted: the model's lateral acceleration (m/s^2) at PREDICTION_HORIZONS_S, or None when unavailable.
    """
    if not active:
      self.reset(kappa if math.isfinite(kappa) else 0.0)
      return kappa
    if not (math.isfinite(kappa) and math.isfinite(v_ego) and math.isfinite(gain) and dt > 0.0):
      return kappa   # leave the state alone and pass the command through

    gain = min(max(gain, GAIN_MIN), GAIN_MAX)
    # tracked even while bypassed or off, so fading in never steps the output
    self.baseline += (1.0 - math.exp(-dt / BASELINE_TAU_S)) * (kappa - self.baseline)
    self.curve_level += (1.0 - math.exp(-dt / CURVE_TAU_S)) * (kappa - self.curve_level)

    v2 = v_ego * v_ego
    held = abs(self.curve_level) * v2
    in_curve = _smoothstep(held, *CURVE_BLEND)
    trend = in_curve * self.curve_level
    envelope = self.envelope.update(abs(kappa - trend) * v2, dt)

    if predicted and all(math.isfinite(p) for p in predicted):
      gate = 1.0 - _smoothstep(envelope, DEVIATION_ON, DEVIATION_OFF)
      ahead = max(abs(p - trend * v2) for p in predicted)
      on = PREDICTED_ON + in_curve * (PREDICTED_ON_CURVE - PREDICTED_ON)
      off = PREDICTED_OFF + in_curve * (PREDICTED_OFF_CURVE - PREDICTED_OFF)
      gate *= 1.0 - _smoothstep(self.prediction_envelope.update(ahead, dt), on, off)
    else:
      self.prediction_envelope.reset()
      on = DEVIATION_ON + in_curve * (DEVIATION_ON_CURVE - DEVIATION_ON)
      off = DEVIATION_OFF + in_curve * (DEVIATION_OFF_CURVE - DEVIATION_OFF)
      gate = 1.0 - _smoothstep(envelope, on, off)
    gate *= 1.0 - _smoothstep(held, TIGHT_CURVE_ON, TIGHT_CURVE_OFF)

    step = dt / BYPASS_FADE_S
    target = 0.0 if bypass else 1.0
    self.bypass_weight += min(max(target - self.bypass_weight, -step), step)

    self.weight = self.bypass_weight * _smoothstep(v_ego, SPEED_OFF, SPEED_FULL) * gate
    k = 1.0 - self.weight * (1.0 - gain)
    if k >= 1.0:
      return kappa
    return self.baseline + k * (kappa - self.baseline)

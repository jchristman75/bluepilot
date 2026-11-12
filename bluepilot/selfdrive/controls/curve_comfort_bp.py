"""
BluePilot: learned curve comfort (choice A3 of the StarPilot ideas plan), a layer on the curve limiter.

Idea from StarPilot's curve speed controller: remember how much lateral acceleration the *driver* is happy
with at each curvature, from the curves driven with the driver in charge of speed, and slow for that. Two
differences, both because the Mach-E has a physical steering limit (curve_limit_bp.py):

  * The prior is non-restrictive (A_MAX), not StarPilot's 1.5-2.9 m/s^2 table, so an untrained table never
    trims a curve the car tracks fine; only your own gentler driving makes it restrictive.
  * The limiter uses min(car ceiling, learned level), so this layer can only make the car gentler than the
    steering allows, never push past what the steering delivers. It is clamped at A_COMFORT_MIN below.

24 log-spaced curvature buckets hold (average lateral acceleration, sample count). A bucket's level blends its
average with the prior by confidence n / (n + PRIOR_COUNT); levels are interpolated in log-curvature. Driver
overrides while the limiter is controlling nudge one bucket (brake: gentler, gas: bolder) by a fixed weight.
Persisted as JSON in the BPCurveComfortData param by the caller (this module has no I/O).
"""
import math

import numpy as np

from openpilot.bluepilot.selfdrive.controls import curve_limit_bp as cl

K_MIN, K_MAX = 0.0005, 0.02
N_BUCKETS = 24
CURVATURE_GRID = K_MIN * (K_MAX / K_MIN) ** (np.arange(N_BUCKETS) / (N_BUCKETS - 1))
_LOG_GRID = np.log(CURVATURE_GRID)

A_MAX = 3.2              # m/s^2: prior and upper clamp (above the car ceiling, so an untrained table restricts nothing)
A_MIN = cl.A_COMFORT_MIN
PRIOR_COUNT = 100        # samples (model frames) worth of prior
COUNT_CAP = 600          # a bucket adapts at most this slowly
NUDGE_WEIGHT = 20
SAMPLE_RANGE = (0.2, 6.0)


def bucket_index(k: float) -> int:
  k = float(np.clip(abs(k), K_MIN, K_MAX))
  return int(np.argmin(np.abs(_LOG_GRID - math.log(k))))


def bucket_key(k: float) -> str:
  return str(round(float(CURVATURE_GRID[bucket_index(k)]), 6))


class CurveComfort:
  def __init__(self, data: dict | None = None):
    self.data: dict[str, dict] = {}
    self.dirty = False
    self._levels = np.full(N_BUCKETS, A_MAX)
    for key, v in (data or {}).items():
      try:
        k, avg, n = float(key), float(v["average"]), int(v["count"])
      except (KeyError, TypeError, ValueError):
        continue
      if n <= 0 or not (math.isfinite(k) and math.isfinite(avg)) or not SAMPLE_RANGE[0] <= avg <= SAMPLE_RANGE[1]:
        continue
      b = bucket_key(k)
      if b in self.data:
        old = self.data[b]
        total = old["count"] + n
        self.data[b] = {"average": (old["average"] * old["count"] + avg * n) / total, "count": total}
      else:
        self.data[b] = {"average": avg, "count": n}
    self._rebuild()

  # ---- persistence: `data` is stored as the BPCurveComfortData JSON param by ComfortLearner ----
  def mark_saved(self):
    self.dirty = False

  # ---- learning ----
  def _rebuild(self):
    levels = np.full(N_BUCKETS, A_MAX)
    for i, k in enumerate(CURVATURE_GRID):
      d = self.data.get(bucket_key(k))
      if d:
        conf = d["count"] / (d["count"] + PRIOR_COUNT)
        levels[i] = conf * d["average"] + (1.0 - conf) * A_MAX
    self._levels = np.clip(levels, A_MIN, A_MAX)

  def _add(self, k: float, lat_acc: float, weight: int):
    key = bucket_key(k)
    d = self.data.get(key, {"average": lat_acc, "count": 0})
    eff = min(d["count"], COUNT_CAP)
    self.data[key] = {"average": (d["average"] * eff + lat_acc * weight) / (eff + weight), "count": d["count"] + weight}
    self.dirty = True
    self._rebuild()

  def observe(self, k: float, lat_acc: float):
    """One sample: the curve of |curvature| k was driven at lateral acceleration lat_acc with the driver in charge."""
    if not (math.isfinite(k) and math.isfinite(lat_acc)) or not SAMPLE_RANGE[0] <= lat_acc <= SAMPLE_RANGE[1]:
      return
    self._add(k, float(lat_acc), 1)

  def nudge(self, k: float, delta: float):
    """Driver override while the limiter was controlling: move the bucket's level by about delta."""
    if not (math.isfinite(k) and math.isfinite(delta)):
      return
    target = float(np.clip(float(self.lat_accel_for_curvature(np.array([abs(k)]))[0]) + delta, A_MIN, A_MAX))
    self._add(k, target, NUDGE_WEIGHT)

  # ---- use ----
  def lat_accel_for_curvature(self, k):
    """Learned comfortable lateral acceleration (m/s^2) for |curvature| k; usable as CurveLimiter(comfort=...)."""
    k = np.clip(np.abs(np.asarray(k, dtype=float)), K_MIN, K_MAX)
    return np.interp(np.log(k), _LOG_GRID, self._levels)


# ---- the driver-in-charge learner ----
SAVE_PERIOD_S = 15.0         # a dirty comfort table is written this often
LEARN_MIN_LAT_ACC = 0.8      # m/s^2: only real curves teach
LEARN_SETTLE_S = 2.0         # driver in charge this long before samples count
LEARN_QUIET_S = 5.0          # after the limiter controlled, wait (its curve is not the driver's choice)
RECENT_S = 3.0               # a driver override counts for the limiter this long after it last controlled
NUDGE = 0.15                 # m/s^2 per override episode


def load_comfort(params) -> CurveComfort:
  try:
    data = params.get("BPCurveComfortData")
  except Exception:
    return CurveComfort()
  return CurveComfort(data if isinstance(data, dict) else {})


class ComfortLearner:
  """Feeds a CurveComfort from the driving: samples while the driver is in charge of speed, and a nudge when
  the driver overrides right after the limiter controlled. Persists the table every SAVE_PERIOD_S while dirty."""

  def __init__(self, comfort: CurveComfort):
    self.comfort = comfort
    self.active_age = 1e9      # s since the limiter last controlled
    self.manual_t = 0.0        # s the driver has been in charge of speed
    self.prev_brake = False
    self.prev_override = False
    self.nudged = False
    self.save_t = 0.0

  def step(self, dt: float, limiter_active: bool, long_enabled: bool, long_override: bool, brake_pressed: bool,
           gas_pressed: bool, v_ego: float, blinker: bool, lead_near: bool, kappa: float, bind_curvature: float) -> None:
    """One model frame. kappa: the curvature being driven; bind_curvature: the limiter's binding curve."""
    self.active_age = 0.0 if limiter_active else self.active_age + dt
    manual = (not long_enabled) or long_override or brake_pressed or gas_pressed
    self.manual_t = self.manual_t + dt if manual else 0.0

    # a driver override right after the limiter controlled is feedback on its level, once per episode
    override = long_override or gas_pressed
    brake_edge = brake_pressed and not self.prev_brake
    override_edge = override and not self.prev_override
    self.prev_brake, self.prev_override = brake_pressed, override
    if self.active_age > RECENT_S + 2.0:
      self.nudged = False
    if not self.nudged and self.active_age <= RECENT_S and bind_curvature > 1e-4 and (brake_edge or override_edge):
      self.comfort.nudge(bind_curvature, -NUDGE if brake_edge else NUDGE)
      self.nudged = True

    lat_acc = v_ego ** 2 * kappa
    if (manual and self.manual_t >= LEARN_SETTLE_S and v_ego >= cl.V_ACTIVE_MIN and not blinker and not lead_near
        and self.active_age >= LEARN_QUIET_S and lat_acc >= LEARN_MIN_LAT_ACC):
      self.comfort.observe(kappa, lat_acc)

  def maybe_save(self, params, dt: float) -> None:
    self.save_t += dt
    if self.comfort.dirty and self.save_t >= SAVE_PERIOD_S:
      self.save_t = 0.0
      try:
        params.put("BPCurveComfortData", self.comfort.data)
        self.comfort.mark_saved()
      except Exception:
        pass   # an unknown key (stale params library) or a full disk must not take the planner down

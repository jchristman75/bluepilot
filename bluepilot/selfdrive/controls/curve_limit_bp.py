"""
BluePilot: curve speed limit from the car's steering ability (choice A2 of the StarPilot ideas plan).

Op long takes its speed through curves from the driving model alone. The Mach-E's steering (PSCM, angle mode)
follows what is asked up to about 2.3 m/s^2 of lateral acceleration and falls behind above that
(8 drives, 52k hands-free frames: actual/asked 0.96-1.00 below 2.3, 0.85 at 2.4-3.0), so the car runs wide
and the driver has to grab the wheel. This module slows the car *before* a curve whose planned lateral
acceleration would exceed what the steering can deliver.

Method, after StarPilot's curve speed controller, with the car's measured ceiling in place of a learned
comfort table:
  1. Curvature along the model's planned path (spatial curvature of the plan polyline). The plan geometry
     predicts the curvature the car then drives better than the yaw-rate plan does (median predicted/driven
     0.94 vs 0.86 at 15-35 m, 0.82 vs 0.62 at 35-60 m on CT v3), but still under-reads far firm curves, so a
     distance-dependent gain (1.2 at 35 m and beyond, firm curves only) corrects it.
  2. Speed at each point where the lateral acceleration reaches the ceiling: v_i = sqrt(a_max / k_i), never
     below a floor.
  3. A comfortable-decel envelope back from each point: allowed_i = sqrt(v_i^2 + 2 * a_env * s_i). The lowest
     one binds. The car is over the envelope when v_ego > allowed_i, and then the constant decel that reaches
     v_i at s_i is requested (capped, jerk-limited), so braking starts early and gently.
It only ever lowers the request, and only for curves the model has already planned.

Optional comfort layer (choice A3, curve_comfort_bp.py): a gentler learned limit for a curvature can lower
a_max further, never raise it above the car ceiling.

Pure numpy, no cereal or planner imports, so it is unit-testable and replayable on logged routes.
"""
from dataclasses import dataclass

import numpy as np

from openpilot.common.realtime import DT_MDL

DT = DT_MDL

# ---- steering ceiling: lateral acceleration the PSCM reliably delivers (m/s^2) by speed (m/s) ----
CEILING_BP = (0.0, 13.0, 25.0)
CEILING_V = (2.1, 2.1, 2.2)   # highway: 2.2 leaves the logged 65-73 mph sweepers (2.0-2.2) alone; tracking falls off above ~2.4

# ---- envelope ----
V_FLOOR = 20 * 0.44704       # m/s (20 mph): never ask for less than this
V_ACTIVE_MIN = 8.0           # m/s: below this the car is crawling, turns belong to the driver
DECEL_ENVELOPE = 0.45        # m/s^2: comfortable approach decel
DECEL_CAP = 1.0              # m/s^2: never request more than this
D_MIN = 8.0                  # m: a curve closer than this is treated as 8 m away (finite decel)
S_MIN = 5.0                  # m: plan points closer than this are noise
S_MAX = 100.0                # m: farthest plan point considered
K_EPS = 1e-4
K_MAX = 0.05
A_COMFORT_MIN = 1.2          # m/s^2: a learned comfort level can never push a_max below this

# ---- far-field under-read correction (CT v3: predicted/driven 0.82 at 35-60 m) ----
FAR_BP = (20.0, 35.0)
FAR_V = (1.0, 1.2)
FAR_GAIN_MAX = FAR_V[-1]
FAR_MIN_K = 0.004

# ---- smoothing ----
JERK_DOWN = 1.0              # m/s^3: how fast the brake request may build
JERK_UP = 3.0                # m/s^3: how fast it may release
HOLD_S = 0.5                 # s: keep the request (decaying through the jerk limit) after the curve clears
TARGET_DOWN_RATE = 4.0       # m/s^2: reported speed target
TARGET_UP_RATE = 3.0
TARGET_HEADROOM = 2.0        # m/s above ego the reported target is seeded at


def ceiling_lat_accel(v) -> float:
  """Lateral acceleration (m/s^2) the steering reliably delivers at speed v (m/s)."""
  return np.interp(v, CEILING_BP, CEILING_V)


def far_field_gain(s, k):
  """Gain on the plan's curvature at distance s (m): the model under-reads firm far curves."""
  s = np.asarray(s, dtype=float)
  k = np.asarray(k, dtype=float)
  return np.where(k >= FAR_MIN_K, np.interp(s, FAR_BP, FAR_V), 1.0)


def plan_curvature_profile(x, y) -> tuple[np.ndarray, np.ndarray]:
  """|Spatial curvature| (1/m) along the planned path: (arc length s of each interior point, curvature).

  Empty arrays when the plan is unusable (standstill, too short, non-finite)."""
  empty = (np.zeros(0), np.zeros(0))
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  if x.size < 5 or x.size != y.size or not (np.isfinite(x).all() and np.isfinite(y).all()):
    return empty
  dx, dy = np.diff(x), np.diff(y)
  ds = np.hypot(dx, dy)
  keep = ds > 1e-3
  if keep.sum() < 4:
    return empty
  dx, dy, ds = dx[keep], dy[keep], ds[keep]
  s = np.concatenate([[0.0], np.cumsum(ds)])
  if s[-1] < 15.0:
    return empty
  heading = np.unwrap(np.arctan2(dy, dx))
  s_mid = 0.5 * (s[1:] + s[:-1])
  k = np.diff(heading) / np.maximum(np.diff(s_mid), 1e-3)
  k = np.convolve(np.pad(k, 1, mode='edge'), np.ones(3) / 3.0, mode='valid')   # 3-point smooth, no zero-padding droop at the ends
  return s[1:-1], np.abs(k)


@dataclass(frozen=True)
class CurveTarget:
  active: bool           # a curve caps the speed (the envelope is below the set speed): hold the ceiling, don't accelerate into it
  braking: bool          # the car is over the envelope and the limiter asks for decel (a_cmd < 0)
  v_target: float        # speed ceiling (m/s), rate-limited; v_cruise when not active
  v_raw: float           # unfiltered envelope speed at the current position (m/s)
  a_req: float           # raw requested accel (m/s^2, <= 0, >= -DECEL_CAP); 0 when over no envelope
  a_cmd: float           # a_req shaped by the jerk limits: what the planner should apply
  bind_distance: float   # distance to the binding point (m), 0 when none
  bind_speed: float      # speed at the binding point (m/s)
  bind_curvature: float  # the plan's curvature at the binding point (1/m), before the far-field gain
  pred_lat_acc: float    # lateral acceleration the plan asks at the current speed (m/s^2), for logs and UI


def _inactive(v_cruise: float) -> CurveTarget:
  return CurveTarget(False, False, float(v_cruise), float(v_cruise), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class CurveLimiter:
  def __init__(self):
    self.reset()

  def reset(self):
    self.a_cmd = 0.0
    self.v_filt = None
    self.hold = 0.0

  def _shape_brake(self, a_req: float, dt: float) -> None:
    """a_cmd follows a_req: it builds at a limited jerk, and releases quickly after a short hold once the envelope clears."""
    if a_req < 0.0:
      self.hold = HOLD_S
      if a_req < self.a_cmd:
        self.a_cmd = max(a_req, self.a_cmd - JERK_DOWN * dt)
      else:
        self.a_cmd = min(a_req, self.a_cmd + JERK_UP * dt)
    else:
      self.hold = max(self.hold - dt, 0.0)
      self.a_cmd = min(self.a_cmd + (JERK_UP if self.hold <= 0.0 else JERK_DOWN) * dt, 0.0)
    self.a_cmd = min(self.a_cmd, 0.0)

  def _envelope(self, s, k, v_ego, comfort):
    k = np.clip(k, K_EPS, K_MAX)
    a_lim = ceiling_lat_accel(np.clip(np.sqrt(ceiling_lat_accel(v_ego) / k), 0.0, 60.0))
    if comfort is not None:
      learned = np.clip(np.asarray(comfort(k), dtype=float), A_COMFORT_MIN, None)
      a_lim = np.minimum(a_lim, learned)
    v_pt = np.maximum(np.sqrt(a_lim / k), V_FLOOR)
    allowed = np.sqrt(v_pt ** 2 + 2.0 * DECEL_ENVELOPE * s)
    need = (v_pt ** 2 - v_ego ** 2) / (2.0 * np.maximum(s, D_MIN))
    return v_pt, allowed, need

  def update(self, x, y, v_ego: float, v_cruise: float, enabled: bool = True, blinker: bool = False,
             comfort=None, s_shift: float = 0.0, dt: float = DT) -> CurveTarget:
    """One model frame. x, y: the plan polyline (car frame, m). comfort: optional callable k -> learned
    lateral acceleration. s_shift: meters to add to every plan distance (offline replays where the simulated
    car is behind the logged one)."""
    v_ego = float(max(v_ego, 0.0))
    if not enabled or blinker or v_ego < V_ACTIVE_MIN or not np.isfinite(v_ego):
      self.reset()
      return _inactive(v_cruise)
    s, k = plan_curvature_profile(x, y)
    s = s + s_shift
    sel = (s >= S_MIN) & (s <= S_MAX)
    if sel.sum() < 3:
      self.reset()
      return _inactive(v_cruise)
    s, k = s[sel], k[sel]
    k_eff = np.minimum(k * far_field_gain(s, k), K_MAX)
    v_pt, allowed, need = self._envelope(s, k_eff, v_ego, comfort)

    i = int(np.argmin(allowed))
    v_raw = float(min(allowed[i], v_cruise))
    over = v_ego > allowed + 1e-6
    a_req = float(max(need[over].min(), -DECEL_CAP)) if over.any() else 0.0
    a_req = min(a_req, 0.0)
    pred = float(v_ego ** 2 * k_eff.max())

    self._shape_brake(a_req, dt)
    braking = self.a_cmd < -1e-3

    # speed ceiling: on whenever a curve ahead binds below the set speed, so the planner can't accelerate back up
    # to cruise inside the curve (that +1 m/s^2 burst and re-brake showed up in the closed-loop test). Rate-limited,
    # seeded near the car so a step in the envelope doesn't jump it.
    binding = v_raw < v_cruise - 1e-6
    if not binding:
      self.v_filt = min(float(v_cruise), v_ego + TARGET_HEADROOM)
      return CurveTarget(False, braking, float(v_cruise), v_raw, a_req, float(self.a_cmd), 0.0, 0.0, 0.0, pred)
    if self.v_filt is None:
      self.v_filt = max(v_raw, min(float(v_cruise), v_ego + TARGET_HEADROOM))
    self.v_filt = float(np.clip(v_raw, self.v_filt - TARGET_DOWN_RATE * dt, self.v_filt + TARGET_UP_RATE * dt))
    v_target = float(min(self.v_filt, v_cruise))
    return CurveTarget(True, braking, v_target, v_raw, a_req, float(self.a_cmd), float(s[i]), float(v_pt[i]), float(k[i]), pred)

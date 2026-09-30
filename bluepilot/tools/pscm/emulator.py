"""
BluePilot: Ford Mach-E PSCM + vehicle emulator for LateralMotionControl2 path-angle (C1) commands.

Grey-box model: the stage order follows the PSCM Walkthrough (F-150 Lightning reference firmware),
every number is fitted to Mach-E rlogs (fit.py). The Lightning constants are NOT used -- on this car
the servo is fast and the delivery droops instead (README, "Emulator").

  wire C1 --[transport delay]--> held C1 (slew-limited) --> feedforward angle ff = F(v)*C1*N(v)
          --> minus droop state b (leaky: b -> c(v)*ff with time constant tau(v))
          --> + angle offset + bank compensation = internal angle target
          --> angle servo: rate = clip(P(v) * (target - angle), +-R(v))  -> carState.steeringAngleDeg
          --> linear single-track (bicycle) model (+ bank) --> yaw-rate sensor lag -> carState.yawRate

N(v) converts a yaw-rate request into the steering angle that produces it at speed v: the Mach-E
PSCM treats C1 roughly as a yaw-rate request. A yaw-feedback alternative to the droop (the PSCM
integrating yaw error toward a setpoint -- it would also explain the bank rejection) was fitted and
rejected: held-out angle R2 0.87 vs 0.96, post-release error 9.0 vs 2.0 deg.

Everything is in carState sign (steering angle left +, yaw rate left +). The wire C1 shares that
sign: carcontroller sends -lat.path_angle, and the controller's internal kappa is right-positive.

Every state is a numpy array, so one emulator steps a batch of independent cars at once (fitting)
or a batch of one (closed-loop harness).
"""
import json
import os
from dataclasses import asdict, dataclass, field

import numpy as np

DT = 0.01  # s, carState cadence; the PSCM itself ticks at 8 ms but we observe it at 100 Hz
DEFAULT_PARAMS = os.path.join(os.path.dirname(__file__), 'mache_pscm_params.json')

# The PSCM's own yaw-rate -> angle conversion N(v) = CK*(1+KUS*v^2)/v. Fixed at the car's measured
# steady state (Mach-E: 224k steady samples) so the PSCM tables stay independent of the vehicle fit;
# other platforms carry their own (PSCMParams.pscm_ck / pscm_kus).
_PSCM_CK = 2446.2
_PSCM_KUS = 0.000952

# Lightning reference-image angle loop (PSCM Walkthrough p.3): speed breakpoints, angle P (1/s) and
# requested-rate bound (deg/s). Used as the SHAPE of the servo schedule; fit.py scales both.
_SERVO_V_BP = np.array([0., 15., 30., 60., 100., 130.]) / 3.6
_SERVO_P = np.array([15., 15., 30., 30., 40., 40.])
_SERVO_RATE = np.array([720., 300., 150., 120., 100., 100.])


@dataclass
class PSCMParams:
  # speed schedule (m/s) shared by the tables below
  v_bp: list = field(default_factory=lambda: [5., 10., 15., 20., 25., 30., 35.])
  pscm_ck: float = _PSCM_CK        # nominal N(v): deg of steering per 1/m of curvature at v -> 0
  pscm_kus: float = _PSCM_KUS      # s^2/m^2
  ff_gain: list = field(default_factory=lambda: [1.0] * 7)       # immediate delivery vs nominal
  droop_ratio: list = field(default_factory=lambda: [0.2] * 7)   # share of ff the droop state removes
  droop_tau: list = field(default_factory=lambda: [0.8] * 7)     # s
  sat_knee: float = 99.0           # m/s^2: delivered lateral accel where the PSCM's ceiling bends in (99 = off)
  sat_slope: float = 1.0           # delivered per requested lateral accel above the knee
  small_gain: float = 0.0          # extra delivery for small commands: ff *= 1 + small_gain*exp(-|C1/v|/small_k0)
  small_k0: float = 0.0005         # 1/m, curvature scale of that small-command boost
  delay_s: float = 0.08            # wire -> PSCM internal target transport delay
  c1_slew: float = 0.3             # rad/s, held-C1 slew (Lightning normal build 0.1)
  servo_p_scale: float = 1.0       # x the Lightning angle-P schedule
  servo_rate_scale: float = 1.0    # x the Lightning requested-rate bound
  angle_offset_deg: float = 0.0    # PSCM target offset (carState sign)
  bank_comp: float = 0.0           # deg of target per rad of road roll the PSCM adds by itself
  release_tau: float = 0.5         # s, mode 0: angle relaxes toward the free-wheel angle (fit_mode0.py)
  # vehicle: linear single-track model. Mass, wheelbase and CG from the Mach-E CarSpecs. The steady
  # state is pinned to the measured ang = CK*(1+KUS v^2)*k (steer ratio and understeer gradient; a free
  # dynamic fit let high-speed understeer drift 14% off it), the dynamic terms are fitted.
  veh_mass: float = 2200.0         # kg
  veh_wheelbase: float = 2.984     # m
  veh_cg_front: float = 0.44       # centre-to-front / wheelbase
  veh_sr: float = 14.3             # steering-wheel deg per road-wheel deg
  veh_cf: float = 93000.0          # N/rad front axle cornering stiffness; rear follows from veh_us
  veh_us: float = _PSCM_KUS * 2.984  # understeer gradient, rad s^2/m (= KUS * wheelbase)
  veh_iz: float = 3500.0           # kg m^2
  veh_yaw_tau: float = 0.03        # s, yaw-rate sensor filter
  veh_offset_deg: float = -0.656   # steering angle at zero yaw on a flat road
  veh_roll: float = 19.44          # deg of steering per rad of roll to hold zero yaw

  def save(self, path=DEFAULT_PARAMS, meta=None):
    with open(path, 'w') as f:
      json.dump({'params': asdict(self), 'meta': meta or {}}, f, indent=1)

  @classmethod
  def load(cls, path=DEFAULT_PARAMS):
    with open(path) as f:
      return cls(**json.load(f)['params'])


class PSCMEmulator:
  """
  Step at DT with the latest wire frame (sample-and-hold, as the car sees a 20 Hz message).

    emu = PSCMEmulator(PSCMParams.load(), n=1)
    emu.reset(angle_deg, yaw_rate, v)
    out = emu.step(mode, c1, v, roll)   # -> dict(steeringAngleDeg, yawRate, ...)
  """
  def __init__(self, params: PSCMParams | None = None, n: int = 1):
    self.p = params or (PSCMParams.load() if os.path.exists(DEFAULT_PARAMS) else PSCMParams())
    self.n = n
    self._nd = max(0, int(round(self.p.delay_s / DT)))
    self.reset(np.zeros(n), np.zeros(n), np.full(n, 10.0))

  # --- helpers -----------------------------------------------------------------------------------
  def _tab(self, name, v):
    return np.interp(v, self.p.v_bp, getattr(self.p, name))

  def _nominal_deg_per_c1(self, v):
    # C1 behaves as a yaw-rate request: angle for yaw rate w at speed v is CK*(1+KUS v^2)*w/v
    return self.p.pscm_ck * (1 + self.p.pscm_kus * v**2) / np.maximum(v, 1.0)

  # --- state -------------------------------------------------------------------------------------
  def reset(self, angle_deg, yaw_rate, v, c1=None, roll=0.0):
    """Start from a measured state, as if the current command had been held long enough to settle."""
    n = self.n
    angle_deg = np.broadcast_to(np.asarray(angle_deg, float), (n,)).copy()
    yaw_rate = np.broadcast_to(np.asarray(yaw_rate, float), (n,)).copy()
    v = np.broadcast_to(np.asarray(v, float), (n,))
    c1 = np.zeros(n) if c1 is None else np.broadcast_to(np.asarray(c1, float), (n,)).copy()
    self.fifo = np.repeat(c1[None], self._nd + 1, axis=0)
    self.mfifo = np.ones((self._nd + 1, n), dtype=bool)  # mode > 0, delayed like C1
    self.c1h = c1.copy()
    ff = self._tab('ff_gain', v) * self._nominal_deg_per_c1(v) * c1
    self.b = self._tab('droop_ratio', v) * ff
    self.ang = angle_deg
    # vehicle: lateral velocity and true yaw rate at their steady-state ratio, sensor at the measurement
    p = self.p
    a, b, cr = self._axles()
    u = np.maximum(v, 3.0)
    self.r = yaw_rate.copy()
    self.vy = yaw_rate * (b - p.veh_mass * a * u**2 / (cr * p.veh_wheelbase))
    self.yaw = yaw_rate.copy()

  _SAT_WIDTH = 0.2  # m/s^2, softness of the knee

  def _saturate(self, cmd_deg, v):
    """Soft lateral-acceleration ceiling on the command-driven part of the target: below the knee
    unchanged, above it the delivered lateral accel grows at sat_slope. Fitted on the Mach-E, where
    the PSCM's LatCtlLim flag comes on as delivered accel passes ~2 m/s^2 (routes 1b0/1b1)."""
    p = self.p
    if p.sat_knee >= 50:
      return cmd_deg
    x = np.abs(v * cmd_deg / self._nominal_deg_per_c1(v))  # steady lateral accel this target delivers
    w = self._SAT_WIDTH
    y = x - (1 - p.sat_slope) * w * np.logaddexp(0.0, (x - p.sat_knee) / w)
    return cmd_deg * np.where(x > 1e-6, np.maximum(y, 0.0) / np.maximum(x, 1e-6), 1.0)

  def sync_vehicle(self, yaw_rate, v):
    """Put the car (not the PSCM) on a measured yaw rate, e.g. while a replayed driver is steering."""
    p = self.p
    a, b, cr = self._axles()
    u = np.maximum(np.broadcast_to(np.asarray(v, float), (self.n,)), 3.0)
    self.r = np.broadcast_to(np.asarray(yaw_rate, float), (self.n,)).copy()
    self.vy = self.r * (b - p.veh_mass * a * u**2 / (cr * p.veh_wheelbase))
    self.yaw = self.r.copy()

  def step(self, mode, c1, v, roll=0.0, driver_angle=None, bias_deg=0.0, ang_dist=0.0, yaw_dist=0.0, c2=0.0):
    """
    mode: LatCtl_D2_Rq (0 = off). c1: wire LatCtlPath_An_Actl, rad. v: m/s. roll: road roll, rad.
    c2: wire LatCtlCurv_No_Actl, 1/m (curvature mode). It enters as the equivalent yaw-rate request
      v*C2, so a params file fitted on curvature-mode data (fleet.py) has its tables in the same
      units as the C1 fit and the two can be compared directly. Mixing both on one params file is
      not validated.
    driver_angle: if given (hands-on replay), the wheel is forced there (NaN = not forced) and the
      PSCM's states follow it, so hands-free emulation resumes from the logged state on release.
    bias_deg: extra steering target (deg), for injecting the post-release PSCM bias the model
      itself doesn't produce (harness disturbance scenarios).
    ang_dist / yaw_dist: replayed disturbances (harness): added to the wheel angle the car sees and
      reports, and to the yaw-rate sensor. They carry what a recorded drive did that the model
      doesn't explain (road, wind, PSCM quirks), so replaying the recorded commands reproduces it.
    """
    p = self.p
    mode = np.broadcast_to(np.asarray(mode, float), (self.n,))
    c1 = np.broadcast_to(np.asarray(c1, float), (self.n,))
    v = np.broadcast_to(np.asarray(v, float), (self.n,))
    roll = np.broadcast_to(np.asarray(roll, float), (self.n,))

    # transport delay (mode and C1 alike), then held copy slews toward the delayed wire value
    self.fifo = np.roll(self.fifo, -1, axis=0)
    self.mfifo = np.roll(self.mfifo, -1, axis=0)
    c2 = np.broadcast_to(np.asarray(c2, float), (self.n,))
    self.fifo[-1] = np.where(mode > 0, c1 + v * c2, 0.0)
    self.mfifo[-1] = mode > 0
    c1d = self.fifo[0]
    active = self.mfifo[0]
    step = p.c1_slew * DT
    self.c1h = np.where(active, self.c1h + np.clip(c1d - self.c1h, -step, step), 0.0)

    # feedforward (with the small-command boost) minus the leaky droop state
    boost = 1.0 + p.small_gain * np.exp(-np.abs(self.c1h) / (np.maximum(v, 1.0) * p.small_k0))
    ff = self._tab('ff_gain', v) * self._nominal_deg_per_c1(v) * self.c1h * boost
    a_b = 1 - np.exp(-DT / np.maximum(self._tab('droop_tau', v), DT))
    self.b = np.where(active, self.b + (self._tab('droop_ratio', v) * ff - self.b) * a_b, 0.0)
    target = self._saturate(ff - self.b, v) + p.angle_offset_deg + p.bank_comp * roll + bias_deg

    # angle servo (Lightning schedule shape, scaled)
    kp = np.minimum(p.servo_p_scale * np.interp(v, _SERVO_V_BP, _SERVO_P), 0.9 / DT)
    rmax = p.servo_rate_scale * np.interp(v, _SERVO_V_BP, _SERVO_RATE)
    rate_req = np.clip(kp * (target - self.ang), -rmax, rmax)
    # mode 0: the wheel relaxes toward the angle that holds the car straight (self-aligning)
    free = p.veh_offset_deg + p.veh_roll * roll
    rate_req = np.where(active, rate_req, (free - self.ang) / max(p.release_tau, DT))
    self.ang = self.ang + rate_req * DT
    if driver_angle is not None:
      da = np.broadcast_to(np.asarray(driver_angle, float), (self.n,))
      forced = np.isfinite(da)
      self.ang = np.where(forced, da, self.ang)
      self.b = np.where(forced, self._tab('droop_ratio', v) * ff, self.b)

    ang_out = self.ang + ang_dist
    yaw = self.vehicle_step(ang_out, v, roll) + yaw_dist
    return {'steeringAngleDeg': ang_out.copy(), 'yawRate': yaw.copy(), 'heldC1': self.c1h.copy(),
            'target': target, 'droop': self.b.copy()}

  def _axles(self):
    p = self.p
    a = p.veh_cg_front * p.veh_wheelbase
    b = p.veh_wheelbase - a
    # understeer gradient K = m/L * (b/Cf - a/Cr)  ->  Cr from Cf and K
    cr = a / max(b / p.veh_cf - p.veh_us * p.veh_wheelbase / p.veh_mass, 1e-7)
    return a, b, cr

  def vehicle_step(self, angle_deg, v, roll):
    """Advance the vehicle one DT from a steering angle; also usable on its own with a measured angle."""
    p = self.p
    a, b, cr = self._axles()
    m, cf, iz = p.veh_mass, p.veh_cf, p.veh_iz
    u = np.maximum(v, 3.0)
    delta = np.radians(angle_deg - p.veh_offset_deg - p.veh_roll * roll) / p.veh_sr
    for _ in range(2):  # two half steps keep explicit integration well inside stability at low speed
      h = DT / 2
      vy_d = -(cf + cr) / (m * u) * self.vy + (-(cf * a - cr * b) / (m * u) - u) * self.r + cf / m * delta
      r_d = -(cf * a - cr * b) / (iz * u) * self.vy - (cf * a**2 + cr * b**2) / (iz * u) * self.r + cf * a / iz * delta
      self.vy = self.vy + vy_d * h
      self.r = self.r + r_d * h
    self.yaw = self.yaw + (self.r - self.yaw) * (1 - np.exp(-DT / max(p.veh_yaw_tau, 1e-3)))
    return self.yaw

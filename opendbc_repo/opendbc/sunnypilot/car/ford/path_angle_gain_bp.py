"""
BluePilot: the Mach-E steering module's own path-angle gain curve (replaces the hand-tuned angle-mode gain).

The Ford PSCM turns the path angle it is sent (c1, LatCtlPath_An_Actl, rad) into curvature with a gain K2 that
it reads from its data file at a preview distance d:

    curvature = c2 + K2(d) * c1          (c2 = 0 in angle mode)      =>      c1 = kappa / K2(d)
    d         = clamp(0.28 * v_kph * T, d_low(v), d_high(v))         T = preview time (s)

so the steady-state wire gain angle mode has been tuning by hand (path_angle = kappa * v * K) is K = 1 / (K2(d) * v).
The numbers below are the module's own, read from its flash by the ford-op/bluepilot-clean library
(core/module_tables.py PREVIEW_MACHE_2021, panda gain curve 6, evidence 'executed': start-up, path law, preview,
slews, state machine, output stage and limiter run in an emulator) for the 2021-22 Mustang Mach-E:
  strategy LJ9C-14D003-GA with data files LJ9C-14D007-ANA / AMA / ARA, and LJ9C-14D003-HA with LJ9C-14D007-KNB.
The 2023-24 Mach-E (LJ9C-14D003-FA) has no preview table in the library, so it keeps the hand-tuned gain.

T: the library's controller copy of T follows the curvature carried in c2 (t_per_curvature = 250 s per 1/m, floor
3.5 s, starts at 5 s, recovers at 0.2 s/s). Angle mode sends c2 = 0, so T stays at t_max and the gain is the T = t_max
curve. T is still a parameter here so the c2 case stays expressible.

Not in this model: the PSCM's measured delivery droop in time (0.91 of the wire at 0.5 s, 0.75-0.8 after ~1.5 s,
bluepilot/tools/pscm). That is dynamics, not the table, and stays with the lead-lag shaping in lateral_angle_ext.
"""
from dataclasses import dataclass

from numpy import interp

# F188 strategies this table was read from (data files: see docstring); anything else keeps the hand-tuned gain.
# This build's firmware query only logs F188 (brand label "mazda" in carFw), not F10A, so the data file is not checked.
MACHE_2021_STRATEGIES = ("LJ9C-14D003-GA", "LJ9C-14D003-HA")
PSCM_ADDR = 0x730
DID_STRATEGY = 0xF188


@dataclass(frozen=True)
class PathAngleGainCurve:
  t_max: float                       # s: preview time on a straight (and in angle mode, c2 = 0)
  t_min: float                       # s
  t_per_curvature: float             # s per 1/m
  speeds_kph: tuple[float, ...]
  d_low: tuple[float, ...]           # m
  d_high: tuple[float, ...]          # m
  k2_distances: tuple[float, ...]    # m
  k2: tuple[float, ...]              # 1/m per rad

  def preview_time(self, curvature: float = 0.0) -> float:
    """T for a curvature (1/m) carried in c2 held steady."""
    return min(max(self.t_max - self.t_per_curvature * abs(curvature), self.t_min), self.t_max)

  def preview_distance(self, v_ego: float, t: float) -> float:
    v_kph = v_ego * 3.6
    d = 0.28 * v_kph * t
    d_high = float(interp(v_kph, self.speeds_kph, self.d_high))
    # the module's own order: past the upper table it is the upper table, else at least the lower one
    return d_high if d > d_high else max(d, float(interp(v_kph, self.speeds_kph, self.d_low)))

  def k2_at(self, v_ego: float, t: float | None = None) -> float:
    """K2, 1/m per rad."""
    return float(interp(self.preview_distance(v_ego, self.t_max if t is None else t), self.k2_distances, self.k2))

  def path_angle(self, kappa: float, v_ego: float, t: float | None = None) -> float:
    """The path angle (rad) that makes the module steer for kappa (1/m)."""
    return kappa / self.k2_at(v_ego, t)

  def curvature_factor(self, v_ego: float, t: float | None = None) -> float:
    """Steady-state K in path_angle = kappa * v * K (v floored at 1 m/s, like the controller's own speed floor)."""
    return 1.0 / (self.k2_at(v_ego, t) * max(v_ego, 1.0))


# PREVIEW_MACHE_2021 (ford-op/bluepilot-clean core/module_tables.py, generated from the module's flash)
MACHE_2021 = PathAngleGainCurve(
  t_max=5.0,
  t_min=3.5,
  t_per_curvature=250.0,
  speeds_kph=(0.0, 15.0, 30.0, 60.0, 100.0, 130.0, 160.0, 220.0),
  d_low=(30.0, 30.0, 30.0, 30.0, 60.0, 60.0, 60.0, 60.0),
  d_high=(150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0),
  k2_distances=(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 100.0, 150.0),
  k2=(0.4, 0.2, 0.1333, 0.1, 0.08, 0.0667, 0.0571, 0.05, 0.04, 0.0266),
)


def pscm_strategy(car_fw) -> str:
  """The steering module's F188 answer (CarParams.carFw), '' when it gave none."""
  for fw in car_fw:
    request = bytes(fw.request[-1]) if len(fw.request) else b""
    if fw.address == PSCM_ADDR and len(request) == 3 and request[0] == 0x22 \
       and int.from_bytes(request[1:], "big") == DID_STRATEGY:
      return bytes(fw.fwVersion).rstrip(b"\x00").decode(errors="replace")
  return ""


def gain_curve_for(CP) -> PathAngleGainCurve | None:
  """The module's own curve for this car, or None (keep the hand-tuned gain)."""
  if getattr(CP, "carFingerprint", "") != "FORD_MUSTANG_MACH_E_MK1":
    return None
  return MACHE_2021 if pscm_strategy(getattr(CP, "carFw", ())) in MACHE_2021_STRATEGIES else None

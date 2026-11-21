"""Braking indicator on the steering wheel icon: orange = regen (including blended braking that is mostly regen), red = friction."""
from enum import IntEnum

REGEN_MIN_DECEL = 0.2  # m/s^2; below this the regen request is just coasting noise
MASS_KG = 2300.0       # Mach-E, same as the regen estimate in longitudinal_ext.py
SHARE_MIN_DECEL = 0.5  # m/s^2 and
SHARE_MIN_SPEED = 3.0  # m/s: slower or gentler than this the pack/braking power ratio is not meaningful
REGEN_SHARE_ORANGE = 0.6  # the brake path / pedal counts as regen when this much of the braking power lands in the pack

COLORS = {  # (r, g, b)
  "none": (255, 255, 255),
  "brake": (255, 60, 60),
  "regen": (255, 150, 30),
}


class BrakeKind(IntEnum):
  NONE = 0
  REGEN = 1
  BRAKE = 2


def regen_share(pack_kw: float, decel: float, v_ego: float) -> float | None:
  """Fraction of the braking power (mass x decel x speed) that goes into the pack (pack_kw > 0 = charging), or None when
  the car is too slow or the braking too gentle to tell. The Mach-E blends regen first when the brake path (the friction
  flag or the pedal) is used: 80-90% of the power lands in the pack at 20-45 mph (routes 59e/5a0/5ab)."""
  if decel < SHARE_MIN_DECEL or v_ego < SHARE_MIN_SPEED:
    return None
  return max(0.0, pack_kw * 1000.0 / (MASS_KG * decel * v_ego))


def brake_kind(lights_on: bool, friction: bool, regen_decel: float, pedal: bool = False, decel: float = 0.0,
               share: float | None = None) -> BrakeKind:
  """pedal = the driver is on the brake pedal; decel = measured deceleration (m/s^2, positive); share = regen_share().

  Lifting off the accelerator slows the car with regen only (the Mach-E turns the brake lights on for it), so with
  no pedal and no openpilot friction request, a real deceleration counts as regen. The pedal is friction (it is not
  known how much of it is regen unless the pack power says so)."""
  if friction or pedal:
    return BrakeKind.REGEN if share is not None and share >= REGEN_SHARE_ORANGE else BrakeKind.BRAKE
  if regen_decel >= REGEN_MIN_DECEL or (lights_on and decel >= REGEN_MIN_DECEL):
    return BrakeKind.REGEN
  return BrakeKind.BRAKE if lights_on else BrakeKind.NONE


def brake_color(kind: BrakeKind) -> tuple[int, int, int]:
  return COLORS[("none", "regen", "brake")[int(kind)]]


def brake_kind_from_car_state_bp(car_state_bp, car_state=None) -> BrakeKind:
  """Reads carStateBP (and carState for the pedal and measured decel); anything missing counts as not braking."""
  try:
    pedal = bool(car_state.brakePressed) if car_state is not None else False
    decel = -float(car_state.aEgo) if car_state is not None else 0.0
    bls = car_state_bp.brakeLightStatus
    lights = bool(bls.dataAvailable and bls.brakeLightsOn)
    lng = car_state_bp.longitudinal
    share = None
    hb = getattr(car_state_bp, 'hybridBattery', None)
    if car_state is not None and hb is not None and hb.dataAvailable:
      share = regen_share(float(hb.voltActual) * float(hb.ampsActual) / 1000.0, decel, float(getattr(car_state, 'vEgo', 0.0)))
    return brake_kind(lights, bool(lng.frictionBrake), float(lng.regenDecel), pedal, decel, share)
  except (KeyError, AttributeError):
    return BrakeKind.NONE

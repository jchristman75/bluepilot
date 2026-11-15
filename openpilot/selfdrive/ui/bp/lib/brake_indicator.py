"""Braking indicator on the steering wheel icon: red = friction brakes (or the driver's pedal), orange = regen only."""
from enum import IntEnum

REGEN_MIN_DECEL = 0.2  # m/s^2; below this the regen request is just coasting noise

COLORS = {  # (r, g, b)
  "none": (255, 255, 255),
  "brake": (255, 60, 60),
  "regen": (255, 150, 30),
}


class BrakeKind(IntEnum):
  NONE = 0
  REGEN = 1
  BRAKE = 2


def brake_kind(lights_on: bool, friction: bool, regen_decel: float) -> BrakeKind:
  if friction:
    return BrakeKind.BRAKE
  if regen_decel >= REGEN_MIN_DECEL:
    return BrakeKind.REGEN
  return BrakeKind.BRAKE if lights_on else BrakeKind.NONE


def brake_color(kind: BrakeKind) -> tuple[int, int, int]:
  return COLORS[("none", "regen", "brake")[int(kind)]]


def brake_kind_from_car_state_bp(car_state_bp) -> BrakeKind:
  """Reads carStateBP; anything missing counts as not braking."""
  try:
    bls = car_state_bp.brakeLightStatus
    lights = bool(bls.dataAvailable and bls.brakeLightsOn)
    lng = car_state_bp.longitudinal
    return brake_kind(lights, bool(lng.frictionBrake), float(lng.regenDecel))
  except (KeyError, AttributeError):
    return BrakeKind.NONE

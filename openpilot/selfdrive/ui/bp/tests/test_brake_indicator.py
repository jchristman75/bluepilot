from types import SimpleNamespace as NS

from openpilot.selfdrive.ui.bp.lib.brake_indicator import BrakeKind, brake_color, brake_kind, brake_kind_from_car_state_bp


def test_friction_is_red_even_with_regen():
  assert brake_kind(True, True, 1.0) == BrakeKind.BRAKE


def test_regen_only_is_orange_even_with_the_lights_on():
  assert brake_kind(True, False, 0.8) == BrakeKind.REGEN
  assert brake_color(BrakeKind.REGEN) != brake_color(BrakeKind.BRAKE)


def test_driver_pedal_lights_without_a_request_is_red():
  assert brake_kind(True, False, 0.0) == BrakeKind.BRAKE


def test_tiny_regen_is_not_braking():
  assert brake_kind(False, False, 0.1) == BrakeKind.NONE


def test_from_message_and_missing_fields():
  m = NS(brakeLightStatus=NS(dataAvailable=True, brakeLightsOn=True), longitudinal=NS(frictionBrake=False, regenDecel=0.5))
  assert brake_kind_from_car_state_bp(m) == BrakeKind.REGEN
  assert brake_kind_from_car_state_bp(NS()) == BrakeKind.NONE

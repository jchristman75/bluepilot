from types import SimpleNamespace as NS

import pytest

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


def test_lift_off_regen_is_orange_until_the_pedal_is_pressed():
  assert brake_kind(True, False, 0.0, pedal=False, decel=1.5) == BrakeKind.REGEN      # lifted off: regen only
  assert brake_kind(True, False, 0.0, pedal=True, decel=1.5) == BrakeKind.BRAKE       # pedal: red
  assert brake_kind(True, True, 0.0, pedal=False, decel=1.5) == BrakeKind.BRAKE       # op friction: red
  assert brake_kind(False, False, 0.0, pedal=False, decel=1.5) == BrakeKind.NONE      # no lights (e.g. a hill): nothing
  assert brake_kind(True, False, 0.0, pedal=False, decel=0.1) == BrakeKind.BRAKE      # lights without a real decel


def test_from_message_uses_carstate_for_the_pedal_and_decel():
  m = NS(brakeLightStatus=NS(dataAvailable=True, brakeLightsOn=True), longitudinal=NS(frictionBrake=False, regenDecel=0.0))
  assert brake_kind_from_car_state_bp(m, NS(brakePressed=False, aEgo=-1.6)) == BrakeKind.REGEN
  assert brake_kind_from_car_state_bp(m, NS(brakePressed=True, aEgo=-1.6)) == BrakeKind.BRAKE


def test_regen_share_is_pack_power_over_braking_power():
  from openpilot.selfdrive.ui.bp.lib.brake_indicator import regen_share
  # 20 m/s at 2.0 m/s^2 with a 2300 kg car = 92 kW of braking; 80 kW into the pack
  assert regen_share(80.0, 2.0, 20.0) == pytest.approx(80.0 / 92.0)
  assert regen_share(80.0, 0.3, 20.0) is None            # too gentle to judge
  assert regen_share(80.0, 2.0, 2.0) is None             # too slow to judge
  assert regen_share(-5.0, 2.0, 20.0) == 0.0              # discharging: no regen


def test_blended_braking_with_most_of_the_power_in_the_pack_is_orange():
  # the brake path / pedal: the car blends regen first, pads take the remainder
  assert brake_kind(True, True, 0.0, decel=2.0, share=0.85) == BrakeKind.REGEN
  assert brake_kind(True, False, 0.0, pedal=True, decel=2.0, share=0.85) == BrakeKind.REGEN
  assert brake_kind(True, True, 0.0, decel=2.0, share=0.40) == BrakeKind.BRAKE       # the pads are doing a lot
  assert brake_kind(True, True, 0.0, decel=2.0, share=None) == BrakeKind.BRAKE       # unknown: red as before
  assert brake_kind(True, False, 0.0, pedal=True, decel=2.0, share=0.59) == BrakeKind.BRAKE


def test_from_message_computes_the_share_from_the_pack_and_the_motion():
  m = NS(brakeLightStatus=NS(dataAvailable=True, brakeLightsOn=True), longitudinal=NS(frictionBrake=True, regenDecel=0.0),
         hybridBattery=NS(dataAvailable=True, voltActual=380.0, ampsActual=210.0))      # 79.8 kW charging
  cs = NS(brakePressed=False, aEgo=-2.0, vEgo=20.0)
  assert brake_kind_from_car_state_bp(m, cs) == BrakeKind.REGEN
  m.hybridBattery.ampsActual = 60.0                                                       # 22.8 kW of 92 kW
  assert brake_kind_from_car_state_bp(m, cs) == BrakeKind.BRAKE

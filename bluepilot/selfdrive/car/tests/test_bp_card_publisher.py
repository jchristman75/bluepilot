"""carStateBP.longitudinal: whether openpilot is braking with the friction brakes or decelerating with regen."""
from types import SimpleNamespace

import pytest

from openpilot.bluepilot.selfdrive.car import bp_card_publisher as pub

INACTIVE_GAS = -5.0


@pytest.mark.parametrize("brake,gas,friction,regen", [
  (True, INACTIVE_GAS, True, 0.0),       # the brake flag is on: propulsion is inactive (never gas and brake together)
  (False, -0.6, False, 0.6),             # a regen request: gas is the negative propulsion request
  (False, -1.5, False, 1.5),
  (False, 0.4, False, 0.0),              # accelerating
  (False, INACTIVE_GAS, False, 0.0),     # long off / propulsion inactive without the brake flag: not braking
  (False, 0.0, False, 0.0),
])
def test_friction_and_regen_from_what_was_sent_to_the_car(brake, gas, friction, regen):
  assert pub.longitudinal_indicator(brake, gas) == (friction, pytest.approx(regen))


def test_the_publisher_fills_the_carstatebp_message_from_the_controller():
  from openpilot.cereal import messaging
  msg = messaging.new_message("carStateBP")
  CI = SimpleNamespace(CS=SimpleNamespace(car_state_bp_msg=msg), CC=SimpleNamespace(bp_brake_actuate=False, gas=-0.9))
  sent = []
  pm = SimpleNamespace(send=lambda name, m: sent.append((name, m)))
  pub.publish_car_state_bp(CI, pm, True)
  lng = sent[0][1].carStateBP.longitudinal
  assert not lng.frictionBrake and lng.regenDecel == pytest.approx(0.9)
  CI.CC.bp_brake_actuate, CI.CC.gas = True, INACTIVE_GAS
  pub.publish_car_state_bp(CI, pm, True)
  lng = sent[1][1].carStateBP.longitudinal
  assert lng.frictionBrake and lng.regenDecel == 0.0


def test_a_controller_without_the_attributes_publishes_not_braking():
  from openpilot.cereal import messaging
  msg = messaging.new_message("carStateBP")
  CI = SimpleNamespace(CS=SimpleNamespace(car_state_bp_msg=msg), CC=SimpleNamespace())
  sent = []
  pub.publish_car_state_bp(CI, SimpleNamespace(send=lambda n, m: sent.append(m)), True)
  lng = sent[0].carStateBP.longitudinal
  assert not lng.frictionBrake and lng.regenDecel == 0.0

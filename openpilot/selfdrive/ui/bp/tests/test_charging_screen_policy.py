"""The charging screen keeps the display on only while the car is on; with the car off the screen times out (burn-in)."""
from openpilot.selfdrive.ui.bp.charging import screen_policy as sp


def test_the_screen_is_held_awake_while_the_car_is_on():
  assert sp.screen_timeout_s(ignition=True) == sp.STAY_AWAKE_TIMEOUT_S


def test_with_the_car_off_the_screen_gets_a_short_timeout_not_the_hold():
  assert sp.screen_timeout_s(ignition=False) == sp.CAR_OFF_TIMEOUT_S
  assert 0 < sp.CAR_OFF_TIMEOUT_S <= 10                       # near-immediate, but a touch still shows it briefly
  assert sp.CAR_OFF_TIMEOUT_S < sp.STAY_AWAKE_TIMEOUT_S


def test_the_hold_reports_a_timeout_only_when_it_changes():
  hold = sp.ChargingScreenHold()
  assert hold.update(ignition=True) == sp.STAY_AWAKE_TIMEOUT_S     # first frame applies it
  assert hold.update(ignition=True) is None
  assert hold.update(ignition=True) is None
  assert hold.update(ignition=False) == sp.CAR_OFF_TIMEOUT_S       # the car turned off: start the short timeout once
  assert hold.update(ignition=False) is None                       # not re-applied every frame (that would keep resetting the timer)
  assert hold.update(ignition=True) == sp.STAY_AWAKE_TIMEOUT_S     # car back on: hold again


def test_opening_the_screen_with_the_car_already_off_applies_the_short_timeout():
  hold = sp.ChargingScreenHold()
  assert hold.update(ignition=False) == sp.CAR_OFF_TIMEOUT_S


def test_a_new_show_starts_from_scratch():
  hold = sp.ChargingScreenHold()
  hold.update(ignition=False)
  hold.reset()
  assert hold.update(ignition=False) == sp.CAR_OFF_TIMEOUT_S

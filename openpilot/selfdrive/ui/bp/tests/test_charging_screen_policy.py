"""The charging screen keeps the display on only while the car is on; with the car off the display really turns off
(short timeout, and no screensaver in its place) so it cannot burn in."""
from openpilot.selfdrive.ui.bp.charging import screen_policy as sp


def test_the_screen_is_held_awake_while_the_car_is_on():
  policy = sp.screen_policy(ignition=True)
  assert policy.timeout_s == sp.STAY_AWAKE_TIMEOUT_S
  assert not policy.suppress_screensaver


def test_with_the_car_off_the_screen_gets_a_short_timeout_and_no_screensaver():
  policy = sp.screen_policy(ignition=False)
  assert policy.timeout_s == sp.CAR_OFF_TIMEOUT_S
  assert 0 < sp.CAR_OFF_TIMEOUT_S <= 10                       # near-immediate, but a touch still shows it briefly
  assert policy.suppress_screensaver                          # the sunnypilot screensaver would keep the display lit instead


def test_the_hold_reports_a_policy_only_when_it_changes():
  hold = sp.ChargingScreenHold()
  assert hold.update(ignition=True) == sp.screen_policy(True)      # first frame applies it
  assert hold.update(ignition=True) is None
  assert hold.update(ignition=True) is None
  assert hold.update(ignition=False) == sp.screen_policy(False)    # the car turned off: start the short timeout once
  assert hold.update(ignition=False) is None                       # not re-applied every frame (that would keep resetting the timer)
  assert hold.update(ignition=True) == sp.screen_policy(True)      # car back on: hold again


def test_opening_the_screen_with_the_car_already_off_applies_the_short_policy():
  hold = sp.ChargingScreenHold()
  assert hold.update(ignition=False) == sp.screen_policy(False)


def test_a_new_show_starts_from_scratch():
  hold = sp.ChargingScreenHold()
  hold.update(ignition=False)
  hold.reset()
  assert hold.update(ignition=False) == sp.screen_policy(False)


# ---- the Device side: the screensaver is skipped only while suppressed ----

class _Screensaver:
  was_dismissed = False
  is_active = False

  def __init__(self):
    self.initialized = False

  def initialize(self):
    self.initialized = True


class _FakeUiState:
  screensaver_enabled = True
  boot_offroad_mode = 0

  def __init__(self):
    self.screensaver = _Screensaver()


def _sleep(device, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot import ui_state as sp_ui
  pushed = []
  monkeypatch.setattr(sp_ui.gui_app, "push_widget", lambda w: pushed.append(w))
  monkeypatch.setattr(sp_ui.gui_app, "get_active_widget", lambda: None)
  fake = _FakeUiState()
  device._set_awake(False, fake)
  return fake, pushed


def test_a_sleeping_device_shows_the_screensaver_by_default(monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.ui_state import DeviceSP
  device = DeviceSP()
  fake, pushed = _sleep(device, monkeypatch)
  assert fake.screensaver.initialized and pushed and device._blocked_by_screensaver


def test_a_suppressed_screensaver_lets_the_display_turn_off(monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.ui_state import DeviceSP
  device = DeviceSP()
  device.suppress_screensaver(True)
  fake, pushed = _sleep(device, monkeypatch)
  assert not fake.screensaver.initialized and not pushed
  assert not device._blocked_by_screensaver                   # so Device goes on to power the display off
  device.suppress_screensaver(False)
  fake, pushed = _sleep(device, monkeypatch)
  assert fake.screensaver.initialized and device._blocked_by_screensaver

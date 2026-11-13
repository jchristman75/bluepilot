"""With the car off and the charging screen open, the display really turns off: a short timeout and no screensaver in its
place. With the charging screen not showing, the normal device timeout and screensaver apply (on purpose)."""
import pytest

from openpilot.selfdrive.ui.bp.charging import screen_policy as sp


@pytest.mark.parametrize("ignition,screen_open,expected", [
  (True, True, False),       # car on: the charging screen stays up as long as you want
  (True, False, False),
  (False, True, True),       # car off with the charging screen open
  (False, False, False),     # charging screen not showing: normal timeout and screensaver
])
def test_the_display_is_forced_off_only_with_the_car_off_and_the_charging_screen_open(ignition, screen_open, expected):
  assert sp.screen_off_with_car_off(ignition, screen_open) is expected


def test_the_car_off_timeout_is_short_but_a_touch_still_shows_the_screen_briefly():
  assert 0 < sp.CAR_OFF_TIMEOUT_S <= 10


def test_the_rule_reports_a_change_once_so_the_countdown_is_not_reset_every_frame():
  rule = sp.CarOffScreenRule()
  assert rule.update(True, False) is False                   # nothing changed
  assert rule.update(False, True) is True                    # car turned off with the charging screen open
  assert rule.active
  assert rule.update(False, True) is False                   # not reported again
  assert rule.update(True, True) is True                     # car back on: released
  assert not rule.active
  assert rule.update(False, False) is False                  # car off without the charging screen: never active


# ---- the Device side ----

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


def test_the_device_uses_the_short_timeout_while_the_rule_is_active():
  from openpilot.selfdrive.ui.ui_state import Device
  device = Device()
  device.set_override_interactive_timeout(99999)               # the charging screen's hold
  assert device.interactive_timeout == 99999
  device.set_car_off_charging_screen_off(True)
  assert device.interactive_timeout == sp.CAR_OFF_TIMEOUT_S    # takes precedence over the hold
  device.set_car_off_charging_screen_off(False)
  assert device.interactive_timeout == 99999

"""hardwared's power-save decision with the charge monitor: the screen may go dark, the comma must keep running."""
from openpilot.bluepilot.system.power_save_bp import should_power_save


def test_stock_rule_ignition_off_and_screen_dark_means_power_save():
  assert should_power_save(ignition=False, screen_brightness_pct=0.0, charge_monitor_active=False)


def test_the_screen_being_on_or_the_car_being_on_means_no_power_save():
  assert not should_power_save(ignition=False, screen_brightness_pct=40.0, charge_monitor_active=False)
  assert not should_power_save(ignition=True, screen_brightness_pct=0.0, charge_monitor_active=False)


def test_while_the_charge_monitor_tracks_a_session_the_comma_stays_out_of_power_save_with_the_screen_dark():
  # the charging screen now lets the display time out when the car is off; the comma must keep running at full speed
  assert not should_power_save(ignition=False, screen_brightness_pct=0.0, charge_monitor_active=True)


def test_the_monitor_flag_never_turns_power_save_on_by_itself():
  assert not should_power_save(ignition=True, screen_brightness_pct=50.0, charge_monitor_active=True)
  assert not should_power_save(ignition=False, screen_brightness_pct=50.0, charge_monitor_active=True)

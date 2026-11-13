"""bp_chargemon's keep-alive decisions: DC fast charging only, for at most one hour."""
from openpilot.bluepilot.selfdrive.car import chargemon_policy_bp as pol

T = 1000.0


def test_the_keep_alive_is_limited_to_one_hour():
  assert pol.MAX_MONITOR_S == 3600.0


def test_only_a_recent_dc_fast_charge_keeps_the_comma_awake_when_the_car_turns_off():
  assert pol.keep_alive_on_ignition_drop(T, last_onroad_dc_t=T - 1.0)
  assert not pol.keep_alive_on_ignition_drop(T, last_onroad_dc_t=T - pol.ONROAD_DC_WINDOW_S - 1.0)
  assert not pol.keep_alive_on_ignition_drop(T, last_onroad_dc_t=-1e9)          # AC charging or no charge: never seen DC


def test_the_monitor_ends_when_dc_charging_has_been_inactive_for_a_minute():
  assert pol.release_reason(T, mode_start_t=T - 100, last_active_t=T - 10) is None
  assert pol.release_reason(T, mode_start_t=T - 100, last_active_t=T - pol.INACTIVE_RELEASE_S - 1) == "charge ended"


def test_the_monitor_ends_after_one_hour_even_while_still_charging():
  assert pol.release_reason(T, mode_start_t=T - 3599, last_active_t=T) is None
  assert pol.release_reason(T, mode_start_t=T - 3601, last_active_t=T) == "max monitor time"


def test_an_ended_charge_is_reported_before_the_time_cap():
  assert pol.release_reason(T, mode_start_t=T - 7200, last_active_t=T - 500) == "charge ended"


def test_a_probe_needs_dc_fast_charging_but_an_entered_monitor_keeps_tracking_on_the_charge_status():
  # the monitor is only entered for DC fast charging; once running with the car off, the 0x440 status that says "DC"
  # may not be forwarded by the gateway, and losing it must not end the tracking
  assert pol.session_active("probe", charging_active=True, dc_fast=True)
  assert not pol.session_active("probe", charging_active=True, dc_fast=False)
  assert pol.session_active("monitor", charging_active=True, dc_fast=True)
  assert pol.session_active("monitor", charging_active=True, dc_fast=False)
  assert not pol.session_active("monitor", charging_active=False, dc_fast=True)
  assert not pol.session_active("monitor", charging_active=False, dc_fast=False)

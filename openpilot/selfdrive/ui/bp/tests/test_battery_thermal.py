"""Battery temperature icon logic: the arrow (coolant heating or cooling the pack), units and color bands."""
import pytest

from openpilot.selfdrive.ui.bp.lib import battery_thermal as bt

DT = 0.05


def run(th, pack, coolant, seconds):
  for _ in range(int(seconds / DT)):
    out = th.update(pack, coolant, DT)
  return out


def test_a_coolant_near_the_pack_temperature_shows_no_arrow():
  th = bt.BatteryThermal()
  assert run(th, 26.0, 26.5, 10) == bt.Trend.STEADY


def test_a_much_warmer_coolant_means_the_pack_is_being_heated():
  th = bt.BatteryThermal()
  assert run(th, 12.0, 21.0, 10) == bt.Trend.HEATING


def test_a_much_cooler_coolant_means_the_pack_is_being_cooled():
  th = bt.BatteryThermal()
  assert run(th, 36.0, 18.0, 10) == bt.Trend.COOLING        # the DC fast charge on route 464


def test_a_short_blip_does_not_flip_the_arrow():
  th = bt.BatteryThermal()
  run(th, 26.0, 26.0, 10)
  assert run(th, 26.0, 33.0, 0.5) == bt.Trend.STEADY       # half a second of 7 C
  assert run(th, 26.0, 26.0, 5) == bt.Trend.STEADY


def test_the_arrow_has_hysteresis_so_it_does_not_flicker_around_the_threshold():
  th = bt.BatteryThermal()
  run(th, 20.0, 24.0, 10)                                    # 4 C: on
  assert th.trend == bt.Trend.HEATING
  assert run(th, 20.0, 22.0, 10) == bt.Trend.HEATING         # 2 C: between exit and enter, stays on
  assert run(th, 20.0, 20.8, 10) == bt.Trend.STEADY          # under 1.5 C: off
  assert run(th, 20.0, 22.0, 10) == bt.Trend.STEADY          # 2 C does not turn it back on


def test_it_can_swing_from_heating_to_cooling():
  th = bt.BatteryThermal()
  run(th, 20.0, 26.0, 10)
  assert th.trend == bt.Trend.HEATING
  assert run(th, 30.0, 20.0, 15) == bt.Trend.COOLING


def test_unavailable_data_clears_the_arrow_and_the_state():
  th = bt.BatteryThermal()
  run(th, 20.0, 26.0, 10)
  assert th.update(None, None, DT) == bt.Trend.STEADY
  assert run(th, 20.0, 20.5, 1) == bt.Trend.STEADY


def test_temperature_text_is_whole_degrees_celsius():
  assert bt.temp_text(26.4) == "26"
  assert bt.temp_text(26.6) == "27"
  assert bt.temp_text(0.0) == "0"
  assert bt.temp_text(13.5) == "14"
  assert bt.temp_text(None) == "--"


def test_color_bands_run_from_cold_blue_through_green_to_hot_red():
  cold, cool, ok, warm, hot = (bt.temp_color(t) for t in (2.0, 12.0, 25.0, 38.0, 46.0))
  assert cold[2] > cold[0] and cold[2] > cold[1]             # blue
  assert ok[1] > ok[0] and ok[1] > ok[2]                     # green
  assert hot[0] > hot[1] and hot[0] > hot[2]                 # red
  assert len({cold, cool, ok, warm, hot}) == 5
  assert bt.temp_color(None) == bt.NO_DATA_COLOR


def test_the_fill_fraction_maps_the_working_range_and_clamps():
  assert bt.fill_fraction(-30.0) == pytest.approx(0.08)
  assert bt.fill_fraction(80.0) == pytest.approx(1.0)
  assert bt.fill_fraction(15.0) < bt.fill_fraction(35.0)


class FakeSM:
  def __init__(self, pack=31.0, coolant=24.0, available=True, alive=True, recv_frame=10):
    hb = type("HB", (), {"packTempC": pack, "coolantTempC": coolant, "thermalDataAvailable": available})()
    self.data = {"carStateBP": type("CS", (), {"hybridBattery": hb})()}
    self.alive = {"carStateBP": alive}
    self.recv_frame = {"carStateBP": recv_frame} if recv_frame is not None else {}

    class _V(dict):
      pass
    self.valid = _V(carStateBP=True)

  def __getitem__(self, k):
    return self.data[k]


def test_telemetry_is_read_from_a_live_current_carstatebp():
  assert bt.telemetry_from(FakeSM(), started_frame=5) == (31.0, 24.0)


@pytest.mark.parametrize("sm", [
  FakeSM(available=False),             # sensor fault or no battery data
  FakeSM(alive=False),                 # the message stopped
  FakeSM(recv_frame=3),                # left over from before this drive started
  FakeSM(recv_frame=None),             # never received
])
def test_unavailable_stale_or_missing_telemetry_is_none(sm):
  assert bt.telemetry_from(sm, started_frame=5) is None


def test_a_sm_without_the_service_is_none():
  class Empty:
    alive: dict = {}
    recv_frame: dict = {}
    valid: dict = {}
  assert bt.telemetry_from(Empty(), started_frame=0) is None


def test_the_icon_yields_to_the_set_speed_block_and_needs_the_toggle_and_data():
  data = (30.0, 24.0)
  assert bt.should_show(True, data, hud_busy=False)
  assert not bt.should_show(True, data, hud_busy=True)       # set-speed block showing in the same corner
  assert not bt.should_show(False, data, hud_busy=False)     # toggle off
  assert not bt.should_show(True, None, hud_busy=False)      # no telemetry


# ---- the "good for charging" range: green outline ----

def test_the_optimal_range_is_25_to_40_c():
  assert (bt.OPTIMAL_LOW_C, bt.OPTIMAL_HIGH_C) == (25.0, 40.0)


def test_the_pack_is_optimal_inside_the_range_and_not_outside():
  th = bt.BatteryThermal()
  for pack, expected in ((20.0, False), (26.0, True), (33.0, True), (39.5, True), (44.0, False)):
    th = bt.BatteryThermal()
    th.update(pack, pack, DT)
    assert th.optimal is expected, pack


def test_the_range_edges_have_hysteresis_so_a_half_degree_wobble_does_not_flicker():
  th = bt.BatteryThermal()
  th.update(26.0, 26.0, DT)
  assert th.optimal
  th.update(24.5, 24.5, DT)                  # just under the 25 C edge: stays on (leaves below 24)
  assert th.optimal
  th.update(23.5, 23.5, DT)
  assert not th.optimal
  th.update(24.5, 24.5, DT)                  # back just under the edge: stays off (re-enters at 25)
  assert not th.optimal
  th.update(25.5, 25.5, DT)
  assert th.optimal
  th.update(40.5, 40.5, DT)                  # just over the top edge: stays on (leaves above 41)
  assert th.optimal
  th.update(41.5, 41.5, DT)
  assert not th.optimal


def test_no_data_is_not_optimal():
  th = bt.BatteryThermal()
  th.update(30.0, 30.0, DT)
  th.update(None, None, DT)
  assert not th.optimal


def test_the_outline_is_green_in_range_and_white_otherwise():
  assert bt.outline_color(True) == bt.OPTIMAL_OUTLINE
  assert bt.outline_color(False) == bt.NORMAL_OUTLINE
  assert bt.OPTIMAL_OUTLINE[1] > bt.OPTIMAL_OUTLINE[0] and bt.OPTIMAL_OUTLINE[1] > bt.OPTIMAL_OUTLINE[2]

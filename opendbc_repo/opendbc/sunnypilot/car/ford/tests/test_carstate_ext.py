"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Regression test for BluePilotDev/bluepilot#193: holding a Ford combo cruise button
# (Set+Inc/Set+Dec/Cancel+Res) produced a self-sustaining stream of spurious press/release
# events every frame, because the "did my signal change" check compared against derived,
# shared button_states slots instead of each raw signal's own previous value. See the
# comments in CarStateExt.update() for the full mechanism.

import unittest

from opendbc.car import Bus, structs
from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt

ButtonType = structs.CarState.ButtonEvent.Type


class _FakeCANParser:
  def __init__(self, vl):
    self.vl = vl


def _cs(cruise_enabled: bool) -> structs.CarState:
  ret = structs.CarState.new_message()
  ret.cruiseState.enabled = cruise_enabled
  return ret


def _parsers(dec_press: int, inc_press: int = 0, cncl_res_press: int = 0, on_off_press: int = 0):
  vl = {
    "Steering_Data_FD1": {
      "CcAslButtnSetDecPress": dec_press,
      "CcAslButtnSetIncPress": inc_press,
      "CcAslButtnCnclResPress": cncl_res_press,
      "CcButtnOnOffPress": on_off_press,
    },
  }
  return {Bus.pt: _FakeCANParser(vl)}


class TestComboButtonEvents(unittest.TestCase):
  def setUp(self):
    self.ext = CarStateExt(structs.CarParams(), structs.CarParamsSP())

  def _events(self):
    return [(e.type, e.pressed) for e in self.ext.button_events]

  def test_sustained_hold_emits_exactly_one_press_and_one_release(self):
    """The core #193 regression: holding Set+Dec should emit exactly one press event
    (on the rising edge) and exactly one release event (on the falling edge) -- not a
    fresh press/release pair every single frame it stays held."""
    ret = _cs(cruise_enabled=False)

    # Rising edge.
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
    self.assertEqual(self._events(), [(ButtonType.setCruise, True)])

    # Held steady for many frames: must be silent every time.
    for _ in range(50):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
      self.assertEqual(self._events(), [], "spurious event while button held steady (#193)")

    # Cruise engages partway through the hold (as it would in the real failure) -- still silent.
    ret = _cs(cruise_enabled=True)
    for _ in range(20):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
      self.assertEqual(self._events(), [], "spurious event after cruise_enabled changed mid-hold (#193)")

    # Falling edge: exactly one release, for the event type that was actually emitted on press.
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0))
    self.assertEqual(self._events(), [(ButtonType.setCruise, False)])

  def test_inc_and_dec_do_not_cross_contaminate_setcruise(self):
    """Holding Set+Dec (emitting setCruise) must not be disturbed by the Inc signal's own
    steady 0 state being reprocessed each frame -- the original bug's exact mechanism."""
    ret = _cs(cruise_enabled=False)
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1, inc_press=0))
    self.assertEqual(self._events(), [(ButtonType.setCruise, True)])

    for _ in range(10):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1, inc_press=0))
      self.assertEqual(self._events(), [])

  def test_tap_while_engaged_emits_decel_not_setcruise(self):
    """A quick tap while cruise is already enabled should emit decelCruise, matching stock
    adjust-speed behavior -- unaffected by the fix."""
    ret = _cs(cruise_enabled=True)
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
    self.assertEqual(self._events(), [(ButtonType.decelCruise, True)])

    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0))
    self.assertEqual(self._events(), [(ButtonType.decelCruise, False)])

  def test_cancel_resume_combo_unaffected(self):
    ret = _cs(cruise_enabled=True)
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0, cncl_res_press=1))
    self.assertEqual(self._events(), [(ButtonType.cancel, True)])

    for _ in range(10):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0, cncl_res_press=1))
      self.assertEqual(self._events(), [])

    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0, cncl_res_press=0))
    self.assertEqual(self._events(), [(ButtonType.cancel, False)])


if __name__ == "__main__":
  unittest.main()


class TestLatCtlLimStat(unittest.TestCase):
  """LatCtlLim_D_Stat is read on CAN FD cars only (lateral_angle_ext's anti-windup)."""

  def _parsers(self, lim):
    p = _parsers(dec_press=0)
    p[Bus.pt].vl["Lane_Assist_Data3_FD1"] = {"LatCtlLim_D_Stat": lim}
    return p

  def test_canfd_reads_the_flag(self):
    from opendbc.car.ford.values import FordFlags
    CP = structs.CarParams()
    CP.flags = int(FordFlags.CANFD)
    ext = CarStateExt(CP, structs.CarParamsSP())
    ext.update(_cs(False), structs.CarStateSP(), self._parsers(2))
    self.assertEqual(ext.lat_ctl_lim_stat, 2)

  def test_can_stays_zero(self):
    ext = CarStateExt(structs.CarParams(), structs.CarParamsSP())
    ext.update(_cs(False), structs.CarStateSP(), self._parsers(2))
    self.assertEqual(ext.lat_ctl_lim_stat, 0)


class TestBatteryThermalTelemetry(unittest.TestCase):
  """carStateBP.hybridBattery carries the pack and battery-coolant temperatures (BattTrac_Te_Actl on 0x24C,
  BattTracClnt_Te_Actl on 0x24B) for the battery temperature HUD icon."""

  def _bp(self, pack_c, coolant_c, flags=None):
    from opendbc.car.ford.values import FordFlags
    cp_params = structs.CarParams()
    cp_params.flags = int(FordFlags.HEV_BATTERY_DATA if flags is None else flags)
    ext = CarStateExt(cp_params, structs.CarParamsSP())
    vl = {
      "Battery_Traction_1_FD1": {"BattTrac_U_LimHi": 400.0, "BattTrac_U_LimLo": 280.0, "BattTrac_U_Actl": 360.0, "BattTrac_I_Actl": 0.0},
      "Battery_Traction_3_FD1": {"BattTracSoc_Pc_MnPrtct": 2.5, "BattTracSoc_Pc_MxPrtct": 97.3, "BattTracClnt_Te_Actl": coolant_c},
      "Battery_Traction_4_FD1": {"BattTracSoc2_Pc_Actl": 55.0, "BattTrac_Te_Actl": pack_c},
      "HV_Battery_Current_BP": {"BattTracI_Actl_BP": 0.0},
      "MtrTracData_1_FD1": {"MtrTrac2_I_Actl": 0.0},
    }
    return ext.update_car_state_bp(_FakeCANParser(vl), _FakeCANParser({})).carStateBP.hybridBattery

  def test_temperatures_are_reported_when_the_battery_data_is_available(self):
    hb = self._bp(pack_c=31.5, coolant_c=24.0)
    self.assertTrue(hb.thermalDataAvailable)
    self.assertAlmostEqual(hb.packTempC, 31.5)
    self.assertAlmostEqual(hb.coolantTempC, 24.0)

  def test_the_pack_sensor_fault_and_unknown_codes_read_as_unavailable(self):
    # BattTrac_Te_Actl raw 1023 "Fault" / 1022 "Unknown" decode to 461.5 / 461.0 degC
    for bad in (461.5, 461.0):
      hb = self._bp(pack_c=bad, coolant_c=24.0)
      self.assertFalse(hb.thermalDataAvailable, bad)

  def test_cars_without_battery_data_report_nothing(self):
    hb = self._bp(pack_c=30.0, coolant_c=25.0, flags=0)
    self.assertFalse(hb.thermalDataAvailable)
    self.assertEqual(hb.packTempC, 0.0)

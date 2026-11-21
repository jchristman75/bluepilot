"""
BluePilot: tests for the Ford APIM GPS decoder (APIMGPS_Data_Nav_1/2/3_FD1, 0x462-0x464).

Frames are built with the real DBC packer and read back with the real CAN parser, so the tests cover the bit
layout as well as the decoding. The coordinates are made up.
"""
import datetime
import math
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.sunnypilot.car.ford.gps_ext import (NAV_MESSAGES, HORIZONTAL_ACCURACY_MIN, FordGpsFix, parse_apim_gps,
                                                 nav_frame_nanos)

DBC = "ford_lincoln_base_pt"


def _signals(lat=37.5125, lon=-122.2511, alt_ft=120.0, utc=datetime.datetime(2026, 11, 21, 15, 41, 7),
             speed_mph=30, heading=128.25, hdop=0.4, vdop=0.6, dim=2, sats=12, fault=0, inferred=0):
  def split(value):
    deg = int(abs(value))
    minutes = (abs(value) - deg) * 60.0
    whole = int(minutes)
    return deg, whole, round(minutes - whole, 4)

  lat_deg, lat_min, lat_dec = split(lat)
  lon_deg, lon_min, lon_dec = split(lon)
  nav1 = {
    "GPS_Latitude_Degrees": lat_deg, "GPS_Latitude_Minutes": lat_min, "GPS_Latitude_Min_dec": lat_dec,
    "GpsHsphLattSth_D_Actl": 2 if lat >= 0 else 1,
    # The car sends a western longitude's degrees as a negative number; the decoder must not double-negate it.
    "GPS_Longitude_Degrees": -lon_deg if lon < 0 else lon_deg, "GPS_Longitude_Minutes": lon_min,
    "GPS_Longitude_Min_dec": lon_dec, "GpsHsphLongEast_D_Actl": 1 if lon >= 0 else 2,
  }
  nav2 = {
    "GpsUtcYr_No_Actl": utc.year, "GpsUtcMnth_No_Actl": utc.month, "GpsUtcDay_No_Actl": utc.day,
    "GPS_UTC_hours": utc.hour, "GPS_UTC_minutes": utc.minute, "GPS_UTC_seconds": utc.second,
    "Gps_B_Falt": fault, "GPS_Actual_vs_Infer_pos": inferred, "GPS_Pdop": 0.8,
  }
  nav3 = {
    "GPS_Speed": speed_mph, "GPS_Heading": heading, "GPS_Hdop": hdop, "GPS_Vdop": vdop, "GPS_dimension": dim,
    "GPS_Sat_num_in_view": sats, "GPS_MSL_altitude": alt_ft,
  }
  return nav1, nav2, nav3


def _decode(nav1, nav2, nav3, t_ns=1_000_000_000):
  """Pack the three messages with the DBC, parse them back, and return (parser, fix)."""
  packer = CANPacker(DBC)
  frames = [packer.make_can_msg(name, 0, values) for name, values in zip(NAV_MESSAGES, (nav1, nav2, nav3), strict=True)]
  parser = CANParser(DBC, [(name, float("nan")) for name in NAV_MESSAGES], 0)
  parser.update([(t_ns, [(addr, dat, 0) for addr, dat, _ in frames])])
  return parser, parse_apim_gps(*(parser.vl[name] for name in NAV_MESSAGES))


class TestParseApimGps(unittest.TestCase):
  def test_round_trip_through_the_dbc(self):
    _, fix = _decode(*_signals())
    self.assertIsInstance(fix, FordGpsFix)
    self.assertTrue(fix.has_fix)
    self.assertAlmostEqual(fix.latitude, 37.5125, places=4)
    self.assertAlmostEqual(fix.longitude, -122.2511, places=4)
    self.assertAlmostEqual(fix.speed, 30 * 0.44704, places=3)
    self.assertAlmostEqual(fix.bearing_deg, 128.25, places=2)
    self.assertAlmostEqual(fix.altitude, 120 * 0.3048, places=1)
    self.assertEqual(fix.unix_timestamp_millis,
                     int(datetime.datetime(2026, 11, 21, 15, 41, 7, tzinfo=datetime.UTC).timestamp() * 1000))
    self.assertEqual(fix.satellite_count, 12)

  def test_southern_and_eastern_hemispheres(self):
    _, fix = _decode(*_signals(lat=-33.8568, lon=151.2153))
    self.assertAlmostEqual(fix.latitude, -33.8568, places=4)
    self.assertAlmostEqual(fix.longitude, 151.2153, places=4)

  def test_velocity_points_along_the_heading(self):
    _, fix = _decode(*_signals(speed_mph=40, heading=90.0))
    north, east, down = fix.v_ned
    self.assertAlmostEqual(north, 0.0, places=3)
    self.assertAlmostEqual(east, 40 * 0.44704, places=3)
    self.assertEqual(down, 0.0)

  def test_accuracy_is_never_claimed_better_than_measured(self):
    # Against the u-blox on route 4a3 the APIM position was 4.9 m median / 10.9 m p90 off at HDOP 0.4, so a low
    # HDOP must not be turned into a 2 m claim.
    _, fix = _decode(*_signals(hdop=0.4))
    self.assertGreaterEqual(fix.horizontal_accuracy, HORIZONTAL_ACCURACY_MIN)
    _, worse = _decode(*_signals(hdop=4.0))
    self.assertGreater(worse.horizontal_accuracy, fix.horizontal_accuracy)

  def test_two_d_fix_has_a_loose_altitude(self):
    _, fix3d = _decode(*_signals(dim=2))
    _, fix2d = _decode(*_signals(dim=1))
    self.assertTrue(fix2d.has_fix)
    self.assertGreater(fix2d.vertical_accuracy, fix3d.vertical_accuracy)

  def test_unknown_satellite_count_reads_zero(self):
    for sats in (30, 31):
      _, fix = _decode(*_signals(sats=sats))
      self.assertEqual(fix.satellite_count, 0, sats)

  def test_no_fix_states_are_rejected(self):
    # Before the APIM has a fix it sends a stale clock (an hour off on route 4e8), so none of these may produce a fix.
    for kwargs in ({"dim": 0}, {"fault": 1}, {"inferred": 1}):
      _, fix = _decode(*_signals(**kwargs))
      self.assertIsNone(fix, kwargs)

  def test_fault_and_unknown_codes_are_rejected(self):
    # (message index, signal, decoded value of the DBC's Fault/Unknown/Invalid code)
    cases = ((0, "GpsHsphLattSth_D_Actl", 0), (0, "GpsHsphLattSth_D_Actl", 3),
             (0, "GpsHsphLongEast_D_Actl", 0), (0, "GpsHsphLongEast_D_Actl", 3),
             (0, "GPS_Latitude_Minutes", 62), (0, "GPS_Longitude_Minutes", 63),
             (0, "GPS_Latitude_Min_dec", 1.6382), (0, "GPS_Longitude_Min_dec", 1.6383),
             (0, "GPS_Latitude_Degrees", 165), (0, "GPS_Longitude_Degrees", 331),
             (1, "GPS_UTC_hours", 30), (1, "GPS_UTC_minutes", 62),
             (1, "GPS_UTC_seconds", 63), (1, "GpsUtcMnth_No_Actl", 15))
    for index, name, value in cases:
      navs = [dict(m) for m in _signals()]
      navs[index][name] = value
      _, fix = _decode(*navs)
      self.assertIsNone(fix, (name, value))

  def test_the_year_fault_code_is_rejected(self):
    # VAL_ tables are raw values: GpsUtcYr_No_Actl raw 31 "Fault" decodes to 2041, a valid-looking year.
    _, fix = _decode(*_signals(utc=datetime.datetime(2041, 3, 1, 12, 0, 0)))
    self.assertIsNone(fix)

  def test_invalid_speed_keeps_the_position_without_motion(self):
    _, fix = _decode(*_signals(speed_mph=254))
    self.assertTrue(fix.has_fix)
    self.assertEqual(fix.speed, 0.0)
    self.assertGreaterEqual(fix.bearing_accuracy_deg, 180.0)

  def test_standing_still_has_no_heading(self):
    _, fix = _decode(*_signals(speed_mph=0))
    self.assertGreaterEqual(fix.bearing_accuracy_deg, 180.0)

  def test_frame_time_comes_from_the_parser(self):
    parser, _ = _decode(*_signals(), t_ns=7_000_000_000)
    self.assertEqual(nav_frame_nanos(parser), 7_000_000_000)

  def test_frame_time_is_zero_without_timestamps(self):
    class Plain:
      vl = {}
    self.assertEqual(nav_frame_nanos(Plain()), 0)


class _Cam:
  vl: dict = {}


class TestCarStateBPGps(unittest.TestCase):
  """carStateBP.gps is filled from the parsed Nav messages."""

  def _ext(self):
    from opendbc.car import structs
    from opendbc.car.ford.values import FordFlags
    from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt
    cp = structs.CarParams()
    cp.flags = int(FordFlags.CANFD)
    return CarStateExt(cp, structs.CarParamsSP())

  def test_a_fix_is_copied_with_its_frame_time(self):
    parser, _ = _decode(*_signals(), t_ns=5_000_000_000)
    gps = self._ext().update_car_state_bp(parser, _Cam()).carStateBP.gps
    self.assertTrue(gps.hasFix)
    self.assertEqual(gps.frameMonoTime, 5_000_000_000)
    self.assertAlmostEqual(gps.latitude, 37.5125, places=4)
    self.assertAlmostEqual(gps.longitude, -122.2511, places=4)
    self.assertAlmostEqual(gps.speed, 30 * 0.44704, places=3)
    self.assertEqual(len(gps.vNED), 3)
    self.assertGreaterEqual(gps.horizontalAccuracy, HORIZONTAL_ACCURACY_MIN)

  def test_no_fix_keeps_the_frame_time_but_no_position(self):
    parser, _ = _decode(*_signals(dim=0), t_ns=5_000_000_000)
    gps = self._ext().update_car_state_bp(parser, _Cam()).carStateBP.gps
    self.assertFalse(gps.hasFix)
    self.assertEqual(gps.frameMonoTime, 5_000_000_000)
    self.assertEqual(gps.latitude, 0.0)

  def test_a_parser_without_the_nav_messages_is_left_alone(self):
    # CANParser.vl adds any message it is asked for, with alive checks on; a car whose parser never registered the
    # Nav messages (non-CAN FD) must not get them added, or they would time out and invalidate CAN.
    parser = CANParser(DBC, [("BCM_Lamp_Stat_FD1", float("nan"))], 0)
    before = set(parser.addresses)
    gps = self._ext().update_car_state_bp(parser, _Cam()).carStateBP.gps
    self.assertFalse(gps.hasFix)
    self.assertEqual(gps.frameMonoTime, 0)
    self.assertEqual(parser.addresses, before)

  def test_registered_but_never_received_is_no_fix(self):
    parser = CANParser(DBC, [(name, float("nan")) for name in NAV_MESSAGES], 0)
    gps = self._ext().update_car_state_bp(parser, _Cam()).carStateBP.gps
    self.assertFalse(gps.hasFix)
    self.assertEqual(gps.frameMonoTime, 0)

  def test_the_ford_parser_registers_the_nav_messages_as_non_critical_on_can_fd_only(self):
    from opendbc.car import Bus, structs
    from opendbc.car.ford.carstate import CarState
    from opendbc.car.ford.values import CAR, FordFlags
    from opendbc.car.ford.interface import CarInterface
    for car, canfd in ((CAR.FORD_MUSTANG_MACH_E_MK1, True), (CAR.FORD_BRONCO_SPORT_MK1, False)):
      cp = CarInterface.get_non_essential_params(car)
      self.assertEqual(bool(cp.flags & FordFlags.CANFD), canfd, car)
      pt = CarState.get_can_parsers(cp, structs.CarParamsSP())[Bus.pt]
      for name in NAV_MESSAGES:
        addr = pt.dbc.name_to_msg[name].address
        self.assertEqual(addr in pt.addresses, canfd, (car, name))
        if canfd:
          self.assertTrue(pt.message_states[addr].ignore_alive, name)


if __name__ == "__main__":
  unittest.main()

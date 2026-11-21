"""BluePilot: tests for the car-GPS fallback that ubloxd uses while the u-blox has no fix."""
import math
import unittest
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.bluepilot.system.car_gps_bp import (CarGpsFallback, EXTRAPOLATION_ACCURACY_PER_S, MAX_CAR_AGE_NS,
                                                   PUBLISH_INTERVAL_NS, UBLOX_HOLD_NS)

SEC = 1_000_000_000
CAR = log.GpsLocationData.SensorSource.car
UBLOX = log.GpsLocationData.SensorSource.ublox
EARTH_RADIUS = 6_378_137.0


def _car(frame_ns, has_fix=True, lat=37.5, lon=-122.25, speed=20.0, bearing=90.0, acc=10.0, millis=1_790_000_000_000):
  v = (speed * math.cos(math.radians(bearing)), speed * math.sin(math.radians(bearing)), 0.0)
  return SimpleNamespace(hasFix=has_fix, frameMonoTime=frame_ns, latitude=lat, longitude=lon, altitude=30.0,
                         speed=speed, bearingDeg=bearing, vNED=list(v), horizontalAccuracy=acc, verticalAccuracy=15.0,
                         speedAccuracy=1.0, bearingAccuracyDeg=5.0, unixTimestampMillis=millis, satelliteCount=0)


def _ublox(has_fix):
  dat = messaging.new_message('gpsLocationExternal', valid=True)
  dat.gpsLocationExternal.source = UBLOX
  dat.gpsLocationExternal.hasFix = has_fix
  return dat


class TestCarGpsFallback(unittest.TestCase):
  def setUp(self):
    self.fb = CarGpsFallback()
    self.fb.enabled = True

  def test_a_ublox_fix_always_passes_through(self):
    self.fb.update_car(_car(10 * SEC))
    out = self.fb.on_ublox(_ublox(True), 10 * SEC)
    self.assertEqual(out.gpsLocationExternal.source, UBLOX)
    self.assertTrue(out.gpsLocationExternal.hasFix)

  def test_the_car_replaces_a_ublox_without_a_fix(self):
    self.fb.update_car(_car(10 * SEC))
    out = self.fb.on_ublox(_ublox(False), 10 * SEC)
    g = out.gpsLocationExternal
    self.assertEqual(g.source, CAR)
    self.assertTrue(g.hasFix)
    self.assertAlmostEqual(g.latitude, 37.5, places=6)
    self.assertEqual(g.satelliteCount, 0)

  def test_the_ublox_keeps_the_channel_for_a_while_after_losing_its_fix(self):
    self.fb.update_car(_car(10 * SEC))
    self.fb.on_ublox(_ublox(True), 10 * SEC)
    out = self.fb.on_ublox(_ublox(False), 10 * SEC + UBLOX_HOLD_NS - 1)
    self.assertEqual(out.gpsLocationExternal.source, UBLOX)
    self.fb.update_car(_car(10 * SEC + UBLOX_HOLD_NS))
    out = self.fb.on_ublox(_ublox(False), 10 * SEC + UBLOX_HOLD_NS + 1)
    self.assertEqual(out.gpsLocationExternal.source, CAR)

  def test_a_stale_car_fix_is_not_used(self):
    self.fb.update_car(_car(10 * SEC))
    out = self.fb.on_ublox(_ublox(False), 10 * SEC + MAX_CAR_AGE_NS + 1)
    self.assertEqual(out.gpsLocationExternal.source, UBLOX)
    self.assertFalse(out.gpsLocationExternal.hasFix)

  def test_a_car_message_without_a_fix_clears_the_last_one(self):
    self.fb.update_car(_car(10 * SEC))
    self.fb.update_car(_car(11 * SEC, has_fix=False))
    out = self.fb.on_ublox(_ublox(False), 11 * SEC)
    self.assertEqual(out.gpsLocationExternal.source, UBLOX)

  def test_disabled_never_substitutes(self):
    self.fb.enabled = False
    self.fb.update_car(_car(10 * SEC))
    self.assertEqual(self.fb.on_ublox(_ublox(False), 10 * SEC).gpsLocationExternal.source, UBLOX)
    self.assertIsNone(self.fb.on_idle(10 * SEC))

  def test_extrapolation_moves_along_the_velocity_and_loosens_accuracy(self):
    self.fb.update_car(_car(10 * SEC, speed=20.0, bearing=90.0))   # due east
    g = self.fb.on_ublox(_ublox(False), 10 * SEC + SEC // 2).gpsLocationExternal
    east_m = math.radians(g.longitude - -122.25) * EARTH_RADIUS * math.cos(math.radians(37.5))
    self.assertAlmostEqual(east_m, 10.0, delta=0.05)
    self.assertAlmostEqual(g.latitude, 37.5, places=6)
    self.assertAlmostEqual(g.horizontalAccuracy, 10.0 + 0.5 * EXTRAPOLATION_ACCURACY_PER_S, places=3)
    self.assertEqual(g.unixTimestampMillis, 1_790_000_000_000 + 500)

  def test_stopped_car_stays_put(self):
    self.fb.update_car(_car(10 * SEC, speed=0.0))
    g = self.fb.on_ublox(_ublox(False), 11 * SEC).gpsLocationExternal
    self.assertAlmostEqual(g.latitude, 37.5, places=9)
    self.assertAlmostEqual(g.longitude, -122.25, places=9)

  def test_the_message_time_is_now_so_locationd_sees_a_current_fix(self):
    self.fb.update_car(_car(10 * SEC))
    out = self.fb.on_ublox(_ublox(False), 10 * SEC + 300_000_000)
    self.assertEqual(out.logMonoTime, 10 * SEC + 300_000_000)

  def test_a_silent_ublox_still_gets_the_car_at_ten_hz(self):
    self.fb.update_car(_car(10 * SEC))
    first = self.fb.on_idle(10 * SEC)
    self.assertEqual(first.gpsLocationExternal.source, CAR)
    self.assertIsNone(self.fb.on_idle(10 * SEC + PUBLISH_INTERVAL_NS - 1))
    self.assertIsNotNone(self.fb.on_idle(10 * SEC + PUBLISH_INTERVAL_NS))

  def test_idle_does_not_double_up_with_ublox_messages(self):
    self.fb.update_car(_car(10 * SEC))
    self.fb.on_ublox(_ublox(False), 10 * SEC)
    self.assertIsNone(self.fb.on_idle(10 * SEC + PUBLISH_INTERVAL_NS // 2))

  def test_idle_without_a_car_fix_publishes_nothing(self):
    self.assertIsNone(self.fb.on_idle(10 * SEC))

  def test_the_accuracy_claims_satisfy_locationd(self):
    # sunnypilot locationd drops a fix whose vertical, speed or bearing accuracy is <= 0
    self.fb.update_car(_car(10 * SEC))
    g = self.fb.on_ublox(_ublox(False), 10 * SEC).gpsLocationExternal
    self.assertGreater(g.verticalAccuracy, 0)
    self.assertGreater(g.speedAccuracy, 0)
    self.assertGreater(g.bearingAccuracyDeg, 0)
    self.assertEqual(len(g.vNED), 3)


if __name__ == "__main__":
  unittest.main()

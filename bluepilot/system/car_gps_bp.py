"""
BluePilot: the car's own GPS as the comma's fallback gpsLocationExternal (BPCarGps).

The comma four's u-blox often has no fix (see the GPS outage notes: 0 satellites on most drives since October),
which leaves the clock at its boot floor, mapd without a position and routes without a location. The Ford's SYNC
APIM broadcasts a GPS fix on CAN (decoded into carStateBP.gps by opendbc/sunnypilot/car/ford/gps_ext.py).

ubloxd stays the only publisher of gpsLocationExternal. While the u-blox has had no fix for UBLOX_HOLD_NS, each of
its no-fix messages is replaced by the car's fix (source = car), and a silent u-blox is covered at 10 Hz. The
car fix arrives at 1 Hz, so every message is moved forward from the frame time to now along the car's velocity,
with the accuracy loosened by EXTRAPOLATION_ACCURACY_PER_S for each second of age and the clock advanced to match.
The message is stamped now: sunnypilot locationd times a fix by its logMonoTime. On route 4a3 the APIM position
had no measurable lag against the u-blox, so no latency is subtracted.
"""
import math
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log

PARAM = "BPCarGps"

MAX_CAR_AGE_NS = 2_500_000_000       # a car fix older than this is not used (the APIM sends at 1 Hz)
UBLOX_HOLD_NS = 2_500_000_000        # after the u-blox's last fix, wait this long before the car takes over
PUBLISH_INTERVAL_NS = 100_000_000    # 10 Hz, the u-blox's own rate, so SubMaster consumers see it alive
EXTRAPOLATION_ACCURACY_PER_S = 2.0   # m of horizontal accuracy lost per second of extrapolation
EARTH_RADIUS = 6_378_137.0           # m (WGS-84 equatorial)


class CarGpsFallback:
  def __init__(self):
    self.enabled = False
    self.car = None                 # latest carStateBP.gps with a fix
    self.last_ublox_fix_ns = -UBLOX_HOLD_NS
    self.last_publish_ns = 0

  def update_car(self, gps):
    """Feed carStateBP.gps; a message without a fix drops the last one."""
    self.car = gps if gps.hasFix and gps.frameMonoTime > 0 else None

  def on_ublox(self, dat, now_ns: int):
    """The message to send in place of the u-blox's gpsLocationExternal `dat`."""
    if dat.gpsLocationExternal.hasFix:
      self.last_ublox_fix_ns = now_ns
      self.last_publish_ns = now_ns
      return dat
    car = self._car_message(now_ns)
    self.last_publish_ns = now_ns
    return car if car is not None else dat

  def on_idle(self, now_ns: int):
    """A car message when the u-blox has sent nothing for a publish interval, else None."""
    if now_ns - self.last_publish_ns < PUBLISH_INTERVAL_NS:
      return None
    car = self._car_message(now_ns)
    if car is not None:
      self.last_publish_ns = now_ns
    return car

  def _car_message(self, now_ns: int):
    car = self.car
    if not self.enabled or car is None or now_ns - self.last_ublox_fix_ns < UBLOX_HOLD_NS:
      return None
    age_ns = now_ns - car.frameMonoTime
    if not 0 <= age_ns <= MAX_CAR_AGE_NS:
      return None
    age = age_ns / 1e9

    north, east = car.vNED[0] * age, car.vNED[1] * age
    lat = car.latitude + math.degrees(north / EARTH_RADIUS)
    lon = car.longitude + math.degrees(east / (EARTH_RADIUS * max(math.cos(math.radians(car.latitude)), 1e-6)))

    dat = messaging.new_message('gpsLocationExternal', valid=True)
    dat.logMonoTime = now_ns
    gps = dat.gpsLocationExternal
    gps.source = log.GpsLocationData.SensorSource.car
    gps.hasFix = True
    gps.flags = 1   # gnssFixOK, as the u-blox sets it with a fix
    gps.latitude, gps.longitude, gps.altitude = lat, lon, car.altitude
    gps.speed, gps.bearingDeg, gps.vNED = car.speed, car.bearingDeg, list(car.vNED)
    gps.horizontalAccuracy = car.horizontalAccuracy + EXTRAPOLATION_ACCURACY_PER_S * age
    gps.verticalAccuracy = car.verticalAccuracy
    gps.speedAccuracy = car.speedAccuracy
    gps.bearingAccuracyDeg = car.bearingAccuracyDeg
    gps.unixTimestampMillis = car.unixTimestampMillis + int(round(age * 1000))
    gps.satelliteCount = car.satelliteCount
    return dat


def read_enabled(params) -> bool:
  """BPCarGps, tolerating a params library that predates the key (see param_flag_bp)."""
  try:
    return bool(params.get_bool(PARAM))
  except Exception:
    return False


def now_ns() -> int:
  return int(time.monotonic() * 1e9)

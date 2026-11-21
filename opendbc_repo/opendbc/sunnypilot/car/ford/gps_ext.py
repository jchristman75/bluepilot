"""
BluePilot: Ford APIM GPS decoder (APIMGPS_Data_Nav_1/2/3_FD1, 0x462-0x464, 1 Hz on the powertrain bus).

The car's own GPS (the SYNC APIM) is a fallback position and clock for the comma, whose u-blox often has no fix.
On route 4a3 (19 min, 1,093 fixes, 26 m/s median) the APIM position was 4.9 m median / 10.9 m p90 from the u-blox
with no measurable lag (best fit -0.05 s), and its speed was within 0.37 m/s median. The accuracies reported below
are set from that comparison rather than from HDOP alone, which would claim ~2 m.

Only a real fix is returned. Until the APIM has one (GPS_dimension 0, GPS_Actual_vs_Infer_pos 1) it sends a stale
clock and a dead-reckoned position: on route 4e8 the clock was an hour off, on 3e3 five hours. Every Fault/Unknown/
Invalid code in the DBC's value tables is rejected.
"""
import datetime
import math
from dataclasses import dataclass

from opendbc.car.common.conversions import Conversions as CV

NAV_MESSAGES = ("APIMGPS_Data_Nav_1_FD1", "APIMGPS_Data_Nav_2_FD1", "APIMGPS_Data_Nav_3_FD1")

HORIZONTAL_ACCURACY_MIN = 10.0    # m: the measured p90 error vs the u-blox at HDOP 0.4
DOP_TO_METERS = 5.0               # m per unit of DOP, above the floor
VERTICAL_ACCURACY_3D_MIN = 15.0   # m
VERTICAL_ACCURACY_2D = 100.0      # m: a 2D fix has no measured altitude
SPEED_ACCURACY = 1.0              # m/s: speed is whole mph (0.45 m/s steps); measured 0.37 m/s median error
BEARING_ACCURACY_MOVING = 5.0     # deg
BEARING_ACCURACY_UNKNOWN = 180.0  # deg
MOVING_SPEED = 2.0                # m/s: below this the heading means nothing
MIN_YEAR = 2025
YEAR_FAULT = 2041                 # GpsUtcYr_No_Actl raw 31 "Fault" (the DBC's VAL_ tables are raw values)
FOOT_TO_METER = 0.3048

_HEMISPHERE_NORTH, _HEMISPHERE_SOUTH = 2, 1   # GpsHsphLattSth_D_Actl
_HEMISPHERE_EAST, _HEMISPHERE_WEST = 1, 2     # GpsHsphLongEast_D_Actl
_DIMENSION_2D, _DIMENSION_3D = 1, 2           # GPS_dimension (0 = No_Fix)


@dataclass(frozen=True)
class FordGpsFix:
  has_fix: bool
  latitude: float              # deg
  longitude: float             # deg
  altitude: float              # m above mean sea level
  speed: float                 # m/s
  bearing_deg: float           # deg, 0 = north
  v_ned: tuple[float, float, float]  # m/s
  horizontal_accuracy: float   # m
  vertical_accuracy: float     # m
  speed_accuracy: float        # m/s
  bearing_accuracy_deg: float  # deg
  unix_timestamp_millis: int
  satellite_count: int         # 0 = unknown


def _coordinate(degrees: float, minutes: float, min_dec: float, max_degrees: float) -> float | None:
  """Unsigned decimal degrees, or None for a Fault/Unknown/Invalid code (out-of-range degrees, minutes 62/63,
  decimal minutes 1.6382/1.6383). The car sends a western longitude's degrees as a negative number, so the sign
  comes from the hemisphere signal, never from here."""
  degrees = abs(degrees)
  if not (degrees <= max_degrees and 0.0 <= minutes < 60.0 and 0.0 <= min_dec < 1.0):
    return None
  return degrees + (minutes + min_dec) / 60.0


def _utc_millis(nav2) -> int | None:
  try:
    t = datetime.datetime(int(nav2["GpsUtcYr_No_Actl"]), int(nav2["GpsUtcMnth_No_Actl"]),
                          int(nav2["GpsUtcDay_No_Actl"]), int(nav2["GPS_UTC_hours"]),
                          int(nav2["GPS_UTC_minutes"]), int(nav2["GPS_UTC_seconds"]), tzinfo=datetime.UTC)
  except ValueError:   # month 15 Fault, hours 30/31, minutes/seconds 62/63
    return None
  return int(t.timestamp() * 1000) if MIN_YEAR <= t.year < YEAR_FAULT else None


def parse_apim_gps(nav1, nav2, nav3) -> FordGpsFix | None:
  """The fix in the three APIM messages (CAN parser value dicts), or None when there is no trustworthy fix."""
  try:
    dimension = int(nav3["GPS_dimension"])
    if dimension not in (_DIMENSION_2D, _DIMENSION_3D) or int(nav2["Gps_B_Falt"]) != 0 or \
       int(nav2["GPS_Actual_vs_Infer_pos"]) != 0:
      return None

    lat_hemisphere, lon_hemisphere = int(nav1["GpsHsphLattSth_D_Actl"]), int(nav1["GpsHsphLongEast_D_Actl"])
    if lat_hemisphere not in (_HEMISPHERE_NORTH, _HEMISPHERE_SOUTH) or lon_hemisphere not in (_HEMISPHERE_EAST, _HEMISPHERE_WEST):
      return None
    latitude = _coordinate(nav1["GPS_Latitude_Degrees"], nav1["GPS_Latitude_Minutes"], nav1["GPS_Latitude_Min_dec"], 90.0)
    longitude = _coordinate(nav1["GPS_Longitude_Degrees"], nav1["GPS_Longitude_Minutes"], nav1["GPS_Longitude_Min_dec"], 180.0)
    millis = _utc_millis(nav2)
    if latitude is None or longitude is None or millis is None or latitude > 90.0 or longitude > 180.0:
      return None
    latitude = latitude if lat_hemisphere == _HEMISPHERE_NORTH else -latitude
    longitude = longitude if lon_hemisphere == _HEMISPHERE_EAST else -longitude

    speed_mph, heading = float(nav3["GPS_Speed"]), float(nav3["GPS_Heading"])
    motion_valid = 0.0 <= speed_mph <= 253.0 and 0.0 <= heading < 360.0   # 254/255 mph: not available
    speed = speed_mph * CV.MPH_TO_MS if motion_valid else 0.0
    moving = motion_valid and speed >= MOVING_SPEED
    bearing = heading if motion_valid else 0.0
    v_ned = (speed * math.cos(math.radians(bearing)), speed * math.sin(math.radians(bearing)), 0.0)

    hdop, vdop = float(nav3["GPS_Hdop"]), float(nav3["GPS_Vdop"])
    horizontal_accuracy = max(HORIZONTAL_ACCURACY_MIN, DOP_TO_METERS * hdop)
    vertical_accuracy = max(VERTICAL_ACCURACY_3D_MIN, DOP_TO_METERS * vdop) if dimension == _DIMENSION_3D else VERTICAL_ACCURACY_2D
    satellites = int(nav3["GPS_Sat_num_in_view"])

    return FordGpsFix(
      has_fix=True,
      latitude=latitude,
      longitude=longitude,
      altitude=float(nav3["GPS_MSL_altitude"]) * FOOT_TO_METER if dimension == _DIMENSION_3D else 0.0,
      speed=speed,
      bearing_deg=bearing,
      v_ned=v_ned,
      horizontal_accuracy=horizontal_accuracy,
      vertical_accuracy=vertical_accuracy,
      speed_accuracy=SPEED_ACCURACY if motion_valid else 50.0,
      bearing_accuracy_deg=BEARING_ACCURACY_MOVING if moving else BEARING_ACCURACY_UNKNOWN,
      unix_timestamp_millis=millis,
      satellite_count=satellites if 0 <= satellites < 30 else 0,   # 30/31: unknown
    )
  except (KeyError, TypeError, ValueError):
    return None


def nav_frame_nanos(cp) -> int:
  """CAN receive time (ns, logMonoTime clock) of the newest Nav_1 frame, or 0 when the parser has none."""
  try:
    return int(max(cp.ts_nanos[NAV_MESSAGES[0]].values()))
  except (AttributeError, KeyError, TypeError, ValueError):
    return 0

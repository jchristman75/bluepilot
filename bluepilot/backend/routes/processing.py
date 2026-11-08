#!/usr/bin/env python3
"""
BluePilot Route Processing Module

Shared module for route processing logic used by both:
1. web_routes_server.py - On-demand processing when routes are accessed
2. route_preprocessor.py - Background processing during idle time

Provides:
- GPS metrics extraction (mileage, speed)
- Reverse geocoding (location names)
- Thumbnail generation
- Processing status checking for recovery from interruptions
"""

import os
import json
import logging
import subprocess
import tempfile
from math import radians, cos, sin, asin, sqrt

logger = logging.getLogger(__name__)

# ============================================================================
# Atomic File Operations (to prevent corruption from crashes)
# ============================================================================
def atomic_json_write(filepath, data):
    """Write JSON file atomically to prevent corruption"""
    try:
        # Create directory if needed
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        # Write to temp file first
        temp_fd, temp_path = tempfile.mkstemp(
            dir=os.path.dirname(filepath),
            prefix='.tmp_',
            suffix=os.path.basename(filepath)
        )

        try:
            json_str = json.dumps(data, indent=2)
            os.write(temp_fd, json_str.encode('utf-8'))
            os.close(temp_fd)

            # Atomic rename (overwrites target on POSIX)
            os.replace(temp_path, filepath)
            return True

        except Exception as e:
            os.close(temp_fd)
            if os.path.exists(temp_path):
                os.remove(temp_path)
            raise e

    except Exception as e:
        logger.error(f"Atomic JSON write failed for {filepath}: {e}")
        return False

# Configuration paths
ROUTES_DIR = "/data/media/0/realdata" if os.path.exists("/data/media/0/realdata") else os.path.expanduser("~/comma_data/media/0/realdata")
METRICS_CACHE = "/data/bluepilot/routes/metrics_cache" if os.path.exists("/data") else os.path.expanduser("~/comma_data/bluepilot/routes/metrics_cache")
THUMBNAIL_CACHE = "/data/bluepilot/routes/thumbs_cache" if os.path.exists("/data") else os.path.expanduser("~/comma_data/bluepilot/routes/thumbs_cache")
FINGERPRINT_CACHE = "/data/bluepilot/routes/fingerprint_cache" if os.path.exists("/data") else os.path.expanduser("~/comma_data/bluepilot/routes/fingerprint_cache")
DRIVE_STATS_CACHE = "/data/bluepilot/routes/drive_stats_cache" if os.path.exists("/data") else os.path.expanduser("~/comma_data/bluepilot/routes/drive_stats_cache")

# Ensure cache directories exist
os.makedirs(METRICS_CACHE, exist_ok=True)
os.makedirs(THUMBNAIL_CACHE, exist_ok=True)
os.makedirs(FINGERPRINT_CACHE, exist_ok=True)
os.makedirs(DRIVE_STATS_CACHE, exist_ok=True)

# Geocoding cache (in-memory)
_geocoding_cache = {}
_last_geocode_time = 0
GEOCODE_RATE_LIMIT = 1.0  # seconds between requests (Nominatim requires 1 req/sec)
GEOCODE_CACHE_FILE = os.path.join(METRICS_CACHE, "geocoding_cache.json")


def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates using Haversine formula

    Args:
        lat1, lon1: First coordinate (degrees)
        lat2, lon2: Second coordinate (degrees)

    Returns:
        Distance in meters
    """
    R = 6371000  # Earth radius in meters
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))

    return R * c


def load_geocoding_cache():
    """Load geocoding cache from disk"""
    global _geocoding_cache
    try:
        if os.path.exists(GEOCODE_CACHE_FILE):
            with open(GEOCODE_CACHE_FILE) as f:
                _geocoding_cache = json.load(f)
            logger.info(f"Loaded {len(_geocoding_cache)} cached geocoding results")
    except Exception as e:
        logger.warning(f"Error loading geocoding cache: {e}")
        _geocoding_cache = {}


def save_geocoding_cache():
    """Save geocoding cache to disk atomically"""
    if not atomic_json_write(GEOCODE_CACHE_FILE, _geocoding_cache):
        logger.warning(f"Error saving geocoding cache")


# Load cache on module import
load_geocoding_cache()


def reverse_geocode(lat, lon):
    """Reverse geocode coordinates to get location name (city, state)

    Uses OpenStreetMap Nominatim API with caching and rate limiting.
    Gracefully handles offline/error cases.

    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees

    Returns:
        String like "Ypsilanti, MI" or None if unavailable
    """
    global _last_geocode_time
    import time

    # Round coordinates to reduce cache size (0.01 degree = ~1km resolution)
    cache_key = f"{lat:.2f}_{lon:.2f}"

    # Check cache first
    if cache_key in _geocoding_cache:
        return _geocoding_cache[cache_key]

    # Rate limiting for Nominatim API (max 1 req/sec)
    current_time = time.monotonic()
    time_since_last = current_time - _last_geocode_time
    if time_since_last < GEOCODE_RATE_LIMIT:
        time.sleep(GEOCODE_RATE_LIMIT - time_since_last)

    try:
        import urllib.request
        import urllib.error

        # Nominatim API (free, no API key required)
        # User-Agent is required by Nominatim usage policy
        url = f"https://nominatim.openstreetmap.org/reverse?lat={lat}&lon={lon}&format=json&zoom=10"
        headers = {'User-Agent': 'BluePilot/1.0 (openpilot route viewer)'}

        req = urllib.request.Request(url, headers=headers)

        with urllib.request.urlopen(req, timeout=5) as response:
            data = json.loads(response.read().decode())
            _last_geocode_time = time.monotonic()

            # Extract city/town/village and state
            address = data.get('address', {})
            city = address.get('city') or address.get('town') or address.get('village') or address.get('county')
            state = address.get('state')

            if city and state:
                # Format: "City, State" or "City, STATE" (depending on country)
                location = f"{city}, {state}"
            elif city:
                location = city
            elif state:
                location = state
            else:
                location = None

            # Cache the result (even if None) and save to disk
            _geocoding_cache[cache_key] = location
            save_geocoding_cache()
            return location

    except (urllib.error.URLError, urllib.error.HTTPError, ConnectionError, TimeoutError) as e:
        # Network error (offline, no internet, timeout)
        logger.debug(f"Geocoding failed (offline/timeout): {e}")
        _geocoding_cache[cache_key] = None
        return None
    except Exception as e:
        # Other errors (JSON parsing, etc.)
        logger.warning(f"Geocoding error for {lat},{lon}: {e}")
        _geocoding_cache[cache_key] = None
        return None


# comma four (u-blox) logs gpsLocationExternal; qcom devices log gpsLocation
GPS_SERVICES = ('gpsLocationExternal', 'gpsLocation')


def extract_gps_metrics_from_segment(segment_path):
    """Extract GPS metrics from a single segment's qlog

    Args:
        segment_path: Path to segment directory

    Returns:
        dict with distance_meters, avg_speed_ms, max_speed_ms, gps_count
        Returns None if no GPS data or error
    """
    try:
        # Import here to avoid issues if tools not available
        from openpilot.tools.lib.logreader import LogReader
    except ImportError:
        logger.warning("LogReader not available - GPS metrics disabled")
        return None

    # Try qlog first (7.8x faster), fallback to rlog
    log_path = None
    for filename in ['qlog.zst', 'qlog.bz2', 'rlog.zst', 'rlog.bz2']:
        candidate = os.path.join(segment_path, filename)
        if os.path.exists(candidate):
            log_path = candidate
            break

    if not log_path:
        return None

    try:
        lr = LogReader(log_path)
        gps_coords = []

        # Extract GPS data only (fast parsing)
        for msg in lr:
            service = msg.which()
            if service in GPS_SERVICES:
                gps = getattr(msg, service)
                lat, lon = float(gps.latitude), float(gps.longitude)
                if lat != 0 and lon != 0:  # 0,0 = no fix
                    gps_coords.append({'lat': lat, 'lon': lon, 'speed': float(gps.speed)})

        if not gps_coords:
            return None

        # Calculate total distance
        total_distance = 0
        for i in range(1, len(gps_coords)):
            dist = haversine_distance(
                gps_coords[i-1]['lat'], gps_coords[i-1]['lon'],
                gps_coords[i]['lat'], gps_coords[i]['lon']
            )
            total_distance += dist

        # Calculate speed metrics
        speeds = [c['speed'] for c in gps_coords if c['speed'] > 0]
        avg_speed = sum(speeds) / len(speeds) if speeds else 0
        max_speed = max(speeds) if speeds else 0

        # Reduce coordinate precision for smaller payload (6 decimals = ~0.1m accuracy)
        coordinates_reduced = [
            {
                'lat': round(c['lat'], 6),
                'lon': round(c['lon'], 6),
                'speed': round(c['speed'], 1)
            }
            for c in gps_coords
        ]

        return {
            'distance_meters': total_distance,
            'avg_speed_ms': avg_speed,
            'max_speed_ms': max_speed,
            'gps_count': len(gps_coords),
            'coordinates': coordinates_reduced
        }

    except Exception as e:
        logger.debug(f"Error extracting GPS metrics from {segment_path}: {e}")
        return None


def get_route_gps_metrics(route_base, segments, include_coordinates=False):
    """Get GPS metrics for entire route with caching

    Args:
        route_base: Base route name
        segments: List of segment dictionaries with 'path' key
        include_coordinates: If True, also extract coordinate array for map overlay

    Returns:
        dict with total_distance_meters, avg_speed_ms, max_speed_ms, has_gps_data
        If include_coordinates=True, also includes 'coordinates' array
    """
    # Check cache first
    cache_file = os.path.join(METRICS_CACHE, f"{route_base}.json")
    coords_cache_file = os.path.join(METRICS_CACHE, f"{route_base}_coords.json")

    # Check if cache exists and is newer than all segments
    if os.path.exists(cache_file):
        try:
            cache_mtime = os.path.getmtime(cache_file)
            segments_mtime = max(os.path.getmtime(seg['path']) for seg in segments)

            # Use cache if it's newer than all segments
            if cache_mtime >= segments_mtime:
                with open(cache_file) as f:
                    cached_data = json.load(f)
                    logger.debug(f"Using cached GPS metrics for {route_base}")

                    # If coordinates requested, try to load them from separate cache
                    if include_coordinates and os.path.exists(coords_cache_file):
                        try:
                            with open(coords_cache_file) as cf:
                                cached_data['coordinates'] = json.load(cf)
                        except Exception as e:
                            logger.debug(f"Error loading coordinates cache: {e}")

                    return cached_data
        except Exception as e:
            logger.debug(f"Error reading cache for {route_base}: {e}")

    # Extract metrics from all segments
    total_distance = 0
    all_speeds = []
    max_speed_overall = 0
    gps_segments = 0
    all_coordinates = [] if include_coordinates else None

    for seg in segments:
        metrics = extract_gps_metrics_from_segment(seg['path'])
        if metrics:
            total_distance += metrics['distance_meters']
            if metrics['avg_speed_ms'] > 0:
                all_speeds.append(metrics['avg_speed_ms'])
            if metrics['max_speed_ms'] > max_speed_overall:
                max_speed_overall = metrics['max_speed_ms']
            gps_segments += 1

            # Collect coordinates if requested
            if include_coordinates and 'coordinates' in metrics:
                all_coordinates.extend(metrics['coordinates'])

    # Calculate overall metrics
    has_gps = gps_segments > 0
    avg_speed = sum(all_speeds) / len(all_speeds) if all_speeds else 0

    result = {
        'total_distance_meters': total_distance,
        'avg_speed_ms': avg_speed,
        'max_speed_ms': max_speed_overall,
        'has_gps_data': has_gps
    }

    # Save to cache atomically
    if atomic_json_write(cache_file, result):
        logger.debug(f"Cached GPS metrics for {route_base}")
    else:
        logger.debug(f"Error caching GPS metrics for {route_base}")

    # Save coordinates to separate cache file if requested (atomically)
    if include_coordinates and all_coordinates:
        result['coordinates'] = all_coordinates
        if atomic_json_write(coords_cache_file, all_coordinates):
            logger.debug(f"Cached GPS coordinates for {route_base}")
        else:
            logger.debug(f"Error caching GPS coordinates for {route_base}")

    return result


def generate_thumbnail(route_base):
    """Generate thumbnail for a route by extracting first frame from fcamera.hevc

    Args:
        route_base: Base route name (e.g., '2024-09-18--14-30-00')

    Returns:
        Path to generated thumbnail, or None if generation failed
    """
    thumbnail_path = os.path.join(THUMBNAIL_CACHE, f"{route_base}.jpg")

    # Check if thumbnail already exists
    if os.path.exists(thumbnail_path):
        return thumbnail_path

    # Find segment 0 for this route
    segment_0_path = os.path.join(ROUTES_DIR, f"{route_base}--0")
    if not os.path.exists(segment_0_path):
        logger.warning(f"Segment 0 not found for route: {route_base}")
        return None

    # Look for fcamera.hevc
    video_path = os.path.join(segment_0_path, "fcamera.hevc")
    if not os.path.exists(video_path):
        logger.warning(f"fcamera.hevc not found for route: {route_base}")
        return None

    logger.info(f"Generating thumbnail for route: {route_base}")

    # Extract frame 5-10 frames into the video using FFmpeg
    # At 20fps, 0.33 seconds = ~6-7 frames
    # Size: 480x270 (16:9 ratio) to match old Qt panel
    cmd = [
        'ffmpeg',
        '-y',  # Overwrite output file
        '-nostdin',  # Disable interaction
        '-i', video_path,  # Input file
        '-ss', '0.33',  # Seek to 0.33 seconds (5-10 frames at 20fps)
        '-vframes', '1',  # Extract one frame
        '-an',  # Disable audio
        '-vf', 'scale=480:270,format=yuvj420p',  # Scale to thumbnail size and convert to JPEG color space
        '-q:v', '2',  # High quality JPEG
        '-update', '1',  # Write single image (not sequence)
        thumbnail_path
    ]

    try:
        # Run FFmpeg with 15 second timeout (same as Qt version)
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=15,
            check=False
        )

        if result.returncode == 0 and os.path.exists(thumbnail_path):
            logger.info(f"Successfully generated thumbnail: {thumbnail_path}")
            return thumbnail_path
        else:
            logger.warning(f"FFmpeg failed to generate thumbnail for {route_base}. Exit code: {result.returncode}")
            if result.stderr:
                logger.debug(f"FFmpeg stderr: {result.stderr.decode('utf-8', errors='ignore')}")
            return None

    except subprocess.TimeoutExpired:
        logger.warning(f"FFmpeg timeout while generating thumbnail for {route_base}")
        return None
    except Exception as e:
        logger.error(f"Error generating thumbnail for {route_base}: {e}")
        return None


def _segment_log(segment_path):
    """rlog when present (has every message), else qlog; None when the segment has no log."""
    for filename in ('rlog.zst', 'rlog.bz2', 'qlog.zst', 'qlog.bz2'):
        candidate = os.path.join(segment_path, filename)
        if os.path.exists(candidate):
            return candidate
    return None


FINGERPRINT_SOURCES = ('can', 'fw', 'fixed')


def _firmware_display(fw_bytes):
    """Printable firmware (Ford part numbers) as text, up to 32 chars; anything else as hex of 16 bytes."""
    try:
        text = fw_bytes.decode('ascii').strip('\x00')
    except UnicodeDecodeError:
        text = ''
    if text and all(c.isprintable() for c in text):
        return text[:32]
    return fw_bytes[:16].hex()


def _fingerprint_info(cp):
    """CarParams (live or logged) -> the portal's fingerprint dict."""
    source = str(cp.fingerprintSource)
    return {
        'carFingerprint': str(cp.carFingerprint),
        'carVin': str(cp.carVin),
        'brand': str(cp.brand),
        'fingerprintSource': source if source in FINGERPRINT_SOURCES else None,
        'fuzzyFingerprint': bool(cp.fuzzyFingerprint),
        'firmwareVersions': [{'ecu': str(fw.ecu), 'version': _firmware_display(bytes(fw.fwVersion))} for fw in cp.carFw],
    }


def _cached_car_params(segment_path):
    """The live CarParams from Params when this segment belongs to the current route, else None."""
    try:
        from opendbc.car.structs import car
        import openpilot.cereal.messaging as messaging
        from openpilot.common.params import Params

        params = Params()
        current_route = params.get("CurrentRoute")
        if current_route and current_route in segment_path:
            car_params_bytes = params.get("CarParams", block=False)
            if car_params_bytes:
                logger.debug("Using cached CarParams from Params")
                return messaging.log_from_bytes(car_params_bytes, car.CarParams)
    except Exception as e:
        logger.debug(f"Could not use Params method: {e}")
    return None


def extract_fingerprint_from_segment(segment_path):
    """Extract vehicle fingerprint data from a single segment's log

    Args:
        segment_path: Path to segment directory

    Returns:
        dict with fingerprint data or None if not found:
        - carFingerprint: Platform string (e.g., "FORD_ESCAPE_MK4")
        - carVin: Vehicle VIN
        - brand: Brand name (e.g., "ford")
        - fingerprintSource: Source of fingerprint (can/fw/fixed)
        - fuzzyFingerprint: True if not exact match
        - firmwareVersions: List of all ECU firmware versions
    """
    # Current route: the live CarParams (works on device without pycapnp)
    cp = _cached_car_params(segment_path)
    if cp is not None:
        return _fingerprint_info(cp)

    # Otherwise the carParams message in the log (LogReader needs pycapnp)
    try:
        from openpilot.tools.lib.logreader import LogReader
    except ImportError:
        logger.debug("LogReader not available - fingerprint extraction disabled")
        return None
    log_path = _segment_log(segment_path)
    if not log_path:
        return None
    try:
        for msg in LogReader(log_path):
            if msg.which() == 'carParams':
                return _fingerprint_info(msg.carParams)
        return None
    except Exception as e:
        logger.debug(f"Error extracting fingerprint from {segment_path}: {e}")
        return None


def get_route_fingerprint(route_base, segments):
    """Get vehicle fingerprint for a route with caching

    Args:
        route_base: Base route name
        segments: List of segment dictionaries with 'path' key

    Returns:
        dict with fingerprint data or None if not found
    """
    # Check cache first
    cache_file = os.path.join(FINGERPRINT_CACHE, f"{route_base}.json")

    # Check if cache exists and is newer than first segment
    if os.path.exists(cache_file) and segments:
        try:
            cache_mtime = os.path.getmtime(cache_file)
            segment_mtime = os.path.getmtime(segments[0]['path'])

            # Use cache if it's newer than first segment
            if cache_mtime >= segment_mtime:
                with open(cache_file) as f:
                    cached_data = json.load(f)
                    # logger.debug(f"Using cached fingerprint for {route_base}")
                    return cached_data
        except Exception as e:
            logger.debug(f"Error reading fingerprint cache for {route_base}: {e}")

    # Extract from first segment (carParams is in all segments, but usually segment 0)
    if not segments:
        return None

    fingerprint_data = extract_fingerprint_from_segment(segments[0]['path'])

    # If not found in first segment, try second segment
    if not fingerprint_data and len(segments) > 1:
        fingerprint_data = extract_fingerprint_from_segment(segments[1]['path'])

    # Save to cache atomically
    if fingerprint_data:
        if atomic_json_write(cache_file, fingerprint_data):
            logger.debug(f"Cached fingerprint for {route_base}")
        else:
            logger.debug(f"Error caching fingerprint for {route_base}")

    return fingerprint_data


MPS_TO_MPH = 2.23694
METERS_PER_MILE = 1609.34
DISENGAGE_REASONS = ('userDisable', 'immediateDisable', 'softDisable')
SAFETY_EVENTS = ('fcw', 'aeb', 'stockFcw', 'stockAeb')
ENGAGED_STATES = ('enabled', 'overriding')


class DriveStats:
    """Drive statistics accumulated over a route's log messages.

    Call start_segment() before each segment's messages, handle() for every message, and
    result() at the end. State that spans segments (time, speed, engagement, active events)
    carries over; per-segment state (control loop timing) resets in start_segment().
    """

    def __init__(self):
        self.duration = 0.0
        self.distance_m = 0.0
        self.speed_sum_mph = 0.0
        self.speed_count = 0
        self.max_speed_mph = 0.0
        self.engaged_time = 0.0
        self.state_times = {'disabled': 0.0, 'preEnabled': 0.0, 'enabled': 0.0, 'softDisabling': 0.0, 'overriding': 0.0}
        self.disengagements = []
        self.alerts = []
        self.lane_changes = 0
        self.safety_events = dict.fromkeys(SAFETY_EVENTS, 0)
        self.steering_saturated = 0
        self.lag_sum_ms = 0.0
        self.lag_count = 0
        self.lag_max_ms = 0.0

        self._prev_time = None
        self._state = None          # selfdriveState.state
        self._mads_active = False   # MADS steering-only engagement (selfdriveStateSP)
        self._prev_car_state = None  # (time, vEgo) of the previous carState
        self._event_names = set()   # onroadEvents is republished while events stay active
        self._prev_controls_time = None
        self._full_rate_log = True
        self._handlers = {
            'carState': self._on_car_state,
            'selfdriveState': self._on_selfdrive_state,
            'selfdriveStateSP': self._on_selfdrive_state_sp,
            'controlsState': self._on_controls_state,
            'onroadEvents': self._on_onroad_events,
        }

    def start_segment(self, full_rate_log):
        """qlogs decimate controlsState, so loop timing is only measured from rlogs."""
        self._full_rate_log = full_rate_log
        self._prev_controls_time = None

    def handle(self, msg):
        t = msg.logMonoTime / 1e9
        if self._prev_time is not None and 0 < t - self._prev_time < 1.0:
            dt = t - self._prev_time
            self.duration += dt
            if self._state is not None:
                self.state_times[self._state] = self.state_times.get(self._state, 0.0) + dt
                if self._state in ENGAGED_STATES or self._mads_active:
                    self.engaged_time += dt
        handler = self._handlers.get(msg.which())
        if handler is not None:
            handler(msg, t)
        self._prev_time = t

    def _on_car_state(self, msg, t):
        v = float(msg.carState.vEgo)
        speed_mph = v * MPS_TO_MPH
        self.max_speed_mph = max(self.max_speed_mph, speed_mph)
        self.speed_sum_mph += speed_mph
        self.speed_count += 1
        if self._prev_car_state is not None:
            t_prev, v_prev = self._prev_car_state
            if 0 < t - t_prev < 1.0:
                self.distance_m += 0.5 * (v + v_prev) * (t - t_prev)
        self._prev_car_state = (t, v)

    def _on_selfdrive_state(self, msg, t):
        self._state = str(msg.selfdriveState.state)

    def _on_selfdrive_state_sp(self, msg, t):
        self._mads_active = bool(msg.selfdriveStateSP.mads.active)

    def _on_controls_state(self, msg, t):
        # lateralControlState is a union (angleState/pidState/torqueState/...): read the active member
        lat = msg.controlsState.lateralControlState
        if getattr(lat, lat.which()).saturated:
            self.steering_saturated += 1
        # cumLagMs is gone upstream: measure how late each 100 Hz controlsState is past its 10 ms period
        if self._full_rate_log and self._prev_controls_time is not None and 0 < t - self._prev_controls_time < 1.0:
            lag_ms = max(0.0, (t - self._prev_controls_time - 0.01) * 1000.0)
            self.lag_sum_ms += lag_ms
            self.lag_count += 1
            self.lag_max_ms = max(self.lag_max_ms, lag_ms)
        self._prev_controls_time = t

    def _on_onroad_events(self, msg, t):
        names_now = set()
        for event in msg.onroadEvents:
            name = str(event.name)
            names_now.add(name)
            if name not in self._event_names:  # count each event once, when it first appears
                self._on_new_event(event, name, t)
        self._event_names = names_now

    def _on_new_event(self, event, name, t):
        reason = next((r for r in DISENGAGE_REASONS if getattr(event, r)), None)
        if reason is not None:
            self.disengagements.append({'time': t, 'type': name, 'reason': reason})
        if event.warning:
            self.alerts.append({'time': t, 'type': name, 'level': 'warning'})
        if name in self.safety_events:
            self.safety_events[name] += 1
        if name == 'laneChange':  # each lane change also raises preLaneChange*
            self.lane_changes += 1

    def result(self):
        hours, rem = divmod(int(self.duration), 3600)
        minutes, seconds = divmod(rem, 60)
        avg_lag = self.lag_sum_ms / self.lag_count if self.lag_count else 0.0
        return {
            'duration': self.duration,
            'durationStr': f"{hours}h {minutes}m {seconds}s" if hours else f"{minutes}m {seconds}s",
            'distance': round(self.distance_m / METERS_PER_MILE, 2),
            'avgSpeed': round(self.speed_sum_mph / self.speed_count, 1) if self.speed_count else 0.0,
            'maxSpeed': round(self.max_speed_mph, 1),
            'opEngagedTime': round(self.engaged_time, 1),
            'opEngagedPercent': round(self.engaged_time / self.duration * 100, 1) if self.duration > 0 else 0.0,
            'opStateTimes': {k: round(v, 1) for k, v in self.state_times.items()},
            'disengagements': self.disengagements,
            'disengagementCount': len(self.disengagements),
            'alerts': self.alerts,
            'alertCount': len(self.alerts),
            'laneChanges': self.lane_changes,
            'safetyEvents': self.safety_events,
            'controlMetrics': {
                'steeringSaturated': self.steering_saturated,
                'avgControlLag': round(avg_lag, 2),
                'maxControlLag': round(self.lag_max_ms, 2),
            },
        }


def _read_messages_manual(log_path):
    """Yield log messages by decompressing by hand (for devices without LogReader)."""
    import bz2
    import struct
    import openpilot.cereal.messaging as messaging

    try:
        if log_path.endswith('.zst'):
            try:
                import zstandard as zstd
                with open(log_path, 'rb') as f, zstd.ZstdDecompressor().stream_reader(f) as reader:
                    data = reader.read()
            except ImportError:
                data = subprocess.run(['zstd', '-d', '-c', log_path], capture_output=True, check=True).stdout
        elif log_path.endswith('.bz2'):
            with bz2.open(log_path, 'rb') as f:
                data = f.read()
        else:
            return
    except Exception as e:
        logger.debug(f"Error decompressing {log_path}: {e}")
        return

    # Each message is: 4 bytes length + message data
    offset = 0
    while offset + 4 <= len(data):
        msg_len = struct.unpack('<I', data[offset:offset + 4])[0]
        offset += 4
        if offset + msg_len > len(data) or msg_len > 10_000_000:  # sanity check
            break
        msg_data = data[offset:offset + msg_len]
        offset += msg_len
        try:
            yield messaging.log_from_bytes(msg_data)
        except Exception as e:
            logger.debug(f"Error deserializing message: {e}")


def _message_reader():
    """The best available log reader: LogReader, else manual decompression, else None."""
    try:
        from openpilot.tools.lib.logreader import LogReader
        return LogReader
    except ImportError as e:
        logger.warning(f"✗ LogReader not available: {e}; trying manual method")
    try:
        import openpilot.cereal.messaging  # noqa: F401
        return _read_messages_manual
    except ImportError as e:
        logger.debug(f"cereal.messaging not available, drive stats disabled: {e}")
        return None


def extract_drive_stats_from_segments(segments):
    """Extract comprehensive drive statistics from route segments

    Args:
        segments: List of segment dictionaries with 'path' key

    Returns:
        dict with drive statistics (see DriveStats.result()) or None if extraction fails
    """
    if not segments:
        return None
    reader = _message_reader()
    if reader is None:
        return None

    stats = DriveStats()
    try:
        for segment in segments:
            log_path = _segment_log(segment['path'])
            if not log_path:
                logger.debug(f"No log file found in {segment['path']}")
                continue
            stats.start_segment(full_rate_log=os.path.basename(log_path).startswith('rlog'))
            try:
                for msg in reader(log_path):
                    stats.handle(msg)
            except Exception as e:
                logger.debug(f"Error reading log {log_path}: {e}")

        result = stats.result()
        logger.info(f"✓ Extracted drive stats: {result['durationStr']}, {result['opEngagedPercent']}% engaged, "
                    f"{result['disengagementCount']} disengagements")
        return result
    except Exception as e:
        logger.error(f"✗ Error extracting drive stats: {e}", exc_info=True)
        return None


def get_route_drive_stats(route_base, segments):
    """Get drive statistics for a route with caching

    Args:
        route_base: Base route name
        segments: List of segment dictionaries with 'path' key

    Returns:
        dict with drive statistics or None if not found
    """
    logger.info(f"get_route_drive_stats called for {route_base} ({len(segments)} segments)")

    # Check cache first
    cache_file = os.path.join(DRIVE_STATS_CACHE, f"{route_base}.json")

    # Check if cache exists and is newer than first segment
    if os.path.exists(cache_file) and segments:
        try:
            cache_mtime = os.path.getmtime(cache_file)
            segment_mtime = os.path.getmtime(segments[0]['path'])

            # Use cache if it's newer than first segment
            if cache_mtime >= segment_mtime:
                with open(cache_file) as f:
                    cached_data = json.load(f)
                    logger.info(f"✓ Using cached drive stats for {route_base}")
                    return cached_data
        except Exception as e:
            logger.warning(f"Error reading drive stats cache for {route_base}: {e}")

    # Extract from all segments
    if not segments:
        logger.warning(f"No segments provided for {route_base}")
        return None

    logger.info(f"Extracting drive stats from {len(segments)} segments for {route_base}...")
    drive_stats = extract_drive_stats_from_segments(segments)

    # Save to cache atomically
    if drive_stats:
        if atomic_json_write(cache_file, drive_stats):
            logger.info(f"✓ Cached drive stats for {route_base}")
        else:
            logger.warning(f"✗ Error caching drive stats for {route_base}")
    else:
        logger.warning(f"✗ Failed to extract drive stats for {route_base}")

    return drive_stats


def get_route_drive_stats_cached_only(route_base):
    """Get drive statistics from cache only (no extraction)

    This is used for fast route listing where we don't want to block
    on log extraction. Returns None if not cached.

    Args:
        route_base: Base route name

    Returns:
        dict with drive statistics or None if not cached
    """
    cache_file = os.path.join(DRIVE_STATS_CACHE, f"{route_base}.json")

    if os.path.exists(cache_file):
        try:
            with open(cache_file) as f:
                cached_data = json.load(f)
                # logger.debug(f"Using cached drive stats for {route_base}")
                return cached_data
        except Exception as e:
            logger.debug(f"Error reading drive stats cache for {route_base}: {e}")

    return None


def check_processing_status(route_base):
    """Check what has already been processed for this route

    This allows resuming from partial processing if interrupted.

    Returns:
        dict with boolean flags for each processing step:
        - gps_metrics: GPS metrics extracted and cached
        - coordinates: GPS coordinates extracted and cached
        - geocoding_checked: Geocoding was attempted (even if failed)
        - thumbnail: Thumbnail generated
        - fingerprint: Vehicle fingerprint extracted and cached
        - drive_stats: Drive statistics extracted and cached
    """
    status = {
        'gps_metrics': False,
        'coordinates': False,
        'geocoding_checked': False,
        'thumbnail': False,
        'fingerprint': False,
        'drive_stats': False
    }

    # Check for GPS metrics cache
    gps_cache = os.path.join(METRICS_CACHE, f"{route_base}.json")
    if os.path.exists(gps_cache):
        status['gps_metrics'] = True

    # Check for coordinates cache
    coords_cache = os.path.join(METRICS_CACHE, f"{route_base}_coords.json")
    if os.path.exists(coords_cache):
        status['coordinates'] = True

    # Check for geocoding - if either GPS metrics or coordinates exist,
    # we consider geocoding as "checked" since it's done inline during GPS extraction
    # For more granular control, we could add a separate marker file
    if status['gps_metrics'] or status['coordinates']:
        status['geocoding_checked'] = True

    # Check for thumbnail
    thumbnail_path = os.path.join(THUMBNAIL_CACHE, f"{route_base}.jpg")
    if os.path.exists(thumbnail_path):
        status['thumbnail'] = True

    # Check for fingerprint cache
    fingerprint_cache = os.path.join(FINGERPRINT_CACHE, f"{route_base}.json")
    if os.path.exists(fingerprint_cache):
        status['fingerprint'] = True

    # Check for drive stats cache
    drive_stats_cache = os.path.join(DRIVE_STATS_CACHE, f"{route_base}.json")
    if os.path.exists(drive_stats_cache):
        status['drive_stats'] = True

    return status


def process_route(route_base, segments, check_idle_fn=None):
    """Process a single route with interruption recovery

    Performs GPS extraction, geocoding, and thumbnail generation.
    Can be interrupted and will resume from where it left off.

    Args:
        route_base: Base route name (e.g., '2024-09-18--14-30-00')
        segments: List of segment dictionaries with 'path' key
        check_idle_fn: Optional callback function that returns False if processing should stop
                      Used by background preprocessor to check if device is still idle

    Returns:
        True if fully completed, False if interrupted
    """
    try:
        logger.info(f"Processing route: {route_base}")

        # Check what's already been processed
        status = check_processing_status(route_base)

        # Step 1: Extract GPS metrics (includes coordinates for geocoding)
        if not status['gps_metrics'] or not status['coordinates']:
            # Check for interruption before expensive operation
            if check_idle_fn and not check_idle_fn():
                logger.info("  Interrupted before GPS extraction (no longer idle)")
                return False

            logger.debug("  Extracting GPS metrics...")
            gps_metrics = get_route_gps_metrics(route_base, segments, include_coordinates=True)

            if not gps_metrics.get('has_gps_data'):
                logger.debug(f"  No GPS data for {route_base}")
                return True  # Not an error, just no GPS
        else:
            logger.debug("  GPS metrics already cached, skipping")
            gps_metrics = get_route_gps_metrics(route_base, segments, include_coordinates=True)

        # Step 2: Geocode start and end locations
        if not status['geocoding_checked']:
            coordinates = gps_metrics.get('coordinates', [])
            if coordinates:
                # Check for interruption before geocoding
                if check_idle_fn and not check_idle_fn():
                    logger.info("  Interrupted before geocoding (no longer idle)")
                    return False

                logger.debug("  Geocoding locations...")
                start_coord = coordinates[0]
                end_coord = coordinates[-1]

                start_location = reverse_geocode(start_coord['lat'], start_coord['lon'])
                end_location = reverse_geocode(end_coord['lat'], end_coord['lon'])

                if start_location:
                    logger.debug(f"    Start: {start_location}")
                if end_location:
                    logger.debug(f"    End: {end_location}")

                # Save location names to GPS metrics cache atomically
                cache_file = os.path.join(METRICS_CACHE, f"{route_base}.json")
                try:
                    # Reload existing cache and add location data
                    if os.path.exists(cache_file):
                        with open(cache_file, 'r') as f:
                            cached_data = json.load(f)
                    else:
                        cached_data = {}

                    cached_data['start_location'] = start_location
                    cached_data['end_location'] = end_location

                    # Write back to cache atomically
                    if atomic_json_write(cache_file, cached_data):
                        logger.debug("  Saved location names to cache")
                    else:
                        logger.warning("  Error saving location names to cache")
                except Exception as e:
                    logger.warning(f"Error saving location names to cache: {e}", exc_info=True)
        else:
            logger.debug("  Geocoding already attempted, skipping")

        # Step 3: Generate thumbnail
        if not status['thumbnail']:
            # Check for interruption before thumbnail generation
            if check_idle_fn and not check_idle_fn():
                logger.info("  Interrupted before thumbnail (no longer idle)")
                return False

            logger.debug("  Generating thumbnail...")
            thumbnail_path = generate_thumbnail(route_base)
            if thumbnail_path:
                logger.debug(f"    Thumbnail: {os.path.basename(thumbnail_path)}")
        else:
            logger.debug("  Thumbnail already cached, skipping")

        # Step 4: Extract fingerprint data
        if not status['fingerprint']:
            # Check for interruption before fingerprint extraction
            if check_idle_fn and not check_idle_fn():
                logger.info("  Interrupted before fingerprint extraction (no longer idle)")
                return False

            logger.debug("  Extracting fingerprint data...")
            fingerprint_data = get_route_fingerprint(route_base, segments)
            if fingerprint_data:
                logger.debug(f"    Platform: {fingerprint_data.get('carFingerprint', 'Unknown')}")
                if fingerprint_data.get('carVin'):
                    logger.debug(f"    VIN: {fingerprint_data.get('carVin')}")
        else:
            logger.debug("  Fingerprint already cached, skipping")

        # Step 5: Extract drive statistics
        if not status['drive_stats']:
            # Check for interruption before drive stats extraction
            if check_idle_fn and not check_idle_fn():
                logger.info("  Interrupted before drive stats extraction (no longer idle)")
                return False

            logger.debug("  Extracting drive statistics...")
            drive_stats = get_route_drive_stats(route_base, segments)
            if drive_stats:
                logger.debug(f"    Distance: {drive_stats.get('distance', 0)} mi, Duration: {drive_stats.get('durationStr', 'N/A')}")
                logger.debug(f"    OP Engaged: {drive_stats.get('opEngagedPercent', 0)}%, Disengagements: {drive_stats.get('disengagementCount', 0)}")
        else:
            logger.debug("  Drive stats already cached, skipping")

        logger.info(f"✓ Fully processed {route_base}")
        return True

    except Exception as e:
        logger.error(f"Error processing route {route_base}: {e}", exc_info=True)
        return False


def kill_existing_process(script_name):
    """Kill any existing instances of the specified Python script

    Args:
        script_name: Name of the Python script (e.g., 'web_routes_server.py', 'route_preprocessor.py')
    """
    try:
        import signal

        # Try to use psutil for accurate process detection
        try:
            import psutil

            current_pid = os.getpid()
            killed = False

            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    # Check if it's a Python process running the target script
                    if proc.info['cmdline'] and script_name in ' '.join(proc.info['cmdline']):
                        # Don't kill ourselves
                        if proc.info['pid'] != current_pid:
                            logger.info(f"Killing existing {script_name} instance (PID {proc.info['pid']})")
                            proc.send_signal(signal.SIGTERM)
                            try:
                                proc.wait(timeout=3)  # Wait up to 3 seconds for graceful shutdown
                            except psutil.TimeoutExpired:
                                logger.warning(f"Process {proc.info['pid']} didn't terminate, sending SIGKILL")
                                proc.send_signal(signal.SIGKILL)
                            killed = True
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

            if killed:
                import time
                time.sleep(1)  # Give the OS time to clean up

        except ImportError:
            # psutil not available, fall back to pkill
            logger.debug(f"psutil not available, using pkill fallback for {script_name}")
            try:
                # Use pkill with exact match to avoid killing ourselves during startup
                # The -o flag ensures we only kill older processes
                result = subprocess.run(
                    ['pkill', '-o', '-f', f'python.*{script_name}'],
                    capture_output=True,
                    timeout=2
                )
                if result.returncode == 0:
                    logger.info(f"Killed existing {script_name} instances using pkill")
                    import time
                    time.sleep(1)
                elif result.returncode == 1:
                    logger.debug(f"No existing {script_name} processes found")
                else:
                    logger.debug(f"pkill returned {result.returncode}")
            except subprocess.TimeoutExpired:
                logger.warning(f"pkill command timed out for {script_name}")
            except Exception as e:
                logger.debug(f"Could not kill existing {script_name} instances: {e}")

    except Exception as e:
        logger.warning(f"Error killing existing {script_name} instances: {e}")

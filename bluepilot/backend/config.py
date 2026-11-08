#!/usr/bin/env python3
"""
BluePilot Backend Configuration
Centralized configuration for the backend server
"""

import os
import sys
import shutil
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

# Directories
ROUTES_DIR = "/data/media/0/realdata" if os.path.exists("/data/media/0/realdata") else os.path.expanduser("~/comma_data/media/0/realdata")
WEBAPP_DIR = Path(__file__).parent.parent / "web" / "public"

# Cache directories
DATA_ROOT = "/data/bluepilot/routes" if os.path.exists("/data") else os.path.expanduser("~/comma_data/bluepilot/routes")
THUMBNAIL_CACHE = os.path.join(DATA_ROOT, "thumbs_cache")
REMUX_CACHE = os.path.join(DATA_ROOT, "remux_cache")
METRICS_CACHE = os.path.join(DATA_ROOT, "metrics_cache")
ROUTE_EXPORT_CACHE = os.path.join(DATA_ROOT, "exports")
VIDEOS_ZIP_CACHE = os.path.join(DATA_ROOT, "videos_zip")
BACKUP_CACHE = os.path.join(DATA_ROOT, "backups")
IMPORT_TEMP_DIR = os.path.join(DATA_ROOT, "import_temp")

# User preferences
BLUEPILOT_DATA_DIR = "/data/bluepilot" if os.path.exists("/data") else os.path.expanduser("~/comma_data/bluepilot")
FAVORITE_SETTINGS_FILE = os.path.join(BLUEPILOT_DATA_DIR, "favorite_settings.json")

# Ensure all cache directories exist
for cache_dir in [DATA_ROOT, THUMBNAIL_CACHE, REMUX_CACHE, METRICS_CACHE,
                  ROUTE_EXPORT_CACHE, VIDEOS_ZIP_CACHE, BACKUP_CACHE, IMPORT_TEMP_DIR,
                  BLUEPILOT_DATA_DIR]:
    os.makedirs(cache_dir, exist_ok=True)

# Camera configuration
CAMERA_FILES = {
    'front': 'fcamera.hevc',
    'wide': 'ecamera.hevc',
    'driver': 'dcamera.hevc',
    'lq': 'qcamera.ts'
}
HEVC_CAMERAS = {'front', 'wide', 'driver'}
CAMERA_LABELS = {
    'front': 'front',
    'wide': 'wide',
    'driver': 'driver',
    'lq': 'interior'
}

# WebSocket configuration
WEBSOCKET_HOST = '0.0.0.0'
WEBSOCKET_PORT = 8089  # Different port from HTTP server

# Power management
IDLE_TIMEOUT_SECONDS = 300  # 5 minutes of no remuxing = idle

# FFmpeg settings
MAX_CONCURRENT_FFMPEG = 3  # Max concurrent FFmpeg processes
FFMPEG_RESERVED_FOR_PLAYBACK = 1  # Always keep 1 slot free for playback

# Server configuration
DEFAULT_PORT = 8088

# Rate limiting (per-second for burst protection)
RATE_LIMIT_WINDOW_SECONDS = 1  # 1 second window for smoother rate limiting
RATE_LIMIT_REQUESTS_PER_SECOND_OFFROAD = 30  # 30 req/s when parked - allows fast settings loading
RATE_LIMIT_REQUESTS_PER_SECOND_ONROAD = 30   # 30 req/s when driving - allows settings management

# FFmpeg binary detection
def _find_binary(name):
    """PATH first, then the usual install locations. On the comma the venv's ffmpeg/ffprobe (pip
    wrappers around bundled binaries) aren't on the manager's PATH, so which() misses them."""
    path = shutil.which(name)
    if path:
        logger.info(f"Found {name} in PATH: {path}")
        return path
    for path in (f'/usr/bin/{name}', f'/usr/local/bin/{name}',
                 os.path.join(os.path.dirname(sys.executable), name), f'/usr/local/venv/bin/{name}',
                 f'/data/openpilot/third_party/{name}', f'/opt/homebrew/bin/{name}'):
        if os.path.exists(path) and os.access(path, os.X_OK):
            logger.info(f"Found {name} at fallback path: {path}")
            return path
    return None


def find_ffmpeg_binary():
    path = _find_binary('ffmpeg')
    if path is None:
        logger.error("FFmpeg binary not found! Video playback will not work. Install with: apt-get install ffmpeg")
    return path


def find_ffprobe_binary():
    path = _find_binary('ffprobe')
    if path is None:
        logger.warning("ffprobe not found - HLS playlists will use estimated durations")
    return path


# Initialize FFmpeg binaries
FFMPEG_BINARY = find_ffmpeg_binary()
FFPROBE_BINARY = find_ffprobe_binary()

"""GPS route metrics (routes/processing.py) read the GPS the device actually logs.

comma four (u-blox) logs gpsLocationExternal only; qcom devices log gpsLocation.
Reading just gpsLocation meant no GPS metrics, ever, on a comma four.
"""
import openpilot.cereal.messaging as messaging
import openpilot.tools.lib.logreader as logreader_mod
import pytest

from bluepilot.backend.routes import processing


def gps(service, lat, lon, speed):
  m = messaging.new_message(service)
  g = getattr(m, service)
  g.latitude, g.longitude, g.speed = lat, lon, speed
  return m.as_reader()


@pytest.fixture
def segment(tmp_path, monkeypatch):
  def make(msgs):
    (tmp_path / 'qlog.zst').write_bytes(b'')
    monkeypatch.setattr(logreader_mod, 'LogReader', lambda p: iter(msgs))
    return str(tmp_path)
  return make


@pytest.mark.parametrize('service', ['gpsLocationExternal', 'gpsLocation'])
def test_reads_either_gps_service(segment, service):
  path = segment([gps(service, 42.0, -83.0, 10.0), gps(service, 42.001, -83.0, 12.0), gps(service, 0.0, 0.0, 0.0)])
  m = processing.extract_gps_metrics_from_segment(path)
  assert m is not None and m['gps_count'] == 2                 # the 0,0 (no fix) sample is skipped
  assert m['distance_meters'] == pytest.approx(111.2, abs=0.5)  # 0.001 deg of latitude
  assert m['avg_speed_ms'] == pytest.approx(11.0) and m['max_speed_ms'] == pytest.approx(12.0)


def test_no_fix_means_no_metrics(segment):
  assert processing.extract_gps_metrics_from_segment(segment([gps('gpsLocationExternal', 0.0, 0.0, 0.0)])) is None

"""Characterization tests for the portal's fingerprint extraction (routes/processing.py).

Both sources must produce the same dict: the live CarParams cached in Params (current route)
and the carParams message in a segment's log.
"""
import openpilot.cereal.messaging as messaging
import openpilot.common.params as params_mod
import openpilot.tools.lib.logreader as logreader_mod
import pytest

from bluepilot.backend.routes import processing

FW = [
  ('eps', b'LJ8C-14D003-AE\x00\x00'),          # printable Ford part number (NULs stripped)
  ('abs', bytes(range(0, 40))),                 # binary: first 16 bytes as hex
  ('fwdCamera', b''),                           # empty
]


def fill(cp, source='fw', fuzzy=True):
  cp.carFingerprint = 'FORD_MUSTANG_MACH_E_MK1'
  cp.carVin = '3FMTK3SS9MMXXXXXX'
  cp.brand = 'ford'
  cp.fuzzyFingerprint = fuzzy
  cp.fingerprintSource = source
  fws = cp.init('carFw', len(FW))
  for f, (ecu, ver) in zip(fws, FW, strict=True):
    f.ecu = ecu
    f.fwVersion = ver
  return cp


def car_params_msg(source='fw', fuzzy=True):
  m = messaging.new_message('carParams')
  fill(m.carParams, source, fuzzy)
  return m


EXPECTED = {
  'carFingerprint': 'FORD_MUSTANG_MACH_E_MK1',
  'carVin': '3FMTK3SS9MMXXXXXX',
  'brand': 'ford',
  'fingerprintSource': 'fw',
  'fuzzyFingerprint': True,
  'firmwareVersions': [
    {'ecu': 'eps', 'version': 'LJ8C-14D003-AE'},
    {'ecu': 'abs', 'version': bytes(range(16)).hex()},
    {'ecu': 'fwdCamera', 'version': ''},
  ],
}


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key, block=False):
    return self.values.get(key)


@pytest.fixture
def segment(tmp_path, monkeypatch):
  def make(msgs, log_name='rlog.zst', params=None):
    d = tmp_path / '00000511--625a4ca2e7--0'
    d.mkdir(exist_ok=True)
    (d / log_name).write_bytes(b'')
    monkeypatch.setattr(logreader_mod, 'LogReader', lambda p: iter([m.as_reader() for m in msgs]))
    monkeypatch.setattr(params_mod, 'Params', lambda: FakeParams(params or {}))
    return str(d)
  return make


def test_from_log(segment):
  path = segment([messaging.new_message('carState'), car_params_msg()])
  assert processing.extract_fingerprint_from_segment(path) == EXPECTED


def test_from_cached_car_params_for_current_route(segment):
  from opendbc.car.structs import car
  cp_bytes = fill(car.CarParams.new_message()).to_bytes()  # how the device stores the CarParams param
  path = segment([], params={'CurrentRoute': '00000511--625a4ca2e7', 'CarParams': cp_bytes})
  assert processing.extract_fingerprint_from_segment(path) == EXPECTED


@pytest.mark.parametrize('source', ['can', 'fixed'])
def test_fingerprint_sources(segment, source):
  path = segment([car_params_msg(source=source, fuzzy=False)])
  out = processing.extract_fingerprint_from_segment(path)
  assert out['fingerprintSource'] == source and out['fuzzyFingerprint'] is False


def test_no_car_params_or_no_log(segment, tmp_path):
  assert processing.extract_fingerprint_from_segment(segment([messaging.new_message('carState')])) is None
  empty = tmp_path / 'empty'
  empty.mkdir()
  assert processing.extract_fingerprint_from_segment(str(empty)) is None

"""ParamsWatcher._read_param_value: what the portal broadcasts for each param type."""
import datetime

import pytest

from bluepilot.backend.params.params_watcher import ParamsWatcher


class FakeParams:
  VALUES = {
    'BPLongRegenBlend': True, 'IsOffroad': False, 'LongitudinalPersonality': 1, 'vbatt_pause_charging': 12.0,
    'GithubUsername': 'jchristman75', 'BPConnectBackend': 0, 'ApiCache_Device': {'a': 1},
    'DongleId': '34b9c9a331c780da', 'NumericString': '123', 'Empty': '', 'Raw': b'abc',
    'LastUpdateTime': datetime.datetime(2026, 11, 7),
  }

  def get(self, key, block=False):
    if key not in self.VALUES:
      raise KeyError(key)  # unknown param
    return self.VALUES[key]

  def get_bool(self, key, block=False):
    return bool(self.get(key))


@pytest.fixture
def watcher():
  w = ParamsWatcher.__new__(ParamsWatcher)
  w.params = FakeParams()
  return w


@pytest.mark.parametrize('key,expected', [
  ('BPLongRegenBlend', True), ('IsOffroad', False), ('LongitudinalPersonality', 1), ('vbatt_pause_charging', 12.0),
  ('GithubUsername', 'jchristman75'), ('BPConnectBackend', 0), ('ApiCache_Device', {'a': 1}),
  ('DongleId', '34b9c9a331c780da'), ('Empty', None), ('Raw', 'abc'), ('Unknown', None),
  ('LastUpdateTime', None),   # TIME values aren't broadcast (not JSON)
  ('NumericString', '123'),   # a STRING param stays a string (was parsed into int 123)
])
def test_read_param_value(watcher, key, expected):
  got = watcher._read_param_value(key)
  assert got == expected and type(got) is type(expected)

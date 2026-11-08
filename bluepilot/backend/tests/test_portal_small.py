"""Small portal helpers: ffmpeg/ffprobe lookup and hotspot detection."""
import pytest

from bluepilot.backend import config
from bluepilot.backend.network import utils as net


@pytest.fixture
def fs(monkeypatch):
  """Pretend only these paths exist (and are executable); nothing is on PATH."""
  present = set()
  monkeypatch.setattr(config.shutil, 'which', lambda name: None)
  monkeypatch.setattr(config.os.path, 'exists', lambda p: p in present)
  monkeypatch.setattr(config.os, 'access', lambda p, mode: p in present)
  return present


@pytest.mark.parametrize('finder,name', [(config.find_ffmpeg_binary, 'ffmpeg'), (config.find_ffprobe_binary, 'ffprobe')])
def test_finds_the_venv_binary_off_path(fs, finder, name):
  fs.add(f'/usr/local/venv/bin/{name}')  # where the comma has them
  assert finder() == f'/usr/local/venv/bin/{name}'


@pytest.mark.parametrize('finder,name', [(config.find_ffmpeg_binary, 'ffmpeg'), (config.find_ffprobe_binary, 'ffprobe')])
def test_prefers_path_then_system(fs, monkeypatch, finder, name):
  fs.update({f'/usr/bin/{name}', f'/usr/local/venv/bin/{name}'})
  assert finder() == f'/usr/bin/{name}'
  monkeypatch.setattr(config.shutil, 'which', lambda n: f'/opt/x/{n}')
  assert finder() == f'/opt/x/{name}'


@pytest.mark.parametrize('finder', [config.find_ffmpeg_binary, config.find_ffprobe_binary])
def test_not_found(fs, finder):
  assert finder() is None


def test_tethering_from_hotspot_address():
  assert net.is_tethering_active([('wlan0', '192.168.1.143'), ('wlan1', '192.168.43.1')])
  assert not net.is_tethering_active([('wlan0', '192.168.1.143')])
  assert not net.is_tethering_active([])

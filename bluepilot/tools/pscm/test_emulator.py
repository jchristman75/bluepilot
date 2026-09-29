"""BluePilot: sanity checks for the PSCM emulator (sign conventions, steady state, batching, mode 0)."""
import numpy as np

from bluepilot.tools.pscm.emulator import DT, PSCMEmulator, PSCMParams


def _run(emu, seconds, **kw):
  out = None
  for _ in range(int(seconds / DT)):
    out = emu.step(**kw)
  return out


def test_left_command_turns_left():
  emu = PSCMEmulator(PSCMParams.load(), n=1)
  emu.reset(0.0, 0.0, 20.0)
  o = _run(emu, 5.0, mode=1, c1=0.05, v=20.0)
  assert o['steeringAngleDeg'][0] > 1.0 and o['yawRate'][0] > 0.01


def test_steady_state_delivery_matches_droop():
  # a held C1 settles at (1 - droop_ratio) * ff_gain of the nominal yaw-rate request (the measured ~0.75-0.87)
  p = PSCMParams.load()
  for v in (10.0, 20.0, 30.0):
    emu = PSCMEmulator(p, n=1)
    emu.reset(p.angle_offset_deg, 0.0, v)
    c1 = 0.03
    o = _run(emu, 15.0, mode=1, c1=c1, v=v)
    delivered = o['yawRate'][0] / c1
    expected = np.interp(v, p.v_bp, p.ff_gain) * (1 - np.interp(v, p.v_bp, p.droop_ratio))
    assert 0.6 < delivered < 1.0, delivered
    assert abs(delivered - expected) < 0.08, (v, delivered, expected)


def test_batch_matches_single():
  p = PSCMParams.load()
  c1s, vs = np.array([0.02, -0.04, 0.08]), np.array([12.0, 22.0, 31.0])
  batch = PSCMEmulator(p, n=3)
  batch.reset(0.0, 0.0, vs)
  ob = _run(batch, 3.0, mode=1, c1=c1s, v=vs)
  for i in range(3):
    one = PSCMEmulator(p, n=1)
    one.reset(0.0, 0.0, vs[i])
    oi = _run(one, 3.0, mode=1, c1=c1s[i], v=vs[i])
    assert np.isclose(ob['steeringAngleDeg'][i], oi['steeringAngleDeg'][0])
    assert np.isclose(ob['yawRate'][i], oi['yawRate'][0])


def test_mode_zero_ignores_command_and_forcing_holds_angle():
  p = PSCMParams.load()
  emu = PSCMEmulator(p, n=1)
  emu.reset(0.0, 0.0, 15.0)
  o = _run(emu, 2.0, mode=0, c1=0.2, v=15.0)
  assert o['heldC1'][0] == 0.0
  o = _run(emu, 1.0, mode=1, c1=0.0, v=15.0, driver_angle=30.0)
  assert o['steeringAngleDeg'][0] == 30.0


def test_harness_yaw_frame_roundtrip_and_checksum():
  from bluepilot.tools.pscm.harness import _yaw_frame
  dat = bytes([0x12, 0x34, 0x7F, 0x00, 0x00, 0x05, 0xF0, 0x00])
  out = _yaw_frame(dat, 0.1234)
  raw = (out[2] << 8) | out[3]
  assert abs(raw * 0.0002 - 6.5 - 0.1234) < 1e-4
  # ford.h's Yaw_Data_FD1 checksum over the same bytes
  cs = 0xFF - ((out[0] + out[1] + out[2] + out[3] + out[5] + (out[6] >> 6) + ((out[6] >> 4) & 0x3)) & 0xFF)
  assert out[4] == cs & 0xFF
  assert out[:2] == dat[:2] and out[5:] == dat[5:]


def test_harness_params_are_typed_like_openpilot():
  from bluepilot.tools.pscm.harness import _Params
  p = _Params({'a': b'1', 'b': b'1.13', 'c': b'main_en', 'd': b'0'})
  assert p.get('a') == 1 and isinstance(p.get('a'), int)
  assert p.get('b') == 1.13 and p.get('c') == 'main_en' and p.get('missing') is None
  assert p.get_bool('a') and not p.get_bool('d') and not p.get_bool('missing')

"""Tests for LongitudinalExt: regen blend (BPLongRegenBlend) and lead detection."""
from types import SimpleNamespace

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.longitudinal_ext import Lead, LongitudinalExt, encode_gas, follow_limits

INACTIVE = CarControllerParams.INACTIVE_GAS


class FakeSM(dict):
  def __init__(self, lead=None):
    super().__init__()
    self.valid = {'radarState': lead is not None}
    self['radarState'] = SimpleNamespace(leadOne=lead)


class Ctrl(LongitudinalExt):
  def __init__(self, regen=False, lead=None, disable_bp_long=False):
    super().__init__(None, None)
    self.regen_blend_UI = regen
    self.disable_BP_long_UI = disable_bp_long
    self.sm = FakeSM(lead)


def cc(active=True):
  return SimpleNamespace(longActive=active)


def cs(v_ego=15.0):
  return SimpleNamespace(out=SimpleNamespace(vEgo=v_ego, gasPressed=False, brakePressed=False))


def step(c, accel, v_ego=15.0, stopping=False, active=True, pitch=0.0):
  # carcontroller passes the raw planner request as gas; LongitudinalExt encodes INACTIVE itself
  return c.update(cc(active), cs(v_ego), accel, accel, pitch, v_ego * 2.23694, stopping, 0.0)


def test_stock_brake_flag_unchanged_when_off():
  c = Ctrl(regen=False)
  r = step(c, -0.2)
  assert r.brake_actuate and r.gas == INACTIVE and r.accel_pred_send == INACTIVE


def test_regen_handles_gentle_decel():
  c = Ctrl(regen=True)
  for a in (-0.05, -0.2, -0.4, -0.5):
    r = step(c, a)
    assert not r.brake_actuate and not r.precharge_actuate
    assert r.gas == a and r.accel_pred_send == a
    assert r.accel == a


def test_regen_brake_hysteresis():
  c = Ctrl(regen=True)
  assert step(c, -0.7).brake_actuate
  r = step(c, -0.7)
  assert r.gas == INACTIVE and r.accel_pred_send == INACTIVE
  assert step(c, -0.45).brake_actuate          # still inside the band
  r = step(c, -0.3)
  assert not r.brake_actuate and r.gas == -0.3  # released back to regen


def test_regen_never_positive_gas_while_decelerating():
  c = Ctrl(regen=True)
  r = c.update(cc(), cs(), -0.3, 0.4, 0.0, 33.0, False, 0.0)
  assert r.gas <= 0.0 and r.gas == -0.3


def test_regen_positive_accel_passes_through():
  c = Ctrl(regen=True)
  r = step(c, 0.6)
  assert not r.brake_actuate and r.gas == 0.6 and r.accel_pred_send == 0.6


def test_regen_falls_back_when_stopping_or_slow():
  c = Ctrl(regen=True)
  assert step(c, -0.2, stopping=True).brake_actuate
  c = Ctrl(regen=True)
  assert step(c, -0.2, v_ego=2.0).brake_actuate


def test_inactive_long_sends_inactive_gas():
  c = Ctrl(regen=True)
  r = step(c, -0.2, active=False)
  assert r.gas == INACTIVE and not r.brake_actuate


def test_lead_present_is_read():
  # 60 mph, closing on a lead 1 s ahead: BP long must keep the brake request (not force accel 0)
  lead = SimpleNamespace(present=True, dRel=27.0, vRel=-3.0, vLead=24.0)
  c = Ctrl(regen=False, lead=lead)
  c.bpSpeedAllow = True
  r = step(c, -1.0, v_ego=27.0)
  assert r.bp_long_used and r.accel < 0.0

  no_lead = SimpleNamespace(present=False, dRel=0.0, vRel=0.0, vLead=0.0)
  c = Ctrl(regen=False, lead=no_lead)
  c.bpSpeedAllow = True
  r = step(c, -1.0, v_ego=27.0)
  assert r.accel == 0.0  # no lead: BP long coasts instead of braking (existing behavior)


def test_encode_gas():
  assert encode_gas(0.3, True, False) == 0.3
  assert encode_gas(-0.5, True, False) == -0.5          # MIN_GAS itself is allowed
  assert encode_gas(-0.51, True, False) == INACTIVE      # below what ford.h accepts
  assert encode_gas(0.3, False, False) == INACTIVE       # long off
  assert encode_gas(0.3, True, True) == INACTIVE         # never gas and brake together
  assert encode_gas(9.0, True, False) == CarControllerParams.ACCEL_MAX


def test_follow_limits_per_lead_state():
  g, a = -0.3, -0.4
  assert follow_limits(None, 999.0, g, a, 0.0) == (g, g, 0.0, 0.0)                    # no lead: coast
  assert follow_limits(Lead(20, -1.0, 20), 1.0, g, a, 0.0) == (0.0, 0.0, a, a)         # gaining, close
  assert follow_limits(Lead(60, -1.0, 20), 3.0, g, a, 0.0) == (g, g, a, a)             # gaining, far
  assert follow_limits(Lead(40, 1.0, 25), 2.0, g, a, 0.0) == (g, g, a, a)              # trailing
  assert follow_limits(Lead(40, 0.0, 25), 2.0, g, a, 0.1) == (0.0, 0.2 + 0.1, a, a)    # pacing cap + pitch


def test_follow_limits_pacing_cap_never_below_gas_floor():
  # steep downhill with downhill comp on: 0.2 - 0.9 would undercut MIN_GAS; the old sentinel path
  # sent MIN_GAS in that case, so the cap is clamped to keep the same output
  assert follow_limits(Lead(40, 0.0, 25), 2.0, -2.0, -1.0, -0.9)[1] == CarControllerParams.MIN_GAS


def _bp_ctrl():
  # 60 mph behind a pacing lead with BP long allowed: the BP follow path is in control
  c = Ctrl(regen=False, lead=SimpleNamespace(present=True, dRel=40.0, vRel=0.0, vLead=27.0))
  c.bpSpeedAllow = True
  c.following_accel_ROC = 99.0  # no ramp, so the test drives the request directly
  return c


def test_bp_path_brake_flag_has_real_hysteresis():
  """BP path used to reset its flag every call (no memory): inside the -0.14..-0.06 band it
  dropped out. It now shares the one brake hysteresis, so the flag holds in the band."""
  c = _bp_ctrl()
  r = step(c, -0.20, v_ego=27.0)
  assert r.bp_long_used and r.brake_actuate
  assert step(c, -0.10, v_ego=27.0).brake_actuate        # in the band: holds
  assert not step(c, -0.05, v_ego=27.0).brake_actuate    # above release: off
  assert not step(c, -0.10, v_ego=27.0).brake_actuate    # in the band from below: stays off


def test_precharge_follows_brake_flag_everywhere():
  for c, v in ((_bp_ctrl(), 27.0), (Ctrl(regen=False), 15.0), (Ctrl(regen=True), 15.0)):
    for a in (-0.1, -0.3, -0.7, -0.3, 0.2):
      r = step(c, a, v_ego=v)
      assert r.precharge_actuate == r.brake_actuate


def test_bp_path_brake_decision_includes_pitch():
  c = _bp_ctrl()
  assert not step(c, -0.10, v_ego=27.0, pitch=0.0).brake_actuate
  c = _bp_ctrl()
  assert step(c, -0.10, v_ego=27.0, pitch=-0.10).brake_actuate  # -0.2 with the (downhill) pitch


# ---- deep regen (FordSafetyFlagsBP.REGEN_BLEND set at car init) ----

from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsBP, REGEN_MIN_GAS


class DeepCtrl(Ctrl):
  def __init__(self, regen=True, flag=True, **kw):
    LongitudinalExt.__init__(self, None, SimpleNamespace(safetyParam=FordSafetyFlagsBP.REGEN_BLEND if flag else 0))
    self.regen_blend_UI = regen
    self.disable_BP_long_UI = kw.get('disable_bp_long', False)
    self.sm = FakeSM(kw.get('lead'))


def test_deep_regen_covers_medium_slowing_without_friction():
  c = DeepCtrl()
  for a in (-0.3, -0.6, -0.85, -0.9):
    r = step(c, a)
    assert not r.brake_actuate, a
    assert r.gas == a and r.accel_pred_send == a


def test_deep_regen_brake_flag_thresholds():
  c = DeepCtrl()
  r = step(c, -0.93)                       # between the floor and the flag: regen at the floor
  assert not r.brake_actuate and r.gas == REGEN_MIN_GAS
  assert step(c, -1.0).brake_actuate       # below -0.95: friction
  assert step(c, -0.85).brake_actuate      # band -0.95..-0.80 holds
  r = step(c, -0.75)
  assert not r.brake_actuate and r.gas == -0.75


def test_deep_regen_needs_the_init_flag():
  """Toggled on without the init flag (panda still at -0.5): today's regen floor and thresholds."""
  c = DeepCtrl(flag=False)
  r = step(c, -0.6)
  assert r.brake_actuate                    # BRAKE_REGEN on at -0.55
  c = DeepCtrl(flag=False)
  assert step(c, -0.5).gas == -0.5 and step(c, -0.52).gas == -0.5


def test_deep_regen_flag_but_regen_off_is_stock():
  c = DeepCtrl(regen=False)
  r = step(c, -0.3)
  assert r.brake_actuate and r.gas == INACTIVE   # stock: flag below -0.14, gas inactive
  c = DeepCtrl(regen=False)
  assert step(c, -0.7).gas == INACTIVE           # never below -0.5 without regen active


def test_gas_never_below_what_the_panda_allows():
  for flag in (False, True):
    for regen in (False, True):
      c = DeepCtrl(regen=regen, flag=flag)
      floor = REGEN_MIN_GAS if (flag and regen) else CarControllerParams.MIN_GAS
      for a in [x / 20 for x in range(-60, 20)]:
        r = step(c, a)
        if r.gas != INACTIVE:
          assert r.gas >= floor - 1e-9
        if r.accel_pred_send != INACTIVE:
          assert r.accel_pred_send >= floor - 1e-9

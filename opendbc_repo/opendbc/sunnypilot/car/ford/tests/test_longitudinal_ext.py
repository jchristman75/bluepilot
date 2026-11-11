"""Tests for LongitudinalExt: regen blend (BPLongRegenBlend) and lead detection."""
import pytest
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


def cc(active=True, accel=0.0):
  # accel: the planner's raw request (CC.actuators.accel), before the carcontroller's limits
  return SimpleNamespace(longActive=active, actuators=SimpleNamespace(accel=accel))


def cs(v_ego=15.0, a_ego=0.0):
  # regen_power_limit_kw None: the car doesn't report the pack limit
  return SimpleNamespace(out=SimpleNamespace(vEgo=v_ego, aEgo=a_ego, gasPressed=False, brakePressed=False),
                         regen_power_limit_kw=None)


def step(c, accel, v_ego=15.0, stopping=False, active=True, pitch=0.0):
  # carcontroller passes the raw planner request as gas; LongitudinalExt encodes INACTIVE itself
  return c.update(cc(active, accel), cs(v_ego, a_ego=accel), accel, accel, pitch, v_ego * 2.23694, stopping, 0.0)


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
  r = c.update(cc(), cs(a_ego=-0.3), -0.3, 0.4, 0.0, 33.0, False, 0.0)
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


# ---- regen limited by the pack (full battery / cold): live floor + shortfall monitor ----

from opendbc.sunnypilot.car.ford.longitudinal_ext import (BRAKE_REGEN, BRAKE_REGEN_DEEP, BRAKE_STOCK,
                                                           regen_floor, regen_thresholds)


def cs_full(v_ego=25.0, limit_kw=None, a_ego=0.0):
  return SimpleNamespace(out=SimpleNamespace(vEgo=v_ego, aEgo=a_ego, gasPressed=False, brakePressed=False),
                         regen_power_limit_kw=limit_kw)


def step_lim(c, accel, v_ego=18.0, limit_kw=None, a_ego=None):  # 40 mph: below the BP-long switch
  a_ego = accel if a_ego is None else a_ego  # default: the car does what it is asked
  return c.update(cc(True), cs_full(v_ego, limit_kw, a_ego), accel, accel, 0.0, v_ego * 2.23694, False, 0.0)


def test_regen_floor_from_pack_limit():
  # unknown limit or plenty of regen: the panda floor, unchanged
  assert regen_floor(REGEN_MIN_GAS, None, 25.0, 2500.0) == REGEN_MIN_GAS
  assert regen_floor(REGEN_MIN_GAS, 100.0, 25.0, 2500.0) == REGEN_MIN_GAS
  # 20 kW at 25 m/s, 2500 kg, 0.8 margin -> regen can do ~0.256 m/s^2
  assert regen_floor(REGEN_MIN_GAS, 20.0, 25.0, 2500.0) == pytest.approx(-0.256)
  # full battery: no regen at all
  assert regen_floor(REGEN_MIN_GAS, 0.0, 25.0, 2500.0) == 0.0


def test_thresholds_follow_the_floor():
  assert regen_thresholds(REGEN_MIN_GAS, REGEN_MIN_GAS) == BRAKE_REGEN_DEEP     # not limited: exact constants
  assert regen_thresholds(CarControllerParams.MIN_GAS, CarControllerParams.MIN_GAS) == BRAKE_REGEN
  t = regen_thresholds(-0.256, REGEN_MIN_GAS)
  assert t.on == pytest.approx(-0.306) and t.off == pytest.approx(-0.156)
  assert regen_thresholds(0.0, REGEN_MIN_GAS) == BRAKE_STOCK                   # no regen: stock thresholds


def test_high_limit_changes_nothing():
  for flag in (False, True):
    a, b = DeepCtrl(flag=flag), DeepCtrl(flag=flag)
    for x in (-0.2, -0.45, -0.6, -0.85, -1.0, -0.3, 0.2):
      ra, rb = step_lim(a, x, limit_kw=None), step_lim(b, x, limit_kw=120.0)
      assert (ra.gas, ra.brake_actuate, ra.accel_pred_send) == (rb.gas, rb.brake_actuate, rb.accel_pred_send)


def test_limited_regen_hands_off_to_friction_early():
  c = DeepCtrl()
  r = step_lim(c, -0.2, limit_kw=20.0)      # within what regen can do (20 kW, 18 m/s, 3000 kg: -0.296)
  assert not r.brake_actuate and r.gas == -0.2
  r = step_lim(c, -0.4, limit_kw=20.0)      # beyond it: friction now, not after the PID winds up
  assert r.brake_actuate and r.gas == CarControllerParams.INACTIVE_GAS


def test_no_regen_at_all_behaves_like_stock():
  c = DeepCtrl()
  r = step_lim(c, -0.1, limit_kw=0.0)
  assert not r.brake_actuate and r.gas == 0.0   # never ask for regen that isn't there (and never positive gas)
  assert step_lim(c, -0.2, limit_kw=0.0).brake_actuate


def test_shortfall_monitor_forces_friction():
  c = DeepCtrl()
  # asked for -0.6 of regen, the car keeps coasting (aEgo 0): friction after ~0.8 s of clear shortfall
  forced_at = None
  for i in range(100):  # 2 s at 50 Hz
    r = step_lim(c, -0.6, a_ego=0.0)
    if r.brake_actuate:
      forced_at = i
      break
  assert forced_at is not None and 40 <= forced_at <= 80


def test_shortfall_monitor_quiet_when_regen_delivers():
  c = DeepCtrl()
  for _ in range(200):
    assert not step_lim(c, -0.6, a_ego=-0.6).brake_actuate


# ---- BP follow ramp and hand-back (route 52d t=342/529/658: harsh braking behind a slowing lead) ----

def _set_lead(c, lead):
  c.sm['radarState'] = SimpleNamespace(leadOne=lead)
  c.sm.valid['radarState'] = lead is not None


def _drops(seq):
  return [b - a for a, b in zip(seq, seq[1:], strict=False)]


def test_lead_appearing_at_highway_speed_brakes_within_a_second():
  """74 mph, a lead appears 55 m ahead closing at 4.5 m/s (TTC ~12 s) and the planner ramps to -1.5
  over 1 s. The old 0.002/scan limit sent ~-0.1 after a second while the car coasted; the request must
  now follow the planner closely, still without a step."""
  c = Ctrl(regen=False, lead=SimpleNamespace(present=False, dRel=0.0, vRel=0.0, vLead=0.0))
  c.bpSpeedAllow = True
  for _ in range(20):
    step(c, 0.0, v_ego=33.0)
  _set_lead(c, SimpleNamespace(present=True, dRel=55.0, vRel=-4.5, vLead=28.5))
  sent = []
  for i in range(60):
    r = step(c, -1.5 * min(1.0, (i + 1) / 50), v_ego=33.0)
    assert r.bp_long_used
    sent.append(r.accel)
  assert sent[49] <= -1.4
  assert min(_drops(sent)) >= -c.following_accel_ROC - 1e-9


def test_handoff_to_planner_ramps_instead_of_stepping():
  """BP holds -0.3 behind a lead; then the car slows below 45 mph and the planner (wound up) asks -2.4.
  The request must ramp at the follow rate, not jump to -2.4."""
  c = Ctrl(regen=False, lead=SimpleNamespace(present=True, dRel=40.0, vRel=-1.0, vLead=26.0))
  c.bpSpeedAllow = True
  for _ in range(100):
    r = step(c, -0.3, v_ego=27.0)
  assert r.bp_long_used and r.accel == pytest.approx(-0.3)
  sent = []
  for _ in range(100):
    r = step(c, -2.4, v_ego=19.5)  # 43.6 mph: BP long switches off
    assert not r.bp_long_used
    sent.append(r.accel)
  assert sent[0] >= -0.3 - c.following_accel_ROC - 1e-9
  assert min(_drops(sent)) >= -c.following_accel_ROC - 1e-9
  assert sent[-1] == pytest.approx(-2.4)  # caught up, then the planner's request passes through
  assert not c.handoff_ramp


def test_urgent_lead_is_not_ramped():
  c = Ctrl(regen=False, lead=SimpleNamespace(present=True, dRel=12.0, vRel=-6.0, vLead=21.0))  # TTC 2 s
  c.bpSpeedAllow = True
  r = step(c, -3.0, v_ego=27.0)
  assert r.bp_long_used and r.accel == pytest.approx(-3.0)


def test_city_driving_is_not_ramped():
  # below the BP-long speed BP never takes over, so nothing changes for the planner's request
  c = Ctrl(regen=False, lead=SimpleNamespace(present=True, dRel=20.0, vRel=-3.0, vLead=12.0))
  for _ in range(5):
    step(c, 0.0, v_ego=15.0)
  r = step(c, -2.0, v_ego=15.0)
  assert not r.bp_long_used and r.accel == pytest.approx(-2.0)


# ---- launch from a stop (route 537 t=373/501: green light, model asks +0.2..+0.34, car never moves,
#      model gives up after ~1 s and op holds the brake again until the driver presses the gas) ----

from opendbc.sunnypilot.car.ford.longitudinal_ext import LAUNCH_ACCEL, LAUNCH_CONFIRM_FRAMES, LAUNCH_END_SPEED, LAUNCH_MAX_FRAMES


def _hold_at_stop(c, frames=50):
  for _ in range(frames):
    r = step(c, -2.0, v_ego=0.0, stopping=True)
  return r


def _release(c, a=0.2, frames=LAUNCH_CONFIRM_FRAMES):
  """The planner releases the stop and asks `a` for `frames` frames; returns the sent accels."""
  return [step(c, a, v_ego=0.0).accel for _ in range(frames)]


def test_launch_gets_enough_to_roll():
  c = Ctrl(regen=True)
  _hold_at_stop(c)
  sent = _release(c, 0.15)
  assert sent[-1] == pytest.approx(LAUNCH_ACCEL)
  for a in (0.25, 0.34):
    r = step(c, a, v_ego=0.0)
    assert r.accel == pytest.approx(LAUNCH_ACCEL) and r.gas == pytest.approx(LAUNCH_ACCEL)
    assert not r.brake_actuate


def test_a_one_frame_blip_at_a_stop_does_not_launch():
  c = Ctrl()
  _hold_at_stop(c)
  sent = _release(c, 0.2, LAUNCH_CONFIRM_FRAMES - 1)
  assert all(a == pytest.approx(0.2) for a in sent)
  assert step(c, 0.05, v_ego=0.0).accel == pytest.approx(0.05)   # below the request threshold
  assert c.launch_frames_left == 0


def test_launch_boost_ends_once_rolling():
  c = Ctrl()
  _hold_at_stop(c)
  _release(c)
  r = step(c, 0.2, v_ego=LAUNCH_END_SPEED + 0.1)
  assert r.accel == pytest.approx(0.2)
  r = step(c, 0.2, v_ego=0.5)  # does not come back on its own while still moving slowly
  assert r.accel == pytest.approx(0.2)


def test_launch_boost_is_time_limited():
  c = Ctrl()
  _hold_at_stop(c)
  sent = _release(c, 0.2, LAUNCH_CONFIRM_FRAMES - 1 + LAUNCH_MAX_FRAMES + 20)[LAUNCH_CONFIRM_FRAMES - 1:]
  assert all(a == pytest.approx(LAUNCH_ACCEL) for a in sent[:LAUNCH_MAX_FRAMES])
  assert all(a == pytest.approx(0.2) for a in sent[LAUNCH_MAX_FRAMES:])


def test_launch_boost_stops_the_moment_the_planner_stops_again():
  c = Ctrl()
  _hold_at_stop(c)
  _release(c)
  r = step(c, -0.3, v_ego=0.0, stopping=True)
  assert r.accel < 0 and r.brake_actuate
  assert _release(c)[-1] == pytest.approx(LAUNCH_ACCEL)  # a new release from a stop boosts again


def test_launch_boost_ends_when_the_planner_brakes_even_if_the_rate_limited_accel_is_still_positive():
  """The carcontroller passes `accel` rate-limited down from what was last sent (0.6 -> 0.53 -> ...).
  The boost must follow the planner's raw request, or it keeps renewing itself."""
  c = Ctrl()
  _hold_at_stop(c)
  assert _release(c)[-1] == pytest.approx(LAUNCH_ACCEL)
  r = c.update(cc(True, -0.3), cs(0.2, 0.0), 0.53, 0.53, 0.0, 0.0, False, 0.0)
  assert r.accel == pytest.approx(0.53)
  assert c.launch_frames_left == 0


def test_launch_boost_never_lowers_a_bigger_request_or_applies_while_moving():
  c = Ctrl()
  _hold_at_stop(c)
  assert _release(c, 1.5)[-1] == pytest.approx(1.5)
  c = Ctrl()
  for _ in range(20):
    r = step(c, 0.2, v_ego=5.0)  # rolling, never stopped: untouched
  assert r.accel == pytest.approx(0.2)


def test_launch_boost_respects_the_pedals_and_disengage():
  c = Ctrl()
  _hold_at_stop(c)
  s = cs(0.0, 0.0)
  s.out.brakePressed = True
  for _ in range(LAUNCH_CONFIRM_FRAMES + 2):
    r = c.update(cc(True, 0.2), s, 0.2, 0.2, 0.0, 0.0, False, 0.0)
  assert r.accel == pytest.approx(0.2)
  c = Ctrl()
  _hold_at_stop(c)
  for _ in range(LAUNCH_CONFIRM_FRAMES + 2):
    r = step(c, 0.2, v_ego=0.0, active=False)
  assert r.gas == INACTIVE


def test_launch_boost_off_by_toggle():
  c = Ctrl()
  c.launch_boost_UI = False
  _hold_at_stop(c)
  assert _release(c, 0.2, LAUNCH_CONFIRM_FRAMES + 5)[-1] == pytest.approx(0.2)


def test_no_launch_boost_toward_a_stopped_car_close_ahead():
  c = Ctrl(lead=SimpleNamespace(present=True, dRel=5.0, vRel=0.0, vLead=0.0))
  _hold_at_stop(c)
  assert _release(c, 0.2, LAUNCH_CONFIRM_FRAMES + 5)[-1] == pytest.approx(0.2)

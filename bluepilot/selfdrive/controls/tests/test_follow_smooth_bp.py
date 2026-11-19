"""Smooth following (BPSmoothFollow): the lead's speed and accel the MPC sees are averaged so a bus that speeds up and
slows down does not make the car do the same, a little closer follow time, and everything safety-relevant passes through."""
import numpy as np
import pytest

from openpilot.bluepilot.selfdrive.controls import follow_smooth_bp as F

DT = 0.05


def feed(s, speeds, d=30.0, v_ego=15.0, a_lead=0.0, radar=True):
  out = []
  for v in speeds:
    out.append(s.update(True, radar, d, v, a_lead, v_ego, DT))   # (v, a, d, active)
  return out


def test_a_constant_lead_is_passed_through_unchanged():
  s = F.FollowSmoother()
  v, a, d, active = feed(s, [15.0] * 200)[-1]
  assert v == pytest.approx(15.0, abs=1e-6) and a == pytest.approx(0.0, abs=1e-6) and d == pytest.approx(30.0) and active


def test_a_bus_oscillating_by_a_few_mph_is_averaged():
  s = F.FollowSmoother()
  t = np.arange(0, 60, DT)
  lead = 15.0 + 1.5 * np.sin(2 * np.pi * t / 8.0)               # +-3.4 mph every 8 s
  out = np.array([o[0] for o in feed(s, lead)])[600:]            # after the filter settled
  assert out.std() < 0.6 * lead[600:].std()                      # well under two thirds of the swing reaches the MPC


def test_a_lead_that_brakes_hard_passes_through():
  s = F.FollowSmoother()
  feed(s, [15.0] * 200)
  out = feed(s, list(np.linspace(15.0, 8.0, 60)), a_lead=-2.5)
  v, a, _, _ = out[-1]
  assert v == pytest.approx(8.0, abs=0.2)                        # the real speed, within the small optimism that fades by 3 m/s
  assert a == pytest.approx(-2.5)                                # braking accel is never attenuated (not smoothed at all)


def test_the_lead_is_never_assumed_faster_than_a_little_above_what_it_does():
  s = F.FollowSmoother()
  feed(s, [15.0] * 200)
  worst = max(v - lv for (v, _, _, _), lv in zip(feed(s, [13.8] * 100), [13.8] * 100, strict=True))
  assert worst <= F.D0 * (1 - F.K) + 1e-6                        # optimism is bounded (<= 0.9 m/s)


def test_a_lead_speeding_up_is_followed_with_a_lag_not_ignored():
  s = F.FollowSmoother()
  feed(s, [10.0] * 200)
  out = feed(s, [18.0] * 400)                                    # a real step up (green light)
  assert out[0][0] < 18.0 - 0.5                                  # the first moments lag a little (conservative)...
  assert out[-1][0] == pytest.approx(18.0, abs=0.3)              # ...and it converges within 20 s


def test_positive_lead_accel_is_attenuated_and_hard_braking_is_not():
  s = F.FollowSmoother()
  feed(s, [15.0] * 100)
  assert feed(s, [15.0], a_lead=1.0)[0][1] == pytest.approx(F.K_A * 1.0)
  assert feed(s, [15.0], a_lead=-0.4)[0][1] == pytest.approx(F.K_A * -0.4)
  v, a, d, active = s.update(True, True, 30.0, 15.0, -1.5, 15.0, DT)
  assert not active and a == -1.5 and v == 15.0 and d == 30.0   # hard braking: everything raw


def test_a_close_lead_or_a_fast_closing_lead_is_not_smoothed():
  s = F.FollowSmoother()
  feed(s, [15.0] * 200)
  v, a, d, active = s.update(True, True, 12.0, 13.5, -0.5, 15.0, DT)       # 0.8 s gap
  assert not active and v == 13.5 and a == -0.5 and d == 12.0
  v, a, d, active = s.update(True, True, 25.0, 10.0, 0.0, 15.0, DT)        # closing at 5 m/s, TTC 5 s
  assert not active and v == 10.0 and d == 25.0


def test_no_radar_or_no_lead_is_passed_through_and_resets():
  s = F.FollowSmoother()
  feed(s, [15.0] * 200)
  v, a, d, active = s.update(True, False, 30.0, 12.0, 0.3, 15.0, DT)       # a vision-only lead is too noisy to average
  assert not active and v == 12.0 and a == 0.3 and d == 30.0
  s.update(False, False, 0.0, 0.0, 0.0, 15.0, DT)
  v, _, _, _ = s.update(True, True, 30.0, 20.0, 0.0, 15.0, DT)             # a new lead starts from its own speed
  assert v == pytest.approx(20.0)


def test_a_new_lead_or_a_jump_in_distance_resets_the_average():
  s = F.FollowSmoother()
  feed(s, [15.0] * 200, d=30.0)
  v, _, d, _ = s.update(True, True, 90.0, 25.0, 0.0, 15.0, DT)             # cut-out of one lead, another far ahead
  assert v == pytest.approx(25.0) and d == pytest.approx(90.0)


def test_follow_time_is_scaled_and_floored():
  assert F.scaled_t_follow(1.45, 0.85) == pytest.approx(1.2325)
  assert F.scaled_t_follow(1.45, F.DEFAULT_GAP_SCALE) == pytest.approx(1.0875)  # standard: 1.45 s -> 1.09 s
  assert F.scaled_t_follow(1.25, 0.7) == F.MIN_T_FOLLOW                       # 0.875 would be under the floor
  assert F.scaled_t_follow(1.75, 0.5) == F.MIN_T_FOLLOW                 # never below the floor
  assert F.scaled_t_follow(1.45, 1.2) == pytest.approx(1.45)             # never longer than the personality's own


def test_the_distance_the_mpc_sees_never_exceeds_the_real_one_and_floats_below_it():
  s = F.FollowSmoother()
  feed(s, [15.0] * 200, d=30.0)
  band = F.BAND_S * 15.0
  for d in (34.0, 40.0, 26.0, 20.0, 30.0):
    _, _, d_eff, active = s.update(True, True, d, 15.0, 0.0, 15.0, DT)
    assert active and d - band - 1e-9 <= d_eff <= d + 1e-9          # never farther than it really is, within the band below
  s = F.FollowSmoother()
  feed(s, [15.0] * 200, d=30.0)
  seen = [s.update(True, True, 30.0 - 0.05 * k, 15.0, 0.0, 15.0, DT)[2] - (30.0 - 0.05 * k) for k in range(60)]
  assert max(seen) <= 1e-9                                           # a closing gap is never made to look bigger

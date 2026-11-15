"""Soft approach (BPSoftApproach): on the highway, closing on a far lead is handled with the decel that time-to-impact
needs plus a safe margin, not the planner's follow-gap panic. Emergencies (cut-in, lead braking hard, short time to
impact) are left entirely to the planner."""
import pytest

from openpilot.bluepilot.selfdrive.controls import approach_decel_bp as ad

V = 29.0     # 65 mph


def cap(v_ego=V, d=100.0, v_rel=-7.0, a_lead=0.0, radar=True, present=True):
  return ad.approach_decel_cap(v_ego, d, v_rel, a_lead, radar, present)


def test_a_far_lead_closing_gets_a_gentle_cap_that_still_leaves_a_safe_gap():
  c = cap(d=100.0, v_rel=-7.0)                       # 100 m, closing 7 m/s (TTC 14 s)
  assert c is not None and 0.3 <= c <= 0.9
  # the cap, held constant, matches the lead's speed with at least END_GAP_S of gap left
  end_gap = 100.0 - 49.0 / (2.0 * c)
  assert end_gap >= ad.END_GAP_S * V - 1e-6


@pytest.mark.parametrize("d,v_rel", [(66.0, -8.4), (76.0, -8.8), (123.0, -9.3), (104.0, -7.3)])
def test_the_logged_highway_approaches_get_a_cap_well_under_what_the_planner_asked(d, v_rel):
  c = cap(v_ego=28.0, d=d, v_rel=v_rel)
  assert c is not None and c < 1.2                  # the planner asked -1.0..-1.6 and the car delivered up to -3.05


def test_the_cap_grows_as_the_gap_closes_so_it_never_traps_the_car():
  caps = [cap(d=d, v_rel=-7.0) for d in (140.0, 100.0, 80.0, 65.0, 55.0)]
  assert all(c is not None for c in caps)
  assert caps == sorted(caps)                        # monotonic: nearer = harder allowed


def test_no_cap_when_there_is_not_much_gap_left_to_use_up():
  assert cap(d=ad.END_GAP_S * V + 1.0) is None       # inside the safe margin: the planner decides
  assert cap(d=20.0, v_rel=-6.0) is None             # a cut-in


def test_no_cap_with_a_short_time_to_impact():
  assert cap(d=30.0, v_rel=-8.0) is None             # 3.8 s
  assert cap(d=ad.URGENT_TTC_S * 7.0 - 1.0, v_rel=-7.0) is None


def test_a_lead_that_is_braking_hard_gets_no_cap():
  assert cap(a_lead=-1.5) is None
  assert cap(a_lead=-ad.LEAD_BRAKE_URGENT - 0.01) is None


def test_a_lead_slowing_a_little_raises_the_cap_by_its_decel():
  base, braking = cap(a_lead=0.0), cap(a_lead=-0.6)
  assert braking == pytest.approx(base + 0.6)


@pytest.mark.parametrize("kw", [{"present": False}, {"radar": False}, {"v_ego": 15.0}, {"v_rel": 0.2}, {"v_rel": 3.0}])
def test_no_cap_without_a_radar_lead_on_the_highway_that_is_being_closed(kw):
  assert cap(**kw) is None


def test_the_cap_never_asks_for_less_than_a_floor_and_never_above_the_ceiling():
  assert cap(d=400.0, v_rel=-1.0) == pytest.approx(ad.CAP_MIN)
  assert cap(d=70.0, v_rel=-9.5) is None or cap(d=70.0, v_rel=-9.5) <= ad.CAP_MAX


def test_the_cap_is_a_decel_magnitude_and_applying_it_only_softens():
  assert ad.apply_cap(-1.6, 0.8) == -0.8
  assert ad.apply_cap(-0.5, 0.8) == -0.5            # a gentler request is untouched
  assert ad.apply_cap(0.3, 0.8) == 0.3              # accelerating is untouched
  assert ad.apply_cap(-1.6, None) == -1.6           # no cap: unchanged

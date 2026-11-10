"""BluePilot: radard lead matching for raw radar tracks (lead_match.py), on a private copy of radard."""
import importlib.util
from types import SimpleNamespace

import pytest

from openpilot.bluepilot.selfdrive.controls.radar import lead_match
from openpilot.bluepilot.selfdrive.controls.radar.mrr_tracker import RAW_ID_BASE


@pytest.fixture
def R():
  # a private module copy, so install() never patches the radard other tests import
  spec = importlib.util.find_spec("openpilot.selfdrive.controls.radard")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  lead_match.install(mod)
  return mod


def track(R, tid, d, y, v_lead, v_ego):
  t = R.Track(tid, v_lead, R.KalmanParams(R.DT_MDL))
  t.update(d, y, v_lead - v_ego, v_lead)
  return t


def lead(x, y, v, x_std=59874.1, y_std=0.85, v_std=59874.1):
  # the big models report x/v stds around 6e4 (route 52e)
  return SimpleNamespace(x=[x], y=[y], v=[v], xStd=[x_std], yStd=[y_std], vStd=[v_std], prob=0.9)


def test_picks_the_lead_car_not_in_lane_clutter_far_away(R):
  v_ego = 17.0
  tracks = {RAW_ID_BASE + 1: track(R, RAW_ID_BASE + 1, 63.0, -0.1, 16.0, v_ego),    # the car the camera sees
            RAW_ID_BASE + 2: track(R, RAW_ID_BASE + 2, 150.0, -0.2, 0.0, v_ego)}   # stationary, up the lane
  # radard's lateral is left-positive: the car at camera y=+0.15 sits at yRel -0.15
  assert R.match_vision_to_track(v_ego, lead(64.5, 0.15, 16.0), tracks) is tracks[RAW_ID_BASE + 1]


def test_no_sane_track_means_vision_only(R):
  v_ego = 17.0
  tracks = {RAW_ID_BASE + 2: track(R, RAW_ID_BASE + 2, 150.0, -0.2, 0.0, v_ego)}
  assert R.match_vision_to_track(v_ego, lead(64.5, 0.15, 16.0), tracks) is None


def test_matched_track_is_kept_past_the_normal_gate(R):
  v_ego = 20.0
  tracks = {RAW_ID_BASE + 1: track(R, RAW_ID_BASE + 1, 90.0, 0.0, 23.0, v_ego)}
  # camera at ~78 m: 12 m off, inside the 25 % gate -> matched
  assert R.match_vision_to_track(v_ego, lead(78.0 + R.RADAR_TO_CAMERA, 0.0, 18.0), tracks) is not None
  # camera now at ~70 m: 20 m off is outside 25 % (17.5 m) but inside the sticky 35 % gate -> kept
  assert R.match_vision_to_track(v_ego, lead(70.0 + R.RADAR_TO_CAMERA, 0.0, 18.0), tracks) is tracks[RAW_ID_BASE + 1]
  # a fresh matcher (nothing matched yet) would not take it
  fresh = lead_match.LeadMatcher(R)
  assert fresh.match(v_ego, lead(70.0 + R.RADAR_TO_CAMERA, 0.0, 18.0), tracks) is None


def test_lead_one_and_two_keep_separate_sticky_tracks(R):
  v_ego = 20.0
  t1, t2 = track(R, RAW_ID_BASE + 1, 40.0, 0.0, 18.0, v_ego), track(R, RAW_ID_BASE + 2, 80.0, 0.0, 18.0, v_ego)
  tracks = {t1.identifier: t1, t2.identifier: t2}
  m = lead_match.LeadMatcher(R)
  m.key = 'one'
  assert m.match(v_ego, lead(41.5, 0.0, 18.0), tracks) is t1
  m.key = 'two'
  assert m.match(v_ego, lead(81.5, 0.0, 18.0), tracks) is t2
  m.key = 'one'
  assert m.match(v_ego, lead(41.5, 0.0, 18.0), tracks) is t1


def test_raw_tracks_never_make_a_low_speed_lead_on_their_own(R):
  v_ego = 1.0
  raw = track(R, RAW_ID_BASE + 7, 10.0, 0.2, 0.0, v_ego)   # a curb / parked car right ahead
  cmbb = track(R, 3, 10.0, 0.2, 0.0, v_ego)                 # Ford's Steer_Assist_Data object
  assert not raw.potential_low_speed_lead(v_ego)
  assert cmbb.potential_low_speed_lead(v_ego)

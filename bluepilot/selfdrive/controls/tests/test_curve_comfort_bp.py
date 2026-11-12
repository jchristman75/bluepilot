"""Learned curve comfort (choice A3): only ever gentler than the car ceiling, learned from the driver's own curves."""
import json

import numpy as np
import pytest

from openpilot.bluepilot.selfdrive.controls import curve_comfort_bp as cc
from openpilot.bluepilot.selfdrive.controls import curve_limit_bp as cl


K15 = float(cc.CURVATURE_GRID[15])   # a bucket center: lookups there are exactly that bucket's level


def test_no_data_means_no_restriction():
  c = cc.CurveComfort()
  a = c.lat_accel_for_curvature(np.array([0.0005, 0.002, 0.01, 0.05]))
  assert np.all(a >= cl.CEILING_V[0] + 1e-9)      # never below the car ceiling, so min(ceiling, learned) = ceiling


def test_buckets_are_log_spaced_and_clip_to_the_range():
  assert cc.bucket_key(0.00001) == cc.bucket_key(cc.K_MIN)
  assert cc.bucket_key(1.0) == cc.bucket_key(cc.K_MAX)
  assert cc.bucket_key(-0.01) == cc.bucket_key(0.01)
  assert len(cc.CURVATURE_GRID) == cc.N_BUCKETS


def test_learning_pulls_the_comfort_level_toward_the_driver():
  c = cc.CurveComfort()
  k = K15
  before = float(c.lat_accel_for_curvature(np.array([k]))[0])
  for _ in range(400):
    c.observe(k, 1.4)
  after = float(c.lat_accel_for_curvature(np.array([k]))[0])
  assert after < before
  assert after == pytest.approx(1.4 * 0.8 + before * 0.2, abs=0.25)   # 400 samples: confidence 0.8


def test_learned_level_is_clamped_to_the_minimum():
  c = cc.CurveComfort()
  for _ in range(2000):
    c.observe(K15, 0.3)        # a driver crawling through a curve
  assert float(c.lat_accel_for_curvature(np.array([K15]))[0]) >= cl.A_COMFORT_MIN - 1e-9


def test_learning_one_curvature_does_not_restrict_distant_ones():
  c = cc.CurveComfort()
  for _ in range(800):
    c.observe(K15, 1.3)
  far = c.lat_accel_for_curvature(np.array([0.0006, 0.0015]))
  assert np.all(far > 3.0)                       # gentle curves keep the non-restrictive prior
  near = float(c.lat_accel_for_curvature(np.array([K15]))[0])
  assert near < 1.7


def test_a_few_samples_barely_move_the_curve():
  c = cc.CurveComfort()
  base = float(c.lat_accel_for_curvature(np.array([K15]))[0])
  c.observe(K15, 1.2)
  assert float(c.lat_accel_for_curvature(np.array([K15]))[0]) == pytest.approx(base, abs=0.05)


def test_nudges_move_one_bucket_by_a_fixed_weight():
  c = cc.CurveComfort()
  for _ in range(300):
    c.observe(K15, 2.0)
  v0 = float(c.lat_accel_for_curvature(np.array([K15]))[0])
  c.nudge(K15, -0.15)
  v1 = float(c.lat_accel_for_curvature(np.array([K15]))[0])
  for _ in range(8):
    c.nudge(K15, +0.15)
  v2 = float(c.lat_accel_for_curvature(np.array([K15]))[0])
  assert v1 < v0 < v2


def test_stored_data_round_trips_and_bad_data_is_ignored():
  c = cc.CurveComfort()
  for _ in range(250):
    c.observe(0.006, 1.7)
  d = cc.CurveComfort(json.loads(json.dumps(c.data)))
  assert np.allclose(d.lat_accel_for_curvature(cc.CURVATURE_GRID), c.lat_accel_for_curvature(cc.CURVATURE_GRID))
  for junk in ({"x": 1}, {"0.01": {"average": "a", "count": 5}}, {"0.01": {"average": float('nan'), "count": 5}},
               {"0.01": {"average": 1.5, "count": -3}}, {}):
    e = cc.CurveComfort(junk)
    assert np.all(e.lat_accel_for_curvature(cc.CURVATURE_GRID) >= cl.CEILING_V[0] - 1e-9)


def test_observe_ignores_nonsense():
  c = cc.CurveComfort()
  for k, a in ((float('nan'), 1.0), (0.01, float('inf')), (0.01, -1.0), (0.01, 12.0)):
    c.observe(k, a)
  assert not c.dirty


def test_dirty_flag_clears_on_save():
  c = cc.CurveComfort()
  c.observe(0.01, 1.5)
  assert c.dirty
  c.mark_saved()
  assert not c.dirty


def test_limiter_with_a_learned_gentle_driver_brakes_earlier_but_not_beyond_its_cap():
  from openpilot.bluepilot.selfdrive.controls.tests.test_curve_limit_bp import arc, run, MPH
  c = cc.CurveComfort()
  x, y = arc(150.0, start=40.0)                       # R 150 m at 45 mph: 2.7 m/s^2... over the ceiling anyway
  _, plain = run(x, y, v_ego=35 * MPH)
  for _ in range(3000):
    c.observe(1 / 150.0, 1.3)
  _, learned = run(x, y, v_ego=35 * MPH, comfort=c.lat_accel_for_curvature)
  assert learned.v_raw <= plain.v_raw
  assert learned.a_req >= -cl.DECEL_CAP - 1e-9


# ---- ComfortLearner, without the SCC adapter ----

def _learner_step(L, **kw):
  args = {'dt': 0.05, 'limiter_active': False, 'long_enabled': False, 'long_override': False, 'brake_pressed': False,
          'gas_pressed': False, 'v_ego': 14.0, 'blinker': False, 'lead_near': False, 'kappa': 0.008, 'bind_curvature': 0.0}
  args.update(kw)
  L.step(**args)


def test_learner_samples_only_after_the_driver_has_been_in_charge_for_a_while():
  L = cc.ComfortLearner(cc.CurveComfort())
  for _ in range(int(cc.LEARN_SETTLE_S / 0.05) - 2):
    _learner_step(L)
  assert not L.comfort.dirty
  for _ in range(10):
    _learner_step(L)
  assert L.comfort.dirty


def test_learner_nudges_once_per_override_episode_and_saves_when_dirty():
  L = cc.ComfortLearner(cc.CurveComfort())
  _learner_step(L, limiter_active=True, bind_curvature=0.01, long_enabled=True)
  _learner_step(L, brake_pressed=True, bind_curvature=0.01, long_enabled=True)
  n = sum(d["count"] for d in L.comfort.data.values())
  assert n == cc.NUDGE_WEIGHT
  for _ in range(5):
    _learner_step(L, brake_pressed=True, bind_curvature=0.01, long_enabled=True)
  assert sum(d["count"] for d in L.comfort.data.values()) == n

  class P:
    saved = None
    def put(self, key, value): self.saved = (key, value)
  p = P()
  L.maybe_save(p, cc.SAVE_PERIOD_S)
  assert p.saved[0] == "BPCurveComfortData" and not L.comfort.dirty

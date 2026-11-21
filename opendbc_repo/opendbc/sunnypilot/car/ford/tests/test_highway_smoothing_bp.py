"""BluePilot: tests for the highway smoothing filter (highway_smoothing_bp.py)."""
import math
import unittest

import numpy as np

from opendbc.sunnypilot.car.ford.highway_smoothing_bp import (BASELINE_TAU_S, HighwaySmoothing, SPEED_FULL, SPEED_OFF,
                                                              TIGHT_CURVE_OFF)

V = 30.0   # m/s, ~67 mph: above SPEED_FULL
DT = 0.05  # the 20 Hz steering tick


def _run(smoother, kappas, v=V, gain=0.5, bypass=False, dt=DT, predicted=None, active=True):
  return [smoother.update(k, v, active, gain, bypass, dt, predicted) for k in kappas]


def _weave(amp, hz=0.3, seconds=30.0, dt=DT, offset=0.0):
  t = np.arange(0.0, seconds, dt)
  return offset + amp * np.sin(2 * math.pi * hz * t)


def _amplitude(x):
  x = np.asarray(x)
  return float((x.max() - x.min()) / 2.0)


class TestHighwaySmoothing(unittest.TestCase):
  def test_gain_one_is_a_pass_through(self):
    s = HighwaySmoothing()
    kappas = _weave(0.0002)
    out = _run(s, kappas, gain=1.0)
    np.testing.assert_allclose(out, kappas, atol=1e-12)

  def test_small_weave_on_the_highway_is_damped_as_the_filter_predicts(self):
    # 0.0002 1/m at 30 m/s = 0.18 m/s^2: small enough to stay under the maneuver gate. The output is
    # baseline + gain * (kappa - baseline), and the baseline is itself a 1.5 s low-pass of the weave, so the
    # amplitude ratio is |gain + (1 - gain) * H(f)| with H = 1 / (1 + j 2 pi f tau). Faster weaves approach `gain`.
    for hz in (0.3, 0.6, 1.0):
      h = 1.0 / (1.0 + 1j * 2 * math.pi * hz * BASELINE_TAU_S)
      for gain in (0.3, 0.5, 0.7):
        s = HighwaySmoothing()
        kappas = _weave(0.0002, hz=hz, seconds=40.0)
        out = _run(s, kappas, gain=gain)
        settled = slice(int(15 / DT), None)
        ratio = _amplitude(np.array(out)[settled]) / _amplitude(kappas[settled])
        self.assertAlmostEqual(ratio, abs(gain + (1.0 - gain) * h), delta=0.03, msg=(hz, gain))

  def test_a_fast_weave_is_cut_to_about_the_gain(self):
    s = HighwaySmoothing()
    kappas = _weave(0.0002, hz=1.5, seconds=40.0)
    out = _run(s, kappas, gain=0.3)
    ratio = _amplitude(np.array(out)[int(15 / DT):]) / 0.0002
    self.assertAlmostEqual(ratio, 0.3, delta=0.06)

  def test_a_constant_curvature_is_not_changed(self):
    s = HighwaySmoothing()
    out = _run(s, [0.0006] * 400)
    self.assertAlmostEqual(out[-1], 0.0006, places=7)

  def test_off_below_the_low_speed_limit_and_full_above_the_high_one(self):
    kappas = _weave(0.0002 * 30.0 ** 2 / 20.0 ** 2)   # same lateral acceleration at 20 m/s
    for v, expect_damped in ((SPEED_OFF - 1.0, False), (SPEED_FULL + 1.0, True)):
      s = HighwaySmoothing()
      out = np.array(_run(s, kappas, v=v, gain=0.4))
      ratio = _amplitude(out[int(10 / DT):]) / _amplitude(kappas[int(10 / DT):])
      self.assertEqual(ratio < 0.9, expect_damped, (v, ratio))

  def test_a_real_corner_gets_through_at_once(self):
    s = HighwaySmoothing()
    _run(s, [0.0] * 200)
    out = _run(s, [0.004] * 6)                # a step to 3.6 m/s^2 of lateral acceleration
    self.assertAlmostEqual(out[0], 0.004, delta=0.004 * 0.05)

  def test_smoothing_stays_off_while_the_maneuver_peak_is_held(self):
    s = HighwaySmoothing()
    _run(s, [0.0] * 200)
    _run(s, [0.004] * 20)                                  # a corner: 3.6 m/s^2 of deviation
    _run(s, [0.0] * 20)                                    # and straight again
    out = _run(s, _weave(0.0002, hz=1.0, seconds=1.0), gain=0.3)    # one whole cycle, inside the 2 s hold
    self.assertGreater(_amplitude(out) / 0.0002, 0.9)      # undamped: the peak is still held
    out = _run(s, _weave(0.0002, hz=1.0, seconds=40.0), gain=0.3)   # the hold, then ~10 s of release
    self.assertLess(_amplitude(out[-200:]) / 0.0002, 0.5)  # damped again

  def test_a_tight_curve_is_never_smoothed(self):
    s = HighwaySmoothing()
    kappa = (TIGHT_CURVE_OFF + 0.2) / V ** 2
    kappas = _weave(0.00005, seconds=40.0, offset=kappa)
    out = _run(s, kappas, gain=0.3)
    np.testing.assert_allclose(out[-200:], kappas[-200:], atol=1e-6)

  def test_a_predicted_curve_ahead_turns_smoothing_off(self):
    kappas = _weave(0.0002, seconds=20.0)
    damped = np.array(_run(HighwaySmoothing(), kappas, gain=0.3))
    ahead = np.array(_run(HighwaySmoothing(), kappas, gain=0.3, predicted=(2.0, 2.4)))
    self.assertLess(_amplitude(damped[-200:]), 0.6 * _amplitude(kappas[-200:]))
    self.assertGreater(_amplitude(ahead[-200:]), 0.9 * _amplitude(kappas[-200:]))

  def test_bypass_passes_through_and_fades_back_in_without_a_step(self):
    kappas = _weave(0.0002, seconds=30.0)
    s = HighwaySmoothing()
    out = _run(s, kappas[:300], gain=0.3)
    held = _run(s, kappas[300:360], gain=0.3, bypass=True)
    np.testing.assert_allclose(held[-20:], kappas[340:360], atol=1e-9)
    back = _run(s, kappas[360:], gain=0.3)
    # the first tick back moves by one fade step (dt / BYPASS_FADE_S = 0.1 of the full effect), never a jump
    full_effect = abs(kappas[360] - s.baseline) * (1.0 - 0.3)
    self.assertLessEqual(abs(back[0] - kappas[360]), 0.12 * full_effect + 1e-9)
    self.assertLess(_amplitude(back[-100:]), 0.6 * _amplitude(kappas[-100:]))  # and it does come back
    self.assertEqual(len(out), 300)

  def test_inactive_resets_and_passes_through(self):
    s = HighwaySmoothing()
    kappas = _weave(0.0002)
    _run(s, kappas)
    out = _run(s, kappas[:50], active=False)
    np.testing.assert_allclose(out, kappas[:50], atol=1e-12)
    self.assertEqual(s.weight, 0.0)

  def test_the_response_does_not_depend_on_the_tick_rate(self):
    amp = {}
    for dt in (0.01, 0.05):
      kappas = _weave(0.0002, dt=dt, seconds=40.0)
      out = np.array(_run(HighwaySmoothing(), kappas, gain=0.4, dt=dt))
      amp[dt] = _amplitude(out[int(15 / dt):]) / 0.0002
    self.assertAlmostEqual(amp[0.01], amp[0.05], delta=0.06)

  def test_bad_inputs_pass_through_and_leave_the_state_alone(self):
    s = HighwaySmoothing()
    _run(s, _weave(0.0002, seconds=10.0))
    before = (s.baseline, s.weight)
    for bad in (float("nan"), float("inf")):
      self.assertTrue(math.isnan(s.update(bad, V, True, 0.5, False, DT)) or s.update(bad, V, True, 0.5, False, DT) == bad)
    self.assertEqual((s.baseline, s.weight), before)
    self.assertEqual(s.update(0.001, V, True, 0.5, False, 0.0), 0.001)   # dt of 0
    self.assertEqual(s.update(0.001, float("nan"), True, 0.5, False, DT), 0.001)

  def test_the_gain_is_clamped(self):
    kappas = _weave(0.0002, seconds=30.0)
    low = np.array(_run(HighwaySmoothing(), kappas, gain=-5.0))
    floor = np.array(_run(HighwaySmoothing(), kappas, gain=0.1))
    np.testing.assert_allclose(low, floor)
    over = np.array(_run(HighwaySmoothing(), kappas, gain=7.0))
    np.testing.assert_allclose(over, kappas)


if __name__ == "__main__":
  unittest.main()

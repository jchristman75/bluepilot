"""BluePilot: tests for the Ford controller's param poll throttle (param_poll_bp.py)."""
import unittest

from opendbc.sunnypilot.car.ford.param_poll_bp import PARAM_POLL_FRAMES, ParamPoll


class TestParamPoll(unittest.TestCase):
  def test_the_first_frame_always_polls(self):
    # behavior from frame 0 must match a per-frame read, whatever frame the controller starts at
    for first in (0, 1, 37, 100_000):
      self.assertTrue(ParamPoll().due(first), first)

  def test_it_polls_once_per_period(self):
    poll = ParamPoll()
    polled = [f for f in range(1000) if poll.due(f)]
    self.assertEqual(polled, list(range(0, 1000, PARAM_POLL_FRAMES)))

  def test_the_period_is_two_hertz_at_the_control_rate(self):
    self.assertEqual(PARAM_POLL_FRAMES, 50)   # 100 Hz control loop

  def test_a_frame_counter_that_goes_backwards_polls_again(self):
    poll = ParamPoll()
    self.assertTrue(poll.due(5000))
    self.assertFalse(poll.due(5001))
    self.assertTrue(poll.due(0))              # new drive / controller reset
    self.assertFalse(poll.due(1))

  def test_skipped_frames_do_not_skip_a_poll(self):
    poll = ParamPoll()
    self.assertTrue(poll.due(0))
    self.assertFalse(poll.due(49))
    self.assertTrue(poll.due(120))            # late, not lost
    self.assertFalse(poll.due(121))

  def test_force_polls_next_frame(self):
    poll = ParamPoll()
    poll.due(0)
    poll.force()
    self.assertTrue(poll.due(1))
    self.assertFalse(poll.due(2))



class TestCarControllerPolling(unittest.TestCase):
  """End to end on the golden harness: the controller reads its params at 2 Hz, not every frame."""

  @staticmethod
  def _scenario():
    from opendbc.sunnypilot.car.ford.tests import long_harness as H
    sc = H.build_scenarios()[0]
    sc.frames = sc.frames[:1000]
    return H, sc

  def test_reads_drop_about_fifty_fold_and_the_output_is_unchanged(self):
    H, sc = self._scenario()
    baseline_acc, _ = H.run_scenario(sc)

    reads = []
    original = H.FakeParams.get
    H.FakeParams.get = lambda self, key, block=False, return_default=False: (reads.append(key), original(self, key, block, return_default))[1]
    try:
      acc, _ = H.run_scenario(sc)
    finally:
      H.FakeParams.get = original

    polls = len(sc.frames) // PARAM_POLL_FRAMES
    per_poll = len(reads) / polls
    self.assertGreater(per_poll, 15)                       # it does read the ~24 params when it polls...
    self.assertLess(len(reads), polls * 40)                # ...but only then (per frame would be ~24,000)
    self.assertLess(len(reads), len(sc.frames))            # fewer reads than frames
    self.assertTrue((acc == baseline_acc).all())

  def test_a_setting_flip_is_seen_within_one_poll_period(self):
    H, sc = self._scenario()
    flip_frame = 410
    key = "BPLongRegenBlend"
    seen = []

    original = H.FakeParams.get
    state = {"frame": 0}

    def get(self, k, block=False, return_default=False):
      if k == key:
        seen.append((state["frame"], k))
      return original(self, k, block, return_default)

    from opendbc.car.ford import carcontroller as cc_mod
    controller_update = cc_mod.CarController.update

    def counting_update(ctrl, *args, **kwargs):
      state["frame"] += 1    # one per control frame (100 Hz)
      return controller_update(ctrl, *args, **kwargs)

    H.FakeParams.get, cc_mod.CarController.update = get, counting_update
    try:
      H.run_scenario(sc)
    finally:
      H.FakeParams.get, cc_mod.CarController.update = original, controller_update

    frames_read = sorted({f for f, _ in seen})
    # every poll lands on a multiple of the period, so a flip at any frame is read at most one period later
    self.assertTrue(all((f - 1) % PARAM_POLL_FRAMES == 0 for f in frames_read), frames_read[:6])
    next_poll = next(f for f in frames_read if f > flip_frame)
    self.assertLessEqual(next_poll - flip_frame, PARAM_POLL_FRAMES)


if __name__ == "__main__":
  unittest.main()

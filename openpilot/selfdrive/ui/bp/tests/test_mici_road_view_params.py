"""MICI road view: display toggles are read about once a second, not every frame."""
from openpilot.selfdrive.ui.bp.mici.onroad import augmented_road_view_bp as view_mod


class CountingParams:
  def __init__(self, values):
    self.values = values
    self.reads = 0

  def get_bool(self, key, block=False):
    self.reads += 1
    return self.values.get(key, False)


def make_view(values):
  v = view_mod.MiciAugmentedRoadViewBP.__new__(view_mod.MiciAugmentedRoadViewBP)
  v._bp_params = CountingParams(values)
  v._hide_fade = v._hide_border = False
  v._param_frame_counter = view_mod.PARAM_REFRESH_FRAMES
  return v


def test_first_frame_reads_the_toggles():
  v = make_view({'mici_hide_onroad_fade': True, 'BPHideOnroadBorder': True})
  v._refresh_display_params()
  assert v._hide_fade and v._hide_border


def test_toggles_are_reread_once_per_refresh_period():
  v = make_view({})
  for _ in range(3 * view_mod.PARAM_REFRESH_FRAMES):
    v._refresh_display_params()
  assert v._bp_params.reads == 3 * 2  # two params, three refreshes (was two reads every frame)


def test_a_change_shows_up_within_one_period():
  v = make_view({})
  v._refresh_display_params()
  v._bp_params.values['mici_hide_onroad_fade'] = True
  for _ in range(view_mod.PARAM_REFRESH_FRAMES):
    v._refresh_display_params()
  assert v._hide_fade

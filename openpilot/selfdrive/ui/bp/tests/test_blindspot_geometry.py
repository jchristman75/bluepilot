"""Hatched blind spot lane geometry (CarrotPilot-style): placement, fallbacks and stripe layout."""
import numpy as np
import pytest

from openpilot.selfdrive.ui.bp.lib import blindspot_geometry as g


def line(y, x_max=100.0, n=34, z=0.0):
  xs = np.linspace(0.0, x_max, n)
  return np.stack([xs, np.full(n, y), np.full(n, z)], axis=1).astype(np.float32)


# right-positive y: the left side is negative
LEFT_EGO, LEFT_OUTER = line(-1.7), line(-5.1)
RIGHT_EGO, RIGHT_OUTER = line(1.7), line(5.1)
PATH = line(0.0)


def test_left_lane_sits_between_the_ego_line_and_the_outer_line():
  xs, inner, outer = g.adjacent_lane(LEFT_EGO, 0.9, LEFT_OUTER, 0.8, PATH, side=-1)
  assert np.allclose(inner[:, 1], -1.7) and np.allclose(outer[:, 1], -5.1)
  assert xs[0] >= g.X_NEAR and xs[-1] <= g.X_FAR


def test_right_lane_mirrors_the_left():
  _, inner, outer = g.adjacent_lane(RIGHT_EGO, 0.9, RIGHT_OUTER, 0.8, PATH, side=+1)
  assert np.allclose(inner[:, 1], 1.7) and np.allclose(outer[:, 1], 5.1)


def test_missing_or_unsure_outer_line_assumes_a_standard_lane():
  for outer, prob in ((None, 0.0), (LEFT_OUTER, 0.1)):
    _, inner, out = g.adjacent_lane(LEFT_EGO, 0.9, outer, prob, PATH, side=-1)
    assert np.allclose(inner[:, 1] - out[:, 1], g.LANE_WIDTH)


def test_an_outer_line_on_the_wrong_side_or_implausibly_far_is_ignored():
  on_the_wrong_side = line(-0.5)              # inside the ego lane
  far_away = line(-12.0)                       # 10 m out
  for outer in (on_the_wrong_side, far_away):
    _, inner, out = g.adjacent_lane(LEFT_EGO, 0.9, outer, 0.9, PATH, side=-1)
    assert np.allclose(inner[:, 1] - out[:, 1], g.LANE_WIDTH)


def test_faded_ego_line_places_the_lane_off_the_planned_path():
  _, inner, out = g.adjacent_lane(LEFT_EGO, 0.05, LEFT_OUTER, 0.9, PATH, side=-1)
  assert np.allclose(inner[:, 1], -g.LANE_WIDTH / 2)       # half a lane from the path
  assert np.allclose(inner[:, 1] - out[:, 1], g.LANE_WIDTH)  # the outer line is not trusted without the ego line
  _, inner_r, _ = g.adjacent_lane(None, 0.0, None, 0.0, PATH, side=+1)
  assert np.allclose(inner_r[:, 1], g.LANE_WIDTH / 2)


def test_path_fallback_adds_the_camera_height_so_the_lane_sits_on_the_road():
  """Real data: path z is ~0 while lane line z is ~camera height (1.3 m). Without the offset the fallback lane
  floats 1.3 m up and flattens into a line at the horizon (route 511 seg 1, left line confidence 0.22)."""
  path = line(0.0, z=0.03)
  ego = line(-1.7, z=1.33)
  _, inner, _ = g.adjacent_lane(ego, 0.9, None, 0.0, path, side=-1)
  assert np.allclose(inner[:, 2], 1.33)                    # placed off the ego line: its own height
  _, inner_p, out_p = g.adjacent_lane(ego, 0.05, None, 0.0, path, side=-1, path_z_offset=1.33)
  assert np.allclose(inner_p[:, 2], 1.36) and np.allclose(out_p[:, 2], 1.36)


def test_nothing_to_draw_without_usable_geometry():
  assert g.adjacent_lane(None, 0.0, None, 0.0, None, side=-1) is None
  short = line(-1.7, x_max=5.0)               # ends before the drawn range
  assert g.adjacent_lane(short, 0.9, None, 0.0, None, side=-1) is None


def test_lane_follows_a_curving_line_and_keeps_its_height():
  xs = np.linspace(0, 100, 34)
  curved = np.stack([xs, -1.7 - 0.002 * xs ** 2, np.full(34, 0.4)], axis=1).astype(np.float32)
  _, inner, out = g.adjacent_lane(curved, 0.9, None, 0.0, PATH, side=-1)
  assert inner[-1, 1] < inner[0, 1]                         # bends away
  assert np.allclose(inner[:, 2], 0.4) and np.allclose(out[:, 2], 0.4)


def lane():
  return g.adjacent_lane(LEFT_EGO, 0.9, LEFT_OUTER, 0.8, PATH, side=-1)


def test_stripes_are_diagonal_quads_inside_the_lane():
  xs, inner, outer = lane()
  st = g.stripes(xs, inner, outer)
  assert len(st) >= 5
  for quad, fade in st:
    assert quad.shape == (4, 3) and g.MIN_FADE <= fade <= 1.0
    # corners 0,1 on the ego line, corners 2,3 on the outer line; the outer end is further along the road
    assert np.allclose(quad[:2, 1], -1.7) and np.allclose(quad[2:, 1], -5.1)
    assert quad[3, 0] - quad[0, 0] == pytest.approx(g.STRIPE_SLANT, abs=1e-3)
    assert quad[1, 0] - quad[0, 0] == pytest.approx(g.STRIPE_WIDTH, abs=1e-3)
    assert quad[0, 0] >= xs[0] and quad[2, 0] <= xs[-1] + 1e-6


def test_stripes_are_evenly_spaced_and_fade_with_distance():
  xs, inner, outer = lane()
  st = g.stripes(xs, inner, outer)
  starts = np.array([q[0, 0] for q, _ in st])
  assert np.allclose(np.diff(starts), g.STRIPE_PERIOD)
  fades = [f for _, f in st]
  assert fades == sorted(fades, reverse=True) and fades[0] > fades[-1]


def test_phase_slides_the_stripes_toward_the_car():
  xs, inner, outer = lane()
  a = [q[0, 0] for q, _ in g.stripes(xs, inner, outer, 0.0)]
  b = [q[0, 0] for q, _ in g.stripes(xs, inner, outer, 1.0)]
  # every stripe in b is a stripe of a moved 1 m closer (or a new one at the far end)
  assert np.isclose(min(b) % g.STRIPE_PERIOD, (min(a) - 1.0) % g.STRIPE_PERIOD)
  assert all(x0 >= xs[0] for x0 in b)


def test_a_drawing_failure_disables_the_overlay_instead_of_crashing_the_ui():
  from openpilot.selfdrive.ui.bp.onroad.blindspot_path import BlindspotPathMixin

  class Host(BlindspotPathMixin):
    calls = 0
    _bsp_failed = False

    def _draw_blindspot_paths_impl(self):
      Host.calls += 1
      raise RuntimeError("boom")

  h = Host()
  h._draw_blindspot_paths()            # must not raise
  h._draw_blindspot_paths()
  assert Host.calls == 1 and h._bsp_failed   # tried once, then stays off

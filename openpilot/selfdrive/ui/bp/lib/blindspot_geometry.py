"""
BluePilot: road-space geometry for the yellow hatched blind spot lane (CarrotPilot-style).

Pure numpy, no UI imports, so it is unit-testable. Everything is in the model's car space
(x forward, y lateral with the right side positive, z), the same space as the model renderer's
lane line `raw_points`, so the projected result lines up with the lane lines on screen.

The warning covers the lane next to the ego lane on the side with a blind spot detection with diagonal
stripes laid across it every few meters, like CarrotPilot's hatched lane. Stripes stay fixed in the car's
frame (`phase` 0); a caller can advance `phase` with distance traveled to make them slide like paint.

The next lane line is only used when the model sees it and it sits a plausible lane width away;
otherwise the lane is assumed LANE_WIDTH wide. Without a usable ego lane line (faded paint) the lane
is placed off the planned path instead.
"""
import numpy as np

X_NEAR = 4.0              # m, nearest distance drawn
X_FAR = 45.0              # m, farthest distance drawn
BAND_STEP = 2.5           # m between samples of the lane geometry
LANE_WIDTH = 3.4          # m, assumed adjacent lane width when the model has no usable outer line
MIN_LINE_PROB = 0.25      # ego lane line confidence needed to place the band off it
MIN_OUTER_PROB = 0.3      # outer lane line confidence needed to use it
MIN_GAP, MAX_GAP = 2.4, 4.8   # m, plausible ego-line to outer-line spacing
STRIPE_PERIOD = 4.0       # m between stripe starts along the road
STRIPE_WIDTH = 1.4        # m, stripe thickness along the road
STRIPE_SLANT = 3.2        # m the far (outer) end of a stripe is shifted forward: the diagonal (about 45 degrees)
MIN_FADE = 0.35           # farthest stripes keep this fraction of full opacity


def _line_at(line: np.ndarray, xs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  return np.interp(xs, line[:, 0], line[:, 1]), np.interp(xs, line[:, 0], line[:, 2])


def _usable(line) -> bool:
  return line is not None and getattr(line, 'shape', (0,))[0] > 1


def adjacent_lane(inner, inner_prob: float, outer, outer_prob: float, path, side: int,
                  x_near: float = X_NEAR, x_far: float = X_FAR, path_z_offset: float = 0.0):
  """The adjacent lane on `side` (-1 left, +1 right) as sampled (xs, inner3, outer3) arrays, or None.

  inner / outer / path: (N, 3) raw points [x, y, z] of the ego-side lane line, the next lane line out
  and the planned path. The model's path z is relative to the ground under the car while its lane line
  z already includes the camera height, so `path_z_offset` (the renderer's _path_offset_z, the camera
  height) must be passed whenever the path is used in place of the ego line.
  """
  if _usable(inner) and inner_prob >= MIN_LINE_PROB:
    base, base_prob = inner, inner_prob
    iy_off = 0.0
    z_off = 0.0
  elif _usable(path):
    base, base_prob = path, 1.0
    iy_off = side * LANE_WIDTH / 2.0     # the ego lane edge is half a lane from the path
    z_off = path_z_offset
  else:
    return None

  hi = min(float(base[-1, 0]), x_far)
  lo = max(float(base[0, 0]), x_near)
  if hi - lo < 2 * BAND_STEP:
    return None
  xs = np.arange(lo, hi + 1e-6, BAND_STEP)

  iy, iz = _line_at(base, xs)
  iy = iy + iy_off
  iz = iz + z_off
  gap = np.full_like(xs, LANE_WIDTH)
  if _usable(outer) and outer_prob >= MIN_OUTER_PROB and base_prob >= MIN_LINE_PROB:
    oy, _ = _line_at(outer, xs)
    seen = (oy - iy) * side              # lateral distance outward; must point the right way
    if np.all(seen > 0.5) and MIN_GAP <= float(np.median(seen)) <= MAX_GAP:
      gap = np.clip(seen, MIN_GAP, MAX_GAP)
  oy = iy + side * gap
  inner3 = np.stack([xs, iy, iz], axis=1)
  outer3 = np.stack([xs, oy, iz], axis=1)
  return xs, inner3, outer3


def stripes(xs: np.ndarray, inner3: np.ndarray, outer3: np.ndarray, phase: float = 0.0,
            period: float = STRIPE_PERIOD, width: float = STRIPE_WIDTH, slant: float = STRIPE_SLANT):
  """Diagonal stripes across the lane: list of ((4, 3) quad, fade 0..1), nearest first.

  A stripe starts at `s` on the ego line and ends at `s + slant` on the outer line; `phase` (meters, the
  caller advances it with distance traveled) slides the pattern toward the car.
  """
  x_lo, x_hi = float(xs[0]), float(xs[-1])
  out = []
  s = x_lo - ((x_lo + phase) % period)
  s += period if s < x_lo else 0.0
  span = max(x_hi - x_lo, 1e-6)
  while s + slant + width <= x_hi:
    ys_i = np.interp([s, s + width], xs, inner3[:, 1])
    zs_i = np.interp([s, s + width], xs, inner3[:, 2])
    ys_o = np.interp([s + slant, s + slant + width], xs, outer3[:, 1])
    zs_o = np.interp([s + slant, s + slant + width], xs, outer3[:, 2])
    quad = np.array([[s, ys_i[0], zs_i[0]],
                     [s + width, ys_i[1], zs_i[1]],
                     [s + slant + width, ys_o[1], zs_o[1]],
                     [s + slant, ys_o[0], zs_o[0]]], dtype=np.float32)
    fade = float(np.clip(1.0 - (s - x_lo) / span, MIN_FADE, 1.0))
    out.append((quad, fade))
    s += period
  return out

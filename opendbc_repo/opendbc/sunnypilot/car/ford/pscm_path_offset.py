"""
BluePilot: angle-mode path offset (LMC2 c0, ``LatCtlPathOffst_L_Actl``) for the PSCM.

The PSCM Walkthrough (F-150 Lightning reference firmware, Sep 2026) recovered the PSCM's request
stage as ``request curvature = g0 * (held C0 + integral) + g1 * held C1``: only the path offset
feeds the PSCM's own integrator. Angle mode used to hold c0 at zero, so nothing rejected a steady
disturbance -- crown, or the ~2e-3 1/m bias the Mach-E PSCM carries for ~5 s after the driver lets
go -- and our own lead is capped at the deviation-clip tolerance (see _STALL_REVERSED_FLOOR_RATIO in
lateral_angle_ext.py). c0 is the lateral offset of the target path from the car, the signal the
PSCM's path follower is built around; it is bounded by ford.h's path-offset range (+-1 m) and
rate-of-change check, not by the shadow-curvature deviation check.

Target path: the same laneline-confidence blend lane_center_trim.py uses (lane-line center when
lines are trustworthy, the model's own path otherwise), sampled at the PSCM's preview distance
d_ref, plus the user's left/right bias when lane positioning is enabled -- so the curvature-domain
trim and c0 push toward one target instead of fighting.

Supervisor-aware cap: the Walkthrough found the PSCM's path supervisor saturates its c0 term at
1.0 m and, on release, keeps excluding a zero demand until held c0 slews back below ~0.41 m; every
metre of held c0 above that is release hang. _C0_MAX_M stays inside the unsaturated range.
"""
import numpy as np
from numpy import interp

from opendbc.sunnypilot.car.ford.lane_center_trim import laneline_blend

_C0_GAIN = 0.5         # fraction of the geometric offset sent; the planner's c1 already steers toward it
_C0_MAX_M = 0.4        # below the supervisor's ~0.41 m release threshold (hang-free) and its 1.0 m saturation
_SMOOTH_TAU_S = 0.4    # lane-line detection noise filter, same as lane_center_trim.py
_DT = 0.05             # BluePilot lateral tick (20 Hz)

# Authority envelope: none below 9 m/s (the deviation clip and lane lines are both unreliable in
# intersections and nav turns), full from 15 m/s -- lane_center_trim.py's speed ramp.
_SPEED_RAMP_BP = (0.0, 9.0, 15.0)
_SPEED_RAMP_V = (0.0, 0.0, 1.0)

# ford.h path_offset_cmd_checks: interp(vehicle_speed.min - 1, [5, 15, 25], [0.05, 0.025, 0.01]) m
# per LMC2 frame, +1 CAN unit (0.01 m). Mirrored with margin -- extra speed shift and 0.8x -- so
# wire quantization (0.01 m) never lands on the edge.
_PANDA_ROC_SPEED_BP = (5.0, 15.0, 25.0)
_PANDA_ROC_M = (0.05, 0.025, 0.01)
_ROC_MARGIN = 0.8
_ROC_SPEED_SHIFT = 1.5


def c0_roc_per_frame(v_ego: float) -> float:
  return _ROC_MARGIN * float(interp(v_ego - _ROC_SPEED_SHIFT, _PANDA_ROC_SPEED_BP, _PANDA_ROC_M))


class PscmPathOffset:
  def __init__(self):
    self._filtered = 0.0
    self._c0 = 0.0

  def reset(self) -> None:
    """Call on every frame c0 isn't sent (mode 0): the wire is zero, so the next ramp starts there."""
    self._filtered = 0.0
    self._c0 = 0.0

  @property
  def c0(self) -> float:
    return self._c0

  def update(self, model, v_ego: float, d_ref: float, offset: float, lane_change: bool) -> float:
    """Returns c0 (m, internal sign: positive = target path to the RIGHT, like model y; carcontroller
    negates it on the wire exactly as it does path_angle). Always rate-limited toward its target,
    including toward zero during lane changes, so ford.h's c0 rate check can't trip mid-maneuver."""
    speed_factor = float(interp(v_ego, _SPEED_RAMP_BP, _SPEED_RAMP_V))
    target = 0.0
    if not lane_change and speed_factor > 0.0 and model is not None and np.isfinite(offset):
      target_y = self._target_y(model, d_ref)
      if target_y is not None:
        target = float(np.clip(_C0_GAIN * (target_y + offset), -_C0_MAX_M, _C0_MAX_M)) * speed_factor

    alpha = 1.0 - np.exp(-_DT / _SMOOTH_TAU_S)
    self._filtered += alpha * (target - self._filtered)
    roc = c0_roc_per_frame(v_ego)
    self._c0 = float(np.clip(self._filtered, self._c0 - roc, self._c0 + roc))
    return self._c0

  @staticmethod
  def _target_y(model, d_ref: float) -> float | None:
    try:
      pos_x = np.asarray(model.position.x, dtype=float)
      pos_y = np.asarray(model.position.y, dtype=float)
      if pos_x.size < 2 or pos_x.size != pos_y.size or not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all()):
        return None
      model_y = float(np.interp(d_ref, pos_x, pos_y))
    except (AttributeError, IndexError, TypeError, ValueError):
      return None
    scale, center_y = laneline_blend(model, d_ref)
    return model_y * (1.0 - scale) + center_y * scale

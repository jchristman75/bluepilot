"""BluePilot: adjustable follow time for the standard driving personality (BPFollowTimeStandard).

Stock openpilot follows at 1.45 s on standard (1.25 s aggressive, 1.75 s relaxed). The planner adapter below reads the
param every few seconds and sets long_mpc.STANDARD_T_FOLLOW, which get_T_FOLLOW() returns for the standard personality.
Everything that scales or overrides the follow time (BPSmoothFollow, the lane-change gap) starts from that value.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL

DEFAULT_T_FOLLOW = 1.25
MIN_T_FOLLOW = 1.0
MAX_T_FOLLOW = 1.75
REFRESH_S = 3.0


def clamp_t_follow(value) -> float:
  """The follow time to use for a raw param value: a bad or missing value falls back to the default."""
  try:
    v = float(value)
  except (TypeError, ValueError):
    return DEFAULT_T_FOLLOW
  if not np.isfinite(v):
    return DEFAULT_T_FOLLOW
  return float(np.clip(v, MIN_T_FOLLOW, MAX_T_FOLLOW))


class FollowTimePlanner:
  """Planner-side adapter: reads BPFollowTimeStandard every few seconds (a key this build doesn't know reads as the
  default) and publishes it to the MPC module."""

  def __init__(self):
    from openpilot.common.params import Params
    self.params = Params()
    self.frame = 0
    self.t_follow = DEFAULT_T_FOLLOW

  def update(self) -> float:
    if self.frame % int(REFRESH_S / DT_MDL) == 0:
      try:
        raw = self.params.get("BPFollowTimeStandard", return_default=True)
        self.t_follow = DEFAULT_T_FOLLOW if raw in (None, b"", "") else clamp_t_follow(raw)
      except Exception:
        self.t_follow = DEFAULT_T_FOLLOW
      from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
      long_mpc.STANDARD_T_FOLLOW = self.t_follow
    self.frame += 1
    return self.t_follow

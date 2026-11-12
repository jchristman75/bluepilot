"""
BluePilot: shorter follow gap while signaling out of the lane (choice C of the StarPilot ideas plan).

Behind a slower car, op long keeps the full follow gap to it even when you are changing lanes out from behind
it, so it brakes or holds back through the maneuver. While a lane change is pending or in progress, this
ramps the MPC's follow time down to REQUEST_S, so the car can accelerate through the maneuver, and ramps it
back to the personality's follow time quickly afterwards.

Same idea as StarPilot's lane-change close gap, with BluePilot's stricter gates: no reduction toward an
occupied blind spot on the lane-change side, below the lane-change speed, at standstill, behind a lead that is
braking, or behind a lead that has no radar track (a vision-only distance is too noisy to close in on). The
result is never longer than the stock follow time and never shorter than MIN_T_FOLLOW. The MPC's safe-stopping
distance term (v^2 / 2 COMFORT_BRAKE) is unchanged: only the time-headway part of the desired gap shrinks.

Pure logic with no params or messaging: the caller passes the gate inputs and the BPLaneChangeCloseGap flag.
"""
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.bluepilot.selfdrive.controls.param_flag_bp import read_flag
from opendbc.sunnypilot.car.ford.lane_change_gate import ABORT_LEAD_BRAKE, MIN_SPEED, close_gap_ok  # noqa: F401 (re-exported)

DT = DT_MDL
REQUEST_S = 1.0                 # s: follow time while changing lanes
MIN_T_FOLLOW = 0.9              # s: hard floor (StarPilot's is 0.25)
RAMP_IN = 0.6                   # s of follow time per second, toward the shorter gap
RAMP_OUT = 4.0                  # s per second, back to the normal gap


def enum_name(enum, value) -> str:
  """The name of a cereal enum value, given a name, a capnp enum or a plain int ordinal."""
  if isinstance(value, int):
    return next(n for n, i in enum.schema.enumerants.items() if i == value)
  return str(value)


class LaneChangeGap:
  def __init__(self):
    self.reset()

  def reset(self):
    self.t = None

  def update(self, t_follow: float, v_ego: float, lane_change_state, direction, left_blindspot: bool, right_blindspot: bool,
             lead_present: bool, lead_radar: bool, lead_brake: float, standstill: bool, enabled: bool, dt: float = DT) -> float:
    """The follow time (s) the MPC should use this frame."""
    if not enabled:
      self.reset()
      return t_follow
    ok = close_gap_ok(enum_name(log.LaneChangeState, lane_change_state), enum_name(log.LaneChangeDirection, direction),
                      left_blindspot, right_blindspot, v_ego, lead_present, lead_radar, lead_brake, standstill)
    target = min(t_follow, REQUEST_S) if ok else t_follow
    if self.t is None:
      self.t = t_follow
    step = (RAMP_IN if target < self.t else RAMP_OUT) * dt
    self.t = min(max(target, self.t - step), self.t + step)
    return min(t_follow, max(self.t, MIN_T_FOLLOW))


class LaneChangeGapPlanner:
  """Planner-side adapter: reads BPLaneChangeCloseGap and the messages, returns the MPC follow-time override
  (None = stock). The param is re-read every few seconds; a key this build doesn't know yet reads as off."""

  def __init__(self):
    from openpilot.common.params import Params
    self.params = Params()
    self.gap = LaneChangeGap()
    self.enabled = False
    self.frame = 0
    self.last = None        # last follow time returned, for logs and tests

  def t_follow(self, sm, personality):
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
    if self.frame % int(3 / DT) == 0:
      self.enabled = read_flag(self.params, "BPLaneChangeCloseGap")
    self.frame += 1
    if not self.enabled:
      self.gap.reset()
      self.last = None
      return None
    meta, cs, lead = sm['modelV2'].meta, sm['carState'], sm['radarState'].leadOne
    self.last = self.gap.update(get_T_FOLLOW(personality), cs.vEgo, meta.laneChangeState, meta.laneChangeDirection,
                                bool(cs.leftBlindspot), bool(cs.rightBlindspot), bool(lead.present), bool(lead.radar),
                                max(0.0, -float(lead.aLeadK)), bool(cs.standstill), True)
    return self.last

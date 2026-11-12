"""
BluePilot: the one definition of "it is safe to close the follow gap while changing lanes" (BPLaneChangeCloseGap).

Two processes act on it and must never disagree: the planner shortens the MPC's follow time
(bluepilot/selfdrive/controls/lane_change_gap_bp.py) and Ford's BP follow stops capping gas against the lead
being left (longitudinal_ext.py). Pure logic on plain values, so opendbc needs no openpilot imports; states and
directions are the names of cereal's log.LaneChangeState / log.LaneChangeDirection values.
"""

ACTIVE_STATES = ('preLaneChange', 'laneChangeStarting', 'laneChangeFinishing')
MIN_SPEED = 20 * 0.44704        # m/s: the lane-change minimum speed (20 mph)
ABORT_LEAD_BRAKE = 0.8          # m/s^2: a lead braking harder than this ends the reduction


def close_gap_ok(state: str, direction: str, left_blindspot: bool, right_blindspot: bool, v_ego: float,
                 lead_present: bool, lead_radar: bool, lead_brake: float, standstill: bool) -> bool:
  """A lane change is pending or in progress, with a radar-tracked lead that is not braking hard, above the
  lane-change speed, and nothing in the blind spot on the lane-change side (either side if the direction is unknown)."""
  if state not in ACTIVE_STATES:
    return False
  if standstill or v_ego < MIN_SPEED or not lead_present or not lead_radar or lead_brake >= ABORT_LEAD_BRAKE:
    return False
  if direction == 'left':
    return not left_blindspot
  if direction == 'right':
    return not right_blindspot
  return not (left_blindspot or right_blindspot)

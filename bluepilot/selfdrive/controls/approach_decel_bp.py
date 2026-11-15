"""
BluePilot: soft approach to a far lead on the highway (BPSoftApproach).

The planner's MPC keeps a safe following distance that includes a stopping-distance term, so a lead 100 m ahead that
is only slightly slower reads as "too close" and gets -1.0..-1.6 m/s^2, which also lands on the friction brakes. On
your drives (routes 56c/56e) closing on a lead needed at most 0.54 m/s^2 (median 0.01-0.05) at 8-40 s to impact. A
driver eases off and coasts closer; friction brakes are for emergencies.

This turns the time to impact into the decel that is actually needed, with a margin, and caps the MPC's lead request
at that. The gap is still the planner's to manage once it is close, and nothing here applies in an emergency:

  needed = closing_speed^2 / (2 * (gap - END_GAP_S * v_ego))    match the lead's speed with END_GAP_S of gap to spare
  cap    = MARGIN * needed + the lead's own decel (it is already slowing), never below CAP_MIN

No cap (the planner as is) when: not on the highway, no radar-backed lead, not closing, the gap is already inside the
safe margin (a cut-in), time to impact is short, the lead is braking hard, or the cap would be high anyway.

Pure logic with no params or messaging; SoftApproachPlanner is the planner-side adapter.
"""
from openpilot.bluepilot.selfdrive.controls.param_flag_bp import read_flag
from openpilot.common.realtime import DT_MDL

V_MIN = 20.0              # m/s (45 mph): the highway
CLOSING_MIN = 0.5         # m/s
END_GAP_S = 0.8           # s of gap left when the speeds match
MARGIN = 1.3
CAP_MIN = 0.35            # m/s^2: never cap below this (a gentle coast/regen)
CAP_MAX = 2.0             # m/s^2: a cap this high means it is not a gentle approach: leave it to the planner
URGENT_TTC_S = 5.0        # s: a shorter time to impact is an emergency
LEAD_BRAKE_URGENT = 1.0   # m/s^2: a lead braking harder than this is an emergency


def approach_decel_cap(v_ego: float, d_rel: float, v_rel: float, a_lead: float, radar: bool, present: bool) -> float | None:
  """Largest decel (m/s^2, positive) to allow while closing on this lead, or None for no cap."""
  if not present or not radar or v_ego < V_MIN:
    return None
  closing = -v_rel
  if closing < CLOSING_MIN:
    return None
  if a_lead < -LEAD_BRAKE_URGENT:
    return None
  if d_rel / closing < URGENT_TTC_S:
    return None
  usable = d_rel - END_GAP_S * v_ego
  if usable <= 1.0:
    return None
  cap = MARGIN * closing ** 2 / (2.0 * usable) + max(-a_lead, 0.0)
  cap = max(cap, CAP_MIN)
  return None if cap > CAP_MAX else cap


def apply_cap(accel: float, cap: float | None) -> float:
  """accel (m/s^2, negative = decel) limited to a decel of `cap`; only ever softens."""
  return accel if cap is None else max(accel, -cap)


class SoftApproachPlanner:
  """Planner-side adapter: reads BPSoftApproach (re-read every few seconds; a key this build doesn't know reads as
  off) and caps the MPC's lead request. `last` is the cap used last frame, for logs and tests."""

  def __init__(self):
    from openpilot.common.params import Params
    self.params = Params()
    self.enabled = False
    self.frame = 0
    self.last: float | None = None

  def cap_mpc_request(self, sm, accel: float, mpc_source, lead_sources) -> float:
    if self.frame % int(3 / DT_MDL) == 0:
      self.enabled = read_flag(self.params, "BPSoftApproach")
    self.frame += 1
    self.last = None
    if not self.enabled or mpc_source not in lead_sources:
      return accel
    lead = sm['radarState'].leadOne if mpc_source == lead_sources[0] else sm['radarState'].leadTwo
    self.last = approach_decel_cap(float(sm['carState'].vEgo), float(lead.dRel), float(lead.vRel), float(lead.aLeadK),
                                   bool(lead.radar), bool(lead.present))
    return apply_cap(accel, self.last)

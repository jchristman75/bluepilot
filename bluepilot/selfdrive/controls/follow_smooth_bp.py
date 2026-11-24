"""
BluePilot: smoother, slightly closer following (BPSmoothFollow).

Behind a bus (route 59d, a stop-and-go city bus at 30-40 mph) the MPC copies every speed wobble of the lead: it sees
the lead speed (+-4 mph every ~8 s) and the Kalman lead accel (+-1 m/s^2) and speeds up and slows down with it, at a
2 s gap that is larger than the follow time asks for. Two changes, both on what the MPC is given (the MPC and its
safety logic are untouched):

  1. The lead's speed and accel are averaged. The speed the MPC sees is the lead's own average (tau 2.5 s) plus a
     fraction K of how far the lead is from it, so small wobbles are damped and the gap breathes a little instead of
     the car chasing them. Real changes pass through: a deviation beyond D0 counts in full (the optimism fades out
     by D1), a lead braking at least A_BRAKE is not smoothed at all, and nothing is smoothed for a lead without a radar track,
     closer than MIN_GAP_S, or closing with a short time to impact.
  2. The lead's distance is averaged too (tau TAU_D), within +-BAND_S seconds of the real one, so the gap the car
     tries to hold floats a little instead of every wobble of the bus moving the boundary the MPC regulates to.
     The distance the MPC sees is never farther than the real one.
  3. The follow time is scaled (BPFollowGapScale, default 0.75: standard 1.25 s by default -> 0.94 s) and floored at MIN_T_FOLLOW.

Pure logic here; the planner adapter at the bottom reads the params.
"""
from types import SimpleNamespace

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.bluepilot.selfdrive.controls.param_flag_bp import read_flag

TAU = 3.0               # s: the lead-speed average
TAU_D = 3.0             # s: the lead-distance average
BAND_S = 0.4           # s of headway: the distance the MPC sees stays within +-BAND_S x v_ego of the real one
K = 0.5                 # fraction of a small deviation from the average that the MPC sees
K_A = 0.25               # fraction of a small lead accel that the MPC sees
D0 = 2.0                # m/s: deviations up to this are damped; beyond it they count in full
D1 = 4.0                # m/s: the downward (lead slower than its average) optimism has faded out here
A_BRAKE = -1.0          # m/s^2: a lead braking at least this hard is passed through untouched (nothing smoothed)
MIN_GAP_S = 1.0         # s: closer than this nothing is smoothed
TTC_MIN = 6.0           # s: closing on the lead faster than this passes through
RESET_JUMP = 5.0        # m: the lead distance jumped (cut-in/out, new track): start the average over
MIN_T_FOLLOW = 1.0      # s: never follow closer than this in time (plus the MPC's stop distance)
DEFAULT_GAP_SCALE = 0.75
MIN_GAP_SCALE = 0.6


def scaled_t_follow(t_personality: float, scale: float) -> float:
  """The follow time to use: the personality's scaled by `scale`, floored, and never longer than the personality's."""
  return float(min(t_personality, max(t_personality * scale, MIN_T_FOLLOW)))


class FollowSmoother:
  def __init__(self):
    self.reset()

  def reset(self):
    self.v_f = None
    self.d_f = None
    self.d_prev = None

  def update(self, present, radar, d_rel, v_lead, a_lead, v_ego, dt=DT_MDL):
    """(v_lead, a_lead, d_rel, active): what the MPC should be given for this lead."""
    if not present or not radar:
      self.reset()
      return v_lead, a_lead, d_rel, False
    if self.v_f is None or self.d_prev is None or abs(d_rel - self.d_prev) > RESET_JUMP:
      self.v_f = v_lead
      self.d_f = d_rel
    self.d_prev = d_rel
    self.v_f += (v_lead - self.v_f) * dt / TAU
    self.d_f += (d_rel - self.d_f) * dt / TAU_D
    closing = v_ego - v_lead
    gap_s = d_rel / max(v_ego, 1.0)
    if gap_s < MIN_GAP_S or (closing > 0.5 and d_rel / closing < TTC_MIN) or a_lead <= A_BRAKE:
      return v_lead, a_lead, d_rel, False
    delta = v_lead - self.v_f
    ad = abs(delta)
    if delta >= 0.0:
      opt = (1.0 - K) * min(ad, D0)                       # lead faster than its average: conservative, lags a little
      v_eff = self.v_f + delta - opt
    else:
      opt = float(np.interp(ad, [0.0, D0, D1], [(1.0 - K) * ad, (1.0 - K) * D0, 0.0]))   # slower: optimism that fades
      v_eff = self.v_f + delta + opt
    a_eff = K_A * a_lead
    band = BAND_S * max(v_ego, 1.0)
    d_eff = float(np.clip(self.d_f, d_rel - band, d_rel))     # never farther than it really is
    return float(v_eff), float(a_eff), d_eff, True


class LeadProxy:
  """A radarState lead with its speed and accel replaced (everything else read through)."""

  def __init__(self, lead, v_lead, a_lead, d_rel):
    self._lead, self.vLead, self.aLeadK, self.dRel = lead, v_lead, a_lead, d_rel

  def __getattr__(self, name):
    return getattr(self._lead, name)


def _read_float(params, key, default):
  try:
    v = params.get(key, return_default=True)
    return default if v in (None, b"", "") else float(v)
  except Exception:
    return default


class FollowSmoothPlanner:
  """Planner-side adapter: reads BPSmoothFollow (+ BPFollowGapScale) every few seconds (a key this build doesn't know
  reads as off / the default) and hands the MPC the smoothed lead and the follow time."""

  def __init__(self):
    from openpilot.common.params import Params
    self.params = Params()
    self.enabled = False
    self.gap_scale = DEFAULT_GAP_SCALE
    self.frame = 0
    self.smoother = FollowSmoother()
    self.active = False

  def _refresh(self):
    if self.frame % int(3 / DT_MDL) == 0:
      self.enabled = read_flag(self.params, "BPSmoothFollow")
      self.gap_scale = float(np.clip(_read_float(self.params, "BPFollowGapScale", DEFAULT_GAP_SCALE), MIN_GAP_SCALE, 1.0))
    self.frame += 1

  def update(self, sm, t_personality: float, t_override):
    """(radarState for the MPC, follow time or None for the personality's own)."""
    self._refresh()
    radar = sm['radarState']
    self.active = False
    if not self.enabled:
      self.smoother.reset()
      return radar, t_override
    t = scaled_t_follow(t_personality, self.gap_scale)
    t = t if t_override is None else min(t, float(t_override))
    lead = radar.leadOne
    v, a, d, self.active = self.smoother.update(bool(lead.present), bool(lead.radar), float(lead.dRel), float(lead.vLead),
                                             float(lead.aLeadK), float(sm['carState'].vEgo))
    if self.active:
      return SimpleNamespace(leadOne=LeadProxy(lead, v, a, d), leadTwo=radar.leadTwo), t
    return radar, t

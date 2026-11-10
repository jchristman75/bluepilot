"""
BluePilot: radard lead matching for many radar tracks (the raw Delphi MRR tracker).

Stock radard picks the most likely track for the vision lead and only then sanity-checks it. With one
radar point (Steer_Assist_Data) that is fine; with ~20-70 raw tracks it is not:
  1. The big (eGPU) models report lead xStd / vStd around 6e4, so the likelihood is effectively
     lateral-only and the "best" track is often clutter far up the lane; it fails the sanity check and
     the lead drops to vision-only although the right track is there (lead flipped radar<->vision
     30-60 times a minute).
     -> clamp the stds to sane values and gate BEFORE choosing.
  2. Near the 25 % range gate the match toggles between frames (route 530 t=85: the camera put the
     lead at ~75 m, the radar at ~92 m).
     -> keep the track already matched to a lead while it stays inside a looser gate (sticky).
  3. potential_low_speed_lead() makes any stationary radar point within 25 m a lead at low speed
     without the camera; raw tracks include curbs and parked cars (phantom leads in lots).
     -> raw tracks never do that; the Steer_Assist_Data object still can.

install() swaps these into the radard module of the current process only. bp_radard calls it; the
stock radard process is not affected.
"""
from openpilot.bluepilot.selfdrive.controls.radar.mrr_tracker import RAW_ID_BASE

MAX_X_STD_FRAC, MIN_X_STD = 0.1, 2.0   # m: range std clamped to max(2 m, 10 % of range)
MAX_Y_STD, MAX_V_STD = 2.0, 2.0        # m, m/s
GATE_Y = 2.5                           # m, lateral gate (stock has none)
STICKY_D_FRAC, STICKY_D_MIN, STICKY_Y, STICKY_V = 0.35, 8.0, 3.0, 12.0


class LeadMatcher:
  def __init__(self, radard_mod):
    self.R = radard_mod
    self.last: dict[str, int | None] = {}
    self.key = 'one'

  def _sane(self, c, vd, lead, v_ego, d_frac, d_min, gate_y, gate_v):
    return (abs(c.dRel - vd) < max(vd * d_frac, d_min) and abs(c.yRel + lead.y[0]) < gate_y and
            ((abs(c.vRel + v_ego - lead.v[0]) < gate_v) or (v_ego + c.vRel > 3)))

  def match(self, v_ego, lead, tracks):
    vd = lead.x[0] - self.R.RADAR_TO_CAMERA
    last = self.last.get(self.key)
    if last in tracks and self._sane(tracks[last], vd, lead, v_ego, STICKY_D_FRAC, STICKY_D_MIN, STICKY_Y, STICKY_V):
      return tracks[last]
    cands = [c for c in tracks.values() if self._sane(c, vd, lead, v_ego, 0.25, 5.0, GATE_Y, 10.0)]
    if not cands:
      self.last[self.key] = None
      return None
    xs = min(lead.xStd[0], max(MIN_X_STD, MAX_X_STD_FRAC * vd))
    ys = min(lead.yStd[0], MAX_Y_STD)
    vs = min(lead.vStd[0], MAX_V_STD)
    pdf = self.R.laplacian_pdf
    best = max(cands, key=lambda c: pdf(c.dRel, vd, xs) * pdf(c.yRel, -lead.y[0], ys) * pdf(c.vRel + v_ego, lead.v[0], vs))
    self.last[self.key] = best.identifier
    return best


def install(radard_mod) -> LeadMatcher:
  matcher = LeadMatcher(radard_mod)
  stock_get_lead = radard_mod.get_lead
  stock_low_speed = radard_mod.Track.potential_low_speed_lead

  def get_lead(*args, low_speed_override=True, **kwargs):
    # radard calls get_lead for leadOne with low_speed_override=True and for leadTwo with False:
    # keep a separate sticky track per lead
    matcher.key = 'one' if low_speed_override else 'two'
    return stock_get_lead(*args, low_speed_override=low_speed_override, **kwargs)

  def potential_low_speed_lead(self, v_ego):
    return self.identifier < RAW_ID_BASE and stock_low_speed(self, v_ego)

  radard_mod.match_vision_to_track = matcher.match
  radard_mod.get_lead = get_lead
  radard_mod.Track.potential_low_speed_lead = potential_low_speed_lead
  return matcher

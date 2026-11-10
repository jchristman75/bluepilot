"""
BluePilot: radard with the raw Delphi MRR radar for CAN-FD Fords.

Stock radard gets one radar point on CAN-FD Fords: the Steer_Assist_Data object card publishes in
radarTracks. When BPRawRadar is on, radard's main() hands over to this loop, which also reads the raw
MRR detections from the radar bus itself, tracks them (MrrTracker) and gives radard both:
  - the raw tracks (IDs >= RAW_ID_BASE), and
  - card's Steer_Assist_Data point (kept as a fallback: if the raw radar is missing, leads are exactly
    what stock radard would produce, except for the lead matching changes in lead_match.py).
radard's own RadarD (Kalman tracks, leads, radarState) is reused unchanged.

Runs on the little cores: the tracker is ~1.4 ms per 20 Hz scan on a desktop CPU, so it stays off
core 4 (card / controlsd / selfdrived) and core 5 (plannerd / ui).
"""
from types import SimpleNamespace

from openpilot.cereal import messaging
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import FordFlags
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.bluepilot.selfdrive.controls.radar.mrr_tracker import MrrTracker
from openpilot.bluepilot.selfdrive.controls.radar import lead_match

PARAM = "BPRawRadar"
RAW_RADAR_DELAY = 0.05  # s: one 20 Hz scan; aligns vEgo with the radar measurement in RadarD


def raw_radar_enabled(CP, params: Params | None = None) -> bool:
  if CP.brand != 'ford' or not (CP.flags & FordFlags.CANFD) or CP.radarUnavailable:
    return False
  try:
    return (params or Params()).get_bool(PARAM)
  except Exception:
    return False


def merge_points(raw_tracks, radar_tracks):
  """RadarData-like object for RadarD.update(): raw tracks + card's points (Steer_Assist_Data)."""
  return SimpleNamespace(points=list(raw_tracks) + list(radar_tracks.points), errors=radar_tracks.errors)


def main(CP, CP_SP) -> None:
  import openpilot.selfdrive.controls.radard as radard
  config_realtime_process([0, 1, 2, 3], 5)
  lead_match.install(radard)

  bus = CanBus(CP).radar
  tracker = MrrTracker(bus)
  can_sock = messaging.sub_sock('can', conflate=False)
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])
  RD = radard.RadarD(CP, CP_SP, RAW_RADAR_DELAY)
  cloudlog.warning(f"bp radard: raw MRR radar on bus {bus}")

  while True:
    sm.update()
    for nanos, frames in can_capnp_to_list(messaging.drain_sock_raw(can_sock)):
      tracker.update([(nanos, [f for f in frames if f[2] == bus])])
    RD.update(sm, merge_points(tracker.tracks, sm['radarTracks']))
    RD.publish(pm)

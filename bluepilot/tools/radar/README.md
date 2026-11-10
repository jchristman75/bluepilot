# BluePilot raw-radar tools (CAN-FD Fords)

The code on the car lives in `bluepilot/selfdrive/controls/radar/`:

| File | What it does |
|---|---|
| `mrr_tracker.py` | Decodes the raw Delphi MRR detections (`FORD_CADS_64`, 22 frames per 20 Hz scan on the radar bus) and tracks them: merge detections per object, persistent IDs, confirm 6 of 10 scans, drop after 10 misses. Raw IDs start at 1,000,000. |
| `lead_match.py` | radard lead matching for many tracks: clamp the big models' lead stds (~6e4), gate before picking, keep the matched track per lead (sticky), and no low-speed lead from a raw track alone. Installed only in the BluePilot radard process. |
| `radard_bp.py` | The radard loop when `BPRawRadar` is on: raw tracks + card's Steer_Assist_Data point into the stock `RadarD`, on the little cores. |

`openpilot/selfdrive/controls/radard.py` hands over to `radard_bp.main()` when `raw_radar_enabled(CP)`
(CAN-FD Ford, radar not disabled, `BPRawRadar` set). With the param off, nothing changes.

## Why

openpilot's CAN-FD Ford mode reads only the one object Ford's collision-mitigation system picks
(`Steer_Assist_Data`). It skips stopped cars far ahead, so those leads are camera-only and the camera
needs seconds to tell "stopped" from "slow": late, hard braking (route 52e t=257, a stopped car first
seen at 117 m was only recognized as stopped at 67 m). The raw radar had it from ~150 m, closing at
exactly vEgo.

## Tools

```bash
# replay one route both ways (ROUTE_DIR holds <route>--<n>/rlog.zst segment folders)
python -m bluepilot.tools.radar.radar_replay ROUTE_DIR out/52e_logged.npz --mode logged
python -m bluepilot.tools.radar.radar_replay ROUTE_DIR out/52e_bp.npz --mode bp
# compare every <name>_logged / <name>_bp pair in a folder
python -m bluepilot.tools.radar.compare out/
```

`logged` replays the logged radarTracks through the real RadarD (parity with the logged radarState:
99.4% lead presence, 99.6% radar flag). `bp` runs exactly what `radard_bp` runs on the car.

## Results (2026-11-10, Mach-E, 15 drives, 163 min moving, highway + city)

| | Today (Steer_Assist_Data) | BluePilot raw radar |
|---|---|---|
| Lead frames backed by radar | 84.4% | 97.9% |
| Radar <-> vision switches per min | 3.6 | 1.4 |
| Leads only BP has (phantoms) / only today has (lost) | | 0 / 0 |
| Seconds a radar lead was known stopped while today's lead still read as moving | | 20.7 |

Other CAN-FD Fords (CAN-only logs, raw track vs. each truck's own Steer_Assist_Data object):
F-150 Lightning 99.2-99.7% matched, F-150 97.8-99.6%, Mach-E (another owner) 99.4-99.9%,
Ranger 88.7-100%; range within 0.3-0.8 m (p95 < 2 m), closing speed within 0.03-0.15 m/s.
CAN Fords (Explorer, Escape, Bronco Sport, Maverick, Focus) use the 8-byte CAN MRR format and
upstream's parser; this does not apply to them.

CPU: the tracker takes 2.7 ms per scan on a comma little core (5% of one core at 20 Hz).

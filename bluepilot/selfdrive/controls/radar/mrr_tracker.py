"""
BluePilot: tracker for the raw Delphi MRR radar on CAN-FD Fords ('FORD_CADS_64', 22 x 64-byte
MRR_Detection_001..022 frames per scan, 20 Hz, on the radar bus).

openpilot's CAN-FD Ford radar mode (STEER_ASSIST_DATA) only gets the one object Ford's
collision-mitigation system picks (Steer_Assist_Data). That object skips stopped cars far ahead, so
those leads are camera-only and the camera needs ~4 s to tell "stopped" from "slow" (route 52e t=257:
the raw radar had the stopped car, closing at exactly vEgo, from ~150 m).

Each scan carries ~89 valid detections (on the Mach-E all four scan indexes cover ~0-210 m) and ~89%
of them are stationary clutter. Per scan:
  detections -> objects (detections of one object merged)
  -> associated to persistent tracks (predicted with vRel, gated on range / lateral / range rate)
  -> alpha-beta update.
A track is reported once confirmed (seen in CONFIRM_HITS of the last CONFIRM_WINDOW scans) and stays
reported until it has gone MAX_MISSES scans unseen, so its ID (and radard's Kalman track) stays
stable. Raw track IDs start at RAW_ID_BASE so radard can tell them from the Steer_Assist_Data object.

Validated offline on 15 Mach-E drives (163 min) and on F-150 / Lightning / Mach-E / Ranger CAN logs:
see bluepilot/tools/radar/README.md.
"""
from dataclasses import dataclass

import numpy as np

from opendbc.can.dbc import DBC

N_MSGS = 22
POINTS_PER_MSG = 6
LAST_MSG_POINTS = 3
RAW_ID_BASE = 1_000_000
FIRST_ADDR = 0x120
FRAME_BYTES = 64

_FIELDS = ('VALID_LEVEL', 'SCAN_INDEX_2LSB', 'RANGE', 'AZIMUTH', 'RANGE_RATE')


class MrrDecoder:
  """Decodes the 5 fields the tracker needs from one scan's 22 MRR_Detection frames, all 129
  detection slots at once. opendbc's CANParser decodes all ~59 signals of every frame in Python
  (~18 ms per scan on the comma's little cores); every field used here is an unsigned big-endian bit
  field, which is a run of `size` consecutive bits in the MSB-first bit stream starting at
  byte*8 + 7 - msb%8 (the same bits opendbc's Signal.get_raw_value reads)."""

  def __init__(self):
    dbc = DBC('FORD_CADS_64')
    slots = [(i, j) for i in range(1, N_MSGS + 1) for j in range(1, (POINTS_PER_MSG if i < N_MSGS else LAST_MSG_POINTS) + 1)]
    self.fields = {}
    for f in _FIELDS:
      rows, pos = [], []
      size = factor = offset = None
      for i, j in slots:
        sig = dbc.name_to_msg[f"MRR_Detection_{i:03d}"].sigs[f"CAN_DET_{f}_{i:02d}_{j:02d}" if f != 'SCAN_INDEX_2LSB' else f"CAN_{f}_{i:02d}_{j:02d}"]
        assert not sig.is_little_endian and not sig.is_signed
        assert size in (None, sig.size) and factor in (None, sig.factor) and offset in (None, sig.offset)
        size, factor, offset = sig.size, sig.factor, sig.offset
        rows.append(i - 1)
        pos.append((sig.msb // 8) * 8 + 7 - sig.msb % 8)
      cols = np.array(pos)[:, None] + np.arange(size)[None, :]
      self.fields[f] = (np.array(rows)[:, None], cols, 1 << np.arange(size - 1, -1, -1), factor, offset)

  def decode(self, frames: np.ndarray) -> dict[str, np.ndarray]:
    """frames: (22, 64) uint8 -> {field: (129,) physical values}."""
    bits = np.unpackbits(frames, axis=1)
    out = {}
    for f, (rows, cols, w, factor, offset) in self.fields.items():
      out[f] = (bits[rows, cols] @ w) * factor + offset
    return out


@dataclass
class RawTrack:
  """What radard needs from a radar point (duck-types structs.RadarData.RadarPoint)."""
  trackId: int
  dRel: float
  yRel: float
  vRel: float
  measured: bool = True


class MrrTracker:
  DT = 0.05
  MAX_Y = 12.0                                  # m, lateral field kept
  MIN_D, MAX_D = 1.0, 200.0
  MERGE_D, MERGE_Y, MERGE_V = 2.5, 1.5, 1.0     # detections of one object within one scan
  GATE_D, GATE_D_FRAC, GATE_Y, GATE_V = 3.0, 0.03, 1.5, 1.5   # scan-to-track association
  ALPHA_D, ALPHA_Y, ALPHA_V = 0.5, 0.3, 0.3
  CONFIRM_HITS, CONFIRM_WINDOW = 6, 10
  MAX_MISSES = 10

  def __init__(self, bus: int):
    self.bus = bus
    self.decoder = MrrDecoder()
    self.frames = np.zeros((N_MSGS, FRAME_BYTES), dtype=np.uint8)  # latest frame per MRR_Detection message
    self.trigger = FIRST_ADDR + N_MSGS - 1                            # the last message closes a scan
    # track state, one row per track: d, y, v
    self.state = np.zeros((0, 3))
    self.ids = np.zeros(0, dtype=np.int64)
    self.hits = np.zeros((0, self.CONFIRM_WINDOW), dtype=bool)
    self.misses = np.zeros(0, dtype=np.int64)
    self.confirmed = np.zeros(0, dtype=bool)
    self.next_id = RAW_ID_BASE
    self.tracks: list[RawTrack] = []

  def reset(self):
    self.__init__(self.bus)

  # ---- parsing ----
  def detections(self) -> np.ndarray:
    """(N, 3) [d, y, rangeRate] of this scan's valid detections inside the field kept."""
    v = self.decoder.decode(self.frames)
    ok = (v['VALID_LEVEL'] > 0) & (v['SCAN_INDEX_2LSB'] == v['SCAN_INDEX_2LSB'][0])
    if not ok.any():
      return np.zeros((0, 3))
    r, az, rr = v['RANGE'][ok], v['AZIMUTH'][ok], v['RANGE_RATE'][ok]
    # same lateral convention as the DELPHI_MRR_64 parser (matches radard's left-positive yRel on the Mach-E)
    d, y = np.cos(az) * r, np.sin(az) * r
    keep = (d > self.MIN_D) & (d < self.MAX_D) & (np.abs(y) < self.MAX_Y)
    return np.stack([d[keep], y[keep], rr[keep]], axis=1)

  # ---- tracking ----
  def objects(self, dets: np.ndarray) -> np.ndarray:
    """Merge detections that belong to one object (greedy, nearest range first) -> (M, 3) centroids."""
    if len(dets) == 0:
      return dets
    # plain Python: ~90 detections, and per-step numpy calls cost more than the arithmetic
    md, my, mv = self.MERGE_D, self.MERGE_Y, self.MERGE_V
    objs: list[list[float]] = []   # [sum_d, sum_y, sum_v, count, mean_d, mean_y, mean_v]
    for d, y, v in sorted(dets.tolist()):
      for o in objs:
        if abs(d - o[4]) < md and abs(y - o[5]) < my and abs(v - o[6]) < mv:
          o[0] += d
          o[1] += y
          o[2] += v
          o[3] += 1
          o[4], o[5], o[6] = o[0] / o[3], o[1] / o[3], o[2] / o[3]
          break
      else:
        objs.append([d, y, v, 1, d, y, v])
    return np.array([o[4:7] for o in objs]).reshape(-1, 3)

  def step(self, objs: np.ndarray) -> None:
    nt = len(self.state)
    pred = self.state.copy()
    pred[:, 0] += pred[:, 2] * self.DT  # range advances with range rate
    assigned_obj = np.full(nt, -1)
    if nt and len(objs):
      gd = self.GATE_D + self.GATE_D_FRAC * np.abs(pred[:, 0:1])
      ed = np.abs(objs[None, :, 0] - pred[:, 0:1]) / gd
      ey = np.abs(objs[None, :, 1] - pred[:, 1:2]) / self.GATE_Y
      ev = np.abs(objs[None, :, 2] - pred[:, 2:3]) / self.GATE_V
      ok = (ed < 1) & (ey < 1) & (ev < 1)
      cost = np.where(ok, ed ** 2 + ey ** 2 + ev ** 2, np.inf)
      # greedy global assignment, cheapest pairs first
      ti, oi = np.nonzero(ok)
      order = np.argsort(cost[ti, oi])
      used_obj = np.zeros(len(objs), dtype=bool)
      for k in order:
        t, o = ti[k], oi[k]
        if assigned_obj[t] < 0 and not used_obj[o]:
          assigned_obj[t] = o
          used_obj[o] = True
    hit = assigned_obj >= 0
    new_state = pred
    if hit.any():
      z = objs[assigned_obj[hit]]
      p = pred[hit]
      new_state[hit, 0] = p[:, 0] + self.ALPHA_D * (z[:, 0] - p[:, 0])
      new_state[hit, 1] = p[:, 1] + self.ALPHA_Y * (z[:, 1] - p[:, 1])
      new_state[hit, 2] = p[:, 2] + self.ALPHA_V * (z[:, 2] - p[:, 2])
    self.hits = np.concatenate([self.hits[:, 1:], hit[:, None]], axis=1) if nt else self.hits
    self.misses = np.where(hit, 0, self.misses + 1)
    self.state = new_state
    self.confirmed |= self.hits.sum(axis=1) >= self.CONFIRM_HITS
    alive = self.misses < self.MAX_MISSES
    self.state, self.ids, self.hits = self.state[alive], self.ids[alive], self.hits[alive]
    self.misses, self.confirmed = self.misses[alive], self.confirmed[alive]
    # unmatched objects start new (unconfirmed) tracks
    new = np.ones(len(objs), dtype=bool)
    if nt and len(objs):
      new[assigned_obj[assigned_obj >= 0]] = False
    k = int(new.sum())
    if k:
      first = np.zeros((k, self.CONFIRM_WINDOW), dtype=bool)
      first[:, -1] = True
      self.state = np.concatenate([self.state, objs[new]])
      self.ids = np.concatenate([self.ids, np.arange(self.next_id, self.next_id + k)])
      self.hits = np.concatenate([self.hits, first])
      self.misses = np.concatenate([self.misses, np.zeros(k, dtype=np.int64)])
      self.confirmed = np.concatenate([self.confirmed, np.zeros(k, dtype=bool)])
      self.next_id += k

  def update(self, can_packets) -> bool:
    """Feed CAN packets [(nanos, [(addr, dat, src), ...]), ...]. True when a scan completed and
    self.tracks was refreshed with the confirmed tracks."""
    scan_done = False
    for _, frames in can_packets:
      for addr, dat, src in frames:
        k = addr - FIRST_ADDR
        if src == self.bus and 0 <= k < N_MSGS:
          self.frames[k, :] = 0
          self.frames[k, :len(dat)] = np.frombuffer(dat, dtype=np.uint8)[:FRAME_BYTES]
          scan_done |= addr == self.trigger
    if not scan_done:
      return False
    self.step(self.objects(self.detections()))
    self.tracks = [RawTrack(int(i), float(s[0]), float(s[1]), float(s[2]))
                   for i, s, c in zip(self.ids, self.state, self.confirmed, strict=True) if c]
    return True

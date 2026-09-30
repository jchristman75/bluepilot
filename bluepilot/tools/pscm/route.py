"""
BluePilot: load a route extracted by extract.py as named column arrays.

  r = Route('/path/to/cache', '00000442--76daf14e0b')
  r.tr        # seconds from the first carState of the route
  r.lmcPA     # path angle we sent, internal sign
  r.meas_pa   # -yawRate: delivered yaw rate, same sign and units as lmcPA before the gain factor
"""
import glob
import os
import re

import numpy as np

from bluepilot.tools.pscm.extract import COLS, DEFAULT_CACHE

_IDX = {c: i for i, c in enumerate(COLS)}
_LMC_COLS = [_IDX[c] for c in ('lmcMode', 'lmcPA', 'lmcOff', 'lmcRamp', 'lmcCurv', 'lmcCrvRate', 'lmcPrec', 'lmcHOC', 'tLmc')]


def route_ids(cache=DEFAULT_CACHE, pattern='*'):
  ids = set()
  for f in glob.glob(os.path.join(cache, f'{pattern}.npz')):
    m = re.match(r'(\w{8}--\w{10})--\d+\.npz$', os.path.basename(f))
    if m:
      ids.add(m.group(1))
  return sorted(ids)


class Route:
  def __init__(self, cache, rid):
    files = sorted(glob.glob(os.path.join(cache, f'{rid}--*.npz')), key=lambda f: int(f.rsplit('--', 1)[1][:-4]))
    arrs = [a for a in (np.load(f)['a'] for f in files) if len(a)]
    if not arrs:
      raise FileNotFoundError(f'no extracted segments for {rid} in {cache}')
    # Segments are extracted independently, so each one starts with the wire columns zeroed (mode 0)
    # until its first 982 frame (~30 ms): a fake mode 2->0->2 at every 60 s boundary. Carry the
    # previous segment's last wire frame over those rows (tLmc == 0 means none seen yet).
    for prev, a in zip(arrs, arrs[1:]):
      fresh = a[:, _IDX['tLmc']] == 0
      a[np.ix_(fresh, _LMC_COLS)] = prev[-1, _LMC_COLS]
    self.rid = rid
    self.a = np.concatenate(arrs)
    for c, i in _IDX.items():
      setattr(self, c, self.a[:, i])
    self.tr = self.t - self.t[0]
    self.meas_pa = -self.yaw
    self.dt = float(np.median(np.diff(self.t)))

  def __len__(self):
    return len(self.t)

  def since_last(self, mask):
    """Seconds since mask was last true (inf before the first occurrence)."""
    out = np.full(len(self), np.inf)
    idx = np.flatnonzero(mask)
    if len(idx):
      pos = np.searchsorted(idx, np.arange(len(self)), side='right') - 1
      ok = pos >= 0
      out[ok] = self.tr[ok] - self.tr[idx[pos[ok]]]
    return out

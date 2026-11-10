#!/usr/bin/env python3
"""
BluePilot: compare radar replays (radar_replay.py) of the same routes.

usage: python -m bluepilot.tools.radar.compare DIR [NAME ...]
DIR holds <name>_logged.npz and <name>_bp.npz pairs (default: every pair found).
Per route and in total: share of lead frames backed by a radar track, radar<->vision switches per
minute, leads only one side has (bp "extra" = possible phantom, "lost" = missed), seconds where bp
had a radar lead it knew was stopped while the logged lead still read as moving, and the range
difference where both had radar.
"""
import glob
import os
import sys

import numpy as np


def edges(x):
  return np.flatnonzero(np.diff(np.r_[0, x.astype(int)]) == 1)


def stats(A, B):
  mv = A['v'] > 2
  mins = mv.sum() / 20 / 60
  la, lb = (A['p'] > 0) & mv, (B['p'] > 0) & mv
  flips = lambda D, l: int(np.sum(l[1:] & l[:-1] & (D['r'][1:] != D['r'][:-1])))  # noqa: E731
  br = (B['r'] > 0) & (A['r'] > 0) & mv
  dd = np.abs(B['d'] - A['d'])[br]
  stopped_early = (B['p'] > 0) & (B['r'] > 0) & (B['vl'] < 1.0) & (A['p'] > 0) & (A['vl'] > 2.5) & (A['v'] > 5)
  return {
    'min': mins, 'lead_a': int(la.sum()), 'lead_b': int(lb.sum()),
    'radar_a': int((A['r'][la] > 0).sum()), 'radar_b': int((B['r'][lb] > 0).sum()),
    'flips_a': flips(A, la), 'flips_b': flips(B, lb), 'extra': len(edges(lb & ~la)), 'lost': len(edges(la & ~lb)),
    'early': float(stopped_early.sum() / 20),
    'dd50': float(np.median(dd)) if len(dd) else np.nan, 'dd95': float(np.percentile(dd, 95)) if len(dd) else np.nan,
  }


def main():
  d = sys.argv[1]
  names = sys.argv[2:] or sorted(os.path.basename(f)[:-len('_bp.npz')] for f in glob.glob(f'{d}/*_bp.npz'))
  tot = {'min': 0.0, 'lead_a': 0, 'lead_b': 0, 'radar_a': 0, 'radar_b': 0, 'flips_a': 0, 'flips_b': 0, 'extra': 0, 'lost': 0, 'early': 0.0}
  print('route        move min |  radar-backed lead: logged -> bp |    switches/min | extra lost | stopped-early s | range diff p50/p95')
  for n in names:
    A, B = dict(np.load(f'{d}/{n}_logged.npz')), dict(np.load(f'{d}/{n}_bp.npz'))
    if len(A['t']) != len(B['t']) or not len(A['t']):
      print(f'{n:12s} skipped (frame count mismatch)')
      continue
    s = stats(A, B)
    for k in tot:
      tot[k] += s[k]
    ra = s['radar_a'] / max(s['lead_a'], 1) * 100
    rb = s['radar_b'] / max(s['lead_b'], 1) * 100
    m = max(s['min'], 1e-6)
    sw = f'{s["flips_a"] / m:5.1f} -> {s["flips_b"] / m:5.1f}'
    dd = f'{s["dd50"]:.2f}/{s["dd95"]:.2f} m'
    print(f'{n:12s} {s["min"]:8.1f} | {ra:14.1f}% -> {rb:5.1f}%          | {sw} | {s["extra"]:5d} {s["lost"]:4d} | {s["early"]:15.2f} | {dd}')
  m = max(tot['min'], 1e-6)
  ra, rb = tot['radar_a'] / max(tot['lead_a'], 1) * 100, tot['radar_b'] / max(tot['lead_b'], 1) * 100
  sw = f'{tot["flips_a"] / m:5.1f} -> {tot["flips_b"] / m:5.1f}'
  print(f'{"ALL":12s} {tot["min"]:8.1f} | {ra:14.1f}% -> {rb:5.1f}%          | {sw} | {tot["extra"]:5d} {tot["lost"]:4d} | {tot["early"]:15.2f} |')


if __name__ == '__main__':
  main()

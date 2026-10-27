#!/usr/bin/env python3
"""
BluePilot: score harness runs (harness.py output) and compare variants.

Scored time: hands-free engaged driving above 9 m/s (the deviation clip's gate), from 2 s after the
emulator was last forced onto the log (press / disengage) -- so every scored second is the emulated
car answering the controller under test, not a replay.

  track_rms     curvature tracking error, emulated car vs the planner's desired curvature, the
                desired taken lateralDelay earlier (1e-4 1/m)
  path_rms/p95  lateral path error (m): tracking error integrated twice with a 2 s leak -- how far
                the car wanders off the plan before the planner would pull it back
  weave         0.3-0.8 Hz band of the tracking error (1e-4 1/m) -- the "jello"
  cut_p95       p95 of tracking error toward the inside of curves (|desired| > 0.003), 1e-4 1/m
  wide_p95      p95 toward the outside of those curves, 1e-4 1/m
  weave_hwy / wide_p95_hwy  the same above 25 m/s (curves: |desired| > 0.001) -- running wide at
                speed is the failure that matters most
  devlim        share of scored time the deviation clip binds
  unwind        share of scored time the unwind clamp bites
  rel_track     tracking error over the 6 s after each release of a >= 0.5 s press (1e-4 1/m) --
                the window the scored time above skips, where the post-release bias acts
  rel_drift_p95 p95 over releases of the worst lateral path error within those 6 s (m), same 2 s
                leak as path_rms, started from zero at the release
  stall_blips   reactive stall pulses fired (source 2) per hour; handoff_blips = source 1
  blocked       frames ford.h blocked (982 or Lane_Assist_Data1) per hour
  c1_rough      rms 2nd difference of the wire C1 at 20 Hz (mrad) -- command jerkiness

Usage: score.py RUN_DIR [RUN_DIR ...] [--log]   (--log scores the logged car instead of the emulator)
"""
import argparse
import glob
import json
import os

import numpy as np

DT = 0.01
SETTLE_S = 2.0


def _runs(mask):
  e = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
  return list(zip(e[::2], e[1::2]))


def scored_mask(d):
  forced = (d['pressed'] > 0) | (d['latActive'] == 0)  # == the harness's forcing in closed loop
  since = np.full(len(forced), np.inf)
  idx = np.flatnonzero(forced)
  if len(idx):
    pos = np.searchsorted(idx, np.arange(len(forced)), side='right') - 1
    ok = pos >= 0
    since[ok] = (np.arange(len(forced))[ok] - idx[pos[ok]]) * DT
  return (~forced) & (since > SETTLE_S) & (d['v'] > 9.0) & (d['latActive'] > 0) & ~np.isin(d['lcState'], (1, 2, 3))


def tracking_error(d, use_log=False):
  yaw = d['yaw_log'] if use_log else d['yaw_emu']
  k = -yaw / np.maximum(d['v'], 0.1)
  lag = int(round(np.median(d['lat_delay']) / DT)) if len(d['lat_delay']) else 20
  des = np.r_[np.full(lag, d['acurv'][0]), d['acurv'][:len(d['acurv']) - lag]]
  return k - des, des


def _band(x, lo=0.3, hi=0.8):
  f = np.fft.rfftfreq(len(x), DT)
  X = np.fft.rfft(x - x.mean())
  X[(f < lo) | (f > hi)] = 0
  return np.fft.irfft(X, len(x))


def score_run(d, use_log=False):
  m = scored_mask(d)
  e, des = tracking_error(d, use_log)
  v = d['v']
  path, weave, weave_hwy = [], [], []
  for a, b in _runs(m):
    if b - a < 300:
      continue
    # lateral path error with a 2 s leak (the planner re-plans from where the car is)
    psi = y = 0.0
    tau = 2.0
    ys = np.empty(b - a)
    for i in range(a, b):
      psi += (v[i] * e[i] - psi / tau) * DT
      y += (v[i] * psi - y / tau) * DT
      ys[i - a] = y
    path.append(ys)
    wb = _band(e[a:b])
    weave.append(wb)
    weave_hwy.append(wb[v[a:b] > 25])
  if not path:
    return None
  path = np.concatenate(path)
  weave = np.concatenate(weave)
  weave_hwy = np.concatenate(weave_hwy)
  hwy_curve = m & (np.abs(des) > 0.001) & (v > 25)
  inward_hwy = (e * np.sign(des))[hwy_curve]
  mm = m.copy()
  curve = mm & (np.abs(des) > 0.003)
  inward = (e * np.sign(des))[curve]
  # post-release windows
  pressed = d['pressed'] > 0
  rel_e, rel_drift = [], []
  n6 = int(6.0 / DT)
  run_len = 0
  for i in range(1, len(pressed)):
    if pressed[i - 1]:
      run_len += 1
    if pressed[i - 1] and not pressed[i]:
      if run_len * DT >= 0.5 and i + n6 < len(pressed) and not pressed[i:i + n6].any() \
         and (d['latActive'][i:i + n6] > 0).all() and (v[i:i + n6] > 9).all():
        ee = e[i:i + n6]
        rel_e.append(ee)
        psi = y = worst = 0.0
        for j in range(n6):  # same 2 s leak as path_rms, started at zero on the release
          psi += (v[i + j] * ee[j] - psi / 2.0) * DT
          y += (v[i + j] * psi - y / 2.0) * DT
          worst = max(worst, abs(y))
        rel_drift.append(worst)
      run_len = 0
  hours = m.sum() * DT / 3600
  blips = lambda src: int(np.sum(np.diff((d['blipSrc'] == src).astype(int)) == 1))
  c1 = d['c1'][::5]
  return {
    'scored_min': float(m.sum() * DT / 60),
    'track_rms': float(np.sqrt(np.mean(e[m]**2)) * 1e4),
    'path_rms': float(np.sqrt(np.mean(path**2))),
    'path_p95': float(np.percentile(np.abs(path), 95)),
    'weave': float(np.sqrt(np.mean(weave**2)) * 1e4),
    'cut_p95': float(np.percentile(inward, 95) * 1e4) if len(inward) > 100 else float('nan'),
    'wide_p95': float(-np.percentile(inward, 5) * 1e4) if len(inward) > 100 else float('nan'),
    'weave_hwy': float(np.sqrt(np.mean(weave_hwy**2)) * 1e4) if len(weave_hwy) > 500 else float('nan'),
    'wide_p95_hwy': float(-np.percentile(inward_hwy, 5) * 1e4) if len(inward_hwy) > 100 else float('nan'),
    'rel_track': float(np.sqrt(np.mean(np.concatenate(rel_e)**2)) * 1e4) if rel_e else float('nan'),
    'rel_drift_p95': float(np.percentile(rel_drift, 95)) if rel_drift else float('nan'),
    'n_rel': len(rel_e),
    'devlim': float(np.mean(d['devLim'][m] > 0)),
    'unwind': float(np.mean(d['unwind'][m] > 0)),
    'stall_blips_h': blips(2) / max(hours, 1e-9),
    'handoff_blips_h': blips(1) / max(hours, 1e-9),
    'blocked_h': float(np.sum(d['blocked'] > 0) / max(len(d['t']) * DT / 3600, 1e-9)),
    'c1_rough': float(np.sqrt(np.mean(np.diff(c1, 2)**2)) * 1e3),
  }


def score_dir(run_dir, use_log=False):
  out = {}
  for f in sorted(glob.glob(os.path.join(run_dir, '*.npz'))):
    d = dict(np.load(f))
    if 't' not in d or len(d['t']) < 1000:
      continue
    s = score_run(d, use_log)
    if s is not None:
      out[os.path.basename(f)[:-4]] = s
  return out


def aggregate(per_route):
  """Time-weighted mean over routes (rates and shares weight by scored minutes)."""
  w = np.array([s['scored_min'] for s in per_route.values()])
  keys = [k for k in next(iter(per_route.values())) if k != 'scored_min']
  agg = {'scored_min': float(w.sum()), 'routes': len(per_route)}
  for k in keys:
    x = np.array([s[k] for s in per_route.values()])
    ok = np.isfinite(x)
    if k == 'n_rel':
      agg[k] = float(x.sum())
    elif k.endswith('_rms') or k in ('weave', 'weave_hwy', 'c1_rough', 'track_rms', 'rel_track'):
      agg[k] = float(np.sqrt(np.sum(w[ok] * x[ok]**2) / w[ok].sum()))
    else:
      agg[k] = float(np.sum(w[ok] * x[ok]) / w[ok].sum())
  return agg


PAIRED_KEYS = ('track_rms', 'path_rms', 'path_p95', 'weave', 'cut_p95', 'wide_p95', 'weave_hwy', 'wide_p95_hwy',
               'rel_drift_p95')


def paired(base_dir, cand_dir, use_log=False, n_boot=2000):
  """Per-route paired change of cand vs base: time-weighted mean % change with a bootstrap 90% CI
  over routes, and the share of routes that improved (lower is better for every key)."""
  a, b = score_dir(base_dir, use_log), score_dir(cand_dir, use_log)
  common = sorted(set(a) & set(b))
  w = np.array([a[r]['scored_min'] for r in common])
  rng = np.random.default_rng(0)
  out = {}
  for k in PAIRED_KEYS:
    x = np.array([a[r][k] for r in common])
    y = np.array([b[r][k] for r in common])
    ok = np.isfinite(x) & np.isfinite(y) & (x > 0)
    x, y, ww = x[ok], y[ok], w[ok]
    if len(x) < 3:
      continue
    rel = lambda idx: 100 * (np.sum(ww[idx] * y[idx]) / np.sum(ww[idx] * x[idx]) - 1)
    boots = [rel(rng.integers(0, len(x), len(x))) for _ in range(n_boot)]
    out[k] = {'change_pct': float(rel(np.arange(len(x)))), 'ci90': [float(np.percentile(boots, 5)), float(np.percentile(boots, 95))],
              'routes_better': float(np.mean(y < x)), 'n': int(len(x))}
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('runs', nargs='+')
  ap.add_argument('--log', action='store_true')
  ap.add_argument('--json', default=None)
  ap.add_argument('--paired', action='store_true', help='compare every run against the FIRST, per route')
  a = ap.parse_args()
  if a.paired:
    res = {r: paired(a.runs[0], r, a.log) for r in a.runs[1:]}
    for r, d in res.items():
      print(os.path.basename(r.rstrip('/')), 'vs', os.path.basename(a.runs[0].rstrip('/')))
      for k, v in d.items():
        print(f'  {k:14s} {v["change_pct"]:+6.1f}%  CI90 [{v["ci90"][0]:+.1f}, {v["ci90"][1]:+.1f}]  routes better {v["routes_better"]:.0%}  (n={v["n"]})')
    if a.json:
      with open(a.json, 'w') as f:
        json.dump(res, f, indent=1)
    return
  res = {}
  for r in a.runs:
    res[r] = aggregate(score_dir(r, a.log))
  keys = list(next(iter(res.values())).keys())
  print(f'{"run":40s} ' + ' '.join(f'{k[:11]:>11s}' for k in keys))
  for r, s in res.items():
    print(f'{os.path.basename(r.rstrip("/"))[:40]:40s} ' + ' '.join(f'{s[k]:11.4g}' for k in keys))
  if a.json:
    with open(a.json, 'w') as f:
      json.dump(res, f, indent=1)


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
"""
BluePilot: fit the emulator's mode-0 behaviour (release_tau) on recorded mode-0 episodes, and score
it on held-out routes.

fit.py's windows exclude every mode-0 stretch, so release_tau is fitted here instead.
An episode is a mode-0 stretch of LatCtl_D2_Rq (stall/hand-off pulse, or the tail of a human-turn
pause) while lateral stays active, ending in a re-entry the driver doesn't touch for the next
POST_S. The window runs from PRE_S before the mode-0 frames to POST_S after re-entry. While the
driver presses, the emulator is forced onto the logged angle; every hands-free sample from the
first mode-0 frame on is scored (steering angle error).

What the Mach-E does: in a hands-free mode-0 pulse the wheel self-centres (12 -> 1 deg in 0.3 s at
14 m/s, 000001b1 t=910). The old fit had release_tau at its 5 s bound -- the wheel held its angle --
because fit.py's windows hold hardly any hands-free mode 0. 885 episodes on 138 routes (2026-09-30):
release_tau 0.3 s, angle rms 2.84 -> 2.39 deg (held-out 2.96 -> 2.38, mode-2 routes 4.33 -> 3.06).
A re-engagement hold/ramp after mode 0 -> 2 was tried and didn't earn its keep: typical re-entries
match the emulator once the wheel self-centres; the slow ones are LatCtlLim episodes after a
large-command pulse (e.g. 000001b1 t=507, 000001b4 t=762), too few to fit.

Usage:
  fit_mode0.py [--cache DIR] [--params mache_pscm_params.json] [--write]
"""
import argparse
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

from bluepilot.tools.pscm.emulator import DEFAULT_PARAMS, DT, PSCMEmulator, PSCMParams
from bluepilot.tools.pscm.extract import DEFAULT_CACHE
from bluepilot.tools.pscm.fit import TEST_ROUTES
from bluepilot.tools.pscm.route import Route, route_ids

PRE_S, POST_S = 0.5, 1.5
MAX_MODE0_S = 5.0


def episodes(rid, cache=DEFAULT_CACHE):
  try:
    r = Route(cache, rid)
  except FileNotFoundError:
    return []
  m = r.lmcMode
  if not (m > 0).any():
    return []
  pre, post = int(PRE_S / DT), int(POST_S / DT)
  out = []
  for i in np.flatnonzero((m[1:] > 0) & (m[:-1] == 0)) + 1:
    k0 = i - 1
    while k0 > 0 and m[k0 - 1] == 0:
      k0 -= 1
    a, b = k0 - pre, i + post
    if a < 0 or b > len(r) or (i - k0) * DT > MAX_MODE0_S or r.v[k0:b].min() < 5.0:
      continue
    if not r.latActive[a:b].all() or r.pressed[i:b].any() or np.any(np.diff(r.t[a:b]) > 0.03):
      continue
    out.append(dict(rid=rid, t=r.tr[k0], v=r.v[k0], mode0_s=(i - k0) * DT, pressed_in=bool(r.pressed[k0:i].any()),
                    k0=k0 - a, i=i - a, mode=r.lmcMode[a:b].copy(), c1=-r.lmcPA[a:b], vv=r.v[a:b].copy(),
                    roll=r.roll[a:b].copy(), ang=r.ang[a:b].copy(), yaw=r.yaw[a:b].copy(),
                    pressed=r.pressed[a:b] > 0, lim=r.limStat[a:b].copy()))
  return out


def _pad(eps):
  """Stack episodes into (T, n) arrays; padding is forced to the log and not scored."""
  T = max(len(e['ang']) for e in eps)
  def stack(key, fill):
    return np.stack([np.r_[e[key], np.full(T - len(e[key]), fill if fill is not None else e[key][-1])] for e in eps], axis=1)
  d = {k: stack(k, None) for k in ('mode', 'c1', 'vv', 'roll', 'ang', 'yaw')}
  d['pressed'] = stack('pressed', True)
  t = np.arange(T)[:, None]
  k0 = np.array([e['k0'] for e in eps])[None]
  d['forced'] = d['pressed'] | (t < k0)       # pre-roll: follow the log up to the first mode-0 frame
  d['score'] = ~d['forced']
  return d


def simulate(p, d):
  T, n = d['ang'].shape
  emu = PSCMEmulator(p, n=n)
  emu.reset(d['ang'][0], d['yaw'][0], d['vv'][0], c1=d['c1'][0])
  out = np.zeros((T, n))
  for k in range(T):
    da = np.where(d['forced'][k], d['ang'][k], np.nan)
    o = emu.step(d['mode'][k], d['c1'][k], d['vv'][k], d['roll'][k], driver_angle=da)
    out[k] = o['steeringAngleDeg']
  return out


def score(p, d):
  sim = simulate(p, d)
  e = (sim - d['ang'])[d['score']]
  return float(np.sqrt(np.mean(e ** 2)))


def _grid_point(args):
  p, d, tau = args
  return score(replace(p, release_tau=tau), d), tau


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--cache', default=DEFAULT_CACHE)
  ap.add_argument('--params', default=DEFAULT_PARAMS)
  ap.add_argument('--write', action='store_true', help='write the fitted values into --params')
  ap.add_argument('-j', type=int, default=24)
  a = ap.parse_args()
  with Pool(a.j) as pool:
    eps = sum(pool.map(episodes, route_ids(a.cache)), [])
  train = [e for e in eps if e['rid'][:8] not in TEST_ROUTES]
  test = [e for e in eps if e['rid'][:8] in TEST_ROUTES]
  print(f'{len(eps)} episodes ({len(train)} train, {len(test)} test) on {len(set(e["rid"] for e in eps))} routes; '
        f'{sum(e["mode0_s"] < 0.4 for e in eps)} are pulses (<0.4 s)')
  p0 = PSCMParams.load(a.params)
  dtr, dte = _pad(train), _pad(test)
  grid = (0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.7, 1.0, 2.0, 5.0)
  with Pool(a.j) as pool:
    res = sorted(pool.map(_grid_point, [(p0, dtr, g) for g in grid]))
  best = res[0][1]
  pb = replace(p0, release_tau=best)
  base = replace(p0, release_tau=5.0)
  print('best release_tau:', best)
  for name, d, es in (('train', dtr, train), ('test', dte, test)):
    print(f'{name}: angle rms old {score(base, d):.2f} deg -> new {score(pb, d):.2f} deg')
    for label, sel in (('pulses', [e['mode0_s'] < 0.4 for e in es]), ('longer', [e['mode0_s'] >= 0.4 for e in es])):
      sel = np.array(sel)
      if sel.any():
        ds = _pad([e for e, s in zip(es, sel) if s])
        print(f'  {label:7s} n={sel.sum():3d}: old {score(base, ds):.2f} -> new {score(pb, ds):.2f} deg')
  if a.write:
    pb.save(a.params, meta={**_meta(a.params), 'mode0_fit': {'episodes': len(eps), 'best': best}})
    print('wrote', a.params)


def _meta(path):
  import json
  with open(path) as f:
    return json.load(f).get('meta', {})


if __name__ == '__main__':
  main()

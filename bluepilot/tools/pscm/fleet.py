#!/usr/bin/env python3
"""
BluePilot: test the PSCM emulator on the commaCarSegments Ford fleet (stock openpilot, curvature
mode -- the cars steer through C2, LatCtlCurv_No_Actl; extract with extract_fleet.py first).

  1. transfer: the Mach-E C1 emulator (mache_pscm_params.json), fed the fleet Mach-Es' C2 as the
     equivalent yaw-rate request v*C2. Does the angle-mode PSCM model describe curvature mode?
  2. refit: per platform, the same grey-box structure fitted on C2 (steady-state regression ->
     vehicle stage -> PSCM stage), split by CAR: every test window comes from a car (dongle) the
     fit never saw.

Usage: fleet.py [--cache DIR] [--out REPORT.json] [--params-dir DIR] [--platforms A,B]
"""
import argparse
import hashlib
import json
import os
import glob
from collections import defaultdict
from dataclasses import asdict

import numpy as np

from opendbc.car.ford.values import CAR
from bluepilot.tools.pscm import fit
from bluepilot.tools.pscm.emulator import DEFAULT_PARAMS, DT, PSCMParams
from bluepilot.tools.pscm.extract_fleet import DEFAULT_FLEET_CACHE, FLEET_COLS

_I = {c: i for i, c in enumerate(FLEET_COLS)}
PRESS_TQ = 1.0      # Nm column torque: treated as hands-on (and 2 s either side excluded)
TEST_SHARE = 0.3    # of each platform's cars


def _is_test_car(dongle):
  return int(hashlib.md5(dongle.encode()).hexdigest(), 16) % 100 < TEST_SHARE * 100


def platform_windows(cache, platform):
  """{'train': d, 'test': d, 'cars': (n_train, n_test)} window batches, c1 = v * C2."""
  n = int(fit.WIN_S / DT)
  segs = defaultdict(list)
  for f in glob.glob(os.path.join(cache, f'{platform}--*.npz')):
    _, dongle, route, seg = os.path.basename(f)[:-4].split('--')
    segs[(dongle, route)].append((int(seg), f))
  out = {'train': [], 'test': []}
  cars = {'train': set(), 'test': set()}
  for (dongle, route), lst in segs.items():
    arrs = [a for a in (np.load(f)['a'] for _, f in sorted(lst)) if len(a)]
    if not arrs:
      continue
    a = np.concatenate(arrs)
    t, v, tq = a[:, _I['t']], a[:, _I['v']], a[:, _I['tq']]
    hands = np.abs(tq) > PRESS_TQ
    idx = np.flatnonzero(hands)
    near = np.zeros(len(a), bool)
    if len(idx):  # within 2 s of a press
      k = int(2.0 / DT)
      near = np.convolve(hands.astype(float), np.ones(2 * k + 1), mode='same') > 0
    ok = (a[:, _I['mode']] > 0) & (v > 5.0) & ~near & (a[:, _I['tx']] > 0)
    ok &= ~np.r_[False, (np.diff(t) > 0.03) | (np.diff(t) < 0)]
    split = 'test' if _is_test_car(dongle) else 'train'
    edges = np.flatnonzero(np.diff(np.r_[0, ok.astype(int), 0]))
    for s0, s1 in zip(edges[::2], edges[1::2]):
      for s in range(s0, s1 - n + 1, n):
        sl = slice(s, s + n)
        c1 = a[sl, _I['c1']] + v[sl] * a[sl, _I['c2']]
        out[split].append(np.stack([a[sl, _I['mode']], c1, v[sl], np.zeros(n), a[sl, _I['ang']], a[sl, _I['yaw']]]))
        cars[split].add(dongle)
  res = {k: ({s: np.stack([x[i] for x in w], axis=1) for i, s in enumerate(fit.SIGS)} if w else None)
         for k, w in out.items()}
  res['cars'] = (len(cars['train']), len(cars['test']))
  return res


def steady_state(d):
  """ang = a0 + CK * k * (1 + KUS v^2), k = yaw / v, on near-steady samples."""
  v, ang, yaw = d['v'][::10].ravel(), d['ang'][::10].ravel(), d['yaw'][::10].ravel()
  yd = np.abs(np.gradient(d['yaw'][::10], axis=0)).ravel() / (10 * DT)
  m = (yd < 0.01) & (v > 8)
  k = yaw / v
  A = np.c_[np.ones(m.sum()), k[m], k[m] * v[m]**2]
  c, *_ = np.linalg.lstsq(A, ang[m], rcond=None)
  return float(c[0]), float(c[1]), float(c[2] / c[1])


def fit_platform(platform, d, workers):
  spec = CAR[platform].config.specs
  a0, ck, kus = steady_state(d)
  wb = spec.wheelbase
  p = PSCMParams(pscm_ck=ck, pscm_kus=kus, veh_mass=spec.mass, veh_wheelbase=wb, veh_cg_front=0.44,
                 veh_sr=float(np.clip(ck * np.pi / 180 / wb, 8, 25)), veh_us=kus * wb, veh_offset_deg=a0,
                 angle_offset_deg=a0, veh_roll=0.0, bank_comp=0.0, c1_slew=1.0)
  stage_b = [s for s in fit.STAGE_B if s[0] != 'veh_roll']
  stage_a = [s for s in fit.STAGE_A if s[0] != 'bank_comp']
  p = fit.lm_fit(p, stage_b, d, 'B', workers=workers, log=lambda *_: None)
  rng = np.random.default_rng(0)
  idx = rng.choice(d['c1'].shape[1], min(300, d['c1'].shape[1]), replace=False)
  small = {k: v[:, idx] for k, v in d.items()}
  best = None
  for delay in (0.06, 0.09, 0.12, 0.15):
    q = fit.lm_fit(PSCMParams(**{**asdict(p), 'delay_s': delay}), stage_a, small, 'A', iters=6, workers=workers,
                   log=lambda *_: None)
    ang, _ = fit.simulate(q, small, 'A')
    rms = float(np.sqrt(np.mean((ang - small['ang'])[fit._W:]**2)))
    if best is None or rms < best[0]:
      best = (rms, q)
  return fit.lm_fit(best[1], stage_a, d, 'A', workers=workers, log=lambda *_: None)


def _summary(ev):
  pick = lambda m: {k: round(m[k], 3) for k in ('rms', 'r2', 'r2_dynamic')}
  return {k: pick(v) for k, v in ev.items()}


def step_curve(p, v, amp=0.03):
  from bluepilot.tools.pscm.validate import step_response
  _, s = step_response(p, v, amp)
  return [round(float(s[int((0.5 + t) / DT) - 1]), 3) for t in (0.25, 0.5, 1.0, 2.0, 3.0)]


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--cache', default=DEFAULT_FLEET_CACHE)
  ap.add_argument('--out', default='fleet_report.json')
  ap.add_argument('--params-dir', default=None, help='write per-platform C2 params files here')
  ap.add_argument('--platforms', default=None)
  ap.add_argument('-j', type=int, default=30)
  a = ap.parse_args()
  plats = a.platforms.split(',') if a.platforms else sorted({os.path.basename(f).split('--')[0]
                                                             for f in glob.glob(os.path.join(a.cache, '*.npz'))})
  report = {}
  for plat in plats:
    w = platform_windows(a.cache, plat)
    if w['train'] is None or w['test'] is None or w['train']['c1'].shape[1] < 20:
      print(plat, 'not enough hands-free engaged data', w['cars'])
      continue
    r = {'cars_train_test': w['cars'], 'windows_train_test': (int(w['train']['c1'].shape[1]), int(w['test']['c1'].shape[1]))}
    if plat == 'FORD_MUSTANG_MACH_E_MK1':
      mine = PSCMParams.load(DEFAULT_PARAMS)
      allw = {k: np.concatenate([w['train'][k], w['test'][k]], axis=1) for k in fit.SIGS}
      r['transfer_mache_c1_model'] = _summary(fit.evaluate(mine, allw))
      # the same model with only its angle offset refitted (fleet cars differ in alignment, no roll)
      a0 = steady_state(allw)[0]
      r['transfer_mache_c1_model_offset'] = _summary(fit.evaluate(
        PSCMParams(**{**asdict(mine), 'angle_offset_deg': a0, 'veh_offset_deg': a0}), allw))
    p = fit_platform(plat, w['train'], a.j)
    r['test'] = _summary(fit.evaluate(p, w['test']))
    r['train'] = _summary(fit.evaluate(p, w['train']))
    r['params'] = {k: (np.round(v, 4).tolist() if isinstance(v, list) else v) for k, v in asdict(p).items()}
    r['step'] = {f'{v}ms': step_curve(p, v) for v in (8, 15, 22, 30)}
    report[plat] = r
    print(plat, json.dumps({k: v for k, v in r.items() if k != 'params'}))
    if a.params_dir:
      os.makedirs(a.params_dir, exist_ok=True)
      p.save(os.path.join(a.params_dir, f'{plat.lower()}_c2_params.json'), meta={k: v for k, v in r.items() if k != 'params'})
  with open(a.out, 'w') as f:
    json.dump(report, f, indent=1)


if __name__ == '__main__':
  main()

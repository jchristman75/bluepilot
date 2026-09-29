#!/usr/bin/env python3
"""
BluePilot: fit the PSCM emulator (emulator.py) to extracted Mach-E routes and validate it on
held-out routes.

Windows are 23 s of engaged, hands-free driving (no press in the 2 s before or during, no human-turn
pause, no stall pulse, v > 5 m/s, continuous 100 Hz samples). The first WARMUP_S of each window
only settles the model's internal states; scoring starts after it. Split is by route, so a test
window never shares a drive with training.

  stage A  wire C1 -> steering angle   (the PSCM; output-error, i.e. free-running simulation)
  stage B  steering angle -> yaw rate  (the car; driven by the measured angle)
  chain    wire C1 -> steering angle -> yaw rate, fully free-running (what the harness will see)

Usage:
  fit.py [--cache DIR] [--out PARAMS.json] [--report REPORT.json] [--max-train N]
"""
import argparse
import json
import time
from dataclasses import asdict
from multiprocessing import Pool

import numpy as np

from bluepilot.tools.pscm.emulator import DEFAULT_PARAMS, DT, PSCMEmulator, PSCMParams
from bluepilot.tools.pscm.extract import DEFAULT_CACHE
from bluepilot.tools.pscm.route import Route, route_ids

WIN_S = 23.0
WARMUP_S = 3.0
# Held out: the newest b09ba355 drives, the other dongle's mode-2 drives, and one long highway drive
# (0000041b) so the test set spans the speed range. 00000434 (the other long highway drive) trains.
TEST_ROUTES = ('0000041b', '00000438', '00000439', '0000043a', '0000043b', '0000043c', '0000043d', '0000043e',
               '0000043f', '00000440', '00000441', '00000442', '00000443', '00000444', '00000445', '00000446',
               '00000447', '00000448', '00000449', '0000044a', '0000044b', '0000044c', '0000044d', '0000044e',
               '0000044f', '00000450', '000001a7', '000001a8', '000001a9', '000001aa', '000001ab', '000001ac',
               '000001ad', '000001ae', '000001af')
SIGS = ('mode', 'c1', 'v', 'roll', 'ang', 'yaw')


def route_windows(r: Route):
  n = int(WIN_S / DT)
  ok = ((r.latActive > 0) & (r.lmcMode > 0) & (r.v > 5.0) & (r.htPaused == 0) & (r.blip == 0) &
        (r.since_last(r.pressed > 0) > 2.0))
  gap = np.r_[False, np.diff(r.t) > 0.03]  # segment joins / dropped frames
  ok &= ~gap
  out = []
  edges = np.flatnonzero(np.diff(np.r_[0, ok.astype(int), 0]))
  for a, b in zip(edges[::2], edges[1::2]):
    for s in range(a, b - n + 1, n):
      out.append(np.stack([r.lmcMode[s:s + n], -r.lmcPA[s:s + n], r.v[s:s + n], r.roll[s:s + n],
                           r.ang[s:s + n], r.yaw[s:s + n]]))
  return out


REL_PRE_S, REL_POST_S = 3.0, 12.0


def release_windows(r: Route):
  """Driver lets go of the wheel: REL_PRE_S hands-on (angle forced to the log), then REL_POST_S free."""
  npre, npost = int(REL_PRE_S / DT), int(REL_POST_S / DT)
  pressed = r.pressed > 0
  gap = np.r_[False, np.diff(r.t) > 0.03]
  out = []
  for i in np.flatnonzero(pressed[:-1] & ~pressed[1:]) + 1:  # first hands-free sample
    a, b = i - npre, i + npost
    if a < 0 or b > len(r) or pressed[i:b].any() or gap[a:b].any() or (r.v[a:b] < 5).any():
      continue
    if pressed[a:i].sum() * DT < 0.5 or not (r.latActive[i:b] > 0).all() or (r.blip[i:b] > 0).any():
      continue
    force = np.zeros(b - a)
    force[:npre] = 1
    out.append(np.stack([r.lmcMode[a:b], -r.lmcPA[a:b], r.v[a:b], r.roll[a:b], r.ang[a:b], r.yaw[a:b], force,
                         np.full(b - a, r.htPaused[a:i].max())]))
  return out


def build_dataset(cache, kind='hands_free'):
  train, test = [], []
  for rid in route_ids(cache):
    try:
      r = Route(cache, rid)
    except FileNotFoundError:
      continue
    w = route_windows(r) if kind == 'hands_free' else release_windows(r)
    (test if rid[:8] in TEST_ROUTES else train).extend(w)
  sigs = SIGS if kind == 'hands_free' else SIGS + ('force', 'manual_turn')
  as_arr = lambda w: {k: np.stack([x[i] for x in w], axis=1) for i, k in enumerate(sigs)}  # (T, N)
  return as_arr(train), as_arr(test)


# --- simulation ----------------------------------------------------------------------------------
def simulate(p: PSCMParams, d, stage='chain'):
  """Free-run over a (T, N) window batch. Returns (angle, yaw) arrays of shape (T, N)."""
  T, N = d['c1'].shape
  emu = PSCMEmulator(p, n=N)
  emu.reset(d['ang'][0], d['yaw'][0], d['v'][0], c1=d['c1'][0], roll=d['roll'][0])
  ang = np.empty((T, N))
  yaw = np.empty((T, N))
  for k in range(T):
    if stage == 'B':
      yaw[k] = emu.vehicle_step(d['ang'][k], d['v'][k], d['roll'][k])
      ang[k] = d['ang'][k]
    else:
      drv = np.where(d['force'][k] > 0, d['ang'][k], np.nan) if 'force' in d else None
      o = emu.step(d['mode'][k], d['c1'][k], d['v'][k], d['roll'][k], driver_angle=drv)
      ang[k], yaw[k] = o['steeringAngleDeg'], o['yawRate']
  return ang, yaw


# --- parameter vector <-> PSCMParams --------------------------------------------------------------
# (name, index or None, transform) -- 'log' params are fitted in log space to stay positive
STAGE_A = ([('ff_gain', i, 'log') for i in range(7)] + [('droop_ratio', i, 'lin') for i in range(7)] +
           [('droop_tau', i, 'log') for i in range(7)] +
           [('c1_slew', None, 'log'), ('servo_p_scale', None, 'log'), ('servo_rate_scale', None, 'log'),
            ('angle_offset_deg', None, 'lin'), ('bank_comp', None, 'lin'),
            ('small_gain', None, 'lin'), ('small_k0', None, 'log')])
STAGE_SAT = [('sat_knee', None, 'lin'), ('sat_slope', None, 'lin')]
STAGE_R = [('release_tau', None, 'log')]
STAGE_B = [('veh_sr', None, 'log'), ('veh_cf', None, 'log'), ('veh_iz', None, 'log'),
           ('veh_yaw_tau', None, 'log'), ('veh_offset_deg', None, 'lin'), ('veh_roll', None, 'lin')]


# physical bounds, applied after the transform, so the optimiser can't wander into an unstable model
BOUNDS = {'release_tau': (0.02, 5.0), 'sat_knee': (1.0, 99.0), 'sat_slope': (0.05, 1.0), 'small_gain': (-0.5, 1.0), 'small_k0': (5e-5, 0.005), 'ff_gain': (0.3, 4.0), 'droop_ratio': (-0.3, 0.7), 'droop_tau': (0.05, 8.0), 'c1_slew': (0.02, 5.0),
          'servo_p_scale': (0.1, 3.0), 'servo_rate_scale': (0.05, 5.0),
          'angle_offset_deg': (-5, 5), 'bank_comp': (-40, 40), 'veh_sr': (8, 25), 'veh_cf': (4e4, 5e5), 'veh_iz': (1000, 10000), 'veh_yaw_tau': (0.002, 0.5), 'veh_offset_deg': (-5, 5),
          'veh_roll': (-60, 60)}


def get_vec(p, spec):
  out = []
  for name, i, tr in spec:
    x = getattr(p, name) if i is None else getattr(p, name)[i]
    out.append(np.log(x) if tr == 'log' else x)
  return np.array(out)


def set_vec(p, spec, vec):
  d = asdict(p)
  for (name, i, tr), x in zip(spec, vec):
    val = float(np.clip(np.exp(np.clip(x, -30, 30)) if tr == 'log' else x, *BOUNDS[name]))
    if i is None:
      d[name] = val
    else:
      d[name] = list(d[name])
      d[name][i] = val
  return PSCMParams(**d)


# --- residuals / Levenberg-Marquardt with a parallel Jacobian ------------------------------------
_W = int(WARMUP_S / DT)
_SUB = 5  # score at 20 Hz
_ctx = {}


def _residual(vec):
  p = set_vec(_ctx['p0'], _ctx['spec'], vec)
  d = _ctx['d']
  ang, yaw = simulate(p, d, _ctx['stage'])
  if _ctx['stage'] == 'A':
    r = (ang - d['ang'])[_W::_SUB]  # deg
  elif _ctx['stage'] == 'R':
    r = (ang - d['ang'])[int(REL_PRE_S / DT):int((REL_PRE_S + 8) / DT):_SUB]
  else:
    # yaw in deg/s-equivalent of steering so both stages weigh similarly
    r = (yaw - d['yaw'])[_W::_SUB] * 180 / np.pi
  r = r.ravel()
  return np.where(np.isfinite(r), r, 1e3)


def _init(p0, spec, d, stage):
  _ctx.update(p0=p0, spec=spec, d=d, stage=stage)


def lm_fit(p0, spec, d, stage, iters=25, workers=30, log=print):
  x = get_vec(p0, spec)
  _init(p0, spec, d, stage)  # workers inherit _ctx by fork, so the window batch is never pickled
  with Pool(workers) as pool:
    r = _residual(x)
    cost = float(r @ r)
    lam = 1e-2
    for it in range(iters):
      h = 1e-3 * np.maximum(1.0, np.abs(x))
      xs = [x + np.eye(len(x))[j] * h[j] for j in range(len(x))]
      cols = pool.map(_residual, xs)
      J = np.stack([(c - r) / h[j] for j, c in enumerate(cols)], axis=1)
      JtJ, Jtr = J.T @ J, J.T @ r
      improved = False
      for _ in range(8):
        dx = -np.linalg.solve(JtJ + lam * np.diag(np.diag(JtJ) + 1e-9), Jtr)
        rn = pool.apply(_residual, (x + dx,))
        cn = float(rn @ rn)
        if cn < cost:
          x, r, rel = x + dx, rn, (cost - cn) / cost
          cost, lam, improved = cn, max(lam / 3, 1e-6), True
          break
        lam *= 4
      log(f'  {stage} iter {it:2d} rms {np.sqrt(cost / len(r)):.4f} lam {lam:.1e}')
      if not improved or rel < 1e-4:
        break
  return set_vec(p0, spec, x)


# --- metrics --------------------------------------------------------------------------------------
def metrics(sim, meas, d, unit_scale=1.0):
  e = (sim - meas)[_W:] * unit_scale
  m = meas[_W:] * unit_scale
  demean = lambda a: a - a.mean(axis=0, keepdims=True)
  v = d['v'][_W:]
  out = {'rms': float(np.sqrt(np.mean(e**2))),
         'r2': float(1 - np.var(e) / np.var(m)),
         'r2_detrended': float(1 - np.mean(demean(e)**2) / np.mean(demean(m)**2)),
         'p95_abs': float(np.percentile(np.abs(e), 95))}
  # dynamics only: remove each window's 2 s moving average (slow curves, crown, offsets) from both
  k = np.ones(int(2.0 / DT)) / int(2.0 / DT)
  hp = lambda a: a - np.apply_along_axis(lambda c: np.convolve(c, k, mode='same'), 0, a)
  eh, mh = hp(e)[100:-100], hp(m)[100:-100]
  out['r2_dynamic'] = float(1 - np.mean(eh**2) / np.mean(mh**2))
  for lo, hi in ((5, 13), (13, 22), (22, 27), (27, 40)):
    s = (v >= lo) & (v < hi)
    if s.sum() > 1000:
      out[f'r2_v{lo}-{hi}'] = float(1 - np.mean(demean(e)[s]**2) / np.mean(demean(m)[s]**2))
  return out


def baseline_static(d, p: PSCMParams, gain=0.8, lag_s=0.15, tau=0.1):
  """What the tools assumed before: wheel = gain * nominal(C1), lagged. Vehicle = same bicycle."""
  T, N = d['c1'].shape
  nl = int(lag_s / DT)
  c1 = np.vstack([np.repeat(d['c1'][:1], nl, 0), d['c1'][:-nl]])
  v = d['v']
  tgt = gain * PSCMEmulator(p)._nominal_deg_per_c1(v) * c1 + p.veh_offset_deg + p.veh_roll * d['roll']
  ang = np.empty((T, N))
  a = d['ang'][0].copy()
  for k in range(T):
    a += (tgt[k] - a) * DT / tau
    ang[k] = a
  return ang


def release_metrics(p, d):
  """Post-release tracking: error over the first 5 s after the driver lets go."""
  ang, yaw = simulate(p, d, 'chain')
  sl = slice(int(REL_PRE_S / DT), int((REL_PRE_S + 5) / DT))
  ea, ey = (ang - d['ang'])[sl], (yaw - d['yaw'])[sl] * 180 / np.pi
  meas_off = (d['yaw'][sl] - yaw[sl]).mean(0) * 180 / np.pi  # per-release mean yaw offset the model misses
  return {'n': int(d['c1'].shape[1]), 'angle_rms_deg': float(np.sqrt(np.mean(ea**2))),
          'yaw_rms_degps': float(np.sqrt(np.mean(ey**2))),
          'yaw_mean_offset_abs_degps': float(np.mean(np.abs(meas_off)))}


def evaluate(p, d):
  angA, _ = simulate(p, d, 'A')
  _, yawB = simulate(p, d, 'B')
  angC, yawC = simulate(p, d, 'chain')
  return {'stageA_angle_deg': metrics(angA, d['ang'], d),
          'stageB_yaw_degps': metrics(yawB, d['yaw'], d, 180 / np.pi),
          'chain_angle_deg': metrics(angC, d['ang'], d),
          'chain_yaw_degps': metrics(yawC, d['yaw'], d, 180 / np.pi),
          'baseline_static_angle_deg': metrics(baseline_static(d, p), d['ang'], d)}


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--cache', default=DEFAULT_CACHE)
  ap.add_argument('--out', default=DEFAULT_PARAMS)
  ap.add_argument('--report', default=None)
  ap.add_argument('-j', type=int, default=30)
  a = ap.parse_args()

  t0 = time.time()
  train, test = build_dataset(a.cache)
  rtrain, rtest = build_dataset(a.cache, 'release')
  print(f'windows: hands-free train {train["c1"].shape[1]} test {test["c1"].shape[1]}; '
        f'releases train {rtrain["c1"].shape[1]} test {rtest["c1"].shape[1]}  ({time.time() - t0:.0f} s)')
  rng = np.random.default_rng(0)
  idx = rng.choice(train['c1'].shape[1], min(400, train['c1'].shape[1]), replace=False)
  small = {k: v[:, idx] for k, v in train.items()}  # one draw for every signal

  print('stage B (vehicle)')
  p = lm_fit(PSCMParams(), STAGE_B, train, 'B', workers=a.j)
  # the transport delay is integer-sample: grid it on a subset, then fit everything at the best one
  grid = []
  for delay in (0.06, 0.09, 0.12, 0.15):
    q = lm_fit(PSCMParams(**{**asdict(p), 'delay_s': delay}), STAGE_A, small, 'A', iters=6, workers=a.j,
               log=lambda *_: None)
    ang, _ = simulate(q, small, 'A')
    grid.append((float(np.sqrt(np.mean((ang - small['ang'])[_W:]**2))), delay, q))
    print(f'delay {delay:.2f}: stage A rms {grid[-1][0]:.3f} deg')
  p = min(grid, key=lambda g: g[0])[2]
  print('stage A (PSCM) full fit')
  p = lm_fit(p, STAGE_A, train, 'A', workers=a.j)
  print('release fit (mode-0 free-wheel relaxation)')
  p = lm_fit(p, STAGE_R, rtrain, 'R', workers=a.j)

  report = {'n_train_windows': int(train['c1'].shape[1]), 'n_test_windows': int(test['c1'].shape[1]),
            'window_s': WIN_S, 'warmup_s': WARMUP_S, 'test_routes': list(TEST_ROUTES),
            'train': evaluate(p, train), 'test': evaluate(p, test),
            'release_train': release_metrics(p, rtrain), 'release_test': release_metrics(p, rtest)}
  print(json.dumps({k: v for k, v in report.items() if k != 'test_routes'}, indent=1))
  p.save(a.out, meta={k: v for k, v in report.items() if k != 'train'})
  if a.report:
    with open(a.report, 'w') as f:
      json.dump({'params': asdict(p), **report}, f, indent=1)
  print(f'saved {a.out} ({time.time() - t0:.0f} s)')


if __name__ == '__main__':
  main()

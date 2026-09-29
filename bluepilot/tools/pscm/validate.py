#!/usr/bin/env python3
"""
BluePilot: check the fitted PSCM emulator against drives it was not fitted on.

  1. Held-out accuracy of the emulator vs two references on the same test windows:
       static  -- wheel = 0.8 x nominal(C1), lagged: the assumption before the emulator
       fir     -- best linear black box: a 3 s FIR per speed band fitted on the TRAIN routes
  2. Ablation: refit without the held-C1 slew limit, to see whether the fitted ~0.1 rad/s is real.
  3. Plots (PNG): the most dynamic test windows per speed band, post-release windows, step responses.

Usage: validate.py [--params PARAMS.json] [--out DIR] [--no-ablation]
"""
import argparse
import json
import os
from dataclasses import asdict

import numpy as np

from bluepilot.tools.pscm import fit
from bluepilot.tools.pscm.emulator import DEFAULT_PARAMS, DT, PSCMEmulator, PSCMParams

BANDS = ((5, 13), (13, 22), (22, 27), (27, 40))
NT = 60  # FIR taps at 20 Hz = 3 s


def _fir_design(d, sub=5):
  """Rows of the last NT (20 Hz) C1 samples + roll + 1, for every scored sample of every window."""
  c1 = d['c1'][::sub]
  T, N = c1.shape
  H = np.lib.stride_tricks.sliding_window_view(c1, NT, axis=0)[..., ::-1]  # (T-NT+1, N, NT)
  i0 = NT - 1
  X = np.concatenate([H, d['roll'][::sub][i0:, :, None], np.ones((T - i0, N, 1))], axis=2)
  return X, d['ang'][::sub][i0:], d['v'][::sub][i0:], i0 * sub


def fir_fit(train):
  X, y, v, _ = _fir_design(train)
  models = {}
  for lo, hi in BANDS:
    m = (v >= lo) & (v < hi)
    A, b = X[m], y[m]
    D = np.zeros((NT - 1, A.shape[1]))
    D[:, :NT] = np.diff(np.eye(NT), axis=0)
    lam = 1e-3 * np.trace(A.T @ A) / A.shape[1]
    models[(lo, hi)] = np.linalg.solve(A.T @ A + lam * D.T @ D, A.T @ b)
  return models


def fir_predict(models, d):
  X, _, v, off = _fir_design(d)
  pred = np.zeros(v.shape)
  for (lo, hi), h in models.items():
    m = (v >= lo) & (v < hi)
    pred[m] = X[m] @ h
  out = np.repeat(pred, 5, axis=0)  # back to 100 Hz (sample-and-hold)
  full = np.full(d['ang'].shape, np.nan)
  full[off:off + len(out)] = out[:len(full) - off]
  return np.where(np.isnan(full), d['ang'], full)  # the unscored warm-up keeps the measurement


def step_response(p, v, amp=0.03, t_end=4.0):
  emu = PSCMEmulator(p, n=1)
  emu.reset(p.angle_offset_deg, 0.0, v)
  nom = emu._nominal_deg_per_c1(v)
  n = int(t_end / DT)
  out = np.zeros(n)
  for k in range(n):
    o = emu.step(1, amp if k * DT >= 0.5 else 0.0, v)
    out[k] = (o['steeringAngleDeg'][0] - p.angle_offset_deg) / (nom * amp)
  return np.arange(n) * DT - 0.5, out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--params', default=DEFAULT_PARAMS)
  ap.add_argument('--cache', default=fit.DEFAULT_CACHE)
  ap.add_argument('--out', default='pscm_validation')
  ap.add_argument('--no-ablation', action='store_true')
  ap.add_argument('-j', type=int, default=30)
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=True)
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt

  p = PSCMParams.load(a.params)
  train, test = fit.build_dataset(a.cache)
  _, rtest = fit.build_dataset(a.cache, 'release')
  res = {}

  ang_emu, yaw_emu = fit.simulate(p, test, 'chain')
  ang_static = fit.baseline_static(test, p)
  firm = fir_fit(train)
  ang_fir = fir_predict(firm, test)
  for name, sim in (('emulator', ang_emu), ('static', ang_static), ('fir_linear_blackbox', ang_fir)):
    res[f'test_angle_{name}'] = fit.metrics(sim, test['ang'], test)
  res['test_yaw_emulator'] = fit.metrics(yaw_emu, test['yaw'], test, 180 / np.pi)
  res['release_test_emulator'] = fit.release_metrics(p, rtest)

  if not a.no_ablation:
    spec = [s for s in fit.STAGE_A if s[0] != 'c1_slew']
    q = fit.lm_fit(PSCMParams(**{**asdict(p), 'c1_slew': 50.0}), spec, train, 'A', workers=a.j, log=lambda *_: None)
    res['ablation_no_c1_slew'] = fit.metrics(fit.simulate(q, test, 'A')[0], test['ang'], test)
    # where would the slew bind? fraction of engaged time the wire C1 moves faster than the fitted slew
    rate = np.abs(np.diff(test['c1'][::5], axis=0)) / 0.05
    res['c1_rate_above_slew_frac'] = float(np.mean(rate > p.c1_slew))
    res['c1_rate_p99'] = float(np.percentile(rate, 99))

  print(json.dumps(res, indent=1))
  with open(os.path.join(a.out, 'validation.json'), 'w') as f:
    json.dump(res, f, indent=1)

  # --- plots ----------------------------------------------------------------------------------------
  t = np.arange(test['c1'].shape[0]) * DT
  vm = test['v'].mean(0)
  dyn = np.std(np.diff(test['ang'], axis=0), axis=0)
  fig, axes = plt.subplots(4, 2, figsize=(16, 14))
  for row, (lo, hi) in enumerate(BANDS):
    cand = np.flatnonzero((vm >= lo) & (vm < hi))
    for col, i in enumerate(cand[np.argsort(dyn[cand])[-2:]][::-1] if len(cand) else []):
      ax = axes[row, col]
      ax.plot(t, test['ang'][:, i], 'k', lw=1.4, label='measured')
      ax.plot(t, ang_emu[:, i], 'C0', lw=1.1, label='emulator')
      ax.plot(t, ang_fir[:, i], 'C2', lw=0.8, alpha=0.8, label='FIR (linear)')
      ax.plot(t, ang_static[:, i], 'C3', lw=0.8, alpha=0.7, label='static 0.8x')
      ax2 = ax.twinx()
      ax2.plot(t, test['c1'][:, i], color='0.6', lw=0.7, ls='--')
      ax2.set_ylabel('wire C1 (rad)', color='0.5')
      ax.axvspan(0, fit.WARMUP_S, color='0.9')
      ax.set_title(f'test window {i}: {vm[i]:.1f} m/s')
      ax.set_ylabel('steering angle (deg)')
      if row == 0 and col == 0:
        ax.legend(loc='upper left', fontsize=8)
  axes[-1, 0].set_xlabel('s')
  axes[-1, 1].set_xlabel('s')
  fig.tight_layout()
  fig.savefig(os.path.join(a.out, 'test_windows.png'), dpi=90)

  ra, ry = fit.simulate(p, rtest, 'chain')
  tr_ = np.arange(rtest['c1'].shape[0]) * DT - fit.REL_PRE_S
  err = np.abs(rtest['ang'] - ra)[int(fit.REL_PRE_S / DT):int((fit.REL_PRE_S + 5) / DT)].mean(0)
  pick = np.argsort(err)[[len(err) // 2, int(len(err) * 0.9), -1]]
  fig, axes = plt.subplots(1, 3, figsize=(16, 4))
  for ax, i, lab in zip(axes, pick, ('median', 'p90', 'worst')):
    ax.plot(tr_, rtest['ang'][:, i], 'k', lw=1.4, label='measured')
    ax.plot(tr_, ra[:, i], 'C0', label='emulator (forced while hands-on)')
    ax.axvline(0, color='0.5', ls=':')
    ax.set_title(f'release {lab} error ({"manual turn" if rtest["manual_turn"][0, i] else "press"})')
    ax.set_xlabel('s from release')
  axes[0].legend(fontsize=8)
  fig.tight_layout()
  fig.savefig(os.path.join(a.out, 'releases.png'), dpi=90)

  fig, ax = plt.subplots(figsize=(8, 4.5))
  for v in (8, 15, 22, 30):
    ts, s = step_response(p, v)
    ax.plot(ts, s, label=f'{v} m/s')
  ax.axhline(1, color='0.7', lw=0.8)
  ax.set_xlabel('s after C1 step')
  ax.set_ylabel('wheel angle / nominal')
  ax.set_title('emulator step response (0.03 rad C1 step)')
  ax.legend()
  fig.tight_layout()
  fig.savefig(os.path.join(a.out, 'step_response.png'), dpi=90)
  print('plots ->', a.out)


if __name__ == '__main__':
  main()

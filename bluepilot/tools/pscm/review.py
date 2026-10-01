#!/usr/bin/env python3
"""
BluePilot: review a road-test route's driver grabs -- one table row and one plot sheet per grab.

A grab is a steering press of at least --min-press seconds while lateral was active. Presses less
than MERGE_S apart are one grab (steeringPressed chatters), and the grab starts at the torque onset:
the driver is often already steering with 1-3 Nm, below the press threshold, a second or two before
steeringPressed -- "the car departed from the plan" before that point is the driver, not the PSCM.
For each one:

  t, dur      grab start (torque onset, s from route start) and length to the last release
  pre_tq      seconds the torque onset precedes steeringPressed
  v           speed at the press (m/s)
  ang_max     max |steering angle| during the press (deg; > 90 is usually a navigation turn)
  blink       blinker on within 3 s before the press
  plan, car   planner and measured curvature at the press (1e-3 1/m, controller sign)
  short       mean (plan - car) toward the outside of the curve over the 3 s before (1e-3 1/m;
              > 0 = car turning less than the plan, i.e. running wide)
  lag_s       lag of car curvature behind the plan over the 6 s before (xcorr peak)
  ay_max      max measured lateral accel over the 3 s before (m/s^2; PSCM ceiling ~1.84)
  lane_off    lane-line offset (llL + llR) / 2 at the press (m; + = car left of lane center)
  devlim/lim  share of the 3 s before with the deviation clip binding / LatCtlLim_D_Stat set
  kind        nav (ang_max > 90 or blinker) | wide | cut | other -- a first guess; check the sheet

Each sheet (--sheets DIR) has plan / car / wire curvature, speed and PSCM flags over -8 s .. +3 s
and five qcamera frames (needs qcamera.ts beside each rlog).

--releases reviews the hand-back instead: one row per grab, from its last release, while lateral
stays active --

  settle      s until |plan - car| stays within max(1.5e-3, 25% of plan) for 1 s (plan taken
              REL_LAG_S earlier, the planner's delay)
  drift       worst lateral offset (m) the curvature error builds within 6 s (2 s leak)
  mode0       s of mode 0 on the wire in the first 3 s (human-turn tail + pulses)
  pulse_s     when a stall/hand-off pulse fired after the release (s), nan if none
  devlim/lim  share of the first 3 s with the deviation clip binding / LatCtlLim_D_Stat set
  regrab      s until the next grab

and, with --sheets, a release-centred sheet (-4 .. +7 s) that also marks mode 0 and pulses and
plots the wheel angle and driver torque.

Usage:
  review.py ROUTE_DIR [--cache DIR] [--sheets DIR] [--min-press 0.4] [--releases]
  (ROUTE_DIR is the CommaRoutes folder that extract.py was run on)
"""
import argparse
import glob
import os
import re

import numpy as np

from bluepilot.tools.pscm.extract import DEFAULT_CACHE
from bluepilot.tools.pscm.route import Route

PRE_S = 3.0
REL_LAG_S = 0.2   # planner -> car delay used when comparing after a release
REL_S = 6.0
MERGE_S = 1.0     # presses closer than this are one grab
ONSET_TQ = 1.0    # Nm; hands-free |steeringTorque| p99.9 is ~1.6, p99 ~0.7


def _runs(mask):
  e = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
  return list(zip(e[::2], e[1::2]))


def _lag(plan, car, dt, max_s=1.5):
  plan, car = plan - plan.mean(), car - car.mean()
  if plan.std() < 1e-5 or car.std() < 1e-5:
    return np.nan
  lags = np.arange(0, int(max_s / dt))
  c = [np.dot(plan[:len(plan) - k], car[k:]) for k in lags]
  return lags[int(np.argmax(c))] * dt


def grabs(r, min_press=0.4):
  car = -r.yaw / np.maximum(r.v, 0.5)
  out = []
  merged = []
  for i0, i1 in _runs(r.pressed > 0):
    if merged and r.tr[i0] - r.tr[merged[-1][1] - 1] < MERGE_S:
      merged[-1] = (merged[-1][0], i1, max(merged[-1][2], r.tr[i1 - 1] - r.tr[i0]))
    else:
      merged.append((i0, i1, r.tr[i1 - 1] - r.tr[i0]))
  for i0, i1, longest in merged:
    if longest < min_press or not r.latActive[max(i0 - 5, 0)]:
      continue
    ip = i0
    while i0 > 0 and abs(r.tq[i0 - 1]) > ONSET_TQ and np.sign(r.tq[i0 - 1]) == np.sign(r.tq[ip]):
      i0 -= 1
    dur = r.tr[i1 - 1] - r.tr[i0]
    pre = slice(max(i0 - int(PRE_S / r.dt), 0), i0)
    pre6 = slice(max(i0 - int(6.0 / r.dt), 0), i0)
    plan_p = r.acurv[pre]
    side = np.sign(np.mean(plan_p)) or 1.0
    ay = np.abs(car[pre]) * r.v[pre] ** 2
    ang_max = np.max(np.abs(r.ang[i0:i1]))
    blink = bool(np.any(r.lblink[pre6] + r.rblink[pre6]))
    short = float(np.mean((plan_p - car[pre]) * side))
    kind = ('nav' if ang_max > 90 or blink else
            'wide' if short > 1e-3 else 'cut' if short < -1e-3 else 'other')
    out.append(dict(i0=i0, i1=i1, t=r.tr[i0], dur=dur, pre_tq=r.tr[ip] - r.tr[i0], v=r.v[i0], ang_max=ang_max, blink=blink,
                    plan=r.acurv[i0] * 1e3, car=car[i0] * 1e3, short=short * 1e3,
                    lag_s=_lag(r.acurv[pre6], car[pre6], r.dt), ay_max=float(np.max(ay)) if len(ay) else 0.0,
                    lane_off=(r.llL[i0] + r.llR[i0]) / 2, devlim=float(np.mean(r.devLim[pre] > 0)),
                    lim=float(np.mean(r.limStat[pre] > 0)), kind=kind))
  return out


def releases(r, min_press=0.4):
  car = -r.yaw / np.maximum(r.v, 0.5)
  plan = np.roll(r.acurv, int(REL_LAG_S / r.dt))
  gl = grabs(r, min_press)
  out = []
  for k, g in enumerate(gl):
    i = g['i1']
    j = min(i + int(REL_S / r.dt), len(r))
    if j - i < int(1.0 / r.dt) or not r.latActive[i:min(i + 50, len(r))].all():
      continue
    e = (plan - car)[i:j]
    ok = np.abs(e) < np.maximum(1.5e-3, 0.25 * np.abs(plan[i:j]))
    settle, run = np.nan, 0
    for n, o in enumerate(ok):
      run = run + 1 if o else 0
      if run >= int(1.0 / r.dt):
        settle = (n - run + 1) * r.dt
        break
    h = y = worst = 0.0
    for n in range(i, j):  # heading and lateral offset from the curvature error, 2 s leak
      h += (r.v[n] * e[n - i] - h / 2.0) * r.dt
      y += (r.v[n] * h - y / 2.0) * r.dt
      worst = max(worst, abs(y))
    w3 = slice(i, min(i + int(3.0 / r.dt), len(r)))
    pulses = np.flatnonzero(r.blipSrc[i:j] > 0)
    out.append(dict(i=i, t=r.tr[i], grab_t=g['t'], v=r.v[i], ang=r.ang[i], plan=plan[i] * 1e3, car=car[i] * 1e3,
                    settle=settle, drift=worst, mode0=float(np.sum(r.lmcMode[w3] == 0) * r.dt),
                    pulse_s=pulses[0] * r.dt if len(pulses) else np.nan, devlim=float(np.mean(r.devLim[w3] > 0)),
                    lim=float(np.mean(r.limStat[w3] > 0)),
                    regrab=(gl[k + 1]['i0'] - i) * r.dt if k + 1 < len(gl) else np.nan, kind=g['kind']))
  return out


class _QCam:
  """qcamera frames by route time; segments are 60 s, decoded lazily."""
  def __init__(self, route_dir, rid):
    self.files = {}
    for f in glob.glob(os.path.join(route_dir, f'{rid}--*', 'qcamera.ts')):
      self.files[int(re.search(r'--(\d+)/qcamera', f).group(1))] = f
    self.cache = {}

  def frame(self, t):
    seg = int(t // 60)
    if seg not in self.files:
      return None
    if seg not in self.cache:
      import av
      with av.open(self.files[seg]) as c:
        self.cache = {seg: [f.to_ndarray(format='rgb24') for f in c.decode(video=0)]}
    frames = self.cache[seg]
    return frames[min(int((t - seg * 60) * len(frames) / 60.0), len(frames) - 1)] if frames else None


def sheet(r, g, cam, path, title):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  s = slice(max(g['i0'] - int(8 / r.dt), 0), min(g['i1'] + int(3 / r.dt), len(r)))
  t = r.tr[s] - g['t']
  v = np.maximum(r.v[s], 0.5)
  fig = plt.figure(figsize=(16, 9))
  gs = fig.add_gridspec(3, 5, height_ratios=[2.2, 1, 1.4])
  ax = fig.add_subplot(gs[0, :])
  ax.plot(t, r.acurv[s] * 1e3, label='plan', lw=1.6)
  ax.plot(t, -r.yaw[s] / v * 1e3, label='car (yaw/v)', lw=1.6)
  ax.plot(t, r.lmcPA[s] / v * 1e3, label='wire C1/v', lw=1, alpha=0.8)
  ax.axvspan(0, g['dur'], color='k', alpha=0.08)
  for col, c in (('devLim', 'tab:red'), ('limStat', 'tab:purple')):
    m = getattr(r, col)[s] > 0
    for a, b in _runs(m):
      ax.axvspan(t[a], t[b - 1], ymin=0.95 if col == 'devLim' else 0.9, ymax=1.0 if col == 'devLim' else 0.95, color=c)
  ax.set_ylabel('curvature 1e-3 1/m  (red: dev clip, purple: PSCM limit)')
  ax.legend(loc='lower left')
  ax.grid(alpha=0.3)
  ax.set_title(title)
  ax2 = fig.add_subplot(gs[1, :], sharex=ax)
  ax2.plot(t, r.v[s], color='k', label='v m/s')
  ax2.plot(t, (r.llL[s] + r.llR[s]) / 2 * 10, color='tab:green', label='lane offset x10 (m)')
  ax2.plot(t, np.abs(r.yaw[s]) * r.v[s], color='tab:orange', label='|ay| m/s^2')
  ax2.axhline(1.84, color='tab:orange', ls=':', lw=0.8)
  ax2.legend(loc='upper left', ncol=3)
  ax2.grid(alpha=0.3)
  for k, dt in enumerate((-6, -3, -1, 0, 1.5)):
    fr = cam.frame(g['t'] + dt)
    a = fig.add_subplot(gs[2, k])
    if fr is not None:
      a.imshow(fr)
    a.set_title(f'{dt:+.1f} s')
    a.axis('off')
  fig.tight_layout()
  fig.savefig(path, dpi=80)
  plt.close(fig)


def release_sheet(r, rel, cam, path, title):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  i = rel['i']
  s = slice(max(i - int(4 / r.dt), 0), min(i + int(7 / r.dt), len(r)))
  t = r.tr[s] - rel['t']
  v = np.maximum(r.v[s], 0.5)
  fig = plt.figure(figsize=(16, 10))
  gs = fig.add_gridspec(4, 5, height_ratios=[2.2, 1.1, 1, 1.4])
  ax = fig.add_subplot(gs[0, :])
  ax.plot(t, r.acurv[s] * 1e3, label='plan', lw=1.6)
  ax.plot(t, -r.yaw[s] / v * 1e3, label='car (yaw/v)', lw=1.6)
  ax.plot(t, r.lmcPA[s] / v * 1e3, label='wire C1/v', lw=1)
  for a, b in _runs(r.pressed[s] > 0):
    ax.axvspan(t[a], t[b - 1], color='k', alpha=0.08)
  for col, c, y0 in (('devLim', 'tab:red', 0.95), ('limStat', 'tab:purple', 0.9)):
    for a, b in _runs(getattr(r, col)[s] > 0):
      ax.axvspan(t[a], t[b - 1], ymin=y0, ymax=y0 + 0.05, color=c)
  for a, b in _runs(r.lmcMode[s] == 0):
    ax.axvspan(t[a], t[b - 1], ymin=0.85, ymax=0.9, color='tab:orange')
  for a, b in _runs(r.blipSrc[s] > 0):
    ax.axvspan(t[a], t[b - 1], ymin=0.8, ymax=0.85, color='tab:green')
  ax.axvline(0, color='k', lw=0.8)
  ax.set_ylabel('curvature 1e-3 1/m')
  ax.legend(loc='lower left')
  ax.grid(alpha=0.3)
  ax.set_title(title + '   [grey: pressed, red: dev clip, purple: PSCM limit, orange: mode 0, green: pulse]')
  ax2 = fig.add_subplot(gs[1, :], sharex=ax)
  ax2.plot(t, r.ang[s], label='wheel deg')
  ax2b = ax2.twinx()
  ax2b.plot(t, r.tq[s], color='tab:red', lw=0.8, label='driver Nm')
  ax2.legend(loc='upper left')
  ax2b.legend(loc='upper right')
  ax2.grid(alpha=0.3)
  ax3 = fig.add_subplot(gs[2, :], sharex=ax)
  ax3.plot(t, r.v[s], color='k', label='v m/s')
  ax3.plot(t, (r.llL[s] + r.llR[s]) / 2 * 10, color='tab:green', label='lane offset x10 (m)')
  ax3.legend(loc='upper left')
  ax3.grid(alpha=0.3)
  for k, dt in enumerate((-1, 0.5, 1.5, 3, 5)):
    fr = cam.frame(rel['t'] + dt)
    a = fig.add_subplot(gs[3, k])
    if fr is not None:
      a.imshow(fr)
    a.set_title(f'{dt:+.1f} s')
    a.axis('off')
  fig.tight_layout()
  fig.savefig(path, dpi=75)
  plt.close(fig)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('route_dir')
  ap.add_argument('--cache', default=DEFAULT_CACHE)
  ap.add_argument('--sheets', default=None)
  ap.add_argument('--min-press', type=float, default=0.4)
  ap.add_argument('--releases', action='store_true', help='review what happens after each release instead')
  a = ap.parse_args()
  rid = os.path.basename(a.route_dir.rstrip('/')).split('_')[-1]
  r = Route(a.cache, rid)
  if a.releases:
    rl = releases(r, a.min_press)
    hdr = ('grab_t', 't', 'v', 'ang', 'plan', 'car', 'settle', 'drift', 'mode0', 'pulse_s', 'devlim', 'lim', 'regrab', 'kind')
    print(f'{rid}: {len(rl)} releases')
    print(' '.join(f'{h:>8}' for h in hdr))
    for x in rl:
      print(' '.join(f'{x[h]:8.2f}' if isinstance(x[h], float) else f'{str(x[h]):>8}' for h in hdr))
    if a.sheets:
      os.makedirs(a.sheets, exist_ok=True)
      cam = _QCam(a.route_dir, rid)
      for x in rl:
        release_sheet(r, x, cam, os.path.join(a.sheets, f'{rid}_rel{x["t"]:06.1f}.png'),
                      f'{rid} release t={x["t"]:.1f}  v={x["v"]:.1f} m/s')
      print('sheets ->', a.sheets)
    return
  gl = grabs(r, a.min_press)
  eng = (r.latActive > 0).sum() * r.dt / 60
  print(f'{rid}: {r.tr[-1] / 60:.1f} min, {eng:.1f} engaged, {len(gl)} grabs')
  hdr = ('t', 'dur', 'pre_tq', 'v', 'ang_max', 'blink', 'plan', 'car', 'short', 'lag_s', 'ay_max', 'lane_off', 'devlim', 'lim', 'kind')
  print(' '.join(f'{h:>8}' for h in hdr))
  for g in gl:
    print(' '.join(f'{g[h]:8.2f}' if isinstance(g[h], float) else f'{str(g[h]):>8}' for h in hdr))
  if a.sheets:
    os.makedirs(a.sheets, exist_ok=True)
    cam = _QCam(a.route_dir, rid)
    for g in gl:
      sheet(r, g, cam, os.path.join(a.sheets, f'{rid}_t{g["t"]:06.1f}.png'),
            f'{rid} t={g["t"]:.1f}  v={g["v"]:.1f} m/s  ang_max={g["ang_max"]:.0f} deg  kind={g["kind"]}')
    print('sheets ->', a.sheets)


if __name__ == '__main__':
  main()

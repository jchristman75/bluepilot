#!/usr/bin/env python3
"""
BluePilot: test which of the PSCM Walkthrough's (F-150 Lightning reference firmware) path-angle
pipeline findings hold on this car, from hands-free angle-mode driving.

  A. release hang   -- the supervisor retains demand until held C1 unwinds, so curve-exit lag should
                       exceed curve-entry lag and grow with the path angle held in the curve.
  B. held-C1 slew   -- the PSCM slews an internal copy of C1 at ~0.1 rad/s, so the delivered rise
                       rate should plateau there however fast we command.
  C. small-signal   -- the curvature filter is slow (~0.68 s) near zero C1 and fast (~0.07 s) at
     filter            large C1, so command->response lag should shrink as |C1| grows.

Only hands-free, angle-mode (lmcMode 1) stretches count: no press within 2 s, no human-turn pause,
stall pulse, lane change, or deviation-clip binding within 0.5 s. Delivered response is -yawRate, compared against the wire path angle;
the gain factor between them doesn't matter because A and C normalise each event by its own
peak/scale and B converts to wire-equivalent units with the event's steady-state gain.

Usage: check_dynamics.py [--cache DIR] [--pattern GLOB]
"""
import argparse

import numpy as np

from bluepilot.tools.pscm.extract import DEFAULT_CACHE
from bluepilot.tools.pscm.route import Route, route_ids

PEAK_MIN = 0.05       # rad, smallest curve event considered
ACTIVE_THRESH = 0.02  # rad, |cmd| above this is "in the curve"
QUIET = 0.008         # rad, |cmd| below this is "straight"
SPEED_MIN = 5.0       # m/s


def smooth(x, n):
  if n <= 1:
    return x
  k = np.ones(n) / n
  return np.convolve(x, k, mode='same')


def clean_mask(r):
  press_recent = r.since_last(r.pressed > 0) < 2.0
  # The deviation clip makes the command a copy of the measurement (+- tolerance) while it binds,
  # which would read as zero-lag tracking; keep 0.5 s clear of it on both sides.
  clip_near = smooth((r.devLim > 0).astype(float), int(1.0 / r.dt)) > 0
  return (~clip_near & (r.latActive > 0) & (r.lmcMode > 0) & (r.v > SPEED_MIN) & ~press_recent &
          (r.htPaused == 0) & (r.blip == 0) & ~np.isin(r.lcState, (1, 2, 3)))


def cross_time(x, level, start, stop, rising):
  """First index in [start, stop) where x crosses level (linear interpolation, in samples)."""
  seg = x[start:stop]
  hit = np.flatnonzero(seg >= level) if rising else np.flatnonzero(seg <= level)
  if not len(hit) or hit[0] == 0:
    return None
  i = start + hit[0]
  x0, x1 = x[i - 1], x[i]
  return i - 1 + (level - x0) / (x1 - x0) if x1 != x0 else float(i)


def curve_events(r, ok):
  """Single-signed curve events: straight -> |cmd| peak >= PEAK_MIN -> straight, all clean."""
  cmd = r.lmcPA
  n = len(r)
  active = np.abs(cmd) > ACTIVE_THRESH
  edges = np.flatnonzero(np.diff(active.astype(np.int8)))
  starts = edges[::1][active[edges + 1]] + 1 if len(edges) else []
  events = []
  for s in starts:
    e = s
    while e < n and active[e]:
      e += 1
    if e >= n:
      break
    seg = cmd[s:e]
    if np.any(np.sign(seg) != np.sign(seg[0])):
      continue
    k = int(np.argmax(np.abs(seg))) + s
    if abs(cmd[k]) < PEAK_MIN:
      continue
    # extend to quiet on both sides, within 5 s
    a = s
    while a > 0 and abs(cmd[a]) > QUIET and s - a < 500:
      a -= 1
    b = e
    while b < n - 1 and abs(cmd[b]) > QUIET and b - e < 500:
      b += 1
    if abs(cmd[a]) > QUIET or abs(cmd[b]) > QUIET or not ok[a:b + 150].all():
      continue
    events.append((a, k, b + 150))  # keep 1.5 s after the command settles for the response tail
  return events


def analyse_events(r, ok):
  out = []
  sgn_meas = smooth(r.meas_pa, 10)
  for a, k, b in curve_events(r, ok):
    sg = np.sign(r.lmcPA[k])
    cmd = r.lmcPA * sg
    meas = sgn_meas * sg
    pk_c = cmd[k]
    km = a + int(np.argmax(meas[a:b]))
    pk_m = meas[km]
    if pk_m < 0.3 * pk_c:
      continue
    # A: 50% crossings, each signal normalised by its own peak
    tc_in = cross_time(cmd, 0.5 * pk_c, a, k + 1, True)
    tm_in = cross_time(meas, 0.5 * pk_m, a, km + 1, True)
    tc_out = cross_time(cmd, 0.5 * pk_c, k, b, False)
    tm_out = cross_time(meas, 0.5 * pk_m, km, b, False)
    if None in (tc_in, tm_in, tc_out, tm_out):
      continue
    dt = r.dt
    # B: rise rates in wire units. Delivered is scaled by the event's steady gain near the peak.
    win = slice(max(a, k - 25), min(b, k + 25))
    g = np.mean(cmd[win]) / max(np.mean(meas[win]), 1e-4)
    # 20->80% rise, as an average rate over 60% of the peak: robust where max(d/dt) of yaw noise isn't.
    c20, c80 = cross_time(cmd, 0.2 * pk_c, a, k + 1, True), cross_time(cmd, 0.8 * pk_c, a, k + 1, True)
    m20, m80 = cross_time(meas, 0.2 * pk_m, a, km + 1, True), cross_time(meas, 0.8 * pk_m, a, km + 1, True)
    rc = 0.6 * pk_c / max((c80 - c20) * dt, dt) if None not in (c20, c80) else np.nan
    rm = 0.6 * pk_m * g / max((m80 - m20) * dt, dt) if None not in (m20, m80) else np.nan
    out.append(dict(rid=r.rid, t=r.tr[k], v=float(np.mean(r.v[a:b])), peak=pk_c, gain=g,
                    lag_in=(tm_in - tc_in) * dt, lag_out=(tm_out - tc_out) * dt,
                    rate_cmd=rc, rate_meas=rm))
  return out


def small_signal(r, ok, win_s=4.0):
  """C: lag maximising correlation of d(cmd) and d(meas) in clean windows, with the window's mean |cmd|."""
  # Fit meas ~ gain * firstorder(cmd delayed by L, tau) + bias per window on a (L, tau) grid.
  # A cross-correlation peak would only find the delay (a first-order impulse response peaks at
  # zero lag), so the filter's time constant has to be fitted, not read off a correlation.
  step = max(1, int(round(0.05 / r.dt)))  # decimate to 20 Hz
  cmd = smooth(r.lmcPA, step)[::step]
  meas = smooth(r.meas_pa, step)[::step]
  okd = ok[::step]
  dt = r.dt * step
  n_win, n_warm = int(win_s / dt), int(2.0 / dt)
  delays = np.arange(0.10, 0.55, 0.05)
  taus = np.array([0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.45, 0.68, 1.0])
  res = []
  for s in range(n_warm + int(delays[-1] / dt), len(cmd) - n_win, n_win // 2):
    if not okd[s - n_warm:s + n_win].all():
      continue
    y = meas[s:s + n_win]
    if np.std(cmd[s:s + n_win]) < 0.004:  # too little excitation to see any dynamics
      continue
    best = (np.inf, None, None)
    for L in delays:
      k = int(round(L / dt))
      u = cmd[s - n_warm - k:s + n_win - k]
      for tau in taus:
        alpha = dt / (tau + dt)
        f = np.empty_like(u)
        f[0] = u[0]
        for i in range(1, len(u)):
          f[i] = f[i - 1] + alpha * (u[i] - f[i - 1])
        x = f[n_warm:]
        A = np.column_stack([x, np.ones_like(x)])
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        sse = float(np.sum((A @ coef - y) ** 2))
        if sse < best[0]:
          best = (sse, L, tau)
    r2 = 1 - best[0] / max(float(np.sum((y - y.mean()) ** 2)), 1e-12)
    if r2 < 0.7:
      continue
    res.append((float(np.mean(np.abs(cmd[s:s + n_win]))), best[2], best[1], float(np.mean(r.v[::step][s:s + n_win]))))
  return res


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--cache', default=DEFAULT_CACHE)
  ap.add_argument('--pattern', default='*')
  a = ap.parse_args()

  events, ss = [], []
  for rid in route_ids(a.cache, a.pattern):
    r = Route(a.cache, rid)
    ok = clean_mask(r)
    events += analyse_events(r, ok)
    ss += small_signal(r, ok)

  ev = {k: np.array([e[k] for e in events]) for k in ('peak', 'v', 'lag_in', 'lag_out', 'rate_cmd', 'rate_meas')}
  print(f'{len(events)} clean hands-free curve events\n')

  print('A. release hang: exit lag vs entry lag, by path angle held in the curve')
  print('   peak |C1| rad   n   entry lag (s)   exit lag (s)   exit-entry (s)')
  for lo, hi in ((0.05, 0.08), (0.08, 0.12), (0.12, 0.2), (0.2, 0.3), (0.3, 0.6)):
    m = (ev['peak'] >= lo) & (ev['peak'] < hi)
    if m.sum() >= 3:
      d = ev['lag_out'][m] - ev['lag_in'][m]
      print(f'   {lo:.2f}-{hi:.2f}     {m.sum():4d}   {np.median(ev["lag_in"][m]):6.2f}          {np.median(ev["lag_out"][m]):6.2f}' +
            f'         {np.median(d):+6.2f}  (IQR {np.percentile(d, 25):+.2f}..{np.percentile(d, 75):+.2f})')
  if len(events) > 5:
    d = ev['lag_out'] - ev['lag_in']
    slope, icpt = np.polyfit(ev['peak'], d, 1)
    print(f'   fit: exit-entry = {icpt:+.2f} s {slope:+.2f} s/rad x peak   (hang predicts a positive slope)')

  print('\nB. held-C1 slew: 20->80% rise rate, delivered (wire-equivalent rad/s) vs commanded')
  print('   cmd rate rad/s    n   delivered median   p90')
  for lo, hi in ((0, 0.05), (0.05, 0.1), (0.1, 0.15), (0.15, 0.25), (0.25, 5)):
    m = (ev['rate_cmd'] >= lo) & (ev['rate_cmd'] < hi)
    if m.sum() >= 3:
      print(f'   {lo:.2f}-{hi:.2f}      {m.sum():4d}   {np.nanmedian(ev["rate_meas"][m]):6.3f}          ' +
            f'{np.nanpercentile(ev["rate_meas"][m], 90):6.3f}')
  print('   (a slew limit shows as delivered rates flattening near 0.1 while commanded keeps rising)')

  print('\nC. small-signal filter: fitted first-order time constant (and delay) vs operating |C1|')
  s = np.array(ss) if ss else np.zeros((0, 4))
  print('   mean |C1| rad    n    tau median (IQR)        delay median')
  for lo, hi in ((0, 0.01), (0.01, 0.02), (0.02, 0.05), (0.05, 0.1), (0.1, 0.6)):
    m = (s[:, 0] >= lo) & (s[:, 0] < hi)
    if m.sum() >= 5:
      print(f'   {lo:.2f}-{hi:.2f}     {m.sum():5d}   {np.median(s[m, 1]):.2f} ({np.percentile(s[m, 1], 25):.2f}..{np.percentile(s[m, 1], 75):.2f})' +
            f'        {np.median(s[m, 2]):.2f}')
  print('   (the PDF filter predicts tau ~0.68 s near zero |C1| falling to ~0.07 s at large |C1|)')

if __name__ == '__main__':
  main()

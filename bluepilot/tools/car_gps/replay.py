#!/usr/bin/env python3
"""BluePilot: replay the car-GPS fallback (BPCarGps) on a recorded drive and score it against the u-blox.

Runs the real code path: the Mach-E powertrain parser from CarState.get_can_parsers, CarStateExt.fill_car_gps for
carStateBP.gps at the 100 Hz card rate, and CarGpsFallback fed every logged u-blox gpsLocationExternal with its fix
removed, as if the u-blox had none. Each fallback message is compared with the real u-blox position at the same time.
Use a drive where the u-blox had a fix (e.g. 4a3). Prints error statistics only, never coordinates.

  python -m openpilot.bluepilot.tools.car_gps.replay /mnt/j/CommaRoutes/c4/<route folder> [--segments N]
"""
import argparse
import glob
import math
import os

import numpy as np

EARTH_RADIUS = 6_378_137.0


def _segments(route_dir, limit):
  paths = glob.glob(os.path.join(route_dir, "*--*", "rlog.zst"))
  paths.sort(key=lambda p: int(os.path.basename(os.path.dirname(p)).rsplit("--", 1)[1]))
  return paths[:limit] if limit else paths


def replay(route_dir, limit=None):
  import openpilot.cereal.messaging as messaging
  from opendbc.car import Bus, structs
  from opendbc.car.ford.carstate import CarState
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR
  from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt
  from openpilot.bluepilot.system.car_gps_bp import CarGpsFallback
  from openpilot.tools.lib.logreader import LogReader

  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  pt = CarState.get_can_parsers(cp, structs.CarParamsSP())[Bus.pt]
  fallback = CarGpsFallback()
  fallback.enabled = True
  fallback.last_ublox_fix_ns = -10**18   # the u-blox "never" had a fix
  bp = messaging.new_message("carStateBP")

  ublox, out = [], []   # (t, lat, lon) of the real u-blox fixes / (t, lat, lon, acc) of the fallback output
  for path in _segments(route_dir, limit):
    for msg in LogReader(path):
      which = msg.which()
      if which == "can":
        pt.update([(msg.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in msg.can])])
        CarStateExt.fill_car_gps(bp.carStateBP.gps, pt)
        fallback.update_car(bp.carStateBP.gps)
      elif which == "gpsLocationExternal":
        real = msg.gpsLocationExternal
        if real.hasFix and real.horizontalAccuracy < 5:
          ublox.append((msg.logMonoTime, real.latitude, real.longitude))
        blank = messaging.new_message("gpsLocationExternal", valid=True)
        blank.gpsLocationExternal.hasFix = False
        sent = fallback.on_ublox(blank, msg.logMonoTime)
        g = sent.gpsLocationExternal
        if g.hasFix:
          out.append((msg.logMonoTime, g.latitude, g.longitude, g.horizontalAccuracy))
  return np.array(ublox), np.array(out)


def main():
  p = argparse.ArgumentParser()
  p.add_argument("route_dir")
  p.add_argument("--segments", type=int)
  args = p.parse_args()
  ublox, out = replay(args.route_dir, args.segments)
  if not len(out):
    print("no fallback output (no car fix in these segments)")
    return
  span = (out[-1, 0] - out[0, 0]) / 1e9
  print(f"fallback messages {len(out)} over {span:.0f} s = {len(out) / max(span, 1e-9):.1f} Hz; u-blox fixes {len(ublox)}")
  if len(ublox) < 2:
    print("no u-blox fix to compare against")
    return
  ok = (out[:, 0] >= ublox[0, 0]) & (out[:, 0] <= ublox[-1, 0])
  la = np.interp(out[ok, 0], ublox[:, 0], ublox[:, 1])
  lo = np.interp(out[ok, 0], ublox[:, 0], ublox[:, 2])
  dn = np.radians(out[ok, 1] - la) * EARTH_RADIUS
  de = np.radians(out[ok, 2] - lo) * EARTH_RADIUS * np.cos(np.radians(la))
  err = np.hypot(dn, de)
  acc = out[ok, 3]
  print(f"error vs u-blox: median {np.median(err):.1f} m, p90 {np.percentile(err, 90):.1f} m, p99 {np.percentile(err, 99):.1f} m, "
        f"max {err.max():.1f} m (n={ok.sum()})")
  print(f"claimed accuracy: median {np.median(acc):.1f} m; error within the claim {100 * np.mean(err <= acc):.0f}% of messages, "
        f"within 2x {100 * np.mean(err <= 2 * acc):.0f}%")


if __name__ == "__main__":
  main()

# PSCM study tools

Offline tools for checking how the Ford PSCM responds to BluePilot's LateralMotionControl2 commands,
using recorded rlogs. Nothing here runs on the device.

```bash
# 1. extract per-segment signal arrays (cached in ~/.cache/bp_pscm_v2, one .npz per segment; ~8 min for
#    all 2,700 segments on 30 cores)
python -m bluepilot.tools.pscm.extract /mnt/j/CommaRoutes/c4/*/ /mnt/j/CommaRoutes/3x/*/

# 2. test the PSCM Walkthrough's (Lightning reference firmware) pipeline claims on this car
python -m bluepilot.tools.pscm.check_dynamics [--pattern '0000044*']

# 3. fit the PSCM emulator (writes mache_pscm_params.json, ~4 min) and check it on held-out routes
python -m bluepilot.tools.pscm.fit
python -m bluepilot.tools.pscm.validate --out /tmp/pscm_validation   # metrics JSON + PNG plots

# 4. closed loop: run the real angle-mode code (+ ford.h) against the emulator on recorded routes,
#    then compare variants route by route
python -m bluepilot.tools.pscm.harness ROUTE_DIR... --out /tmp/h_base --ref HEAD
python -m bluepilot.tools.pscm.harness ROUTE_DIR... --out /tmp/h_new            # working tree
python -m bluepilot.tools.pscm.harness ROUTE_DIR... --out /tmp/h_x \
    --patch "lateral_angle_ext._LEADLAG_TAU_S=(0.5, 1.0)" --set enable_lane_positioning_ang=1
python -m bluepilot.tools.pscm.score --paired /tmp/h_base /tmp/h_new /tmp/h_x

# 5. the commaCarSegments Ford fleet (stock curvature mode, CAN-only rlogs)
python -m bluepilot.tools.pscm.extract_fleet /mnt/j/CommaRoutes/Ford
python -m bluepilot.tools.pscm.fleet --params-dir bluepilot/tools/pscm/fleet_params
```

`route.Route` loads an extracted route as named columns; see `extract.COLS` for the list and the
sign convention (wire values are negated back to the controller's internal sign).

## Results so far (Mach-E, 2026-09, 118 clean hands-free curves, routes 415-450)

| PSCM Walkthrough claim (Lightning) | Mach-E, hands-free angle mode |
|---|---|
| Release hang: supervisor holds old demand until held C1 unwinds | Not seen. Curve-exit lag is 0.16-0.38 s *shorter* than entry lag, on both builds |
| Internal C1 copy slews at ~0.1 rad/s | Untestable and irrelevant hands-free: commands never exceed ~0.15 rad/s or ~0.2 rad |
| Curvature filter ~0.68 s near zero C1, ~0.07 s at large C1 | Not seen. Command->yaw is ~0.15 s delay + 0.02-0.05 s time constant at every amplitude; entry 50% lag is 0.16 s total |

So the Lightning firmware constants should not be imported for the Mach-E; its servo is fast in
the range BluePilot drives it hands-free. What does limit it there is the post-release bias
(see `lateral_angle_ext.py` `_STALL_REVERSED_FLOOR_RATIO`), not servo dynamics.

## Command shaping (2026-09-28, routes 41x-45x + 1a7-1af)

A regularized FIR fit of wire path_angle -> yaw rate (hands-free, deviation clip not binding,
R² 0.85-0.97) shows the PSCM's delivery droops: a step reaches ~0.91 in 0.5 s and settles at ~0.75
by ~1.5 s (20-60 mph); 0.98 -> 0.87 above 60 mph. Identical in LatCtl_D2_Rq 1 and 2. A static
~1.33x gain therefore over-drives transients, which is why `lateral_angle_ext.py` now lead-lag shapes
the wire. Note `extract.py` must latch the PSCM's 972 on bus 0: at route
start a short-lived bus-2 copy can arrive first and freeze LatCtlSte_D_Stat at "Available".

## Emulator (2026-09-28)

`emulator.py` is a grey-box model of the Mach-E PSCM plus the car, driven by what we put on the wire
(LatCtl_D2_Rq, LatCtlPath_An_Actl) and returning `carState.steeringAngleDeg` / `carState.yawRate`.
The stage order comes from the PSCM Walkthrough; every number is fitted to this car
(`mache_pscm_params.json`), because the Lightning constants don't hold here:

```
wire C1 -> 60 ms delay -> held C1, slewed at 0.10 rad/s -> feedforward F(v)*N(v)*C1
        -> minus a leaky droop state (share c(v), 0.2-0.5 s below 20 m/s, ~none above 27 m/s)
        -> + offset + bank term = target -> angle servo (Lightning P/rate schedule x fitted scale)
        -> steering angle -> single-track car model (steady state pinned to the measured
           understeer, SR 15.7) -> 50 ms yaw sensor lag -> yaw rate
```

It is batch-vectorised (one instance steps thousands of windows), and `step(driver_angle=...)`
replays hands-on stretches from the log so emulation resumes from the logged state on release.

**Data:** 1,282 hands-free engaged minutes, all angle mode. Fitted on 1,673 23-s windows. Tested on
443 windows from 35 routes it never saw: 0000041b (highway), 438-450, and the other dongle's
mode-2 drives 1a7-1af. Curvature mode (C2) has ~2 min of data and is **not modelled**.

| held-out routes | emulator | FIR, best linear black box | static 0.8x gain |
|---|---|---|---|
| steering angle rms / R² | 0.60 deg / 0.960 | 0.59 deg / 0.961 | 0.87 deg / 0.924 |
| steering angle R², dynamics only (2 s trend removed) | **0.79** | 0.74 | 0.58 |
| yaw rate rms / R², fully free-running | 0.19 deg/s / 0.967 | - | - |
| yaw rate R², dynamics only | 0.62 | - | - |
| after the driver lets go, first 5 s | 2.0 deg / 0.47 deg/s | - | - |

R² by speed band is flat (0.96-0.97 from 5 to 40 m/s). Step response: ~1.0 at 0.5 s drooping to
0.77-0.85 below 25 m/s; no overshoot, 0.83 at 30 m/s. That matches the FIR measurement above.

**What was tested and rejected:**
- A yaw-feedback droop, where the PSCM integrates yaw error toward a setpoint. It would also have
  explained bank rejection and the post-release bias, but it fitted worse: R² 0.87 vs 0.96, and
  9.0 vs 2.0 deg after release.
- A "hold memory" of the driver's offset after release. Its gain fitted to 0.
- The 0.10 rad/s held-C1 slew independently reproduces the PDF's 0.1 rad/s. But on recorded
  commands it binds only 0.4% of the time (p99 C1 rate 0.08 rad/s), and a refit without it is
  just as good. So it only matters for a controller that commands C1 faster than today's.

**Limits -- read before trusting a result:**
- The servo's rate bound never binds in the data, so it stays at the Lightning values (scale 1.0).
  Anything that commands much faster or larger than recorded driving is extrapolation.
- Post-release behaviour is the weak spot. The model misses a car-specific bias that shows up in
  some releases (mean |yaw offset| 0.17 deg/s, but a few cases like 00000442 t=153 are ~1.7 deg/s).
  It isn't predictable from the pre-release state (corr 0.05). The harness should inject it as a
  disturbance, not expect the emulator to produce it.
- Mode 0 (stall blip, human-turn pause): the wheel is modelled as holding its angle
  (release_tau fitted to its 5 s bound). There is little hands-free mode-0 data.
- The yaw rate is the weaker output (dynamics R² 0.62), because PSCM and vehicle errors compound.

Since then the emulator gained a small-command boost: small commands are delivered up to 21% more,
fading out by |C1/v| ~0.0013. That matches a direct regression on the logs, 0.85-0.88 vs 0.78. It
takes held-out angle R² to 0.961 (dynamics-only 0.795 vs 0.744 for a per-band FIR), and yaw to 0.966.

## Closed-loop harness (`harness.py`, `score.py`)

The harness replays each route's recorded messages at 100 Hz. carControl, modelV2, liveDelay,
liveParameters, the drive's own Params (initData) and CarParams go in as recorded. It runs the real
`LateralCurvExt` + `LateralAngleExt`, with carcontroller's lateral dispatch and packers, at the car's
own 20 Hz phase. Every frame goes through the compiled `ford.h`, and a blocked 982 frame never reaches
the emulator. The emulator's yaw is written back into Yaw_Data_FD1 (checksum recomputed), so ford.h
judges the emulated car. While the log shows a press, or lateral is off, the emulator is forced onto
the logged state.

- **Parity:** open loop, with the code each route was driven on, 84% of control frames match the
  logged wire exactly and 94% within ±2 frames (p90 difference 0.0006 rad). What remains is
  message-arrival jitter.
- **Disturbance replay** (default): a shadow emulator on the *logged* wire and a shadow car on the
  *logged* angle give the part of each drive the model doesn't explain (road, wind, PSCM quirks).
  That part is added into the loop. With the recorded code the loop reproduces the drive to 0.07
  deg/s yaw and 0.28 deg angle, and its tracking/weave/clip metrics match the real car's. Without it
  (`--ideal`) the car is about 40% too clean.
- **Blind spot:** the planner is replayed, not re-run from the emulated car's position. Planner-loop
  interaction (e.g. weave from higher small-signal gain) is invisible.
- `score.py` metrics: see its docstring. `--paired` gives per-route changes with a bootstrap 90% CI
  and the share of routes improved.

## Fleet test (`fleet.py`)

commaCarSegments, 5,278 Ford segments, stock openpilot, all curvature mode (C2 only). The C2 path
holds its level on every platform, at 1.0-1.2x the nominal curvature, with **no droop**. Your
Mach-E's C1 path peaks at 1.0 and droops to 0.77-0.85. So the droop belongs to the path-angle channel,
which is why angle mode needs command shaping. Per-platform C2 fits, tested on cars they never saw,
beat a static gain: angle R² 0.84-0.92 vs 0.76-0.84. Params are in `fleet_params/`.

## Tuning results that drove the 2026-09-28 code changes (42 training routes, nonlinear emulator)

| variant vs committed code | path err | weave | run wide | cut in | hwy weave |
|---|---|---|---|---|---|
| lead-lag off (linear emulator) | +0.3% | +12% | -15% | +84% | - |
| full gain at small curvature | -7.6% | +1.5% | -0.9% | 0 | +1.0% |
| small-curvature gain x1.15 | -5.1% | +0.4% | -0.3% | 0 | +1.9% |
| lead-lag retune below 25 m/s | -6.0% | -2.7% | -4.6% | +14% | -0.2% |
| **retune + x1.15 (shipped)** | **-6.2%** | **-2.7%** | **-4.6%** | **+14%** | **+0.6%** |

- Lead-lag is a cut-vs-wide dial: every faster setting runs wide less and cuts in more.
- Changing the highway values always added highway weave, so they stay as they were.
- Stall detector with a 2e-3 post-release bias injected: if a mode-0 pulse clears the bias, the
  hand-off pulse cuts post-release drift 46%. If it doesn't, the pulses cost nothing measurable. A
  faster reactive detector (0.3 s) adds nothing.

## Final check on 32 held-out routes (240 scored min, never used for fitting or tuning)

Committed code vs working tree (lead-lag retune + small-curvature gain), per route, bootstrap 90% CI:

| metric | nonlinear emulator | linear emulator | lead-lag retune only |
|---|---|---|---|
| path error p95 | -4.6% [-8.5, -3.0], 100% of routes | -5.6% | -0.2% |
| path error rms | -2.0% [-7.5, +0.1], 97% of routes | -2.7% | -0.3% |
| weave | -3.3% [-5.7, -1.4] | -3.2% | -3.9% |
| running wide p95 | -3.7% [-7.2, -1.3] | -3.9% | -3.3% |
| cutting in on entry p95 | +10.0% [+8.1, +15.0] | +10.7% | +10.0% |
| highway weave / running wide (route 0000041b) | 0.0% / -3.1% | 0.0% / -3.8% | 0.0% / 0.0% |

- The same holds with the post-release bias injected (path p95 -4.3%). Panda blocks are unchanged
  (3.6 vs 3.8 per hour).
- On the highway route alone, the small-curvature gain is a wash: rms path error +2%, p95 -1.8%.
  Watch near-straight highway lane keeping on the road test.

## Road test on the emulator build, and the fixes it led to (2026-09-29, routes 000001b0/000001b1)

- **Settings you tried.** Params changes mid-drive aren't logged, so they were reconstructed by
  open-loop harness parity under all 8 toggle combinations.
- **Both on was best.** In same-road harness replays, both-on beat every other combination on path
  error on both routes. With the small-curvature gain off, the real car fell about 2x further short of
  the plan near straight.
- **The toggles are gone.** `FordAngleLeadLag_ang` and `FordAngleSmallCurvGain_ang` were removed; both
  behaviours are now always on.
- **What caused the grabs.** Video plus signals for all 25 grabs: most were navigation turns or set-up
  for them. The road-keeping ones were fast curve exits and S-bends where the car stayed turned.
- **Why the car stayed turned.** Three stacked causes:
  1. *Windup at the PSCM ceiling.* Delivered lateral accel bends over at ~1.84 m/s^2 (slope 0.57
     above it); LatCtlLim_D_Stat marks it. At 000001b1 t=280 the plan wanted 3.45 m/s^2 at 41 mph and
     the wire asked 4.2 while the car gave 2.6.
  2. *The clip-bound unwind lock.* A wire equal to the measured curvature holds the turn on a PSCM
     that delivers ~0.78.
  3. *A PSCM-internal hold.* After a limit episode the car answers a falling command ~0.2 s later
     than usual (p50 0.16 vs -0.05 s, n=30 vs 391).
- **Fixes in `lateral_angle_ext.py`.**
  - An anti-windup cap at the ceiling, active only while the plan is unwinding.
  - Unit gain on clip-bound unwinds, so the wire equals the shadow curvature.
  - A "responding" guard on the reactive stall detector. All 4 road reactive pulses ever logged were
    false fires mid S-bend.
- **Result on 000001b1.** Tracking -15%, running wide -13%, false stall pulses 2 -> 0.
- **What the harness can't show.** The disturbance replay re-inserts the real car's post-limit hold,
  and the emulator doesn't model that hold. Exit timing at the ceiling is therefore only
  partly judged here.
- **Emulator update.** `mache_pscm_params.json` now carries the ceiling (`sat_knee`, `sat_slope`).
- **The remaining cure is curve speed.** Openpilot longitudinal was off on these drives, so nothing
  slowed the car for curves that need more than the ~2 m/s^2 angle mode delivers.

# PSCM study tools

Offline tools for checking how the Ford PSCM responds to BluePilot's LateralMotionControl2 commands,
using recorded rlogs. Nothing here runs on the device.

```bash
# 1. extract per-segment signal arrays (cached in ~/.cache/bp_pscm, one .npz per segment)
python -m bluepilot.tools.pscm.extract /mnt/j/CommaRoutes/c4/*bp-pscm-study*

# 2. test the PSCM Walkthrough's (Lightning reference firmware) pipeline claims on this car
python -m bluepilot.tools.pscm.check_dynamics [--pattern '0000044*']
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

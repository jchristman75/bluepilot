"""
BluePilot: Ford CAN-FD path-angle–primary lateral control (developer).

Steering intent is carried on c1 (``LatCtlPath_An_Actl``): planner curvature (optionally blended
with the model's predicted curvature per ``FordPathAngleBlendRatio``) becomes
``path_angle = κ·v·K``, where K is the speed/curvature gain table below (``curvature_factor``).
The Mach-E PSCM treats that path angle roughly as a yaw-rate request: its steady-state delivery
is ~0.75 of the wire value, which is what K ≈ 1.33 corrects. c0, c2 and c3 are sent as zero.

**Command shaping (lead-lag).** The PSCM's delivery is not flat in time: a step on the wire
reaches ~0.91 of itself within 0.5 s, then droops to ~0.75 over the next ~1.5 s (FIR fit over
~20 h of hands-free Mach-E driving, R² 0.85-0.97, identical in LatCtl_D2_Rq 1 and 2). A static K
therefore over-drives every transient: 0.3-0.7 Hz planner content arrived 1.2-1.27x amplified
(the "jello" weave) and curve entries overshot toward the inside edge. The wire now carries
``v·(r·Kκ + (1-r)·lowpass(Kκ, τ))`` -- K is reached in steady state (so the user factors keep
their meaning) but the immediate response is r·K. Open-loop replay: tracking error 2.7x lower at
20-60 mph. r and tau are speed-scheduled (see _LEADLAG_*); the low-speed values were retuned in
closed loop against the PSCM emulator (bluepilot/tools/pscm) and confirmed on the road
(routes 000001b0/000001b1: turning it off ran ~30% more weave and cut curve entries harder).

**Small-curvature gain.** Below |kappa| ~0.0007 the gain map used 1.0, but the Mach-E PSCM delivers
only ~0.85-0.88 of a small command, so near-straight lane keeping ran ~13% short of the plan. On
the Mach-E platform group that gain is now x1.15 (capped at the large-curve gain); see
_SMALL_CURV_GAIN_CANFD_SUV. Road A/B on 000001b1: off ran ~2x further short of the plan near straight.
It fades out between 22 and 27 m/s (_SMALL_CURV_GAIN_FADE_V): on the highway it added straight-road weave.

**Deviation clip.** κ is clipped to measured ± bp_curvature_error before the gain (mirrors curvature
mode and ford.h's shadow-curvature check). Because the gain is applied after the clip, a binding
clip feeds the car's own yaw back into the command; with the static gain that loop gain was
~1.33 x 0.91 ≈ 1.2, and on an unwind 1.33·(meas - tol) stays above meas for any κ > 0.008 -- the car
could not straighten while the planner asked it to (47-62% of clip-bound unwind time on the
recorded routes). Lead-lag drops the loop gain below 1. On a clip-bound unwind the wire is the
clipped κ itself (unit gain): capping it at the measured curvature was not enough, because the PSCM
holds only ~0.78 of a held command, so "command = measured" still held the turn (road-confirmed on
000001b1, see _shape_wire_kappa).

**Lane centering trim (``lane_center_trim.py``):** a small correction applied to ``kappa_cmd``
itself, before the deviation clip / gain / PSCM clamp / soft ROC below, so it inherits every one of
those limiters. Blends toward lane-line center by confidence and falls back to the model's own
predicted path when lines are missing/unreliable. Disabled during lane changes, user-tunable
(enable, offset, authority) via ``enable_lane_positioning_ang`` / ``custom_path_offset_ang`` /
``lane_centering_strength_ang``.

**Wire mode/ramp:** LatCtl_D2_Rq 2 (PathFollowingExtendedMode, set in carcontroller.py) and
LatCtlRampType_D_Rq 3 (Immediately). Immediate ramp is safe here because every mode-0 frame zeroes
path_angle_last, so re-engagement always starts from zero through the soft ROC.

**Human-turn override**: while the driver manually turns (same sustained-press + angle criteria
as ``lateral_curv_ext``, via the shared ``HumanTurnDetector``), lateral is forced inactive (mode
0, all-zero signals) instead of winding path_angle into a stale command the PSCM has to reconcile
on release -- on the Mach-E's PSCM that reconciliation cost 2-3 s of dead time before control
resumed. Mode 0 is panda-clean by construction: every ford.h check has a legitimate
!steer_control_enabled branch, so no reset-bypass latch involvement. On release, path_angle ramps
back in from zero through the soft ROC below (no jump seed) -- generous at human-turn speeds, and
admitted by ford.h's path_angle ROC check (2% looser) without any bypass.
"""
import numpy as np
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralResult
from opendbc.sunnypilot.car.ford.human_turn import HumanTurnDetector
from opendbc.sunnypilot.car.ford.lane_center_trim import LaneCenterTrim
from opendbc.sunnypilot.car.ford.values_ext import BP_ANGLE_LIMITS
from selfdrive.modeld.constants import ModelConstants

# Hard-coded per-platform gain defaults.
# CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
_GAIN_CAN         = (1.00, 1.15)
# CAN-FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
_GAIN_CANFD_BOF   = (0.95, 0.95)
# CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)
_GAIN_CANFD_SUV   = (1.00, 1.05)

# Small-curvature gain: multiplies the gain used below |kappa| ~0.0007 (near-straight lane keeping),
# where the static map used 1.0 -- i.e. no correction for the PSCM's delivery, which measures
# 0.85-0.88 there on the Mach-E (2 s regression over ~20 h of hands-free angle mode; bound <= 1.2).
# Closed-loop PSCM-emulator replay (bluepilot/tools/pscm, 42 routes, disturbances replayed): 1.10
# cuts lateral path error 3.7%, 1.15 more, full gain 7.6% at +1.5% weave. The harness replays the
# planner rather than re-planning, so it can't see planner-loop weave from a higher small-signal
# gain; 1.15 is the measured delivery's inverse, not the harness optimum. Only the platform group
# it was measured on; everything else keeps 1.0. Capped at the large-curve gain.
_SMALL_CURV_GAIN_CANFD_SUV = 1.15
# ...faded out at highway speed. The cap above is 1.05 x FordHighSpeedFactor_ang, so with a typical
# highF (1.13) the full x1.15 applied on highway straights, where the near-straight gain had been
# 1.00 -- and the PSCM droops least there (0.98 -> 0.87 above 60 mph), so it needs the boost least.
# Road test 2026-10-03 (routes 458/463/464 vs the Sept highway routes 41b/434, same settings): wire/plan
# gain 1.08-1.14 vs 0.92-0.97, straight-road yaw weave +20% at 0.2-0.25 Hz, and the plan itself moving
# 20-30% more -- the planner loop the harness can't see. Full gain to 22 m/s, none from 27 m/s.
_SMALL_CURV_GAIN_FADE_V = (22.0, 27.0)  # m/s

_CANFD_BOF_CARS = frozenset({
  CAR.FORD_F_150_MK14,
  CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_EXPEDITION_MK4,
  CAR.FORD_RANGER_MK2,
})
_CANFD_SUV_CARS = frozenset({
  CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_ESCAPE_MK4_5,
})


# DBC ``LatCtlPath_An_Actl`` (rad) — panda safety uses the same in ``ford.h``; PSCM enforces in firmware.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235


# Default blend ratio validated on F-150 fleet data (0.5s lookup time).
_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT = 0.50

# Variable lookup time (VLT): curvature_lookup_time adapts to speed and curvature magnitude.
# t_lookup = t_base + t_extra_max × speed_factor(v) × kappa_factor(|κ|)
# t_base = liveDelay.lateralDelay + DT_MDL — always matches the planner's pre-compensation floor.
# Extra lookahead collapses toward zero at high speed (PSCM responds faster)
# and at large curvature (prevents blend importing a "start unwinding" signal too early).
_DT_MDL = 0.05                       # model loop period (matches common/realtime.py)
_VLT_T_EXTRA_MAX = 0.10              # max extra lookahead above t_base
_VLT_V_LOW_MS   = 25.0 * 0.44704    # 25 mph — full extra lookahead at or below this speed
_VLT_V_HIGH_MS  = 55.0 * 0.44704    # 55 mph — no extra lookahead at or above this speed
_VLT_KAPPA_FULL  = 0.005             # 1/m — full extra lookahead below this curvature (200m+ radius)
_VLT_KAPPA_TAPER = 0.020             # 1/m — no extra lookahead above this curvature (50m radius)

# Rate cap on path_angle magnitude DECREASE during PSCM LimitReached (rad/call = 0.40 rad/s).
# Both model and planner naturally drop path_angle ~0.36 rad/s at a sharp 90° apex, while the PSCM is
# physically pinned and cannot execute the rapidly falling desired angle. The resulting actual-vs-desired
# gap (up to 47° observed) causes a snap correction the moment the PSCM is released. This cap limits
# the desired-angle drop rate to what the PSCM can reasonably track, at the cost of holding the car
# slightly more in the curve during saturation.
# BluePilot: this strategy runs once per STEER_STEP (CarControllerParams.STEER_STEP=5), i.e. once
# every 5th 100Hz control tick = 20Hz, not every tick. The original 0.004 rad/call value (and its
# "50Hz" comment, corrected above) was authored 2026-05-11 on bp-sid-simple, which had already
# switched STEER_STEP 5->1 (true 100Hz) on 2026-04-22 -- so it was tuned at 100Hz real cadence
# even though its own comment mistakenly said 50Hz. Scaled x5 here to restore the same real-world
# 0.40 rad/s (23 deg/s) unwind rate on this branch's actual 20Hz cadence.
_PSCM_SAT_UNWIND_RATE = 0.02        # rad/call (0.02 * 20Hz = 0.40 rad/s)

# Post-override stall blip. Road test 2026-07-14 (route 886240741b067740/000000bd--feb980680f)
# showed that after driver-touch episodes the Mach-E PSCM keeps reporting InProgress but honors
# path_angle at only ~0.56x (healthy hands-free delivery on the same route: ~0.95 median). The
# current-curvature deviation clip below then pins kappa_cmd at measured + CURVATURE_ERROR, so the
# command can never lead the car enough to overcome the attenuation -- a stall equilibrium the
# driver reads as "not engaging" (wire path_angle flat at ~4 deg for 4.5 s while desired kappa
# climbed to 3x measured, EPS motor current ~0 A). A short mode-0 pulse -- the identical
# panda-clean wire pattern the human-turn override sends, no ford.h involvement -- resets the
# PSCM's authority, after which path_angle ramps back in from zero through the soft ROC.
_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick (matches human_turn.py)
_STALL_GAP_RATIO = 2.0  # x the active clip tolerance (bp_curvature_error): stall gap and the real-curve floor
_STALL_HOLD_S = 0.5          # accumulated clip-binding time before a pulse fires
_STALL_BLIP_FRAMES = 6       # mode-0 pulse length (6 frames @ 20 Hz = 300 ms; PSCM acked mode 0 in ~150 ms on-road)
_STALL_COOLDOWN_S = 2.0      # re-arm delay after a pulse (release ramp + PSCM response time)
_STALL_MAX_BLIPS = 3         # give up on a stuck episode; devLim telemetry keeps recording the stall
# Sign-reversal stalls only: a gap that is still closing is the car responding, not a stall.
# Compared against a smoothed |gap| because frame-to-frame differencing is buried in yawRate
# noise. 0.95 admits closing slower than ~0.17 x |gap| per second -- an order below the rate a
# real S-curve reversal closes at.
_STALL_GAP_TAU = 0.3         # s, smoothing for the |gap| reference
_STALL_GAP_CLOSING = 0.95
# ...but when the plan reverses faster than the car can follow, the gap grows while the car is
# turning the right way. Every reactive pulse ever logged on the road (000001af t=224/229,
# 000001b1 t=507/510) was that: the car was already moving toward the plan at 3.6-6.4e-3 1/m/s,
# and the 300 ms mode-0 pulse landed mid S-curve. A car whose (smoothed) curvature moves toward the
# plan faster than this is responding, not stalled; a real post-release stall holds its curvature.
_STALL_RESPONDING_RATE = 0.002  # 1/m/s
# Anti-windup at the PSCM's lateral-acceleration ceiling. Past ~1.8-2.0 m/s^2 delivered the Mach-E
# PSCM bends over (LatCtlLim_D_Stat comes on; each extra 1 m/s^2 of request buys ~0.5): fitted from
# steady curves and from 10 s windows up to every grab, knee 1.84 m/s^2 / slope 0.57. A plan that
# asks for more (000001b1 t=280: 3.45 m/s^2 at 41 mph) piles request above what the car delivers
# (4.2 vs 2.6 m/s^2 there), and when the plan unwinds that excess has to drain before the car moves
# at all -- the "saturates in the turn, won't unwind, drifts toward the other lane" grabs. While the
# car is at the ceiling and the plan is genuinely unwinding, the wire is capped at the command that
# holds the car's current curvature (K x measured), so the car starts unwinding with the plan.
# Curve entry and the in-curve request are untouched (the plan isn't unwinding there).
# Deviation-clip speed gate. ford.h enforces the shadow-curvature band only above
# angle_error_min_speed = 10.0 m/s (FORD_LIMITS, same Veh_V_ActlBrk speed as vEgoRaw); clipping from
# 9 m/s (curvature mode's gate) cost angle mode the 9-10 m/s band where tight residential curves
# live -- 2 of the 3 road-keeping grabs on 000001b2 (t=155 at 9.1 m/s, t=523 at 9.2 m/s) had the
# clip binding there with the car short of the plan. 0.2 m/s margin keeps the clip engaged before
# the panda starts checking.
_DEVIATION_CLIP_MIN_SPEED = 9.8  # m/s
# Below that gate there is no clip, so the clip-bound unwind (see _shape_wire_kappa) never ran there and
# a curve exit at the PSCM ceiling waited for K x plan to fall under the clamp: route 00000470 t=2061 at
# 9.5 m/s, the plan eased 32 -> 24.5e-3 while the wire held 36-39e-3 and the car stayed at 28.7e-3 for
# 0.7 s, until v crossed 9.8 and the wire dropped to 26e-3 in one frame. Below the gate the same unwind
# now runs against a virtual clip (measured -+ tolerance, not applied to kappa_cmd or the shadow ford.h
# checks): 0 = off, 1 = only with the car at the ceiling (_WINDUP_AY), 2 = every unwind.
_LOW_SPEED_UNWIND = 1
# Measured lateral accel at which the car counts as at the ceiling: 0.9 x the emulator's hard-clamp fit
# (1.78/2.66/2.20 m/s^2 at 10/20/30 m/s, 2026-10-06). A flat 1.8 never engaged at 10-12 m/s, where the
# car plateaus at 1.6-1.7 (routes 465-467) and where the ceiling grabs happen. Harness, 19 town routes,
# clean emulator: curve-exit cut -9.5%, wide -2.4%, tracking -2.2%, nothing worse; replay neutral.
_WINDUP_AY_V = (10.0, 20.0, 30.0)  # m/s
_WINDUP_AY = (1.6, 2.4, 2.0)       # m/s^2
_WINDUP_PLAN_RATE = 0.002      # 1/m/s, plan curvature easing off at least this fast = unwinding
_WINDUP_PLAN_TAU = 0.15        # s, smoothing for the plan-rate test
# Post-release drift: the real-curve floor (|measured| > 2x tolerance) keeps curve entry from
# straight out of the detector, but entry has the car turning WITH the plan or not yet at all. A
# car measurably curving AGAINST the plan is the deadlock instead: for ~5 s after the driver lets
# go (worst after a manual turn) the PSCM tracks with a ~2e-3 1/m bias, the deviation clip pins
# the command at measured - tolerance (~0 on a straight road), and the car drifts toward the
# shoulder while the plan's correction grows (routes 00000442 t=153, 00000447 t=51, 00000438
# t=420 -- each ended by a driver grab 1-3 s in). Admit that case down to this fraction of the
# tolerance; the reversal case's not-closing test below still applies. Offline replay over
# 456 hands-free min (b09ba355 + 740c61c6 routes): 7 fires, all genuine stalls, 0 elsewhere;
# at 1.0x the tolerance it caught only 1 of the 7.
_STALL_REVERSED_FLOOR_RATIO = 0.5
# Proactive hand-off blip: any sustained driver press attenuates the PSCM (route 000000be seg 4:
# 3 s of sub-45-deg circle-exit steering left it at ~0x delivery, and the reactive detector's
# fire-after-the-stall-develops timing meant 2.4 s of dead-straight running into the next curve
# before the pulse landed). Firing the same pulse on the falling edge of a sustained press resets
# the PSCM while the car is straight and the command is small -- a 300 ms lateral gap right at
# hand-off, imperceptible, instead of a missed curve. The reactive detector above stays as backstop.
_PRESS_BLIP_MIN_S = 0.5      # press must last this long before its release earns a pulse
# The pulse releases steering for 300 ms; never fire it in a curve.
_BLIP_MAX_PATH_ANGLE = 0.10  # rad
# steeringPressed chatters: a 30 ms dip inside a 1.7 s hold fired a pulse (route 00000399 t=172.04).
_PRESS_RELEASE_S = 0.3       # release must persist this long to count as one
# _PRESS_RELEASE_S is a budget for the whole grab, not per dip. Resetting it on every re-press
# meant a lightly-resting hand -- which chatters with dips shorter than the debounce -- could
# never end its grab, so press_timer_s summed unrelated taps into a "sustained hold" that was
# never held, and a later release fired a pulse on that stale total. Accumulating the dips
# instead costs a genuine 30 ms dip 30 ms of budget (route 00000399 still earns its pulse) and
# ends a chattery grab after 0.3 s of total dip.
# An earned pulse that a guard blocks (cooldown, pulse in flight, mid-curve) is held pending and
# retried rather than dropped -- otherwise the driver has to grab and release again. The window
# outlasts a full _STALL_COOLDOWN_S plus pulse; past that the hand-off moment has gone and the
# reactive detector is the right backstop.
_PRESS_BLIP_PENDING_S = 3.0
# The pulse also leaves path_angle to ramp back in through the soft ROC. The fixed
# _BLIP_MAX_PATH_ANGLE cap above doesn't scale with speed: at 25 m/s, 0.10 rad is a R~250 m
# curve and the ramp adds ~0.6 s of unassisted steering (~15 m). Cap the ramp-recovery
# distance so a hand-off pulse never fires where the recovery would understeer the curve.
_BLIP_MAX_RAMP_M = 10.0
# A mode-0 pulse lets the wheel self-centre (12 -> 1 deg in 0.3 s at 14 m/s, 000001b1 t=910), and the
# PSCM then takes ~0.5 s to answer again -- up to ~1 s in LatCtlLim after a large command. That is the
# right move when the driver hands back a turn the plan no longer wants (manual-turn exit: the free
# wheel straightens the car at once, where a clip-bound unwind takes ~2 s -- 000001af t=223, 000001b4
# t=177). It is the wrong move when the plan still wants a curve: the pulse throws it away and the
# car re-acquires it late (000001b1 t=507/910, 000001b2 t=591, 000001b4 t=762 -> driver re-grab).
# Only fire while the plan asks for less than this share of the car's curvature, or is near straight.
_BLIP_PLAN_MEAS_RATIO = 0.3
_BLIP_PLAN_STRAIGHT = 0.001  # 1/m
# "Near straight" must also hold in lateral accel: 0.001 1/m is 1.2 m/s^2 at 77 mph. Route 00000463
# t=3085 (34.5 m/s, plan 0.9 m/s^2 into an S-bend) passed the curvature test, the pulse freed the wheel,
# the PSCM came back in LatCtlLim and the car overshot ~0.7 m toward the next lane.
_BLIP_PLAN_STRAIGHT_AY = 0.3  # m/s^2
# No hand-off pulse at highway speed. Its value is manual-turn exits, which don't happen there, and
# over every logged release above 22 m/s a pulse brought no tracking gain but more re-grabs within
# 3 s (65-82% vs 47-57% without). An earned pulse just waits out its pending window.
_BLIP_MAX_SPEED = 22.0  # m/s
# The lane trim may not take curvature away from a turn. The model cuts curves on the inside by plan
# (0.15-0.3 m, worse with SCM), so in a curve the trim pulls against the plan; openpilot judges the car
# against the plan, and the shortfall reads as saturation. Route 0000047c (2026-10-10, strength 0.5): at
# 43-47 mph the wire was 0.23-0.65x a 1.0-1.5 m/s^2 plan and "turn exceeds limit" fired on simple curves.
# A trim that opposes the plan fades out between these plan lateral accels (none left at the second);
# a trim that adds to the turn, and every trim on a straight, is untouched. Harness, saturation-like
# events (plan > 1 m/s^2, car < plan/1.2 for 1 s): 47a-47e 19 -> 1 at strength 0.5 (9 -> 1 at 0.25, the
# same as the trim off); highway routes 458/463/464 59 -> 6 with straight-road yaw unchanged.
_TRIM_OPPOSE_AY = (0.5, 1.0)  # m/s^2
# Lead-lag command shaping (see module docstring): the wire carries r*K*kappa immediately and the
# remaining (1-r)*K*kappa through a first-order lag of tau. First fitted open-loop on routes
# 41x-45x + 1a7-1af (r=0.70/tau=0.7 s to 56 mph, r=0.85/tau=1.0 s above 60 mph). Retuned 2026-09-28
# below 25 m/s in closed loop against the fitted PSCM emulator (bluepilot/tools/pscm: the real
# controller + ford.h driving the emulator, recorded disturbances replayed, 42 tuning routes, then
# checked on 32 held-out routes): the emulator's droop (share c, tau_d) inverts to r = 1 - c and a
# lag of tau_d / (1 - c), with the lag shortened 0.6x in closed loop. That cut lateral path error
# ~6% and weave ~3% and ran wide less, at ~14% more curve-entry cut (still ~40% less than no
# shaping). Longer lags ran wider and weaved more; any change above 25 m/s added highway weave, so
# the highway values are unchanged.
_LEADLAG_V_BP = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0)          # m/s
_LEADLAG_FAST_RATIO = (0.50, 0.43, 0.42, 0.49, 0.70, 0.85, 0.85)   # r: immediate share of the steady-state gain
_LEADLAG_TAU_S = (0.40, 0.31, 0.30, 0.33, 0.70, 1.0, 1.0)          # s: lag on the remaining share

# path_angle soft-ROC breakpoints (rad per 20 Hz call) -- shared by the limiter below and the
# hand-off blip's ramp-recovery distance guard above.
_SOFT_ROC_V_NODES = [9., 10., 15., 25.]
_SOFT_ROC_RAD_PER_CALL = [0.055, 0.055, 0.0425, 0.009]


def _soft_roc_rad_per_s(v_ego_ms: float) -> float:
  return float(interp(v_ego_ms, _SOFT_ROC_V_NODES, _SOFT_ROC_RAD_PER_CALL)) / _STEER_DT


class LateralAngleExt:
  def __init__(self, CP=None, CP_SP=None):
    # Predicted-curvature blend for path_angle: pred * b + desired * (1-b); b from ``FordPathAngleBlendRatio``
    self.path_angle_blend_ratio = _FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT
    # Max extra VLT above t_base; from ``FordVLTExtraMax`` param
    self.vlt_extra_max = _VLT_T_EXTRA_MAX
    # Telemetry: final path_angle (rad) after limits (see bp_card_publisher)
    self.bp_path_angle_final = 0.0
    # High-speed gain factors: set per-platform via carFingerprint in update_angle_params.
    self.path_angle_gain_lowC_highV = 1.0   # dampening at high speed, low curvature
    self.path_angle_gain_highC_highV = 1.0  # gain at high speed, high curvature
    self.bp_path_angle_gain_lowC_highV = 1.0
    self.bp_path_angle_gain_highC_highV = 1.0
    # User-tunable "feel" multipliers: read from the angle-tuning Params below.
    self.low_speed_curv_factor = 1.0
    self.high_speed_curv_factor = 1.0
    self.user_dampening_factor = 1.0
    self.bp_low_speed_curv_factor = 1.0
    self.bp_high_speed_curv_factor = 1.0
    # BluePilot: angle mode's own lane-change scaling factor, independent of curvature mode's
    # lane_change_factor_high_curv -- angle needs a boost (>1) where curvature needs a cut (<1).
    self.lane_change_factor_high_ang = 1.0
    # BluePilot: angle-mode lane centering trim (advanced lane positioning) -- see
    # lane_center_trim.py and the module docstring above.
    self.lane_center_trim = LaneCenterTrim()
    self.lane_trim_applied = 0.0  # telemetry: trim actually added to kappa_cmd, after _TRIM_OPPOSE_AY
    self.enable_lane_positioning_ang = False
    self.custom_path_offset_ang = 0.0
    self.lane_centering_strength_ang = 0.25
    # BluePilot: lead-lag command shaping (see module docstring). _wire_slow is the lagged share of
    # K*kappa, in curvature units; None = unseeded (seeded on the first active frame so
    # re-engagement ramps exactly as before, through the soft ROC).
    # BluePilot: small-curvature gain (see _SMALL_CURV_GAIN_CANFD_SUV); platform value set in
    # update_angle_params.
    self.small_curv_gain = 1.0
    self._wire_slow = None
    self.bp_unwind_clamped = False  # clip-bound unwind clamp actually bit this frame
    self.bp_windup_released = False  # anti-windup cap at the PSCM ceiling bit this frame
    self._windup_plan_slow = None    # smoothed planner curvature for the unwinding test
    # Telemetry: variable curvature lookup time used this frame (s)
    self.bp_curvature_lookup_time = _VLT_T_EXTRA_MAX + 0.3725  # warm start at ~0.5s
    # BluePilot: error-clipped kappa path_angle was derived from -- carcontroller.py reads this as
    # shadow_curvature for ford.h's angle-mode deviation check. Actively consumed, not telemetry.
    self.bp_kappa_cmd = 0.0
    # BluePilot: rate-limit diagnostics (controllerStateBP)
    self.bp_angle_rate_limited = False      # path_angle soft-ROC clip actually bit this frame
    self.bp_curvature_rate_limited = False  # equivalent curvature would be rate-limited by curv-mode logic (sim)
    self.bp_curvature_deviation_limited = False  # current_curvature error-clip constrained kappa_cmd this frame
    self.sim_curvature_last = 0.0           # shadow curvature-mode last for the curvatureRateLimited sim
    # Exit detection: track previous desired curvature to sense when planner is actively reducing
    self._desired_curvature_last = 0.0
    # Human-turn override: while the driver manually turns, lateral is forced inactive (mode 0,
    # all-zero signals) instead of winding path_angle into a stale command the PSCM can't cleanly
    # reconcile on release (2-3 s re-engage dead time observed on Mach-E). See module docstring.
    # Note: in CarController this attribute is shared with LateralCurvExt (same mixin instance) --
    # only one lateral strategy runs per frame, so a single detector serves both.
    self.human_turn_detector = HumanTurnDetector()
    self.angle_human_turn_active = False  # read by carcontroller to force mode 0
    # Post-override stall blip state (see module constants). angle_stall_blip_active is read by
    # carcontroller to force mode 0, exactly like angle_human_turn_active.
    self.stall_blip_hold_s = 0.0      # accumulated deviation-clip-binding time toward a pulse
    self._stall_gap_mag_slow = -1.0   # smoothed |desired - current|; < 0 = unseeded
    self._stall_meas_slow = None      # smoothed measured curvature (responding test); None = unseeded
    self.stall_blip_frames_left = 0   # remaining pulse frames; > 0 -> mode 0 on the wire
    self.stall_blip_cooldown_s = 0.0  # re-arm delay after a pulse
    self.stall_blip_count = 0         # pulses fired this stall episode
    self.angle_stall_blip_active = False
    self.angle_stall_blip_source = 0  # 0=none, 1=hand-off pulse, 2=reactive stall pulse
    self.press_timer_s = 0.0          # pressed time in the current grab, for the hand-off blip
    self.release_timer_s = 0.0        # total !steeringPressed time in the grab (see _PRESS_RELEASE_S)
    self.press_blip_pending_s = 0.0   # earned hand-off pulse waiting for a clear frame to fire

  def update_angle_params(self, params):
    """Sets per-platform gain defaults and reads user angle-tuning params."""
    self._ensure_lateral_curv_initialized(self.CP)
    fp = getattr(self.CP, 'carFingerprint', '')
    if fp in _CANFD_BOF_CARS:
      low, high = _GAIN_CANFD_BOF
    elif fp in _CANFD_SUV_CARS:
      low, high = _GAIN_CANFD_SUV
    else:
      low, high = _GAIN_CAN
    self.path_angle_gain_lowC_highV = low
    self.path_angle_gain_highC_highV = high
    self.small_curv_gain = _SMALL_CURV_GAIN_CANFD_SUV if fp in _CANFD_SUV_CARS else 1.0
    if params is not None and hasattr(params, "get"):
      for attr, key, min_value, max_value in (
        ("low_speed_curv_factor", "FordLowSpeedFactor_ang", 0.5, 1.5),
        ("high_speed_curv_factor", "FordHighSpeedFactor_ang", 0.5, 1.5),
        ("user_dampening_factor", "FordHighSpeedDampening_ang", 0.25, 1.25),
      ):
        try:
          raw = params.get(key, return_default=True)
          if raw is not None and raw != b"":
            setattr(self, attr, float(clip(
              float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), min_value, max_value)))
        except Exception:
          pass
      try:
        raw = params.get("lane_change_factor_high_ang", return_default=True)
        if raw is not None and raw != b"":
          self.lane_change_factor_high_ang = float(clip(
            float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), 0.85, 1.50))
      except Exception:
        pass
      # BluePilot: angle-mode lane centering trim (advanced lane positioning) params.
      try:
        self.enable_lane_positioning_ang = bool(params.get_bool("enable_lane_positioning_ang"))
      except Exception:
        pass
      for attr, key, min_value, max_value in (
        ("custom_path_offset_ang", "custom_path_offset_ang", -0.5, 0.5),
        ("lane_centering_strength_ang", "lane_centering_strength_ang", 0.0, 1.0),
      ):
        try:
          raw = params.get(key, return_default=True)
          if raw is not None and raw != b"":
            setattr(self, attr, float(clip(
              float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), min_value, max_value)))
        except Exception:
          pass

  def _shape_wire_kappa(self, target: float, kappa_cmd: float, kappa_pre_clip: float,
                        current_curvature: float, v_ego: float) -> float:
    """Lead-lag shaping of the steady-state wire command ``target`` (= K*kappa_cmd, curvature
    units) plus the clip-bound unwind clamp -- see the module docstring. Returns path_angle / v."""
    r = float(interp(v_ego, _LEADLAG_V_BP, _LEADLAG_FAST_RATIO))
    tau = float(interp(v_ego, _LEADLAG_V_BP, _LEADLAG_TAU_S))
    if self._wire_slow is None:
      # First active frame after mode 0: seed at the target so (re-)engagement behaves exactly as
      # the static gain did -- the soft ROC ramp from zero is the transient there.
      self._wire_slow = target
    else:
      self._wire_slow += (_STEER_DT / (tau + _STEER_DT)) * (target - self._wire_slow)

    # Clip-bound unwind: the planner wants less turn than the car has, and the deviation clip is
    # what's holding kappa_cmd at measured - tolerance. Send that clipped intent itself, at unit
    # gain, not K x it and not "no more than measured": the PSCM holds only ~0.78 of a held
    # command, so a wire at the measured curvature just holds the turn, and the command -- tied to
    # the car by the clip -- waits for a car that waits for it. That was the saturate-then-won't-
    # unwind drift toward the other lane on routes 000001b1 t=280 (41 mph, wire flat at -7.5e-3 for
    # 1.5 s while the plan crossed zero) and t=375; the PSCM emulator on the logged wire tracks the
    # real car through those exits, so the lock is ours, not the PSCM's. The wire now equals the
    # shadow curvature ford.h checks (kappa_cmd), which leads the car by the full tolerance every
    # frame. The lagged share restarts from here so leaving the clip is continuous.
    s = 1.0 if current_curvature >= 0.0 else -1.0
    _virtual_clip = (_LOW_SPEED_UNWIND > 0 and v_ego <= _DEVIATION_CLIP_MIN_SPEED
                     and (kappa_pre_clip - current_curvature) * s < -self.bp_curvature_error
                     and (_LOW_SPEED_UNWIND == 2
                          or v_ego ** 2 * abs(current_curvature) >= float(interp(v_ego, _WINDUP_AY_V, _WINDUP_AY))))
    if _virtual_clip:
      kappa_cmd = current_curvature - s * self.bp_curvature_error
    if ((self.bp_curvature_deviation_limited or _virtual_clip) and abs(current_curvature) > self.bp_curvature_error
        and (kappa_pre_clip - current_curvature) * s < 0.0):
      shaped = r * target + (1.0 - r) * min(self._wire_slow * s, target * s) * s
      if shaped * s > kappa_cmd * s:
        shaped = kappa_cmd
        self.bp_unwind_clamped = True
      self._wire_slow = shaped
      return shaped
    return r * target + (1.0 - r) * self._wire_slow

  def update_angle_strategy(self, CC, CS, actuators, CP):
    """
    Curvature from planner (+ optional predicted blend, + lane centering trim) → path_angle via
    κ·v·K, lead-lag shaped (see module docstring). The lane centering trim stays in the curvature
    domain (kappa_cmd). c0, c2 and c3 are zero.
    Blended κ is not passed through Ford c2 rate / DBC limits (those target the curvature actuator).
    """
    self._ensure_lateral_curv_initialized(CP)

    v_ego = float(CS.out.vEgoRaw)

    curvature_rate = 0.0
    path_offset = 0.0
    path_angle = 0.0
    ramp_type = 0
    lateral_uncertainty = 0.0
    precision = 1

    if not CC.latActive:
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Publish the shadow curvature from the measured curvature while inactive. LKA keeps
      # carrying angle_mode_engaged whenever angle mode is configured (independent of
      # latActive), and ford.h latches the shadow from every LKA frame -- so the latched
      # value must track reality here, not sit at a stale zero. Otherwise the first enabled
      # LMC frame after (re-)engage races LKA's 33Hz latch against LMC's 20Hz enable bit and
      # ford.h's deviation check compares a zero shadow against real measured curvature.
      # (ford.h skips the check while steer_control_enabled is 0, so the value is free to
      # follow the measurement during the inactive period itself.)
      self.bp_kappa_cmd = self.get_current_curvature(CS)
      self.human_turn_detector.reset()
      self.angle_human_turn_active = False
      self.lane_center_trim.reset()
      self._wire_slow = None
      self.stall_blip_hold_s = 0.0
      self._stall_gap_mag_slow = -1.0
      self._stall_meas_slow = None
      self._windup_plan_slow = None
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.angle_stall_blip_source = 0
      self.press_timer_s = 0.0
      self.release_timer_s = 0.0
      self.press_blip_pending_s = 0.0
      self.precision_type = 1
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

    # Human-turn override: sustained driver press + large wheel angle → force lateral inactive
    # (carcontroller drops mode to 0; all signals are zero on the wire) so path_angle can't wind
    # into a stale command while the driver turns. Always on in angle mode (no param gate) -- the
    # curv-suffixed human-turn toggle belongs to curvature mode's reset strategy, and the Mach-E
    # PSCM re-engage stall this prevents is not something a user should be able to opt out of.
    # On release, no jump seed: path_angle_last is 0, so the normal flow below ramps the command
    # back in through the soft ROC -- generous at human-turn speeds, no panda bypass involved.
    self.angle_human_turn_active = self.human_turn_detector.update(
      True, CS.out.steeringPressed, CS.out.steeringAngleDeg)
    if self.angle_human_turn_active:
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Truthful shadow during the override (mirrors the inactive path -- see the comment
      # there): the driver is steering, so the honest command is the car's actual curvature,
      # and the panda-latched shadow stays current for the re-engage frame.
      self.bp_kappa_cmd = self.get_current_curvature(CS)
      # Keep exit detection current so resume doesn't compare against a stale pre-turn value.
      self._desired_curvature_last = float(actuators.curvature)
      self.lane_center_trim.reset()
      self._wire_slow = None
      # A human turn ends any stall episode -- its own mode 0 does the PSCM reset job. That also
      # covers the press so far: only press time accumulated AFTER the latch releases should earn
      # a hand-off pulse.
      self.stall_blip_hold_s = 0.0
      self._stall_gap_mag_slow = -1.0
      self._stall_meas_slow = None
      self._windup_plan_slow = None
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.angle_stall_blip_source = 0
      self.press_timer_s = 0.0
      self.release_timer_s = 0.0
      self.press_blip_pending_s = 0.0
      self.precision_type = 1
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

    # Proactive hand-off blip: the falling edge of a sustained press earns an immediate mode-0
    # pulse (see _PRESS_BLIP_MIN_S) -- resets the PSCM's press-induced attenuation right at
    # hand-off, while the car is straight and the command small, instead of waiting for the
    # reactive stall detector below to watch the car miss the next curve first.
    if CS.out.steeringPressed:
      if self.press_timer_s <= 0.0:
        self.release_timer_s = 0.0  # first press of a new grab
      self.press_timer_s += _STEER_DT
      # A fresh grab supersedes any pulse the previous one earned: that grab's release will
      # earn its own, and firing mid-press would drop lateral while the driver is steering.
      self.press_blip_pending_s = 0.0
    else:
      # Not reset on re-press: dips accumulate across the grab -- see _PRESS_RELEASE_S.
      self.release_timer_s += _STEER_DT
      # press_timer_s survives a sub-debounce dip, so a re-press resumes the same grab.
      if self.press_timer_s > 0.0 and self.release_timer_s >= _PRESS_RELEASE_S:
        if self.press_timer_s >= _PRESS_BLIP_MIN_S:
          self.press_blip_pending_s = _PRESS_BLIP_PENDING_S
        self.press_timer_s = 0.0

    # An earned pulse waits for a frame where every guard is clear instead of being dropped.
    if self.press_blip_pending_s > 0.0:
      self.press_blip_pending_s = max(0.0, self.press_blip_pending_s - _STEER_DT)
      _plan = abs(actuators.curvature)
      _plan_straight = _plan <= min(_BLIP_PLAN_STRAIGHT, _BLIP_PLAN_STRAIGHT_AY / max(v_ego, 1.0) ** 2)
      if (self.stall_blip_cooldown_s <= 0.0
          and self.stall_blip_frames_left <= 0
          and v_ego <= _BLIP_MAX_SPEED
          and abs(self.path_angle_last) < _BLIP_MAX_PATH_ANGLE
          # ramp-recovery distance guard: straight (path_angle ~0) always passes; curves
          # scale with speed through the soft ROC
          and (abs(self.path_angle_last) / _soft_roc_rad_per_s(v_ego)) * v_ego < _BLIP_MAX_RAMP_M
          and (_plan_straight or _plan <= _BLIP_PLAN_MEAS_RATIO * abs(self.get_current_curvature(CS)))):
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
        self.angle_stall_blip_source = 1
        self.press_blip_pending_s = 0.0

    # Stall-blip pulse in progress: hold lateral inactive (mode 0, all-zero signals -- the same
    # wire pattern as the human-turn override, no ford.h involvement) for _STALL_BLIP_FRAMES so the
    # PSCM drops its post-override attenuation, then release; path_angle ramps back in from zero
    # through the soft ROC exactly like a human-turn release. Detection lives at the end of the
    # normal flow below.
    if self.stall_blip_frames_left > 0:
      self.stall_blip_frames_left -= 1
      self.angle_stall_blip_active = True
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Truthful shadow during the blip (see the inactive-path comment).
      self.bp_kappa_cmd = self.get_current_curvature(CS)
      self._desired_curvature_last = float(actuators.curvature)
      self.lane_center_trim.reset()
      self._wire_slow = None
      self.precision_type = 1
      if self.stall_blip_frames_left <= 0:
        self.stall_blip_cooldown_s = _STALL_COOLDOWN_S
        self.angle_stall_blip_source = 0
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )
    self.angle_stall_blip_active = False

    self.precision_type = 1
    precision = 1
    desired_curvature = float(actuators.curvature)

    # Variable lookup time: t_base tracks planner pre-compensation; extra tapers on high speed and large curves.
    # Cap liveDelay at 0.15s for VLT purposes. liveDelay can calibrate up to ~420ms on some runs, which inflates
    # VLT to 0.6s and pushes the model lookahead 5m into the curve. At that depth the model sees full peak
    # curvature, kappa_entering stays True, and the exit-biased blend is permanently disabled — causing the car
    # to command max path_angle through the entire apex. 0.15s gives t_base ≤ 0.20s and VLT ≤ 0.33s, restoring
    # the 2.8m lookahead that kept kappa_entering False at the apex in successful earlier runs.
    _t_base = float(clip(self.sm['liveDelay'].lateralDelay, 0.1, 0.15)) + _DT_MDL
    _speed_factor = float(interp(v_ego, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
    # Direction-aware kappa factor: on curve ENTRY (model shows more curvature at t_base than planner now),
    # keep full lookahead so pre-steering begins early. On exit/apex, taper by magnitude to prevent unwind.
    _kappa_at_t_base = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      _curvatures_ref = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      _kappa_at_t_base = abs(float(interp(_t_base, ModelConstants.T_IDXS, _curvatures_ref)))
    _kappa_entering = _kappa_at_t_base > abs(desired_curvature)
    if _kappa_entering:
      _kappa_factor = 1.0  # curve deepening ahead: full extra lookahead for gradual entry
    else:
      _kappa_factor = float(interp(abs(desired_curvature), [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER], [1.0, 0.0]))
    curvature_lookup_time = _t_base + self.vlt_extra_max * _speed_factor * _kappa_factor
    self.bp_curvature_lookup_time = curvature_lookup_time

    predicted_curvature = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      curvatures = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      predicted_curvature = float(
        interp(curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
      )

    b = float(self.path_angle_blend_ratio)
    b = float(clip(b, 0.0, 1.0))

    # Exit-biased blend: near the PSCM authority limit or while the planner is actively
    # reducing curvature (exit detected), drop model prediction weight from 60% → ~15%.
    # This lets the planner's natural unwind dominate instead of being diluted by a model
    # prediction that still sees the curve (→ seg-14 slow unwind) or that snaps when its
    # lookahead window crosses the curve exit (→ seg-17 snap + reverse PSCM hit).
    # Normal gentle curves are unaffected: no PSCM limit, no falling desired → full b=0.60.
    # BluePilot: LatCtlLim_D_Stat is not wired into CarState (it does fire in angle mode, rarely, but
    # blocking path_angle growth on it would hold the command in post-release stalls), so only the
    # DBC-limit proximity below gates the saturation handling.
    # Previously used angleState.saturated (CtrSat) as a proxy, but CtrSat fires whenever the car
    # lags the commanded path_angle by > 2.5° — which happens during any normal curve entry.
    # That caused a positive-feedback flat-line: under-steer → CtrSat → path_angle frozen → more under-steer.
    # Use DBC-limit proximity instead: only block when path_angle is already near the ±0.5 rad CAN limits,
    # which is the only condition where the anti-snap unwind rate cap makes physical sense.
    _dbc_sat = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * 0.90 or
                self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * 0.90)
    _in_hard_sat = _dbc_sat
    # BluePilot: per-call delta threshold. The original 0.002 was authored 2026-05-07 on
    # bp-sid-simple (9c3d000fd), which ran STEER_STEP=1 (true 100Hz, switched 2026-04-22) -- so it
    # was tuned to trigger on planner unwind faster than 0.2 (1/m)/s. Scaled x5 here to restore
    # that same real-world trigger rate on this branch's actual 20Hz cadence; unscaled it fired at
    # 0.04 (1/m)/s, collapsing the model blend on mild straightening instead of genuine exits.
    # Same bug class and fix as _PSCM_SAT_UNWIND_RATE and _soft_roc above.
    _desired_falling = abs(desired_curvature) < abs(self._desired_curvature_last) - 0.010
    _on_exit_near_limit = not _kappa_entering and (_in_hard_sat or _desired_falling)
    b_blend = float(clip(b * 0.25, 0.0, 1.0)) if _on_exit_near_limit else b
    requested_curvature = predicted_curvature * b_blend + desired_curvature * (1.0 - b_blend)
    self._desired_curvature_last = desired_curvature

    if self.model is not None:
      self.lane_change = self.model.meta.laneChangeState in (1, 2, 3)
    else:
      self.lane_change = False

    lane_change_factor = interp(
      v_ego, self.lane_change_factor_bp, [self.lane_change_factor_low, self.lane_change_factor_high_ang]
    )
    if self.lane_change and self.model is not None:
      if self.model.meta.laneChangeDirection == 1 and requested_curvature < 0:
        requested_curvature *= lane_change_factor
        precision = 0
      elif self.model.meta.laneChangeDirection == 2 and requested_curvature > 0:
        requested_curvature *= lane_change_factor
        precision = 0
    self.precision_type = precision

    # Use planner / predicted κ directly for the κ → path_angle map; we are not sending κ on CAN.
    kappa_cmd = float(requested_curvature)

    # BluePilot: lane centering trim (advanced lane positioning) -- nudges kappa_cmd toward true
    # lane-line center + user offset, gated on lane-line confidence and disabled during lane
    # changes (see lane_center_trim.py). Applied here, before the deviation clip below, so the
    # trimmed value inherits every limiter this file already applies to kappa_cmd instead of
    # bypassing them.
    current_curvature = self.get_current_curvature(CS)
    _kappa_planner = kappa_cmd
    kappa_cmd = self.lane_center_trim.update(
      kappa_cmd, self.model, v_ego, self.enable_lane_positioning_ang,
      self.custom_path_offset_ang, self.lane_centering_strength_ang,
      CC.latActive, self.lane_change)
    _trim = kappa_cmd - _kappa_planner
    if _trim * _kappa_planner < 0.0:
      _trim *= float(interp(v_ego ** 2 * abs(_kappa_planner), _TRIM_OPPOSE_AY, [1.0, 0.0]))
      kappa_cmd = _kappa_planner + _trim
    self.lane_trim_applied = _trim

    # BluePilot: the planner has first claim on the deviation budget clipped below; the trim takes
    # what is left. Symmetric -- a one-sided form lets the trim subtract authority while the planner
    # is already clipped short in a curve.
    if v_ego > _DEVIATION_CLIP_MIN_SPEED:
      _room = max(self.bp_curvature_error - abs(_kappa_planner - current_curvature), 0.0)
      kappa_cmd = _kappa_planner + float(clip(kappa_cmd - _kappa_planner, -_room, _room))

    # BluePilot: clip kappa_cmd to current_curvature (measured) +- bp_curvature_error,
    # mirroring lateral_curv_ext.py's apply_ford_curvature_limits_ext (same formula, same tolerance;
    # gated at _DEVIATION_CLIP_MIN_SPEED, just under ford.h's 10 m/s). Without this, kappa_cmd
    # (and therefore path_angle, and the shadow_curvature sent to ford.h) can legitimately lead the
    # measured curvature by more than ford.h's angle-error tolerance during normal curve entry/exit
    # -- the shadow-curvature deviation check (ford_shadow_curvature_error_check) would then block
    # routinely, not just on genuine pothole/override divergence. Curvature mode has always clipped
    # here; this brings angle mode's actual steering intent in line with that proven behavior rather
    # than only clipping the value reported to panda (which would make the check a no-op).
    self.bp_curvature_deviation_limited = False
    _kappa_cmd_pre_error_clip = kappa_cmd
    if v_ego > _DEVIATION_CLIP_MIN_SPEED:
      kappa_cmd = float(clip(kappa_cmd, current_curvature - self.bp_curvature_error,
                            current_curvature + self.bp_curvature_error))
      # BluePilot: did this clip actually constrain kappa_cmd this frame (deviation from measured,
      # not rate-of-change -- see carcontroller.py)?
      self.bp_curvature_deviation_limited = bool(abs(kappa_cmd - _kappa_cmd_pre_error_clip) > 1e-9)

    lateral_uncertainty = 0.0  # no curvature-limit ladder until angle-mode torque display is defined

    # Speed-interpolated gain: at low speed both curves use 1.0; at high speed the params take effect.
    self.low_gain_calc = interp(
      v_ego, [13.5, 26.82], [1.0, (self.path_angle_gain_lowC_highV * self.user_dampening_factor)]
    )
    self.high_gain_calc = interp(v_ego, [13.5, 26.82], [(1.30 * self.low_speed_curv_factor), (self.path_angle_gain_highC_highV * self.high_speed_curv_factor)])
    _small_gain = float(interp(v_ego, _SMALL_CURV_GAIN_FADE_V, [self.small_curv_gain, 1.0]))
    if _small_gain != 1.0:
      self.low_gain_calc = min(self.low_gain_calc * _small_gain, max(self.high_gain_calc, self.low_gain_calc))

    # As the curve gets bigger, we will need a little boost to the signal to to not understeer
    self.curvature_factor = interp(abs(kappa_cmd), [0.0007, 0.001], [self.low_gain_calc, self.high_gain_calc])

    # Steady-state wire command in curvature units (path_angle / v).
    wire_kappa = kappa_cmd * self.curvature_factor
    self.bp_unwind_clamped = False
    wire_kappa = self._shape_wire_kappa(wire_kappa, kappa_cmd, _kappa_cmd_pre_error_clip, current_curvature, v_ego)

    # Anti-windup at the PSCM ceiling (see _WINDUP_AY): the car is at the ceiling, the plan is
    # unwinding, and we are asking for more than holds the car where it is -> drop the excess now.
    _plan_prev = self._windup_plan_slow if self._windup_plan_slow is not None else desired_curvature
    self._windup_plan_slow = _plan_prev + (_STEER_DT / (_STEER_DT + _WINDUP_PLAN_TAU)) * (desired_curvature - _plan_prev)
    _turn = 1.0 if current_curvature >= 0.0 else -1.0
    _plan_easing = (self._windup_plan_slow - _plan_prev) / _STEER_DT * _turn < -_WINDUP_PLAN_RATE
    _hold = self.curvature_factor * current_curvature
    self.bp_windup_released = False
    _windup_ay = float(interp(v_ego, _WINDUP_AY_V, _WINDUP_AY))
    if (_plan_easing and v_ego ** 2 * abs(current_curvature) >= _windup_ay
        and wire_kappa * _turn > _hold * _turn):
      wire_kappa = _hold
      self._wire_slow = _hold
      self.bp_windup_released = True
    path_angle = wire_kappa * v_ego

    # PSCM authority limit clamp (_in_hard_sat: path_angle near the ±0.5 rad DBC limit).
    # Hard saturation (_in_hard_sat): block increases AND rate-limit decreases to _PSCM_SAT_UNWIND_RATE.
    #   Without the decrease cap, model+planner drop path_angle at ~0.36 rad/s at a sharp apex,
    #   driving desired steering 30°+ ahead of actual while the PSCM is pinned, causing a snap when released.
    if _in_hard_sat:
      _last = self.path_angle_last
      _last_mag = abs(_last)
      _curr_mag = abs(path_angle)
      if _curr_mag > _last_mag:  # magnitude growing — block
        path_angle = _last
      elif _last_mag - _curr_mag > _PSCM_SAT_UNWIND_RATE:  # decreasing too fast — rate-limit
        _limited_mag = _last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = float(_limited_mag if _last >= 0 else -_limited_mag)

    path_angle = min(FORD_DBC_PATH_ANGLE_MAX, max(FORD_DBC_PATH_ANGLE_MIN, path_angle))

    # Soft ROC limit — unconditional, slightly tighter than ford.h, applied before the
    # hardware bypass in ford.h is re-enabled.  Lets us observe whether the limit would
    # suppress control and tune it, while the PSCM still receives the clipped value.
    # BluePilot: this strategy runs once per STEER_STEP (CarControllerParams.STEER_STEP=5), i.e.
    # once every 5th 100Hz control tick = 20Hz, not every tick -- ported "verbatim from bp-sid-simple"
    # (2026-06-13), which runs STEER_STEP=1 (true 100Hz, switched 2026-04-22). The y-values below are
    # scaled x5 from the original [0.011, 0.011, 0.0085, 0.0018] to restore the same real-world rate
    # (63/63/49/10 deg/s at v=9-10/15/25) on this branch's actual 20Hz cadence. See ford.h's
    # FORD_PATH_ANGLE_LIMITS, which must mirror this scaling (x1.02 looser) to stay a true backstop.
    _soft_roc = float(interp(v_ego, _SOFT_ROC_V_NODES, _SOFT_ROC_RAD_PER_CALL))
    _path_angle_pre_roc = path_angle
    path_angle = float(clip(path_angle,
                            self.path_angle_last - _soft_roc,
                            self.path_angle_last + _soft_roc))
    # BluePilot: did the soft ROC clip actually limit the path_angle we wanted to send this frame?
    self.bp_angle_rate_limited = bool(abs(path_angle - _path_angle_pre_roc) > 1e-9)

    # c0 stays zero: a lane-offset c0 (PSCM Walkthrough) was road-tested on routes 1a7-1af and
    # had no measurable effect (|c0| p90 0.09 m, none at post-turn releases).
    path_offset = 0.0

    # Telemetry / state
    self.bp_path_angle_gain_lowC_highV = self.path_angle_gain_lowC_highV
    self.bp_path_angle_gain_highC_highV = self.path_angle_gain_highC_highV
    self.bp_low_speed_curv_factor = self.low_speed_curv_factor
    self.bp_high_speed_curv_factor = self.high_speed_curv_factor
    self.path_angle_last = path_angle
    self.bp_path_angle_final = path_angle
    self.apply_curvature_last = 0.0
    # BluePilot: the error-clipped kappa path_angle was derived from -- carcontroller.py reads this
    # as shadow_curvature for ford.h's angle-mode deviation check (see fordcan_ext.create_lka_msg).
    # Not just telemetry: an actively-consumed value, unlike the removed *_kappa_cmd_raw stubs.
    # While the driver is pressing (before the human-turn override latches), the clipped planner
    # kappa can't follow the wheel: the driver moves the measured curvature faster than the
    # deviation clip tracks it, so the shadow can exit ford.h's error band mid-curve -- the one
    # in-drive lateral safety block observed across ~3h of replayed road-test routes was exactly
    # this (driver fighting a sustained curve with the mode still enabled). The honest command
    # during a press is the driver's actual curvature.
    self.bp_kappa_cmd = self.get_current_curvature(CS) if CS.out.steeringPressed else kappa_cmd

    # BluePilot: would the equivalent curvature (kappa_cmd) have been rate-limited by curvature-mode's
    # ROC (apply_std_steer_angle_limits)? kappa_cmd is already error-clipped above (same clip
    # curvature mode applies), so only the rate-of-change portion remains to simulate here.
    _equiv_curv_rl = apply_std_steer_angle_limits(kappa_cmd, self.sim_curvature_last, v_ego,
                                                  CS.out.steeringAngleDeg, CC.latActive, BP_ANGLE_LIMITS)
    self.bp_curvature_rate_limited = bool(abs(_equiv_curv_rl - kappa_cmd) > 1e-9)
    self.sim_curvature_last = float(_equiv_curv_rl)

    # Post-override stall detection (mechanism in the module constants' comment). Fires the mode-0
    # blip when, hands-free, desired curvature has led measured by more than 2x the deviation
    # clip's tolerance while the clip was actually binding for _STALL_HOLD_S accumulated seconds.
    # devLim flickers mid-stall (~63% duty on the diagnosis route), so off frames hold the
    # accumulator rather than resetting it; a closed gap or driver press ends the episode.
    self.stall_blip_cooldown_s = max(0.0, self.stall_blip_cooldown_s - _STEER_DT)
    _stall_gap = desired_curvature - current_curvature
    _stall_gap_min = _STALL_GAP_RATIO * self.bp_curvature_error
    # Seed on the first frame: a reference rising from zero would call every gap "not closing"
    # for the first _STALL_GAP_TAU and hand the detector a free window.
    if self._stall_gap_mag_slow < 0.0:
      self._stall_gap_mag_slow = abs(_stall_gap)
    else:
      _gap_alpha = _STEER_DT / (_STEER_DT + _STALL_GAP_TAU)
      self._stall_gap_mag_slow += _gap_alpha * (abs(_stall_gap) - self._stall_gap_mag_slow)
    # How fast the car's own curvature is moving toward the plan (smoothed: yawRate is noisy).
    _meas_prev = self._stall_meas_slow if self._stall_meas_slow is not None else current_curvature
    self._stall_meas_slow = _meas_prev + (_STEER_DT / (_STEER_DT + _STALL_GAP_TAU)) * (current_curvature - _meas_prev)
    _toward_plan_rate = (self._stall_meas_slow - _meas_prev) / _STEER_DT * (1.0 if _stall_gap >= 0.0 else -1.0)
    _stalled = (not CS.out.steeringPressed and not self.lane_change and v_ego > 9.0
                and abs(_stall_gap) > _stall_gap_min
                # curve entry from straight satisfies the gap test by construction; require a real
                # curve. Coverage boundary: deliberate trade -- below this floor (R ~ 1/gap_min,
                # ~250 m) the stall signature is smaller than the entry transient, so a
                # post-override stall on gentler curves is only covered by the proactive hand-off
                # blip above, never detected here. Do not "fix" this back into firing at entry.
                # The one exception is a car curving AGAINST the plan, which entry never does --
                # see _STALL_REVERSED_FLOOR_RATIO.
                and (abs(current_curvature) > _stall_gap_min
                     or (desired_curvature * current_curvature < 0.0
                         and abs(current_curvature) > _STALL_REVERSED_FLOOR_RATIO * self.bp_curvature_error))
                # "desired leads measured", stated by sign rather than magnitude. The magnitude
                # form missed the reversal case outright -- mid S-curve with the PSCM stuck on a
                # stale positive curvature while the planner already wants negative, |desired|
                # can sit below |current| with the car pointed the wrong way entirely. What is
                # genuinely NOT a stall is the car turning harder than asked in the same
                # direction, so exclude only that.
                and not (desired_curvature * current_curvature > 0.0
                         and abs(current_curvature) >= abs(desired_curvature))
                # Opposite signs are also the normal way through an S-curve, where the car is
                # lagging but responding. Only a gap that is not closing is a stall, so require
                # the reversal case to show no progress against a _STALL_GAP_TAU-smoothed
                # reference (frame-to-frame differencing is buried in yawRate noise). The
                # same-sign case keeps its road-validated behaviour untouched.
                and (desired_curvature * current_curvature >= 0.0
                     or (abs(_stall_gap) >= _STALL_GAP_CLOSING * self._stall_gap_mag_slow
                         and _toward_plan_rate < _STALL_RESPONDING_RATE)))
    if _stalled:
      if self.bp_curvature_deviation_limited and self.stall_blip_cooldown_s <= 0.0:
        self.stall_blip_hold_s += _STEER_DT
      if (self.stall_blip_hold_s >= _STALL_HOLD_S and self.stall_blip_count < _STALL_MAX_BLIPS
          and abs(self.path_angle_last) < _BLIP_MAX_PATH_ANGLE):
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
        self.stall_blip_hold_s = 0.0
        self.stall_blip_count += 1
        self.angle_stall_blip_source = 2
    else:
      self.stall_blip_hold_s = 0.0
      if CS.out.steeringPressed or abs(_stall_gap) < 0.5 * _stall_gap_min:
        self.stall_blip_count = 0  # episode over: the car is tracking again or the driver took it

    # Immediately (PSCM Walkthrough setting) -- see the module docstring for why it's jump-free.
    ramp_type = 3

    return LateralResult(
      apply_curvature=0.0,
      curvature_rate=curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.precision_type,
      lateralUncertainty=lateral_uncertainty,
    )

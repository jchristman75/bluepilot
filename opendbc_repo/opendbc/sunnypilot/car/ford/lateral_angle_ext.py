"""
BluePilot: Ford CAN-FD path-angle–primary lateral control (developer).

Steering intent is c1 from κ → θ. Converts planner/model curvature into ``path_angle`` using
PSCM short lookahead d_ref and y ≈ ½κ x² ⇒ path_angle = ½ κ d_ref (see
``bluepilot/agent_info/20_FORD_PSCM_KNOWLEDGE_PACK.md``). Predicted curvature (modelV2) is
blended with ``actuators.curvature`` per ``FordPathAngleBlendRatio`` (0 = planner only,
1 = model only).

**c0 (path_offset) is always zero on the wire.** Lane centering is delivered as a small,
clipped additive trim onto path_angle, mirroring ``lateral_curv_ext`` (which also zeros c0 to
avoid the c0/c1 sign-conflict failure mode). The trim is built from model y at
``path_offset_lookup_time`` blended with laneline center, fed through the curv-mode
``LC_PID_controller``, speed-gated, ROC-limited, and hard-clipped to ``FordPathAngleTrimMax``
(rad, default 0.3) so it can never dominate the κ-derived command. Toggled by the
``FordPrefAnglePathOffsetEnable`` param (UI label TBD-renamed in a follow-up cleanup).
"""
import numpy as np
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.lateral_autotune import LateralAutoTuner
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralResult
from selfdrive.modeld.constants import ModelConstants


def _read_factor_param(params, key: str, default: float) -> float:
  """Read a float factor from params, clamp to [0.5, 1.5], fallback to default on any error."""
  try:
    raw = params.get(key, return_default=True)
    if raw is not None and raw != b"":
      return float(clip(
        float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw),
        0.5, 1.5,
      ))
  except Exception:
    pass
  return default


# Hard-coded per-platform gain defaults (not user-tunable).
# CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
_GAIN_CAN         = (1.00, 1.15)
# CAN-FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
_GAIN_CANFD_BOF   = (0.95, 0.95)
# CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)
_GAIN_CANFD_SUV   = (1.00, 1.05)

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


# PSCM d_ref (m) vs speed (m/s) — 6 points; above ~55.6 m/s use plateau + optional cap to 5 m.
_PSCM_DREF_SPEEDS_MS = (0.0, 4.17, 27.78, 41.67, 50.0, 55.56)
_PSCM_DREF_M = (0.5, 0.95, 1.4, 2.075, 2.75, 3.875)

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

# Rate cap on path_angle magnitude DECREASE during PSCM LimitReached (rad/frame at 50 Hz = 0.20 rad/s).
# Both model and planner naturally drop path_angle ~0.36 rad/s at a sharp 90° apex, while the PSCM is
# physically pinned and cannot execute the rapidly falling desired angle. The resulting actual-vs-desired
# gap (up to 47° observed) causes a snap correction the moment the PSCM is released. This cap limits
# the desired-angle drop rate to what the PSCM can reasonably track, at the cost of holding the car
# slightly more in the curve during saturation. 0.004/frame ≈ 12–17 m unwind distance at 34–44 kph.
_PSCM_SAT_UNWIND_RATE = 0.004       # rad/frame

# Curvature gate for the centering trim (LC_PID).
# In tight curves, modelV2.position.y is dominated by road geometry (the car IS in the turn),
# not by a lane-centre deviation — the PID trim pushes the car further into the curve and adds
# asymmetric overshoot (left curves worse than right due to DBC limit asymmetry: -0.5 vs +0.5235).
# Scale trim linearly from 1.0 at gentle curves to 0.0 at sharp ones.
_TRIM_CURV_GATE_LOW  = 0.006   # 1/m — trim fully active below this (~167 m radius)
_TRIM_CURV_GATE_HIGH = 0.012   # 1/m — trim zeroed above this (~83 m radius)

# High-curvature gain attenuation.
# Left curves on route 00000124 show sustained desiredCurvature = -0.022 to -0.025 for 3+ seconds
# after the apex (secondary tightening), accumulating more total lateral displacement than matching
# right curves (peak dc -0.024, duration ~1s). With trim=0, b=0, VLT=0, the only remaining lever
# is the gain itself. Scale from 1.0 at gentle curves to a user-configurable floor at sharp ones.
_CURV_GAIN_SCALE_LOW  = 0.010  # 1/m — full gain below this (~100 m radius)
_CURV_GAIN_SCALE_HIGH = 0.025  # 1/m — user minimum gain applied at or above this (~40 m radius)

# Straight-line ROC limit: suppress model chatter on straights and at very low speed.
# Only active when both current kappa_cmd and the VLT-base lookahead curvature are below this.
# Above the threshold we're entering a curve and want full, unclipped authority.
_STRAIGHT_ROC_KAPPA_MAX = 0.008  # 1/m (~125 m radius)

# Gain calibration: conditions required for stable curve-following data collection.
# isStableCurveFollow is True only after all four conditions hold for _STABLE_FRAMES_MIN consecutive frames.
_STABLE_KAPPA_MIN      = 0.002   # rad/m — minimum curvature (~500 m radius) to qualify as "on a curve"
_STABLE_KAPPA_RATE_MAX = 0.0003  # rad/m/frame — max per-frame kappa change for "steady" curvature
_STABLE_SPEED_MIN      = 5.0     # m/s (~11 mph) — minimum speed for meaningful yaw-rate signal
_STABLE_FRAMES_MIN     = 30      # frames (0.3 s at 100 Hz) of sustained conditions before flag goes True


def pscm_d_ref_m(v_ego_ms: float) -> float:
  v = max(float(v_ego_ms), 0.0)
  d = float(np.interp(v, _PSCM_DREF_SPEEDS_MS, _PSCM_DREF_M))
  if v > _PSCM_DREF_SPEEDS_MS[-1]:
    # Doc: d_ref table ends at 3.875 m; contribution saturates for high speed — cap at 5 m.
    d = min(5.0, d)
  return d


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
    # User-tunable "feel" multipliers: read from FordAngleLowSpeedFactor / FordAngleHighSpeedFactor params.
    self.low_speed_curv_factor = 1.0
    self.high_speed_curv_factor = 1.0
    self.bp_low_speed_curv_factor = 1.0
    self.bp_high_speed_curv_factor = 1.0
    # Gain telemetry stubs — kept for bp_card_publisher compatibility; not used in the formula.
    self.path_angle_gain_mult_low = 1.0
    self.path_angle_gain_mult_high = 1.0
    self.path_angle_equiv_gain_low = 0.0
    self.path_angle_equiv_gain_high = 0.0
    self.bp_path_angle_equiv_gain = 0.0
    # Telemetry: variable curvature lookup time used this frame (s)
    self.bp_curvature_lookup_time = _VLT_T_EXTRA_MAX + 0.3725  # warm start at ~0.5s
    # Telemetry: unconditional ROC bounds (no curvature gate) for PlotJuggler ROC tuning
    self.bp_roc_path_angle_max = 0.0
    self.bp_roc_path_angle_min = 0.0
    # Telemetry: path_angle after ROC smoothing but before _in_hard_sat block; compare with
    # bp_path_angle_final to see when and how much the hard-sat clamp is suppressing the command.
    self.bp_path_angle_pre_sat = 0.0
    # ford.h ROC simulation: running estimate of what ford.h would have accepted if rate-of-change
    # limiting were enabled in panda. Tracks a shadow "last accepted" value advancing at ford.h's
    # per-frame ROC limit so the band [sim_max, sim_min] drifts away from the actual signal when
    # we send jumps larger than the firmware allows. Reset when lat control disengages.
    self._ford_h_sim_last = 0.0
    self.bp_ford_h_sim_max = 0.0
    self.bp_ford_h_sim_min = 0.0
    self.bp_ford_h_violation_high = False
    self.bp_ford_h_violation_low  = False
    # Gain calibration telemetry — see isStableCurveFollow in custom.capnp
    self._kappa_cmd_last_stable = 0.0      # kappa_cmd from previous frame (for per-frame rate check)
    self._stable_curve_counter = 0         # consecutive frames of sustained stable conditions
    self.bp_kappa_cmd = 0.0                # blended kappa fed into path_angle formula (rad/m)
    self.bp_kappa_cmd_raw = 0.0            # raw planner curvature before blend (rad/m)
    self.bp_d_ref = 0.0                    # PSCM look-ahead distance used this frame (m)
    self.bp_yaw_rate_desired = 0.0         # kappaCmd × vEgo — target yaw rate (rad/s)
    self.bp_is_stable_curve_follow = False  # True when conditions are suitable for gain calibration
    # Exit detection: track previous desired curvature to sense when planner is actively reducing
    self._desired_curvature_last = 0.0
    # Centering trim state (Option C): additive on path_angle, c0 stays zero.
    self.path_angle_trim_max = 0.3  # rad (~17°) — large default; tune down via FordPathAngleTrimMax param
    self._path_angle_trim_last = 0.0
    self._ll_centre_error_filt = 0.0
    self.bp_swa_trim = 0.0              # telemetry: PID trim added to path_angle (rad)
    self.bp_ll_centre_error = 0.0       # telemetry: filtered lane center error (m, +ve = left)
    self.bp_ll_centre_error_raw = 0.0   # telemetry: unfiltered lane center error — actual PID input (m)
    # UI: ``FordPrefAnglePathOffsetEnable`` — repurposed to gate the angle-mode centering trim.
    self.angle_trim_enable = True
    # High-curvature gain scale floor — reduces gain in tight turns to prevent sustained over-steer.
    self.high_curv_gain_scale = 0.825
    # Auto-tune: integrating controller that nudges low/high speed factors toward zero tracking error.
    self._auto_tuner = LateralAutoTuner()
    # Telemetry mirrors exposed for UI / debugging.
    self.bp_at_actual_curvature = 0.0
    self.bp_at_curvature_error = 0.0
    self.bp_at_integral_low = 0.0
    self.bp_at_integral_high = 0.0

  def update_angle_params(self, params):
    """Sets per-platform gain defaults and reads user feel-factor params."""
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
    if params is not None and hasattr(params, "get"):
      low_factor = _read_factor_param(params, "FordAngleLowSpeedFactor", self._auto_tuner.low_factor)
      high_factor = _read_factor_param(params, "FordAngleHighSpeedFactor", self._auto_tuner.high_factor)
      try:
        enabled = params.get_bool("FordAngleAutoTuneEnable")
      except Exception:
        enabled = False
      self._auto_tuner.configure(params, enabled, low_factor, high_factor)
      self.low_speed_curv_factor = self._auto_tuner.low_factor
      self.high_speed_curv_factor = self._auto_tuner.high_factor

  def update_angle_strategy(self, CC, CS, actuators, CP):
    """
    Curvature from planner (+ optional predicted blend) → path_angle via ½·κ·d_ref, plus a small
    additive centering trim (clipped to ``self.path_angle_trim_max`` from ``FordPathAngleTrimMax``).
    c0 is always zero on the wire; centering is delivered through the c1 trim. c2 and c3 are zero.
    Blended κ is not passed through Ford c2 rate / DBC limits (those target the curvature actuator).
    """
    self._ensure_lateral_curv_initialized(CP)

    v_ego = float(CS.out.vEgoRaw)
    d_ref = pscm_d_ref_m(v_ego)

    curvature_rate = 0.0
    path_offset = 0.0
    path_angle = 0.0
    ramp_type = 0
    lateral_uncertainty = 0.0
    precision = 1

    if not CC.latActive:
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.bp_path_angle_equiv_gain = 0.0
      self.apply_curvature_last = 0.0
      self._path_angle_trim_last = 0.0
      self.bp_roc_path_angle_max = 0.0
      self.bp_roc_path_angle_min = 0.0
      self.bp_path_angle_pre_sat = 0.0
      self._ford_h_sim_last = 0.0
      self.bp_ford_h_sim_max = 0.0
      self.bp_ford_h_sim_min = 0.0
      self.bp_ford_h_violation_high = False
      self.bp_ford_h_violation_low  = False
      self._kappa_cmd_last_stable = 0.0
      self._stable_curve_counter = 0
      self.bp_kappa_cmd = 0.0
      self.bp_kappa_cmd_raw = 0.0
      self.bp_d_ref = 0.0
      self.bp_yaw_rate_desired = 0.0
      self.bp_is_stable_curve_follow = False
      self._ll_centre_error_filt = 0.0
      self.bp_swa_trim = 0.0
      self.bp_ll_centre_error = 0.0
      self.bp_ll_centre_error_raw = 0.0
      self.LC_PID_controller.reset()
      self.LC_path_angle_reset_counter = 0
      self.precision_type = 1
      # Reset auto-tune accumulators on disengage and persist any dirty factors.
      self._auto_tuner.flush()
      self._auto_tuner.reset()
      self.bp_at_actual_curvature = 0.0
      self.bp_at_curvature_error = 0.0
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

    self.precision_type = 1
    precision = 1
    LP = self.lp
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
    _pscm_lim = getattr(CS, 'lat_ctl_lim_stat', 0)
    # In angle mode, LatCtlLim_D_Stat (→ lat_ctl_lim_stat) does not fire.
    # Previously used angleState.saturated (CtrSat) as a proxy, but CtrSat fires whenever the car
    # lags the commanded path_angle by > 2.5° — which happens during any normal curve entry.
    # That caused a positive-feedback flat-line: under-steer → CtrSat → path_angle frozen → more under-steer.
    # Use DBC-limit proximity instead: only block when path_angle is already near the ±0.5 rad CAN limits,
    # which is the only condition where the anti-snap unwind rate cap makes physical sense.
    _dbc_sat = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * 0.90 or
                self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * 0.90)
    _in_hard_sat = _pscm_lim >= 2 or _dbc_sat
    _desired_falling = abs(desired_curvature) < abs(self._desired_curvature_last) - 0.002
    _on_exit_near_limit = not _kappa_entering and (_pscm_lim >= 1 or _in_hard_sat or _desired_falling)
    b_blend = float(clip(b * 0.25, 0.0, 1.0)) if _on_exit_near_limit else b
    requested_curvature = predicted_curvature * b_blend + desired_curvature * (1.0 - b_blend)
    self._desired_curvature_last = desired_curvature

    if self.model is not None:
      self.lane_change = self.model.meta.laneChangeState in (1, 2, 3)
    else:
      self.lane_change = False

    lane_change_factor = interp(
      v_ego, self.lane_change_factor_bp, [self.lane_change_factor_low, self.lane_change_factor_high]
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
    # Ford curvature rate limits and apply_std_steer_angle_limits are for c2; angle-specific safety later.
    kappa_cmd = float(requested_curvature)
    lateral_uncertainty = 0.0  # no curvature-limit ladder until angle-mode torque display is defined



    # Speed-interpolated gain: at low speed both curves use 1.0; at high speed the params take effect.
    self.low_gain_calc = interp(v_ego, [13.5, 26.82], [1.0, self.path_angle_gain_lowC_highV])
    self.high_gain_calc = interp(v_ego, [13.5, 26.82], [(1.30 * self.low_speed_curv_factor), (self.path_angle_gain_highC_highV * self.high_speed_curv_factor)])

    # As the curve gets bigger, we will need a little boost to the signal to to not understeer
    self.curvature_factor = interp(abs(kappa_cmd), [0.0007, 0.001], [self.low_gain_calc, self.high_gain_calc])

    path_angle_calc = kappa_cmd * v_ego * self.curvature_factor

    # Auto-tune: compare requested curvature against actual vehicle curvature,
    # gradually adjust low/high speed factors to minimize tracking error.
    # Skip if driver is applying torque — their input corrupts the error signal.
    if not CS.out.steeringPressed:
      self._auto_tuner.update(kappa_cmd, v_ego, CS)
    # Mirror tuner's factor values — an adjustment this frame takes effect next frame's gain calc.
    self.low_speed_curv_factor = self._auto_tuner.low_factor
    self.high_speed_curv_factor = self._auto_tuner.high_factor
    # Mirror telemetry
    self.bp_at_actual_curvature = self._auto_tuner.actual_curvature
    self.bp_at_curvature_error = self._auto_tuner.curvature_error
    self.bp_at_integral_low = self._auto_tuner.integral_low
    self.bp_at_integral_high = self._auto_tuner.integral_high

    # Apply trim, then DBC clip.
    path_angle = path_angle_calc


    # PSCM authority limit clamp.
    # On CANFD Fords in angle mode, LatCtlLim_D_Stat does not fire, so _pscm_lim stays 0.
    # _in_hard_sat (computed above) combines _pscm_lim >= 2 with _dbc_sat (path_angle near ±0.5 rad limit).
    # LimitClose (_pscm_lim >= 1 only): block magnitude increases — exit-biased blend provides unwind.
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
    elif _pscm_lim >= 1:  # LimitClose (F150/non-angle-mode only): block increases only
      path_angle = float(clip(path_angle, -abs(self.path_angle_last), abs(self.path_angle_last)))

    path_angle = min(FORD_DBC_PATH_ANGLE_MAX, max(FORD_DBC_PATH_ANGLE_MIN, path_angle))

    # Soft ROC limit — unconditional, slightly tighter than ford.h, applied before the
    # hardware bypass in ford.h is re-enabled.  Lets us observe whether the limit would
    # suppress control and tune it, while the PSCM still receives the clipped value.
    _soft_roc = float(interp(v_ego, [9., 10., 15., 25.], [0.011, 0.011, 0.0085, 0.0018]))
    path_angle = float(clip(path_angle,
                            self.path_angle_last - _soft_roc,
                            self.path_angle_last + _soft_roc))


    # c0 always zero — centering lives on c1 via the trim above (see lateral_curv_ext.py:537).
    path_offset = 0.0

    # Telemetry / state
    self.bp_path_angle_gain_lowC_highV = self.path_angle_gain_lowC_highV
    self.bp_path_angle_gain_highC_highV = self.path_angle_gain_highC_highV
    self.bp_low_speed_curv_factor = self.low_speed_curv_factor
    self.bp_high_speed_curv_factor = self.high_speed_curv_factor
    self.path_angle_last = path_angle
    self.bp_path_angle_final = path_angle
    self.apply_curvature_last = 0.0
    ramp_type = 2


    return LateralResult(
      apply_curvature=0.0,
      curvature_rate=curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.precision_type,
      lateralUncertainty=lateral_uncertainty,
    )


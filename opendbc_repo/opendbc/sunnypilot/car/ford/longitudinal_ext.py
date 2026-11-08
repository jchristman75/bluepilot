"""
BluePilot Ford longitudinal follow control extension.

Implements smoother highway following by classifying lead vehicle behavior
(gaining, pacing, trailing) and applying gas/accel limits per state. Also
adds split brake/precharge hysteresis for smoother deceleration.

Key features:
  - Speed deadband: BP long engages above 50 mph, disengages below 45 mph
  - Lead classification: gaining (closing in), pacing (matching), trailing (falling behind)
  - Gas limits per state: zero gas when gaining within 1.5s, capped gas when pacing
  - Rate-limited accel changes to avoid stomping the brakes
  - TTC-based emergency bypass for imminent collision scenarios
  - Mutual exclusion: brake_actuate forces gas to INACTIVE_GAS
  - Regen blend (BPLongRegenBlend): gentle decel goes through a negative
    propulsion request (regen) like stock Ford ACC, with the brake flag kept for
    real braking. On the Mach-E the brake flag alone produces ~-1 m/s^2 for any
    request, which turned every small ease-off into a brake hit (logs 4d5/50a).

The gas value is the raw planner request until encode_gas() turns it into the CAN
value (INACTIVE_GAS when propulsion is off), once, at the end of update().
"""

from collections import namedtuple

import numpy as np
from numpy import clip

from opendbc.car.ford.values import CarControllerParams

INACTIVE_GAS = CarControllerParams.INACTIVE_GAS
MIN_GAS = CarControllerParams.MIN_GAS

# Result namedtuple returned by LongitudinalExt.update()
LongitudinalResult = namedtuple('LongitudinalResult', [
  'accel',
  'gas',
  'brake_actuate',
  'precharge_actuate',
  'accel_pred_send',
  'stopping',
  'target_speed',
  'bp_long_used',
])

# Lead car as BP long sees it: distance (m), relative speed (m/s), lead speed (m/s)
Lead = namedtuple('Lead', ['d_rel', 'v_rel', 'v_lead'])


def follow_limits(lead, lead_time_sec, op_gas, op_accel, accel_due_to_pitch):
  """(gas_lo, gas_hi, accel_lo, accel_hi) that BP long clips the planner request to.

  gaining (closing in): no gas within 1.5 s of the lead
  pacing (matching speed): gas capped at 0.2 (+ pitch), never below 0
  trailing (falling behind): planner request unchanged
  no lead: gas unchanged, accel held at 0 (coast; no friction brakes)
  """
  if lead is None:
    return op_gas, op_gas, 0.0, 0.0
  if lead.v_rel < -0.1:  # gaining
    if lead_time_sec < 1.5:
      return 0.0, 0.0, op_accel, op_accel
    return op_gas, op_gas, op_accel, op_accel
  if lead.v_rel > 0.1:  # trailing
    return op_gas, op_gas, op_accel, op_accel
  # pacing. The cap never goes below the gas floor: a steep downhill (pitch < -0.7) would
  # otherwise push it under MIN_GAS.
  return 0.0, max(0.2 + accel_due_to_pitch, MIN_GAS), op_accel, op_accel


def encode_gas(gas, long_active, brake_actuate):
  """Planner gas request -> AccPrpl_A_Rq. Propulsion is inactive when long is off, while
  braking (never gas and brake together), or when the request is below what ford.h allows."""
  if not long_active or brake_actuate or gas < MIN_GAS:
    return INACTIVE_GAS
  return float(clip(gas, MIN_GAS, CarControllerParams.ACCEL_MAX))


class LongitudinalExt:
  """
  BluePilot longitudinal follow control extension for Ford vehicles.

  Mixed into CarController via multiple inheritance. The stock carcontroller
  computes op_accel/op_gas using upstream logic, then calls
  LongitudinalExt.update() to apply BP follow control on top.

  The SubMaster (for radarState) is owned by LateralCurvExt and shared via self.sm
  since both classes are mixed into the same CarController instance.
  """

  def __init__(self, CP, CP_SP):
    # BP longitudinal state
    self.bp_accel_last = 0.0
    self.bpSpeedAllow = False

    # Thresholds
    self.MAX_URBAN_SPEED_MPH = 45.0
    self.following_accel_ROC = 0.002  # max accel change per scan in following mode

    # Brake hysteresis thresholds
    self.brake_actuate_target = -0.14   # engage brakes below this accel
    self.brake_actuate_release = -0.06  # release brakes above this accel
    self.precharge_actuate_target = -0.12
    self.precharge_actuate_release = -0.06
    self.op_brake_actuate_last = False

    # Regen blend (BPLongRegenBlend). Stock Ford ACC (Mach-E, routes 4e6/4f8)
    # sends AccPrpl_A_Rq ~= AccBrkTot_A_Rq down to ~-0.9 with the brake flag off,
    # and mirrors the gas value into AccPrpl_A_Pred. Panda limits gas to -0.5.
    self.regen_blend_UI = False
    self.REGEN_BRAKE_ON = -0.55     # brake flag on below this (pitch-compensated accel)
    self.REGEN_BRAKE_OFF = -0.40    # brake flag off above this
    self.REGEN_MIN_SPEED = 3.0      # m/s; below this (and when stopping) use the stock brake logic
    self.regen_brake_actuate_last = False

    # Toggles (updated from Params each frame)
    self.disable_BP_long_UI = False
    self.disable_downhill_comp_UI = True

  def update_long_params(self, params):
    """Read longitudinal-related Params from the UI. Called each frame."""
    self.disable_BP_long_UI = params.get_bool("disable_BP_long_UI")
    self.disable_downhill_comp_UI = params.get_bool("disable_downhill_comp_UI")
    self.regen_blend_UI = params.get_bool("BPLongRegenBlend")

  def _read_lead(self):
    """leadOne from radarState, or None when it is missing or not present."""
    if not self.sm.valid.get('radarState', False):
      return None
    lead = self.sm['radarState'].leadOne
    if not lead.present:
      return None
    return Lead(float(lead.dRel), float(lead.vRel), float(lead.vLead))

  def _bp_follow(self, CC, CS, op_accel, op_gas, accel_due_to_pitch):
    """BP long follow control. Returns (accel, gas, brake_actuate, precharge_actuate, applied)."""
    v_ego = max(CS.out.vEgo, 0.5)
    lead = self._read_lead()

    lead_time_sec = 999.0
    ttc_sec = 120.0
    v_lead_mph = 0.0
    if lead is not None:
      v_lead_mph = lead.v_lead * 2.23694
      if lead.d_rel > 0:
        lead_time_sec = lead.d_rel / v_ego
      ttc_sec = lead.d_rel / (-lead.v_rel) if (lead.d_rel > 0 and lead.v_rel < 0) else 60.0
    lead_time_sec = float(np.clip(lead_time_sec, 0.0, 999.0))
    ttc_sec = float(np.clip(ttc_sec, 0.2, 120.0))

    gas_lo, gas_hi, accel_lo, accel_hi = follow_limits(lead, lead_time_sec, op_gas, op_accel, accel_due_to_pitch)
    bp_gas = clip(op_gas, gas_lo, gas_hi)
    bp_accel = clip(op_accel, accel_lo, accel_hi)

    # Rate limit downward accel changes (dampen initial brake hit); skip on imminent collision risk
    if ttc_sec > 8.0 and lead_time_sec > 0.5:
      bp_accel = clip(bp_accel, self.bp_accel_last - self.following_accel_ROC, 999)
    self.bp_accel_last = bp_accel

    # BP brake/precharge thresholds
    bp_brake = bool(bp_accel < self.brake_actuate_target)
    bp_precharge = bool(bp_accel < self.precharge_actuate_target)

    applied = (self.bpSpeedAllow and not CS.out.gasPressed and not CS.out.brakePressed and
               (lead is None or v_lead_mph > 40.0))
    return bp_accel, bp_gas, bp_brake, bp_precharge, applied

  def update(self, CC, CS, op_accel, op_gas, accel_due_to_pitch, v_ego_mph, stopping, target_speed):
    """
    Apply BluePilot longitudinal follow control on top of stock op_accel/op_gas.

    Called at 50Hz from CarController.update() inside the ACC_CONTROL_STEP block,
    after stock creep compensation and rate limiting have been applied.

    Args:
      CC: CarControl with longActive
      CS: CarState with vEgo, gasPressed, brakePressed
      op_accel: Stock openpilot accel after creep comp + rate limit (m/s^2)
      op_gas: Planner gas request (m/s^2), raw: encode_gas() decides when propulsion is inactive
      accel_due_to_pitch: Pitch compensation value (m/s^2, clamped by the downhill toggle in the caller)
      v_ego_mph: Current speed in mph
      stopping: True if in stopping state
      target_speed: Target cruise speed (km/h)

    Returns:
      LongitudinalResult namedtuple with final accel, gas, brake, precharge values.
    """
    # Op brake actuate hysteresis
    accel_pitch_compensated = op_accel + accel_due_to_pitch
    op_brake_actuate = self.op_brake_actuate_last
    if accel_pitch_compensated > self.brake_actuate_release or not CC.longActive:
      op_brake_actuate = False
    elif accel_pitch_compensated < self.brake_actuate_target:
      op_brake_actuate = True
    self.op_brake_actuate_last = op_brake_actuate

    # Speed deadband: engage above 50 mph, disallow below 45 mph
    if v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5:
      self.bpSpeedAllow = True
    if v_ego_mph < self.MAX_URBAN_SPEED_MPH:
      self.bpSpeedAllow = False

    accel, gas, brake_actuate, precharge_actuate = op_accel, op_gas, op_brake_actuate, op_brake_actuate
    bp_long_used = False
    if not self.disable_BP_long_UI:
      bp_accel, bp_gas, bp_brake, bp_precharge, bp_long_used = self._bp_follow(CC, CS, op_accel, op_gas,
                                                                               accel_due_to_pitch)
      if bp_long_used and CC.longActive:
        accel, gas, brake_actuate, precharge_actuate = bp_accel, bp_gas, bp_brake, bp_precharge

    # Regen blend: decel through negative propulsion (regen), brake flag only for real braking.
    # Any negative request counts as braking: gas is never positive while decelerating.
    regen_active = (self.regen_blend_UI and CC.longActive and not stopping and
                    CS.out.vEgo > self.REGEN_MIN_SPEED)
    if regen_active:
      accel_pitch = accel + accel_due_to_pitch
      regen_brake = self.regen_brake_actuate_last
      if accel_pitch < self.REGEN_BRAKE_ON:
        regen_brake = True
      elif accel_pitch > self.REGEN_BRAKE_OFF:
        regen_brake = False
      brake_actuate = precharge_actuate = regen_brake

      decel_req = min(accel, gas)
      if not brake_actuate and decel_req < 0.0:
        gas = max(decel_req, MIN_GAS)
    self.regen_brake_actuate_last = brake_actuate

    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    gas = encode_gas(gas, CC.longActive, brake_actuate)
    # Stock Ford mirrors the propulsion request into AccPrpl_A_Pred (op sends INACTIVE)
    accel_pred_send = gas if regen_active else INACTIVE_GAS

    return LongitudinalResult(
      accel=accel,
      gas=gas,
      brake_actuate=brake_actuate,
      precharge_actuate=precharge_actuate,
      accel_pred_send=accel_pred_send,
      stopping=stopping,
      target_speed=target_speed,
      bp_long_used=bp_long_used,
    )

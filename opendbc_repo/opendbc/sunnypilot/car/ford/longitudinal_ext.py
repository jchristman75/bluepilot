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

from collections import deque, namedtuple

import numpy as np
from numpy import clip

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.values_ext import (FORD_REGEN_MASS_DEFAULT_KG, FORD_REGEN_MASS_KG, REGEN_MIN_GAS,
                                                     FordSafetyFlagsBP)

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

# Brake flag (AccBrkDecel_B_Rq) thresholds on pitch-compensated accel: on below `on`, off above `off`
BrakeThresholds = namedtuple('BrakeThresholds', ['on', 'off'])
BRAKE_STOCK = BrakeThresholds(on=-0.14, off=-0.06)
# Regen blend: decel down to MIN_GAS goes through regen, the brake flag only for real braking.
# Stock Ford ACC (Mach-E, routes 4e6/4f8) sends AccPrpl_A_Rq ~= AccBrkTot_A_Rq down to ~-0.9 with
# the flag off and mirrors gas into AccPrpl_A_Pred; panda (ford.h) limits gas to MIN_GAS.
BRAKE_REGEN = BrakeThresholds(on=-0.55, off=-0.40)
# With the panda's deeper regen floor (FordSafetyFlagsBP.REGEN_BLEND at init), regen covers down
# to REGEN_MIN_GAS (-0.9) and the friction brakes only come in below -0.95.
BRAKE_REGEN_DEEP = BrakeThresholds(on=-0.95, off=-0.80)
REGEN_MIN_SPEED = 3.0  # m/s; below this (and when stopping) the stock thresholds apply

# How much of the pack's reported charge-power limit regen is assumed to use. Drivetrain efficiency
# and drag would each add braking, so ignoring them already errs toward less regen.
REGEN_LIMIT_MARGIN = 0.8


def regen_floor(panda_floor, limit_kw, v_ego, mass_kg):
  """Deepest regen (m/s^2) to ask for: the panda's floor, or shallower when the pack can't absorb it
  (full battery, cold pack). limit_kw None = not reported: the panda floor."""
  if limit_kw is None:
    return panda_floor
  max_regen_decel = REGEN_LIMIT_MARGIN * limit_kw * 1000. / (mass_kg * max(v_ego, 1.))
  return max(panda_floor, -max_regen_decel)


def regen_thresholds(floor, panda_floor):
  """Brake flag thresholds for a regen floor: the friction brakes take over just past what regen can
  do. Unlimited regen -> exactly BRAKE_REGEN(_DEEP); no regen at all -> BRAKE_STOCK."""
  base = BRAKE_REGEN_DEEP if panda_floor == REGEN_MIN_GAS else BRAKE_REGEN
  if floor <= panda_floor:
    return base
  on = float(clip(floor - 0.05, base.on, BRAKE_STOCK.on))
  return BrakeThresholds(on=on, off=float(clip(on + 0.15, base.off, BRAKE_STOCK.off)))


class RegenShortfallMonitor:
  """Backup for when regen under-delivers without the limit saying so (stale or unreported limit):
  compares aEgo with how the car normally answers the regen request (0.15 s delay, 0.3 s lag, fit on
  routes 50e/50f/511) and reports a clear shortfall held for 0.8 s. Tuned on drives 50e-515 (52 min):
  0.25/0.5 s fired 7 times, mostly on slow drive-to-regen transitions that settle on their own; these
  settings fire 0 times there and still catch regen that never arrives within 0.9 s."""
  DT = 0.02  # called at 50 Hz
  DELAY_FRAMES = 8
  K = DT / (0.3 + DT)
  MIN_DECEL = 0.25       # only judge a meaningful regen request
  SHORTFALL = 0.35       # m/s^2 less decel than expected...
  HOLD_FRAMES = 40       # ...for 0.8 s

  def __init__(self):
    self.reset(0.0)

  def reset(self, a_ego):
    self.history = deque([a_ego] * self.DELAY_FRAMES, maxlen=self.DELAY_FRAMES)
    self.expected = a_ego
    self.count = 0

  def update(self, regen_cmd, a_ego):
    self.expected += self.K * (self.history[0] - self.expected)
    self.history.append(regen_cmd)
    short = self.expected < -self.MIN_DECEL and (a_ego - self.expected) > self.SHORTFALL
    self.count = self.count + 1 if short else 0
    return self.count >= self.HOLD_FRAMES


class BrakeHysteresis:
  """The single brake flag decision. One state, whichever thresholds are in force."""

  def __init__(self):
    self.on = False

  def update(self, accel_pitch, thresholds, long_active):
    if not long_active or accel_pitch > thresholds.off:
      self.on = False
    elif accel_pitch < thresholds.on:
      self.on = True
    return self.on


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


def encode_gas(gas, long_active, brake_actuate, min_gas=MIN_GAS):
  """Planner gas request -> AccPrpl_A_Rq. Propulsion is inactive when long is off, while
  braking (never gas and brake together), or when the request is below what ford.h allows
  (min_gas: MIN_GAS, or REGEN_MIN_GAS when the panda was told about regen coasting)."""
  if not long_active or brake_actuate or gas < min_gas:
    return INACTIVE_GAS
  return float(clip(gas, min_gas, CarControllerParams.ACCEL_MAX))


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
    self.bp_accel_last = 0.0  # last accel request actually sent (BP's or the planner's): the ramp starts here
    self.bpSpeedAllow = False
    self.bp_used_last = False
    self.handoff_ramp = False  # BP just handed back to the planner: ramp down to its request, don't step
    self.urgent = False

    # Thresholds
    self.MAX_URBAN_SPEED_MPH = 45.0
    # Max downward accel change per 50 Hz scan while BP follows a lead (and on the hand-back): 1.5 m/s^3.
    # It was 0.002 (0.1 m/s^3): a lead appearing at highway speed got ~0 decel for over a second while
    # controlsd's PID wound up, then the hand-back stepped the wound-up request onto the friction
    # brakes (-2..-3.5 m/s^2 behind a lead doing -0.3..-1, route 52d t=342/529/658).
    self.following_accel_ROC = 0.03
    self.URGENT_TTC_S = 4.0         # no ramp when a collision is this close...
    self.URGENT_LEAD_TIME_S = 1.0   # ...or the lead is this close in time

    self.brake = BrakeHysteresis()
    self.regen_blend_UI = False
    # The panda's gas floor is fixed at init (ford.h reads REGEN_BLEND once): never go deeper than it
    self.deep_regen_allowed = CP_SP is not None and bool(CP_SP.safetyParam & FordSafetyFlagsBP.REGEN_BLEND)
    self.regen_mass_kg = FORD_REGEN_MASS_KG.get(CP.carFingerprint if CP is not None else None, FORD_REGEN_MASS_DEFAULT_KG)
    self.regen_monitor = RegenShortfallMonitor()

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
    """BP long follow control. Returns (accel, gas, applied)."""
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

    # Rate limit downward accel changes (dampen initial brake hit), from what was last sent; skip it
    # when a collision is close
    self.urgent = ttc_sec < self.URGENT_TTC_S or lead_time_sec < self.URGENT_LEAD_TIME_S
    if not self.urgent:
      bp_accel = clip(bp_accel, self.bp_accel_last - self.following_accel_ROC, 999)

    applied = (self.bpSpeedAllow and not CS.out.gasPressed and not CS.out.brakePressed and
               (lead is None or v_lead_mph > 40.0))
    return bp_accel, bp_gas, applied

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
    # Speed deadband: engage above 50 mph, disallow below 45 mph
    if v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5:
      self.bpSpeedAllow = True
    if v_ego_mph < self.MAX_URBAN_SPEED_MPH:
      self.bpSpeedAllow = False

    # Which accel/gas request to send: the planner's, or BP long follow's
    accel, gas = op_accel, op_gas
    bp_long_used = False
    if not self.disable_BP_long_UI:
      bp_accel, bp_gas, bp_long_used = self._bp_follow(CC, CS, op_accel, op_gas, accel_due_to_pitch)
      if bp_long_used and CC.longActive:
        accel, gas = bp_accel, bp_gas
    # Hand-back from BP to the planner: the planner's request may have wound up while BP held it back,
    # so ramp down to it instead of stepping onto the friction brakes (unless a collision is close)
    if self.bp_used_last and not (bp_long_used and CC.longActive) and CC.longActive:
      self.handoff_ramp = True
    if self.handoff_ramp:
      if CC.longActive and not self.urgent and accel < self.bp_accel_last - self.following_accel_ROC:
        accel = self.bp_accel_last - self.following_accel_ROC
        gas = max(gas, accel)  # regen no deeper than the ramp either
      else:
        self.handoff_ramp = False
    self.bp_used_last = bp_long_used and CC.longActive
    self.bp_accel_last = accel

    # How to deliver it: one brake flag decision on the final request
    regen_active = (self.regen_blend_UI and CC.longActive and not stopping and
                    CS.out.vEgo > REGEN_MIN_SPEED)
    gas_floor = REGEN_MIN_GAS if (regen_active and self.deep_regen_allowed) else MIN_GAS  # what the panda allows
    if regen_active:
      # how much regen the pack can take right now (full battery / cold pack -> less)
      floor = regen_floor(gas_floor, CS.regen_power_limit_kw, CS.out.vEgo, self.regen_mass_kg)
      thresholds = regen_thresholds(floor, gas_floor)
    else:
      thresholds = BRAKE_STOCK
    brake_actuate = self.brake.update(accel + accel_due_to_pitch, thresholds, CC.longActive)
    # Regen: any negative request is braking, so gas is never positive while decelerating
    regen_only = False
    if regen_active and not brake_actuate:
      decel_req = min(accel, gas)
      if decel_req < 0.0:
        gas = max(decel_req, floor)
        regen_only = gas < 0.0
    # Backup: regen clearly under-delivering -> friction brakes (released by the usual hysteresis)
    if regen_only and self.regen_monitor.update(gas, CS.out.aEgo):
      self.brake.on = brake_actuate = True
    if not regen_only:
      self.regen_monitor.reset(CS.out.aEgo)

    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    gas = encode_gas(gas, CC.longActive, brake_actuate, gas_floor)
    # Stock Ford mirrors the propulsion request into AccPrpl_A_Pred (op sends INACTIVE)
    accel_pred_send = gas if regen_active else INACTIVE_GAS

    return LongitudinalResult(
      accel=accel,
      gas=gas,
      brake_actuate=brake_actuate,
      precharge_actuate=brake_actuate,
      accel_pred_send=accel_pred_send,
      stopping=stopping,
      target_speed=target_speed,
      bp_long_used=bp_long_used,
    )

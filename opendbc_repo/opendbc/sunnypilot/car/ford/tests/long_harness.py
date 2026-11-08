"""Golden-output harness for Ford longitudinal control (BluePilot).

Drives the full Mach-E CarController (carcontroller.py + LongitudinalExt + fordcan packing)
through deterministic scenarios and records every ACCDATA (0x186) frame it sends. The golden
file pins today's CAN output byte for byte, so refactors of the longitudinal path can prove
they don't change what reaches the car.

The scenario inputs are generated from fixed seeds (nothing but the outputs is stored), so the
fixture stays small and readable as code.

Regenerate the fixture only for an intentional behavior change, and say why in the commit:
  python -m opendbc.sunnypilot.car.ford.tests.long_harness --regen
"""
import argparse
import os
import types
from dataclasses import dataclass, field

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.params import Params as _RealParams
from opendbc.can.dbc import DBC
from opendbc.car import structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
import opendbc.car.ford.carcontroller as carcontroller_mod
import opendbc.sunnypilot.car.ford.lateral_curv_ext as lateral_curv_ext_mod
from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalExt
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsBP

GOLDEN_PATH = os.path.join(os.path.dirname(__file__), 'long_golden.npz')
DEEP_REGEN = '_deep_regen_flag'  # scenario key (not a Param): set FordSafetyFlagsBP.REGEN_BLEND at init
ACCDATA = 0x186
DT = 0.01  # CarController runs at 100 Hz
LongCtrlState = structs.CarControl.Actuators.LongControlState
_DEFAULTS = _RealParams()


class FakeParams:
  """Declared params_keys.h defaults, overridden per scenario. Never touches a real param store."""

  def __init__(self, values: dict):
    self.values = values

  def get(self, key, block=False, return_default=False):
    return self.values[key] if key in self.values else _DEFAULTS.get_default_value(key)

  def get_bool(self, key, block=False):
    return bool(self.get(key))


class FakeSubMaster(dict):
  """SubMaster stand-in: default messages, with radarState replaced per frame by the scenario."""

  def __init__(self, services, *args, **kwargs):
    super().__init__({s: getattr(messaging.new_message(s), s) for s in services})
    self.valid = {s: False for s in services}
    self.updated = {s: False for s in services}
    self.alive = dict(self.valid)
    self.seen = dict(self.valid)

  def update(self, timeout=0):
    pass

  def set_lead(self, lead):
    if lead is None:
      self.valid['radarState'] = False
      return
    present, d_rel, v_rel, v_lead = lead
    m = messaging.new_message('radarState')
    l1 = m.radarState.leadOne
    l1.present = bool(present)
    l1.dRel, l1.vRel, l1.vLead = float(d_rel), float(v_rel), float(v_lead)
    self['radarState'] = m.as_reader().radarState
    self.valid['radarState'] = True


@dataclass
class Frame:
  accel: float
  v_ego: float
  pitch: float = 0.0
  long_active: bool = True
  gas_pressed: bool = False
  brake_pressed: bool = False
  stopping: bool = False
  lead: tuple | None = None  # (present, dRel, vRel, vLead), None = radarState invalid


@dataclass
class Scenario:
  name: str
  params: dict
  frames: list = field(default_factory=list)


# ---------------------------------------------------------------- scenario generation

def _ou(rng, n, mean, sigma, theta, x0):
  """Ornstein-Uhlenbeck random walk: a plausible planner accel request."""
  x = np.empty(n)
  x[0] = x0
  for i in range(1, n):
    x[i] = x[i - 1] + theta * (mean - x[i - 1]) * DT + sigma * np.sqrt(DT) * rng.standard_normal()
  return x


def _lead_track(rng, n, v_ego, kind):
  """Lead car around ego. kind: gaining / pacing / trailing / mixed / none / invalid / close."""
  if kind == 'invalid':
    return [None] * n
  if kind == 'none':
    return [(False, 0.0, 0.0, 0.0)] * n
  out = []
  d = rng.uniform(20, 60)
  for i in range(n):
    if kind == 'mixed':
      phase = (i // 300) % 3
      v_rel = (-1.5, 0.0, 1.5)[phase] + 0.05 * rng.standard_normal()
    elif kind == 'close':  # low TTC: fast closing at short range
      v_rel = -6.0
    else:
      v_rel = {'gaining': -1.0, 'pacing': 0.03, 'trailing': 1.0}[kind] + 0.02 * rng.standard_normal()
    d = max(2.0, d + v_rel * DT)
    if kind == 'close' and d < 6.0:
      d = 40.0
    out.append((True, d, v_rel, max(0.0, v_ego[i] + v_rel)))
  return out


def _speed_profile(kind, n):
  t = np.arange(n) * DT
  if kind == 'city':
    return 9.0 + 4.0 * np.sin(2 * np.pi * t / 25.0)
  if kind == 'highway_cross':  # 40 -> 56 -> 40 mph, crossing the 45/50 mph BP deadband both ways
    return (40.0 + 16.0 * np.sin(np.pi * t / t[-1])) / 2.23694
  if kind == 'highway':
    return 28.0 + 1.5 * np.sin(2 * np.pi * t / 30.0)
  if kind == 'stop':  # decel to standstill, hold, pull away
    return np.interp(t, [0, 8, 10, 14, 20], [12.0, 0.0, 0.0, 0.0, 6.0])
  raise ValueError(kind)


def build_scenarios():
  scenarios = []
  param_sets = []
  for regen in (False, True):
    for bp_long_off in (False, True):
      for downhill_off in (False, True):
        param_sets.append({'BPLongRegenBlend': regen, 'disable_BP_long_UI': bp_long_off,
                           'disable_downhill_comp_UI': downhill_off})
  # regen coasting with the panda's deeper floor (FordSafetyFlagsBP.REGEN_BLEND set at init)
  for bp_long_off in (False, True):
    for downhill_off in (False, True):
      param_sets.append({'BPLongRegenBlend': True, 'disable_BP_long_UI': bp_long_off,
                         'disable_downhill_comp_UI': downhill_off, DEEP_REGEN: True})
  shapes = [('city', 'mixed'), ('city', 'invalid'), ('highway_cross', 'pacing'), ('highway_cross', 'none'),
            ('highway', 'gaining'), ('highway', 'trailing'), ('highway', 'close'), ('stop', 'gaining')]
  seed = 0
  for p in param_sets:
    for speed_kind, lead_kind in shapes:
      seed += 1
      rng = np.random.default_rng(seed)
      n = 2000 if speed_kind != 'stop' else 2000
      v = _speed_profile(speed_kind, n)
      accel = _ou(rng, n, mean=-0.1, sigma=1.2, theta=1.5, x0=0.0)
      accel = np.clip(accel, -3.0, 1.8)
      pitch = _ou(rng, n, mean=-0.01, sigma=0.02, theta=0.5, x0=0.0)  # +/- ~3% grade
      lead = _lead_track(rng, n, v, lead_kind)
      frames = []
      for i in range(n):
        # overrides and engagement toggles at fixed points in every scenario
        long_active = not (700 <= i < 760)
        gas = 1200 <= i < 1260
        brake = 1500 <= i < 1530
        stopping = speed_kind == 'stop' and 760 <= i < 1400
        frames.append(Frame(accel=float(accel[i]), v_ego=float(v[i]), pitch=float(pitch[i]),
                            long_active=long_active, gas_pressed=gas, brake_pressed=brake,
                            stopping=stopping, lead=lead[i]))
      flags = f"regen{int(p['BPLongRegenBlend'])}-bpoff{int(p['disable_BP_long_UI'])}-dh{int(p['disable_downhill_comp_UI'])}"
      flags += '-deep' if p.get(DEEP_REGEN) else ''
      name = f"{speed_kind}-{lead_kind}-{flags}"
      scenarios.append(Scenario(name, p, frames))
  return scenarios


# ---------------------------------------------------------------- running the real CarController

_dbc = DBC('ford_lincoln_base_pt')


def _zeros(msg_name):
  m = next(x for x in _dbc.addr_to_msg.values() if x.name == msg_name)
  return {sig: 0.0 for sig in m.sigs}


def _make_cc(fr: Frame):
  m = messaging.new_message('carControl')
  c = m.carControl
  c.enabled = c.longActive = fr.long_active
  c.actuators.accel = fr.accel
  c.actuators.longControlState = LongCtrlState.stopping if fr.stopping else LongCtrlState.pid
  c.orientationNED = [0.0, fr.pitch, 0.0]
  return m.as_reader().carControl


def run_scenario(sc: Scenario):
  """Returns (acc_bytes uint8[N,8], bp_long_used bool[N]) for every ACCDATA frame sent."""
  orig_sm, orig_params = lateral_curv_ext_mod.messaging, carcontroller_mod.Params
  lateral_curv_ext_mod.messaging = types.SimpleNamespace(SubMaster=FakeSubMaster, new_message=messaging.new_message)
  carcontroller_mod.Params = lambda: FakeParams(sc.params)
  orig_update = LongitudinalExt.update
  bp_used = []

  def recording_update(self, *args, **kwargs):
    res = orig_update(self, *args, **kwargs)
    bp_used.append(bool(getattr(res, 'bp_long_used', False)))
    return res

  LongitudinalExt.update = recording_update
  try:
    CP = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
    CP.openpilotLongitudinalControl = True
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.FORD_MUSTANG_MACH_E_MK1)
    if sc.params.get(DEEP_REGEN):
      CP_SP.safetyParam |= FordSafetyFlagsBP.REGEN_BLEND
    CI = CarInterface(CP, CP_SP)
    CS = CI.CS
    CS.buttons_stock_values = _zeros('Steering_Data_FD1')
    CS.acc_tja_status_stock_values = _zeros('ACCDATA_3')
    CS.lkas_status_stock_values = _zeros('IPMA_Data')
    CS.lat_ctl_lim_stat = 0
    CC_SP = messaging.new_message('carControlSP').as_reader().carControlSP
    acc = []
    for i, fr in enumerate(sc.frames):
      CS.out.vEgo = CS.out.vEgoRaw = fr.v_ego
      CS.out.gasPressed, CS.out.brakePressed = fr.gas_pressed, fr.brake_pressed
      CS.out.standstill = fr.v_ego < 0.01
      CI.CC.sm.set_lead(fr.lead)
      _, sends = CI.CC.update(_make_cc(fr), CC_SP, CS, int(i * DT * 1e9))
      acc.extend(np.frombuffer(dat, dtype=np.uint8) for addr, dat, _bus in sends if addr == ACCDATA)
    return np.array(acc, dtype=np.uint8), np.array(bp_used, dtype=bool)
  finally:
    LongitudinalExt.update = orig_update
    lateral_curv_ext_mod.messaging, carcontroller_mod.Params = orig_sm, orig_params


def run_all():
  return {sc.name: run_scenario(sc) for sc in build_scenarios()}


def decode(acc_bytes):
  """ACCDATA bytes -> dict of signal arrays (for readable failure messages)."""
  from opendbc.can.parser import CANParser
  p = CANParser('ford_lincoln_base_pt', [('ACCDATA', 50)], 0)
  out = {k: [] for k in ('AccBrkTot_A_Rq', 'AccPrpl_A_Rq', 'AccPrpl_A_Pred', 'AccBrkDecel_B_Rq', 'AccBrkPrchg_B_Rq')}
  for i, row in enumerate(acc_bytes):
    p.update([(int(i * 2e7), [(ACCDATA, bytes(row), 0)])])
    for k in out:
      out[k].append(p.vl['ACCDATA'][k])
  return {k: np.array(v) for k, v in out.items()}


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('--regen', action='store_true', help='rewrite the golden fixture from the current code')
  args = ap.parse_args()
  results = run_all()
  if args.regen:
    np.savez_compressed(GOLDEN_PATH, **{f'{k}__acc': v[0] for k, v in results.items()},
                        **{f'{k}__bp': v[1] for k, v in results.items()})
    print(f'wrote {GOLDEN_PATH}: {len(results)} scenarios, {sum(len(v[0]) for v in results.values())} ACCDATA frames')
  else:
    for k, (acc, bp) in results.items():
      print(k, len(acc), int(bp.sum()))

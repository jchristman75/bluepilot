#!/usr/bin/env python3
"""
BluePilot: closed-loop test harness -- the real angle-mode lateral code driving the PSCM emulator.

For every route it replays the recorded messages in log order. carControl (the planner's desired
curvature), modelV2, liveDelay, liveParameters, the car's own Params (initData) and CarParams
are fed exactly as the car saw them. The car's response is replaced by the emulator:

  every carState (100 Hz):
    emulator.step(latched wire frame)            -> emulated steeringAngleDeg / yawRate
    CS = logged carState with those two replaced
    every 5th: LateralCurvExt + LateralAngleExt (the real classes, the real carcontroller dispatch)
               -> LateralMotionControl2 + Lane_Assist_Data1 frames (the real packers)
    both frames go through the compiled ford.h (panda safety); a blocked 982 frame never reaches
    the emulator, just as on the car.
  every can message: fed to ford.h's rx hook, with Yaw_Data_FD1's yaw rewritten to the emulated
    yaw (and its checksum recomputed) so ford.h's angle_meas sees the emulated car.

Disturbance replay (default): a shadow emulator follows the LOGGED wire frames and a shadow car
follows the LOGGED steering angle; what the real car did beyond them (road crown, wind, sensor
noise, PSCM quirks) is added into the closed loop. With the recorded code the loop therefore
reproduces the recorded drive, and a code change moves only what the model predicts it moves.
--ideal turns this off (a clean, disturbance-free car).

The driver is a replay: while the log shows a press, or lateral is off, the emulator is forced
onto the logged steering angle and yaw rate, so emulated and logged only differ hands-free.

Variants: a git ref for the Ford controller code (opendbc/sunnypilot/car/ford/*, loaded from git
without touching the checkout), Params overrides, and an optional post-release bias disturbance.

Usage:
  harness.py ROUTE_DIR [ROUTE_DIR ...] --out DIR [--ref REF] [--set KEY=VAL ...] [--open-loop]
             [--bias 0.002] [--name NAME]
"""
import argparse
import ast
import glob
import importlib.abc
import importlib.util
import os
import subprocess
import sys
import tempfile
from multiprocessing import Pool

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_FORD_EXT = 'opendbc_repo/opendbc/sunnypilot/car/ford'
_FORD_PKG = 'opendbc.sunnypilot.car.ford'


class _RefFinder(importlib.abc.MetaPathFinder):
  """Serve opendbc.sunnypilot.car.ford.* from a git ref (extracted to a temp dir), or from a directory
  of edited copies ('dir:/path' -- only the .py files present there are replaced)."""
  def __init__(self, ref):
    self.mods = {}
    if ref.startswith('dir:'):
      for f in glob.glob(os.path.join(ref[4:], '*.py')):
        if not f.endswith('__init__.py'):
          self.mods[f'{_FORD_PKG}.{os.path.basename(f)[:-3]}'] = f
      return
    self.dir = tempfile.mkdtemp(prefix=f'pscm_ref_{ref}_')
    names = subprocess.check_output(['git', '-C', _REPO, 'ls-tree', '--name-only', ref, _FORD_EXT + '/'], text=True).split()
    for path in names:
      if path.endswith('.py') and not path.endswith('__init__.py'):
        src = subprocess.check_output(['git', '-C', _REPO, 'show', f'{ref}:{path}'])
        dst = os.path.join(self.dir, os.path.basename(path))
        with open(dst, 'wb') as f:
          f.write(src)
        self.mods[f'{_FORD_PKG}.{os.path.basename(path)[:-3]}'] = dst

  def find_spec(self, fullname, path=None, target=None):
    if fullname in self.mods:
      return importlib.util.spec_from_file_location(fullname, self.mods[fullname])
    return None


def install_ref(ref):
  if ref and ref != 'HEAD+':  # 'HEAD+' (default) = the working tree, uncommitted changes included
    for m in [m for m in sys.modules if m.startswith(_FORD_PKG + '.')]:
      del sys.modules[m]
    sys.meta_path.insert(0, _RefFinder(ref))


class _Params:
  def __init__(self, values):
    self.values = values

  def get(self, key, return_default=False):
    """Typed like openpilot's Params.get: int, then float, then str."""
    raw = self.values.get(key)
    if raw is None:
      return None
    txt = raw.decode(errors='replace') if isinstance(raw, bytes) else str(raw)
    for conv in (int, float):
      try:
        return conv(txt)
      except ValueError:
        pass
    return txt

  def get_bool(self, key):
    v = self.values.get(key)
    return v is not None and v.strip() in (b'1', b'true', b'True')


class _SM:
  def __init__(self):
    self.msgs = {}
    self.updated = {s: False for s in ('modelV2', 'liveParameters', 'selfdriveState', 'radarState', 'liveDelay')}

  def update(self, timeout=0):
    pass

  def __getitem__(self, key):
    return self.msgs[key]


class _CS:
  def __init__(self, out):
    self.out = out
    self.lat_ctl_lim_stat = 0


def _make_controller(CP, CP_SP):
  from unittest import mock
  from opendbc.sunnypilot.car.ford import lateral_curv_ext
  from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt
  from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt

  class Controller(LateralCurvExt, LateralAngleExt):
    """CarController's lateral mixins, composed exactly as in carcontroller.py."""
    def __init__(self):
      self.CP = CP
      with mock.patch.object(lateral_curv_ext.messaging, 'SubMaster', lambda *a, **k: _SM()):
        LateralCurvExt.__init__(self, CP, CP_SP)
      LateralAngleExt.__init__(self, CP, CP_SP)
  return Controller()


def _pinion_frame(dat, angle_deg):
  """Rewrite StePinComp_An_Est (22|15@0+, 0.1 deg, offset -1600) in SteeringPinion_Data (0x7E).
  ford.h checks this message's counter and quality flag only, both left as logged."""
  d = bytearray(dat)
  raw = max(0, min(0x7FFF, int(round((angle_deg + 1600.0) / 0.1))))
  d[2] = (d[2] & 0x80) | ((raw >> 8) & 0x7F)
  d[3] = raw & 0xFF
  return bytes(d)


def _yaw_frame(dat, yaw):
  """Rewrite VehYaw_W_Actl (bytes 2-3, 0.0002 rad/s, offset -6.5) and the Yaw_Data_FD1 checksum."""
  d = bytearray(dat)
  raw = int(round((yaw + 6.5) / 0.0002))
  raw = max(0, min(0xFFFF, raw))
  d[2], d[3] = raw >> 8, raw & 0xFF
  cs = d[0] + d[1] + d[2] + d[3] + d[5] + (d[6] >> 6) + ((d[6] >> 4) & 0x3)
  d[4] = (0xFF - cs) & 0xFF
  return bytes(d)


COLS = ('t', 'v', 'latActive', 'pressed', 'acurv', 'yaw_emu', 'ang_emu', 'yaw_log', 'ang_log', 'c1', 'c1_log',
        'mode', 'devLim', 'unwind', 'windup', 'blipSrc', 'htActive', 'blocked', 'forced', 'lcState', 'bias', 'lat_delay')


def apply_patches(patches):
  """'module.CONST' -> value on the Ford ext modules (after import), for constant sweeps."""
  for key, val in (patches or {}).items():
    mod, attr = key.rsplit('.', 1)
    m = importlib.import_module(f'{_FORD_PKG}.{mod}')
    assert hasattr(m, attr), f'no {attr} in {mod}'
    setattr(m, attr, val)


def run_route(route_dir, ref=None, overrides=None, open_loop=False, bias=0.0, emu_params=None, ideal=False,
              patches=None, bias_clear_on_mode0=False, pinion=False):
  """Returns a dict of per-tick arrays (COLS) for one route."""
  install_ref(ref)
  apply_patches(patches)
  from opendbc.car.ford.values import CarControllerParams
  from openpilot.tools.lib.logreader import LogReader
  from opendbc.can import CANPacker
  from opendbc.car import structs
  from opendbc.car.ford import fordcan
  from opendbc.sunnypilot.car.ford import fordcan_ext
  from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl
  from opendbc.safety import ALTERNATIVE_EXPERIENCE
  from opendbc.safety.tests.libsafety import libsafety_py
  from bluepilot.tools.pscm.emulator import PSCMEmulator, PSCMParams

  segs = sorted(glob.glob(os.path.join(route_dir, '*--*/rlog.zst')), key=lambda p: int(p.split('--')[-1].split('/')[0]))
  packer = CANPacker('ford_lincoln_base_pt')
  ep = PSCMParams.load(emu_params) if isinstance(emu_params, str) else (emu_params or PSCMParams.load())
  emu = PSCMEmulator(ep, n=1)
  shadow = PSCMEmulator(ep, n=1)      # the PSCM + car under the LOGGED wire frames
  shadow_car = PSCMEmulator(ep, n=1)  # the car alone under the LOGGED steering angle
  wire_log = (0, 0.0)
  safety = libsafety_py.libsafety
  ctrl = CAN = CP = None
  params = {}
  CC = model = None
  frame = 0
  phase = 0                # carState tick (mod 5) the real card ran lateral on; locked to logged 982 sends
  wire = (0, 0.0)          # (mode, C1) latched at the PSCM
  c1_log = 0.0
  emu_ready = False
  yaw_seen = None          # the yaw rate the controller last saw; ford.h must see the same one
  ang_seen = None          # likewise the steering angle (pinion-sourced angle_meas, --pinion)
  press_s, last_press_curv, bias_t = 0.0, 0.0, 1e9
  rows = []

  for seg in segs:
    try:
      msgs = list(LogReader(seg))
    except Exception as e:
      print('skip', seg, e)
      continue
    for m in msgs:
      w = m.which()
      if w == 'initData' and not params:
        params = {e.key: e.value for e in m.initData.params.entries}
        params.update({k: v.encode() if isinstance(v, str) else v for k, v in (overrides or {}).items()})
      elif w == 'carParams' and ctrl is None:
        CP = m.carParams
        CP_SP = structs.CarParamsSP()
      elif w == 'carParamsSP' and CP is not None and ctrl is None:
        CP_SP = m.carParamsSP
      elif w == 'modelV2':
        model = m.modelV2
      elif w in ('liveDelay', 'liveParameters'):
        if ctrl is not None:
          ctrl.sm.msgs[w] = getattr(m, w)
          if w == 'liveParameters':
            ctrl.lp = m.liveParameters
            ctrl.VM.update_params(max(ctrl.lp.stiffnessFactor, 0.1), max(ctrl.lp.steerRatio, 0.1))
      elif w == 'carControl':
        CC = m.carControl
      elif w == 'sendcan':
        for f in m.sendcan:
          if f.address == 982 and f.src < 128:
            phase = (frame - 1) % CarControllerParams.STEER_STEP
            c1_log = -((((f.dat[3] & 0x1F) << 6) | (f.dat[4] >> 2)) * 0.0005 - 0.5)  # internal sign
            wire_log = ((f.dat[0] >> 4) & 0x7, -c1_log)  # LatCtl_D2_Rq, wire C1
      elif w == 'can':
        if ctrl is None:
          continue
        safety.set_timer((m.logMonoTime // 1000) % 0xFFFFFFFF)
        for f in m.can:
          if f.src >= 128:
            continue
          dat = f.dat
          if f.address == 0x91 and f.src == 0 and not open_loop and yaw_seen is not None:
            dat = _yaw_frame(dat, yaw_seen)
          elif f.address == 0x7E and f.src == 0 and not open_loop and ang_seen is not None:
            dat = _pinion_frame(dat, ang_seen)
          safety.safety_rx_hook(libsafety_py.make_CANPacket(f.address, f.src % 4, dat))
      elif w == 'carState':
        if CP is None or CC is None or not params:
          continue
        if ctrl is None:
          if pinion:  # FordPrefSteerAngleCurvature: flag + Mach-E geometry index, as the car interface packs it
            from opendbc.sunnypilot.car.ford.values_ext import (FordSafetyFlagsSP, FORD_PINION_GEOMETRY_INDEX,
                                                                FORD_PINION_GEOMETRY_SHIFT)
            from opendbc.car.ford.values import CAR
            sp = (int(getattr(CP_SP, 'safetyParam', 0)) | FordSafetyFlagsSP.STEER_ANGLE_CURVATURE
                  | (FORD_PINION_GEOMETRY_INDEX[CAR(CP.carFingerprint)] << FORD_PINION_GEOMETRY_SHIFT))
            CP_SP = type('CPSP', (), {'safetyParam': sp})()
          ctrl = _make_controller(CP, CP_SP)
          CAN = fordcan.CanBus(CP)
          cfg = CP.safetyConfigs[-1]
          safety.set_current_safety_param_sp(int(getattr(CP_SP, 'safetyParam', 0)))
          assert safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam) == 0
          alt = int(CP.alternativeExperience)
          safety.set_alternative_experience(alt)
          safety.set_mads_params(bool(alt & ALTERNATIVE_EXPERIENCE.ENABLE_MADS),
                                 bool(alt & ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE),
                                 bool(alt & ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE))
          if 'liveDelay' not in ctrl.sm.msgs:
            ctrl.sm.msgs['liveDelay'] = type('LD', (), {'lateralDelay': 0.2})()
        cs_log = m.carState
        v = cs_log.vEgoRaw
        roll = ctrl.lp.roll if ctrl.lp is not None else 0.0
        pressed = bool(cs_log.steeringPressed)
        forced = pressed or not CC.latActive or open_loop

        # post-release bias disturbance: after a sustained press, the PSCM carries a curvature bias
        # against the turn the driver was holding, decaying over ~4 s (see post-release-clip notes)
        if pressed:
          press_s += 0.01
          last_press_curv = -cs_log.yawRate / max(v, 0.1)
          bias_t = 1e9
        else:
          if press_s >= 0.5 and bias:
            bias_t = 0.0
          press_s = 0.0
          bias_t += 0.01
        if bias_clear_on_mode0 and CC.latActive and not pressed and wire[0] == 0:
          bias_t = 1e9  # hypothesis: a mode-0 pulse (stall blip) resets the PSCM's post-release bias
        bias_k = -np.sign(last_press_curv) * bias * np.exp(-bias_t / 4.0) if bias and bias_t < 12 else 0.0
        bias_deg = float(emu._nominal_deg_per_c1(max(v, 1.0)) * v) * bias_k * -1.0  # internal kappa -> carState deg

        # 1. the car responds to the latched wire frame
        if not emu_ready:
          for e_ in (emu, shadow, shadow_car):
            e_.reset(cs_log.steeringAngleDeg, cs_log.yawRate, v)
          emu_ready = True
        # residuals of the recorded drive against the model
        drv = cs_log.steeringAngleDeg if forced else None
        so = shadow.step(wire_log[0], wire_log[1], v, roll, driver_angle=drv)
        sy = shadow_car.vehicle_step(np.array([cs_log.steeringAngleDeg]), v, roll)
        r_ang = 0.0 if ideal else cs_log.steeringAngleDeg - float(so['steeringAngleDeg'][0])
        r_yaw = 0.0 if ideal else cs_log.yawRate - float(sy[0])
        if forced:
          o = emu.step(wire[0], wire[1], v, roll, driver_angle=cs_log.steeringAngleDeg)
          for e_ in (emu, shadow, shadow_car):
            e_.sync_vehicle(cs_log.yawRate, v)
          yaw_e, ang_e = cs_log.yawRate, cs_log.steeringAngleDeg
        else:
          o = emu.step(wire[0], wire[1], v, roll, bias_deg=bias_deg, ang_dist=r_ang, yaw_dist=r_yaw)
          yaw_e, ang_e = float(o['yawRate'][0]), float(o['steeringAngleDeg'][0])

        yaw_seen = yaw_e
        ang_seen = ang_e

        # 2. CS as the controller sees it
        out = cs_log.as_builder()
        out.yawRate = yaw_e
        out.steeringAngleDeg = ang_e
        CS = _CS(out)
        ctrl.model = model

        # 3. the real lateral dispatch (carcontroller.py, BP lateral enabled, angle mode)
        blocked = 0
        if frame % CarControllerParams.STEER_STEP == phase:
          ctrl.update_lateral_params(_Params(params))
          ctrl.update_angle_params(_Params(params))
          angle_mode = ctrl.primary_lateral_control == PrimaryLateralControl.angle
          assert angle_mode, 'route not in angle mode'
          lat = ctrl.update_angle_strategy(CC, CS, CC.actuators, CP)
          lat_active = CC.latActive and not (ctrl.angle_human_turn_active or ctrl.angle_stall_blip_active)
          mode = 2 if lat_active else 0
          counter = (frame // CarControllerParams.STEER_STEP) % 0x10
          addr, dat, bus = fordcan_ext.create_lat_ctl2_msg(packer, CAN, mode, lat.ramp_type, lat.precision_type,
                                                           -lat.path_offset, -lat.path_angle, -lat.apply_curvature,
                                                           -lat.curvature_rate, counter)
          ok = safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus % 4, dat))
          if ok:
            wire = (mode, -lat.path_angle)  # wire sign = carState sign
          blocked |= int(not ok)
        if frame % CarControllerParams.LKA_STEP == 0:
          addr, dat, bus = fordcan_ext.create_lka_msg(packer, CAN, CC.latActive, CC.hudControl, True, -ctrl.bp_kappa_cmd)
          blocked |= 2 * int(not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus % 4, dat)))
        frame += 1

        rows.append((m.logMonoTime * 1e-9, v, CC.latActive, pressed, CC.actuators.curvature, yaw_e, ang_e,
                     cs_log.yawRate, cs_log.steeringAngleDeg, -wire[1] if wire[0] else 0.0, c1_log, wire[0],
                     ctrl.bp_curvature_deviation_limited, getattr(ctrl, 'bp_unwind_clamped', False),
                     getattr(ctrl, 'bp_windup_released', False),
                     ctrl.angle_stall_blip_source, ctrl.angle_human_turn_active, blocked, forced,
                     model.meta.laneChangeState.raw if model is not None else 0, bias_k,
                     ctrl.sm.msgs['liveDelay'].lateralDelay))
  a = np.array(rows, dtype=np.float64)
  return {c: a[:, i] for i, c in enumerate(COLS)} if len(a) else {}


def _job(args):
  route_dir, out, kw = args
  name = os.path.basename(route_dir.rstrip('/')).split('_')[-1]
  path = os.path.join(out, f'{name}.npz')
  if os.path.exists(path):
    return path
  try:
    r = run_route(route_dir, **kw)
  except AssertionError as e:
    print('skip', name, e)
    r = {}
  np.savez_compressed(path, **r)
  return path


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('routes', nargs='+')
  ap.add_argument('--out', required=True)
  ap.add_argument('--ref', default=None, help='git ref for opendbc/sunnypilot/car/ford (default: working tree)')
  ap.add_argument('--set', action='append', default=[], help='Params override KEY=VALUE')
  ap.add_argument('--open-loop', action='store_true', help='feed logged yaw/angle (parity check)')
  ap.add_argument('--bias', type=float, default=0.0, help='post-release bias disturbance, 1/m')
  ap.add_argument('--bias-clear-on-mode0', action='store_true',
                  help='the injected bias ends at the first mode-0 frame (i.e. a stall blip fixes it)')
  ap.add_argument('--pinion', action='store_true',
                  help='FordPrefSteerAngleCurvature: steering-angle curvature source in controller and ford.h')
  ap.add_argument('--emu', default=None, help='emulator params file (default mache_pscm_params.json)')
  ap.add_argument('--ideal', action='store_true', help='no replayed disturbances (clean emulated car)')
  ap.add_argument('--patch', action='append', default=[],
                  help="module constant override, e.g. lateral_angle_ext._LEADLAG_TAU_S='(0.5, 1.0)'")
  ap.add_argument('-j', type=int, default=30)
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=True)
  kw = dict(ref=a.ref, overrides=dict(s.split('=', 1) for s in a.set), open_loop=a.open_loop, bias=a.bias, ideal=a.ideal,
            bias_clear_on_mode0=a.bias_clear_on_mode0, emu_params=a.emu, pinion=a.pinion,
            patches={k: ast.literal_eval(v) for k, v in (p.split('=', 1) for p in a.patch)})
  with Pool(a.j, maxtasksperchild=1) as p:
    for path in p.imap_unordered(_job, [(r, a.out, kw) for r in a.routes]):
      print('done', path)


if __name__ == '__main__':
  main()

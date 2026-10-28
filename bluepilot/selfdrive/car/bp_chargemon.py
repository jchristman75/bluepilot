#!/usr/bin/env python3
"""
BluePilot offroad charge monitor.

When the car is turned off mid-charge, ignition drops, card stops and pandad puts the panda
into power save (CAN transceivers off), so the charging UI freezes on its last reading. The
car cannot be driven while plugged in, so there is nothing to protect by staying onroad --
but forcing onroad would start cameras, models, logging and fill the screen with CAN alerts.

Instead this process stays offroad and:
  1. Asks pandad to keep the CAN transceivers on (BPChargeCanAwake), nothing else changes:
     safety stays NO_OUTPUT, the relay stays open, no process starts.
  2. Decodes only the charging/battery messages with the same CarStateExt code card uses and
     publishes carStateBP, so the existing charge session/UI keeps working untouched.

It wakes CAN when:
  - ignition drops while carStateBP said chargingActive (the "turned the car off" case), or
  - a periodic probe (every PROBE_INTERVAL_S offroad) sees charging (plugged in with the car
    already off, or a charge that resumes on a schedule).

It releases CAN when charging has read inactive for INACTIVE_RELEASE_S, after MAX_MONITOR_S,
or the moment the device goes onroad (card owns carStateBP again).
"""

import time
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, FordFlags
from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.pandad import can_capnp_to_list

PUBLISH_HZ = 10
ONROAD_CHARGING_WINDOW_S = 10.0  # charging this recently when ignition dropped -> keep monitoring
# Offroad and idle: wake CAN this often to look for a charge (0 = never). Off: waking CAN every 5 min on a
# parked, locked car is the suspected cause of the Mach-E alarm going off (2026-10-28). The ignition-drop
# path (car turned off mid-charge) still monitors.
PROBE_INTERVAL_S = 0.0
PROBE_WINDOW_S = 15.0            # how long a probe listens before giving up
STATUS_STALE_S = 2.0             # MtrTrac_Data2_FD1 (charge status) older than this = no data
INACTIVE_RELEASE_S = 60.0        # charging inactive this long -> release CAN (UI grace is 15 s)
MAX_MONITOR_S = 18 * 3600.0      # hard cap, longer than a Level 2 charge from empty
CHARGE_STATUS_ADDR = 0x442       # MtrTrac_Data2_FD1 (ChrgStat_D_ActlMntr)


def _load_car_params(params: Params):
  """Fields CarStateExt reads, from the last fingerprint. None if not a Ford with charge data."""
  cp_bytes = params.get("CarParamsPersistent")
  if not cp_bytes:
    return None
  with car.CarParams.from_bytes(cp_bytes) as CP:
    if CP.brand != "ford" or not (CP.flags & FordFlags.CHARGING_DATA):
      return None
    return SimpleNamespace(
      carFingerprint=CP.carFingerprint,
      flags=CP.flags,
      openpilotLongitudinalControl=False,  # skips the camera-bus ACC brake-light overlay
      safetyConfigs=[None] * len(CP.safetyConfigs),
    )


def _charge_messages(CP) -> list[tuple[str, float]]:
  """Same charge/battery messages carstate.py registers, all non-critical."""
  msgs = [("Battery_Traction_5_FD1", float('nan')), ("MtrTrac_Data2_FD1", float('nan'))]
  if CP.flags & FordFlags.HEV_BATTERY_DATA:
    msgs += [(m, float('nan')) for m in ("Battery_Traction_1_FD1", "Battery_Traction_3_FD1",
                                          "Battery_Traction_4_FD1", "MtrTracData_1_FD1")]
  if CP.flags & FordFlags.HV_CURRENT_DATA:
    msgs.append(("HV_Battery_Current_BP", float('nan')))
    if not CP.flags & FordFlags.HEV_BATTERY_DATA:
      msgs.append(("Battery_Traction_1_FD1", float('nan')))
  return msgs


class ChargeMonitor:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['deviceState', 'carStateBP'])
    self.can_sock = messaging.sub_sock('can', timeout=0)
    self.pm: messaging.PubMaster | None = None

    self.CP = None
    self.shim = None  # CarStateExt.update_car_state_bp only needs .CP
    self.cp: CANParser | None = None
    self.cp_cam: CANParser | None = None

    self.started = False
    self.last_onroad_charging_t = -1e9
    self.mode = "idle"  # idle | probe | monitor
    self.mode_start_t = 0.0
    self.last_active_t = 0.0
    self.last_probe_t = time.monotonic()
    self.last_status_t = 0.0

  def _setup_parsers(self) -> bool:
    self.CP = _load_car_params(self.params)
    if self.CP is None:
      return False
    dbc = DBC[self.CP.carFingerprint][Bus.pt]
    bus = CanBus(self.CP)
    self.cp = CANParser(dbc, _charge_messages(self.CP), bus.main)
    self.cp_cam = CANParser(dbc, [], bus.camera)
    self.shim = SimpleNamespace(CP=self.CP)
    return True

  def _wake(self, mode: str, now: float) -> None:
    if self.mode == "idle":
      if not self._setup_parsers():
        self.last_probe_t = now
        return
      self.params.put_bool("BPChargeCanAwake", True)
      messaging.drain_sock_raw(self.can_sock)
    cloudlog.info(f"bp_chargemon: {self.mode} -> {mode}")
    if self.mode == "idle":
      self.last_status_t = 0.0
    self.mode = mode
    self.mode_start_t = now
    self.last_active_t = now

  def _release(self, reason: str, now: float) -> None:
    if self.mode != "idle":
      cloudlog.info(f"bp_chargemon: {self.mode} -> idle ({reason})")
      self.params.put_bool("BPChargeCanAwake", False)
    self.mode = "idle"
    self.pm = None  # card publishes carStateBP onroad; never hold a second publisher
    self.cp = self.cp_cam = None
    self.last_probe_t = now

  def step(self) -> None:
    self.sm.update(0)
    now = time.monotonic()

    started = self.sm['deviceState'].started if self.sm.seen['deviceState'] else self.started
    if started:
      if self.sm.updated['carStateBP'] and self.sm['carStateBP'].charging.chargingActive:
        self.last_onroad_charging_t = now
      if self.mode != "idle":
        self._release("onroad", now)
      self.started = True
      return

    if self.started:  # ignition just dropped
      self.started = False
      if now - self.last_onroad_charging_t < ONROAD_CHARGING_WINDOW_S:
        self._wake("monitor", now)
      else:
        self.last_probe_t = now

    if self.mode == "idle":
      if PROBE_INTERVAL_S > 0.0 and now - self.last_probe_t >= PROBE_INTERVAL_S:
        self._wake("probe", now)
      return

    can_strs = messaging.drain_sock_raw(self.can_sock)
    if can_strs and CHARGE_STATUS_ADDR in self.cp.update(can_capnp_to_list(can_strs)):
      self.last_status_t = now

    if now - self.last_status_t > STATUS_STALE_S:
      if self.mode == "probe" and now - self.mode_start_t >= PROBE_WINDOW_S:
        self._release("probe: no CAN", now)
      elif self.mode == "monitor" and now - self.last_active_t >= INACTIVE_RELEASE_S:
        self._release("no charge status on CAN", now)
      return

    dat = CarStateExt.update_car_state_bp(self.shim, self.cp, self.cp_cam)
    active = dat.carStateBP.charging.chargingActive

    if active:
      self.last_active_t = now
      if self.mode == "probe":
        self._wake("monitor", now)
    elif self.mode == "probe":
      if now - self.mode_start_t >= PROBE_WINDOW_S:
        self._release("probe: not charging", now)
      return  # don't publish an idle car from a probe

    if self.mode == "monitor":
      if now - self.last_active_t >= INACTIVE_RELEASE_S:
        self._release("charge ended", now)
        return
      if now - self.mode_start_t >= MAX_MONITOR_S:
        self._release("max monitor time", now)
        return

    if self.pm is None:
      self.pm = messaging.PubMaster(['carStateBP'])
    self.pm.send('carStateBP', dat)


def main():
  monitor = ChargeMonitor()
  monitor.params.put_bool("BPChargeCanAwake", False)
  rk = Ratekeeper(PUBLISH_HZ, print_delay_threshold=None)
  try:
    while True:
      monitor.step()
      rk.keep_time()
  finally:
    monitor.params.put_bool("BPChargeCanAwake", False)


if __name__ == "__main__":
  main()

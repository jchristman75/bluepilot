import struct

import usb1

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, get_usb_state, is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog

# BluePilot: turn the GPU's PCIe power off when parked. The Chestnut firmware boots with PCIe off and the
# model loader (tinygrad CustomASM24Controller) turns it on, but nothing turned it off again, so on an
# always-on 12 V feed the GPU and its fans ran at ~1.6 A (~20 W) offroad. Measured 2026-11-22: 1.64 A -> <0.1 A
# with 0xF3=0, and 0xF3=1 brings the link back to L0 in 0.5 s, which the loader does on its own next drive.
GPU_OFF_CURRENT_MA = 500  # above this the GPU is powered (idle on ~1.6 A, wedged ~1.4 A, off <0.1 A)


def should_power_off_gpu(idle: bool, already_sent: bool, msg) -> bool:
  return idle and not already_sent and msg.valid and msg.chestnutState.supplyCurrent > GPU_OFF_CURRENT_MA


def power_off_gpu(handle, current_ma: int) -> None:
  try:
    handle.controlWrite(0x40, 0xF3, 0, 0, b'', timeout=10000)
    cloudlog.event("chestnut gpu powered off while offroad", supply_current=current_ma)
  except usb1.USBError:
    cloudlog.exception("chestnut gpu power off failed")
# End BluePilot


def read_chestnut_state(handle, gpu_state=None):
  msg = messaging.new_message('chestnutState')
  if gpu_state is not None:
    msg.chestnutState = gpu_state
  state = msg.chestnutState
  try:
    raw = handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100)
    state.supplyVoltage, state.supplyCurrent, state.supplyFault = struct.unpack('<Hh?', bytes(raw))
    raw = handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100)
    state.pcieLtssm, = struct.unpack('B', bytes(raw))
    msg.valid = True
  except (usb1.USBError, struct.error):
    msg.valid = False
  return msg


def chestnut_state_thread(end_event, gpu_idle=None):  # BluePilot: gpu_idle
  pm = messaging.PubMaster(['chestnutState'])
  sm = messaging.SubMaster(['chestnutGpuState'])
  with usb1.USBContext() as context:
    handle = None
    gpu_off_sent = False  # BluePilot: once per offroad period, so a later manual power-on (compile) stays on
    try:
      while not end_event.is_set():
        if handle is None:
          devices = [d for d in get_usb_state() if is_chestnut_usb_id(d['vendorId'], d['productId']) and
                     d['product'] == CHESTNUT_USB_PRODUCT]
          if len(devices) == 1:
            try:
              handle = context.openByVendorIDAndProductID(devices[0]['vendorId'], devices[0]['productId'], skip_on_error=True)
            except usb1.USBError:
              pass
        if handle is not None:
          sm.update(0)
          gpu_valid = sm.alive['chestnutGpuState'] and sm.valid['chestnutGpuState']
          msg = read_chestnut_state(handle, sm['chestnutGpuState'] if gpu_valid else None)
          if not msg.valid:
            handle.close()
            handle = None
          msg.valid &= not sm.seen['chestnutGpuState'] or gpu_valid
          pm.send('chestnutState', msg)
          # BluePilot: power the GPU off once the car has been parked a while (see should_power_off_gpu)
          idle = gpu_idle is not None and gpu_idle.is_set()
          if should_power_off_gpu(idle, gpu_off_sent, msg):
            power_off_gpu(handle, msg.chestnutState.supplyCurrent)
            gpu_off_sent = True
          gpu_off_sent &= idle
          # End BluePilot
        end_event.wait(1 / SERVICE_LIST['chestnutState'].frequency if handle is not None else 1.)
    finally:
      if handle is not None:
        handle.close()

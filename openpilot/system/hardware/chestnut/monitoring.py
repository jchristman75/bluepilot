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
# A 0xF3=0 that lands while the firmware is still bringing the link up is ignored (2026-11-22: sent 0.3 s after
# power on, GPU stayed at 1.6 A), so wait for the draw to settle and re-check after each send.
GPU_OFF_CURRENT_MA = 500  # above this the GPU is powered (idle on ~1.6 A, wedged ~1.4 A, off <0.1 A)
GPU_OFF_SETTLE_FRAMES = 30  # 3 s of powered readings at 10 Hz before sending
GPU_OFF_RETRY_FRAMES = 50   # re-send if still powered 5 s later
GPU_OFF_MAX_ATTEMPTS = 3    # per offroad period, so a later manual power-on (compile) stays on


class GpuPowerOff:
  def __init__(self):
    self.reset()

  def reset(self):
    self.powered_frames = 0
    self.attempts = 0
    self.since_send = 0
    self.done = False  # an off took effect: leave any later power-on (manual compile) alone until the next drive

  def update(self, idle: bool, msg) -> bool:
    """Feed one chestnutState reading. Returns True when 0xF3=0 should be sent now."""
    if not idle:
      self.reset()
      return False
    if not msg.valid:
      return False
    self.since_send += 1
    if msg.chestnutState.supplyCurrent <= GPU_OFF_CURRENT_MA:
      self.powered_frames = 0
      self.done |= self.attempts > 0
      return False
    self.powered_frames += 1
    if self.done or self.attempts >= GPU_OFF_MAX_ATTEMPTS or self.powered_frames < GPU_OFF_SETTLE_FRAMES:
      return False
    if self.attempts > 0 and self.since_send < GPU_OFF_RETRY_FRAMES:
      return False
    self.attempts += 1
    self.since_send = 0
    return True


def power_off_gpu(handle, current_ma: int, attempt: int) -> None:
  try:
    handle.controlWrite(0x40, 0xF3, 0, 0, b'', timeout=10000)
    cloudlog.event("chestnut gpu powered off while offroad", supply_current=current_ma, attempt=attempt)
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
    gpu_power_off = GpuPowerOff()  # BluePilot
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
          # BluePilot: power the GPU off once the car has been parked a while (see GpuPowerOff)
          if gpu_power_off.update(gpu_idle is not None and gpu_idle.is_set(), msg):
            power_off_gpu(handle, msg.chestnutState.supplyCurrent, gpu_power_off.attempts)
          # End BluePilot
        end_event.wait(1 / SERVICE_LIST['chestnutState'].frequency if handle is not None else 1.)
    finally:
      if handle is not None:
        handle.close()

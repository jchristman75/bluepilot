# BluePilot: offroad eGPU PCIe power off (system/hardware/chestnut/monitoring.py)
import struct
import threading

import openpilot.cereal.messaging as messaging
from openpilot.system.hardware.chestnut import monitoring
from openpilot.system.hardware.chestnut.monitoring import should_power_off_gpu


def state(current_ma, valid=True):
  msg = messaging.new_message('chestnutState')
  msg.chestnutState.supplyCurrent = current_ma
  msg.valid = valid
  return msg


def test_gate():
  assert should_power_off_gpu(True, False, state(1640))
  assert should_power_off_gpu(True, False, state(1400))    # wedged link, still powered
  assert not should_power_off_gpu(False, False, state(1640))  # onroad / just parked
  assert not should_power_off_gpu(True, True, state(1640))    # already sent this offroad period
  assert not should_power_off_gpu(True, False, state(50))     # already off
  assert not should_power_off_gpu(True, False, state(1640, valid=False))


class FakeHandle:
  def __init__(self, end_event, frames, current_ma=1640):
    self.end_event, self.frames, self.current_ma, self.writes = end_event, frames, current_ma, []

  def controlRead(self, rtype, req, value, index, length, timeout):
    if req == 0xC0:
      self.frames -= 1
      if self.frames <= 0:
        self.end_event.set()
      return struct.pack('<Hh?', 12000, self.current_ma, False)
    return bytes([0x78 if self.current_ma > 500 else 0])

  def controlWrite(self, rtype, req, value, index, data, timeout):
    self.writes.append((rtype, req, value))
    if (req, value) == (0xF3, 0):
      self.current_ma = 50

  def close(self):
    pass


def run_thread(monkeypatch, idle):
  end_event = threading.Event()
  handle = FakeHandle(end_event, frames=5)

  class Ctx:
    def __enter__(self): return self
    def __exit__(self, *a): return False
    def openByVendorIDAndProductID(self, *a, **k): return handle

  monkeypatch.setattr(monitoring.usb1, "USBContext", Ctx)
  monkeypatch.setattr(monitoring, "get_usb_state", lambda: [{"vendorId": 0x3801, "productId": 0x0001,
                                                             "product": monitoring.CHESTNUT_USB_PRODUCT}])
  monkeypatch.setattr(end_event, "wait", lambda t=None: end_event.is_set())
  gpu_idle = threading.Event()
  if idle:
    gpu_idle.set()
  monitoring.chestnut_state_thread(end_event, gpu_idle)
  return handle


def test_thread_powers_off_once_when_idle(monkeypatch):
  handle = run_thread(monkeypatch, idle=True)
  assert handle.writes == [(0x40, 0xF3, 0)]
  assert handle.current_ma == 50


def test_thread_leaves_gpu_on_when_not_idle(monkeypatch):
  assert run_thread(monkeypatch, idle=False).writes == []

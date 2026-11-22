# BluePilot: offroad eGPU PCIe power off (system/hardware/chestnut/monitoring.py)
import struct
import threading

from openpilot.system.hardware.chestnut import monitoring
from openpilot.system.hardware.chestnut.monitoring import (GPU_OFF_MAX_ATTEMPTS, GPU_OFF_RETRY_FRAMES,
                                                           GPU_OFF_SETTLE_FRAMES, GpuPowerOff)


def feed(gate, n, current_ma, idle=True, valid=True):
  return [i for i in range(n) if gate.update(idle, valid, current_ma)]


def test_waits_for_settled_draw():
  gate = GpuPowerOff()
  assert feed(gate, GPU_OFF_SETTLE_FRAMES - 1, 1640) == []
  assert feed(gate, 1, 1640) == [0]


def test_not_idle_or_off_or_invalid():
  assert feed(GpuPowerOff(), 100, 1640, idle=False) == []
  assert feed(GpuPowerOff(), 100, 50) == []
  assert feed(GpuPowerOff(), 100, 1640, valid=False) == []


def test_wedged_gpu_is_powered_off():
  assert feed(GpuPowerOff(), GPU_OFF_SETTLE_FRAMES, 1400) == [GPU_OFF_SETTLE_FRAMES - 1]


def test_retries_when_ignored_then_gives_up():
  gate = GpuPowerOff()
  sends = feed(gate, GPU_OFF_SETTLE_FRAMES + GPU_OFF_RETRY_FRAMES * 10, 1640)
  assert len(sends) == GPU_OFF_MAX_ATTEMPTS
  assert sends[1] - sends[0] == GPU_OFF_RETRY_FRAMES


def test_later_manual_power_on_left_alone():
  gate = GpuPowerOff()
  assert len(feed(gate, GPU_OFF_SETTLE_FRAMES, 1640)) == 1
  feed(gate, 5, 50)  # the off took effect
  assert feed(gate, 500, 1640) == []  # e.g. compile_modeld's link_up


def test_new_offroad_period_rearms():
  gate = GpuPowerOff()
  feed(gate, GPU_OFF_SETTLE_FRAMES, 1640)
  feed(gate, 5, 50)
  feed(gate, 5, 1640, idle=False)  # drive
  assert len(feed(gate, GPU_OFF_SETTLE_FRAMES, 1640)) == 1


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


class StaleGpuStateSM:
  """chestnutGpuState was published during the drive (modeld) and is stale now -> chestnutState.valid False."""
  def __init__(self, *a, **k):
    self.alive, self.valid, self.seen = {'chestnutGpuState': False}, {'chestnutGpuState': False}, {'chestnutGpuState': True}

  def update(self, timeout=0):
    pass


def run_thread(monkeypatch, idle, frames=GPU_OFF_SETTLE_FRAMES + 20, after_drive=False):
  end_event = threading.Event()
  if after_drive:
    monkeypatch.setattr(monitoring.messaging, "SubMaster", StaleGpuStateSM)
  handle = FakeHandle(end_event, frames)

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


def test_thread_powers_off_after_a_drive(monkeypatch):
  handle = run_thread(monkeypatch, idle=True, after_drive=True)
  assert handle.writes == [(0x40, 0xF3, 0)]


def test_thread_leaves_gpu_on_when_not_idle(monkeypatch):
  assert run_thread(monkeypatch, idle=False).writes == []

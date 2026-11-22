# BluePilot: VBUS cycle a Chestnut that missed boot enumeration (system/hardware/chestnut/enum_recovery.py)
from openpilot.system.hardware.chestnut.enum_recovery import MAX_ATTEMPTS, MISSING_TIME, RETRY_INTERVAL, ChestnutEnumRecovery

CHESTNUT = [{"vendorId": 0x3801, "productId": 0x0001, "product": "custom", "speedMbps": 5000}]
ROM = [{"vendorId": 0x174C, "productId": 0x2464, "product": "", "speedMbps": 5000}]
MODEM = [{"vendorId": 0x2C7C, "productId": 0x6007, "product": "EG916Q-GL", "speedMbps": 480}]


class Clock:
  t = 0.

  def __call__(self):
    return self.t


def make(run=None):
  calls = []
  rec = ChestnutEnumRecovery(run=run or (lambda: calls.append(1)), clock=Clock())
  return rec, calls


def step(rec, t, usb=MODEM, offroad=True, cable=True, flashing=False):
  rec.clock.t = t
  started = rec.update(offroad, usb, cable, flashing)
  if rec.thread is not None:
    rec.thread.join()
  return started


def test_cycles_after_missing_time():
  rec, calls = make()
  assert not step(rec, 0.)
  assert not step(rec, MISSING_TIME - 1)
  assert step(rec, MISSING_TIME + 0.1)
  assert calls == [1]


def test_present_or_rom_bootloader_never_cycles():
  for usb in (CHESTNUT, ROM):
    rec, calls = make()
    for t in range(0, 200, 5):
      step(rec, float(t), usb=usb)
    assert calls == []


def test_no_cable_onroad_or_flashing_never_cycles():
  for kw in ({"cable": False}, {"offroad": False}, {"flashing": True}):
    rec, calls = make()
    for t in range(0, 200, 5):
      step(rec, float(t), **kw)
    assert calls == [], kw


def test_retries_then_gives_up():
  rec, calls = make()
  for t in range(600):
    step(rec, float(t))
  assert len(calls) == MAX_ATTEMPTS


def test_retry_waits_after_previous_cycle():
  rec, calls = make()
  step(rec, 0.)
  step(rec, MISSING_TIME + 1)
  assert not step(rec, MISSING_TIME + RETRY_INTERVAL)
  assert step(rec, MISSING_TIME + RETRY_INTERVAL + 2)


def test_recovered_resets_the_timer():
  rec, calls = make()
  step(rec, 0.)
  step(rec, MISSING_TIME + 1)  # cycle 1 -> recovered
  step(rec, MISSING_TIME + 2, usb=CHESTNUT)
  # lost again later: a fresh MISSING_TIME wait before the last attempt
  assert not step(rec, 100.)
  assert not step(rec, 100. + MISSING_TIME - 1)
  assert step(rec, 100. + MISSING_TIME + 0.1)
  assert len(calls) == 2


def test_runner_exception_is_contained():
  rec, _ = make(run=lambda: 1 / 0)
  step(rec, 0.)
  assert step(rec, MISSING_TIME + 1)
  assert not rec.cycling and rec.last_done > 0

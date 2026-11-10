"""eGPU link watchdog: offer a reboot only for a missing/slow bridge, after the grace period, parked, once."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.ui.bp.lib import egpu_link as el

PARK, DRIVE = el.GearShifter.park, el.GearShifter.drive


def usb(*devs, connected=True):
  return NS(connected=connected, devices=[NS(vendorId=v, productId=p, speedMbps=s) for v, p, s in devs])


GOOD = usb((0x3801, 0x0001, 5000), (0x2c7c, 0x6007, 480))
SLOW = usb((0x3801, 0x0001, 12))
MISSING = usb((0x2c7c, 0x6007, 480))
PARKED = NS(standstill=True, gearShifter=PARK)
MOVING = NS(standstill=False, gearShifter=DRIVE)


def test_problem_classification():
  assert el.egpu_link_problem(GOOD, True) is None
  assert el.egpu_link_problem(SLOW, True) == "slow (12 Mbps)"
  assert el.egpu_link_problem(MISSING, True) == "missing"
  assert el.egpu_link_problem(usb(connected=False), True) is None   # no eGPU plugged in
  assert el.egpu_link_problem(SLOW, False) is None                   # no big model to run
  assert el.egpu_link_problem(usb((0x174C, 0x2464, 480)), True) is None  # being flashed


class Clock:
  t = 0.0

  def __call__(self):
    return self.t


def run(w, clock, seconds, usb_state=SLOW, car=PARKED, active=None, started=True):
  offers = []
  end = clock.t + seconds
  while clock.t < end:
    if (p := w.update(started, usb_state, True, active, car)) is not None:
      offers.append((clock.t, p))
    clock.t += 0.5
  return offers


def test_offers_once_after_grace_and_hold():
  c = Clock()
  w = el.EgpuLinkWatchdog(c)
  offers = run(w, c, 60)
  assert len(offers) == 1
  assert offers[0][0] >= el.ONROAD_GRACE_S + el.BAD_HOLD_S


def test_never_offers_while_driving_but_offers_once_parked():
  c = Clock()
  w = el.EgpuLinkWatchdog(c)
  assert run(w, c, 120, car=MOVING) == []
  assert len(run(w, c, 5, car=PARKED)) == 1


def test_no_offer_when_link_good_or_big_model_running():
  c = Clock()
  w = el.EgpuLinkWatchdog(c)
  assert run(w, c, 120, usb_state=GOOD) == []
  assert run(w, c, 120, active=True) == []


def test_transient_glitch_is_ignored():
  c = Clock()
  w = el.EgpuLinkWatchdog(c)
  run(w, c, el.ONROAD_GRACE_S, usb_state=GOOD)
  assert run(w, c, el.BAD_HOLD_S - 2) == []
  assert run(w, c, 60, usb_state=GOOD) == []


def test_new_drive_offers_again():
  c = Clock()
  w = el.EgpuLinkWatchdog(c)
  assert len(run(w, c, 60)) == 1
  run(w, c, 5, started=False)
  assert len(run(w, c, 60)) == 1

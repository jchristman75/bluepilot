"""
BluePilot: catch an eGPU (chestnut) that came up wrong and offer the driver a reboot.

The eGPU's GPU is powered from the car's ignition. When the comma boots while the eGPU is plugged in
but unpowered (car off), its USB bridge initializes wrong: it is missing from the bus, or it enumerates
at 12/480 Mbps instead of SuperSpeed (5000 Mbps). modeld then can't load the big model and drives on
the small one. No software reset fixes it (xhci rebind leaves it at 12 Mbps; unbinding msm-dwc3
panics the kernel). A reboot with the car on does.

So once the car has been on long enough for the bridge to settle, if the USB-C cable is in, the big
model is compiled and isn't running, and the bridge is missing or slow, offer a reboot -- only while
parked, once per drive.
"""
import time
from collections.abc import Callable

from opendbc.car import structs
from openpilot.common.hardware.usb import CHESTNUT_ROM_USB_IDS, is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog

MIN_SPEED_MBPS = 5000   # SuperSpeed; anything slower can't run the big model
ONROAD_GRACE_S = 20.0   # bridge enumeration after ignition power + modeld's own wait
BAD_HOLD_S = 12.0       # hardwared refreshes usbState every 10 s (sooner on a topology change)

GearShifter = structs.CarState.GearShifter


def egpu_link_problem(usb_state, chestnut_compiled: bool) -> str | None:
  """'missing' / 'slow (N Mbps)' when the eGPU's cable is in but its bridge is not at SuperSpeed."""
  if not chestnut_compiled or not usb_state.connected:
    return None
  speeds = []
  for d in usb_state.devices:
    if (d.vendorId, d.productId) in CHESTNUT_ROM_USB_IDS:
      return None  # bridge in its ROM bootloader: hardwared is flashing it, leave it alone
    if is_chestnut_usb_id(d.vendorId, d.productId):
      speeds.append(d.speedMbps)
  if not speeds:
    return "missing"
  if max(speeds) < MIN_SPEED_MBPS:
    return f"slow ({max(speeds)} Mbps)"
  return None


class EgpuLinkWatchdog:
  """Decides when to offer the reboot. update() once per UI frame; returns the problem to offer."""

  def __init__(self, clock: Callable[[], float] = time.monotonic):
    self._clock = clock
    self._started = False
    self._started_t = 0.0
    self._bad_since: float | None = None
    self._logged = False
    self.offered = False

  def update(self, started: bool, usb_state, chestnut_compiled: bool, chestnut_active: bool | None,
             car_state) -> str | None:
    now = self._clock()
    if started != self._started:
      self._started, self._started_t = started, now
      self._bad_since, self._logged, self.offered = None, False, False
    if not started or now - self._started_t < ONROAD_GRACE_S:
      return None

    problem = None if chestnut_active else egpu_link_problem(usb_state, chestnut_compiled)
    if problem is None:
      self._bad_since = None
      return None
    if self._bad_since is None:
      self._bad_since = now
    if now - self._bad_since < BAD_HOLD_S:
      return None
    if not self._logged:
      self._logged = True
      cloudlog.warning(f"bp egpu link {problem}: offering reboot")
    if self.offered or not (car_state.standstill and car_state.gearShifter == GearShifter.park):
      return None
    self.offered = True
    return problem

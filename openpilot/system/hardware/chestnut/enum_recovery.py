# BluePilot: recover a Chestnut that did not enumerate at boot with a VBUS cycle.
#
# With the eGPU on an always-on 12 V feed, the comma sometimes boots with the Chestnut's USB bridge not showing up
# at all: a600000.ssusb drops to low power ~9 s after boot and never sees it, so modeld falls back to the small
# model for the whole boot. 2026-11-22: 2 of 3 reboots did this, and a VBUS cycle (flash.py vbus_cycle, the same
# one comma's flasher uses) brought it back at 5000 Mbps both times with no reboot. Never unbind msm-dwc3 or
# xhci instead (kernel panic / link stuck at 12 Mbps).
import os
import subprocess
import sys
import threading
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware.usb import is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog

MISSING_TIME = 20.    # seconds offroad with the cable in and no Chestnut on USB before the first cycle
RETRY_INTERVAL = 30.  # seconds after a cycle finishes before trying again
MAX_ATTEMPTS = 2      # per boot


class ChestnutEnumRecovery:
  def __init__(self, run=None, clock=time.monotonic):
    self.run = run or self.vbus_cycle
    self.clock = clock
    self.thread: threading.Thread | None = None
    self.attempts = 0
    self.missing_since: float | None = None
    self.last_done = 0.

  @property
  def cycling(self) -> bool:
    return self.thread is not None and self.thread.is_alive()

  def vbus_cycle(self) -> None:
    ret = subprocess.run(["sudo", sys.executable, os.path.join(BASEDIR, "openpilot/system/hardware/chestnut/flash.py"), "--vbus-cycle"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=60)
    cloudlog.event("chestnut vbus cycle done", attempt=self.attempts, returncode=ret.returncode, output=ret.stdout[-500:],
                   error=ret.returncode != 0)

  def _run(self) -> None:
    try:
      self.run()
    except Exception:
      cloudlog.exception("chestnut vbus cycle failed")
    finally:
      self.last_done = self.clock()

  def update(self, offroad: bool, usb_state: list[dict], cable: bool, flashing: bool) -> bool:
    """Returns True when a VBUS cycle was started."""
    now = self.clock()
    present = any(is_chestnut_usb_id(d["vendorId"], d["productId"], include_bootloader=True) for d in usb_state)
    if present or not offroad or not cable or flashing or self.cycling:
      if not self.cycling:
        self.missing_since = None
      return False
    if self.missing_since is None:
      self.missing_since = now
    if self.attempts >= MAX_ATTEMPTS or now - self.missing_since < MISSING_TIME:
      return False
    if self.attempts > 0 and now - self.last_done < RETRY_INTERVAL:
      return False
    self.attempts += 1
    cloudlog.warning(f"chestnut not enumerated with the cable connected, cycling VBUS (attempt {self.attempts})")
    self.thread = threading.Thread(target=self._run, daemon=True)
    self.thread.start()
    return True

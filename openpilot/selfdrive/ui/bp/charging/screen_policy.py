"""
BluePilot: how long the MICI charging screen keeps the display on.

The screen stays up for as long as you want to watch a charge, which is why it overrides the device's interactive
timeout with a huge value. That is right with the car on. With the car off (the comma is kept awake by the charge
monitor, or woken by plugging in) it left the display lit for hours: burn-in. So the hold applies only while the
ignition is on; with the car off the screen gets a short timeout, and a touch still wakes it for that long.

The sunnypilot screensaver has to be skipped too: with ScreenSaverEnabled the device pushes a bouncing-logo screen in
place of turning the display off, so the screen stays lit for ScreenSaverTimeout seconds. The charging screen asks the
Device to suppress it while the car is off.

Pure logic, no drawing, so it is unit-testable.
"""
from typing import NamedTuple

STAY_AWAKE_TIMEOUT_S = 99999    # car on: no "never timeout" sentinel exists in Device, so a large number
CAR_OFF_TIMEOUT_S = 5           # car off: the screen sleeps this long after the car turns off or after a touch


class ScreenPolicy(NamedTuple):
  timeout_s: int
  suppress_screensaver: bool


def screen_policy(ignition: bool) -> ScreenPolicy:
  return ScreenPolicy(STAY_AWAKE_TIMEOUT_S, False) if ignition else ScreenPolicy(CAR_OFF_TIMEOUT_S, True)


class ChargingScreenHold:
  """Feed it the ignition every frame; it returns a policy to apply only when that changes. Re-applying every frame
  would reset the device's countdown each time and the screen would never time out."""

  def __init__(self):
    self._applied: ScreenPolicy | None = None

  def reset(self) -> None:
    self._applied = None

  def update(self, ignition: bool) -> ScreenPolicy | None:
    policy = screen_policy(ignition)
    if policy == self._applied:
      return None
    self._applied = policy
    return policy

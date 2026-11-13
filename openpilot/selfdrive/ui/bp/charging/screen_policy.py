"""
BluePilot: when the display must really be turned off while the charging screen is showing.

The MICI charging screen holds the display on for as long as you watch a charge. With the car off that left the display
lit for hours (burn-in), either through the hold itself or through the sunnypilot screensaver, which the device pushes in
place of turning the display off (ScreenSaverEnabled). So with the car off and the charging screen open, the Device
uses a short timeout and skips the screensaver. Touching the screen still shows it for that long.

When the charging screen is not showing, nothing changes: the normal device timeout and screensaver apply, on purpose.
The comma itself keeps running: this only concerns the display.

Pure logic, no drawing, so it is unit-testable.
"""

STAY_AWAKE_TIMEOUT_S = 99999    # car on: the charging screen holds the display (no "never timeout" sentinel exists)
CAR_OFF_TIMEOUT_S = 5           # car off: the display sleeps this long after the car turns off or after a touch


def screen_off_with_car_off(ignition: bool, charging_screen_open: bool) -> bool:
  return not ignition and charging_screen_open


class CarOffScreenRule:
  """Tracks the rule; update() returns True only on the frame it changes, so the Device resets its countdown once
  (re-applying every frame would keep resetting it and the screen would never time out)."""

  def __init__(self):
    self.active = False

  def update(self, ignition: bool, charging_screen_open: bool) -> bool:
    active = screen_off_with_car_off(ignition, charging_screen_open)
    changed = active != self.active
    self.active = active
    return changed

"""
BluePilot: hardwared's power-save decision, aware of the offroad charge monitor.

Stock hardwared puts the device into power save (big CPU cores offline, slower governor) whenever the ignition is
off and the screen is dark. The charging screen now lets the display time out once the car is turned off, to avoid
burn-in (ui/bp/charging/screen_policy.py), but bp_chargemon is still decoding the charge for the session history and
the comma has to keep running exactly as it did while the screen was lit. So power save stays off while the monitor
holds the CAN awake (BPChargeCanAwake); it follows the stock rule otherwise.
"""


def should_power_save(ignition: bool, screen_brightness_pct: float, charge_monitor_active: bool) -> bool:
  return not ignition and screen_brightness_pct < 1e-3 and not charge_monitor_active

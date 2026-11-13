"""
BluePilot: when bp_chargemon keeps the comma awake for a charge, and when it lets go.

The keep-alive exists to track a DC fast charge after the car is turned off. It is deliberately narrow:
  - DC fast charging only (carStateBP.charging.dcFastCharging). A Level 1/2 charge runs for hours and the comma
    does not need to stay up for it.
  - At most one hour, which covers a DC session with room to spare.
Pure logic with no messaging, so it is unit-testable.
"""

ONROAD_DC_WINDOW_S = 10.0     # DC fast charging this recently when the ignition dropped -> keep the comma awake
INACTIVE_RELEASE_S = 60.0     # DC charging inactive this long -> release the CAN (the UI's own grace is 15 s)
MAX_MONITOR_S = 3600.0        # hard cap: one hour


def keep_alive_on_ignition_drop(now: float, last_onroad_dc_t: float) -> bool:
  return now - last_onroad_dc_t < ONROAD_DC_WINDOW_S


def release_reason(now: float, mode_start_t: float, last_active_t: float) -> str | None:
  """Why the monitor should stop, or None. "Active" means DC fast charging was seen."""
  if now - last_active_t >= INACTIVE_RELEASE_S:
    return "charge ended"
  if now - mode_start_t >= MAX_MONITOR_S:
    return "max monitor time"
  return None


def session_active(mode: str, charging_active: bool, dc_fast: bool) -> bool:
  """Is the charge still the one to keep tracking? A probe (looking for a charge) must see DC fast charging. A monitor
  was entered for a DC fast charge already, so it only needs the charge itself to stay active: the car-off gateway may
  not forward the 0x440 status that says "DC", and losing it must not end the tracking."""
  if not charging_active:
    return False
  return dc_fast if mode == "probe" else True

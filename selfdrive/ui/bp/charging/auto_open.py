"""
BluePilot charge-session auto-open.

Pops the charging UI up on its own the moment a charge session starts — including the
case where the comma boots with the car already charging, since the first confirmed
active reading is a session start like any other (see ChargeSessionHistory).

Driven from the ui.py main loop rather than from a widget or a gui_app nav-stack tick,
because both of those only run while the screen is rendering: plugging in with the
display timed out has to be seen too. Opening the charging view sets the device's
interactive-timeout override, which wakes the screen on the next device.update().

Each UI registers how it shows the screen (tici: the sidebar overlay panel; mici: a
pushed NavWidget), so nothing here knows about either layout.
"""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.charging.session import charge_session_history, is_in_drive


class ChargingAutoOpen:
  def __init__(self):
    self._opener: Callable[[], None] | None = None

  def set_opener(self, opener: Callable[[], None] | None) -> None:
    self._opener = opener

  def update(self) -> None:
    """Call once per main-loop iteration, screen on or off."""
    charge_session_history.update()

    if not charge_session_history.consume_session_start():
      return

    # Already driving: the charging views auto-dismiss on Drive anyway, so never take
    # the screen away from the road view for a session that starts underway.
    if is_in_drive():
      return

    if self._opener is not None:
      self._opener()


charging_auto_open = ChargingAutoOpen()

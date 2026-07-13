"""
BluePilot charge-session history.

Tracks a rolling buffer of (elapsed_seconds, soc_percent, power_kw) samples for the
current charge session, shared by every UI surface (mici full page, tici sidebar
cards) so the curve is consistent no matter which screen is open.

A "session" starts the moment carStateBP.charging.chargingActive is confirmed True
(see CHARGING_INACTIVE_GRACE_S below), and the buffer is cleared on that transition
so old sessions never bleed into a new one.
"""

import time

from opendbc.car import structs
from openpilot.selfdrive.ui.ui_state import ui_state

SAMPLE_INTERVAL_S = 1.0
MIN_SAMPLES_FOR_ESTIMATE = 15  # ~15s of data before trusting an extrapolation
ESTIMATE_WINDOW_S = 120.0  # only use the most recent N seconds of samples for the rate estimate

# chargingActive can briefly read False mid-session (current renegotiation, thermal
# throttling, taper-related cycling near full, etc.) without the car actually being
# unplugged. Require it to stay False for this long before treating the session as
# over - otherwise a single blip wipes the accumulated curve and closes any charging
# view that auto-dismisses on "session ended".
CHARGING_INACTIVE_GRACE_S = 15.0


class ChargeSessionHistory:
  _instance: 'ChargeSessionHistory | None' = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super().__new__(cls)
      cls._instance._initialize()
    return cls._instance

  def _initialize(self):
    self._samples: list[tuple[float, float, float]] = []  # (elapsed_s, soc_pct, power_kw)
    self._session_start_time: float | None = None
    self._last_sample_time: float = 0.0
    self._was_active = False
    self._inactive_since: float | None = None  # monotonic time chargingActive first read False

  def update(self):
    """Call once per UI frame from any widget that wants fresh data; internally throttled."""
    try:
      sm = ui_state.sm
      if "carStateBP" not in sm.recv_frame:
        return
      car_state_bp = sm['carStateBP']
      charging = car_state_bp.charging
      hybrid_battery = car_state_bp.hybridBattery
      if not charging.dataAvailable:
        return
    except (KeyError, AttributeError, TypeError):
      return

    raw_active = charging.chargingActive
    now = time.monotonic()

    if raw_active:
      self._inactive_since = None
      # New session: only clear the buffer on a *confirmed* start (was truly
      # inactive before), not on every raw-active reading.
      if not self._was_active:
        self._samples = []
        self._session_start_time = now
        self._last_sample_time = 0.0
        self._was_active = True
    elif self._was_active:
      # Still officially "active" until the False reading has persisted long
      # enough to rule out a momentary blip.
      if self._inactive_since is None:
        self._inactive_since = now
      elif now - self._inactive_since >= CHARGING_INACTIVE_GRACE_S:
        self._was_active = False
        self._inactive_since = None

    if not self._was_active:
      return

    # Don't fabricate a sample during a momentary drop - just skip this poll,
    # leaving a small gap in the curve rather than a fake reading.
    if not raw_active:
      return

    elapsed = now - self._session_start_time
    if elapsed - self._last_sample_time < SAMPLE_INTERVAL_S:
      return
    self._last_sample_time = elapsed

    soc = hybrid_battery.socActual if hybrid_battery.dataAvailable else 0.0
    self._samples.append((elapsed, soc, charging.powerKw))

  @property
  def samples(self) -> list[tuple[float, float, float]]:
    return self._samples

  @property
  def is_active(self) -> bool:
    return self._was_active

  def estimate_minutes_to(self, target_pct: float = 80.0) -> float | None:
    """Extrapolate minutes remaining to reach target_pct from the recent SOC trend.

    Returns None if there isn't enough recent data yet, or the target has already
    been reached/passed.
    """
    if len(self._samples) < MIN_SAMPLES_FOR_ESTIMATE:
      return None

    latest_elapsed, latest_soc, _ = self._samples[-1]
    if latest_soc >= target_pct:
      return None

    window_start = latest_elapsed - ESTIMATE_WINDOW_S
    window = [s for s in self._samples if s[0] >= window_start]
    if len(window) < MIN_SAMPLES_FOR_ESTIMATE:
      window = self._samples

    t0, soc0, _ = window[0]
    dt = latest_elapsed - t0
    d_soc = latest_soc - soc0
    if dt <= 0 or d_soc <= 0:
      return None

    rate_pct_per_s = d_soc / dt
    remaining_pct = target_pct - latest_soc
    return (remaining_pct / rate_pct_per_s) / 60.0


charge_session_history = ChargeSessionHistory()


def is_in_drive() -> bool:
  """True once the car has been shifted into Drive (used to auto-dismiss charging views)."""
  try:
    return ui_state.sm['carState'].gearShifter == structs.CarState.GearShifter.drive
  except (KeyError, AttributeError, TypeError):
    return False

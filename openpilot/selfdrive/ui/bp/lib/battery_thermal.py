"""
BluePilot: logic for the battery temperature icon (BPShowBatteryTemp), kept free of drawing so it is unit-testable.

The icon shows the HV pack temperature (carStateBP.hybridBattery.packTempC). An arrow shows which way the battery
coolant loop is pushing it: coolant warmer than the pack heats it (up), cooler cools it (down). The coolant is
compared with the pack, not with its own history, because that is what drives the heat flow. Fleet data (Mach-E, 315
segments) has the coolant within about 1 C of the pack in warm weather and 4+ C above a cold pack when the loop heats
it; a DC fast charge pulled it 18 C below a 36 C pack.
"""
from enum import IntEnum

from openpilot.common.filter_simple import FirstOrderFilter

ENTER_C = 3.0           # |coolant - pack| above this turns an arrow on
EXIT_C = 1.5            # ...and it stays on until the gap falls below this
FILTER_TAU_S = 2.0      # the gap is smoothed so a blip does not flip the arrow

NO_DATA_COLOR = (150, 150, 150)
# (upper bound in C, color): cold blue ... green ... amber ... red
COLOR_BANDS = (
  (8.0, (96, 165, 255)),
  (16.0, (80, 215, 235)),
  (34.0, (70, 225, 120)),
  (41.0, (255, 190, 60)),
  (999.0, (255, 85, 70)),
)
# Pack temperature range marked "good for charging" with a green outline. Ford publishes no figure: it says DC charging
# slows below 10 C (50 F), that the pack needs to warm up before it accepts full power, and that the car limits charge
# power by pack temperature. 25-40 C is the usual lithium-ion sweet spot (an owner reports ~100 F = 38 C once warm).
# Edges have 1 C of hysteresis because the pack reads in 0.5 C steps.
OPTIMAL_LOW_C = 25.0
OPTIMAL_HIGH_C = 40.0
OPTIMAL_HYSTERESIS_C = 1.0
NORMAL_OUTLINE = (245, 245, 245)
OPTIMAL_OUTLINE = (70, 235, 110)
FILL_RANGE_C = (-10.0, 50.0)
FILL_MIN = 0.08


class Trend(IntEnum):
  STEADY = 0
  HEATING = 1     # coolant warmer than the pack
  COOLING = 2     # coolant cooler than the pack


class BatteryThermal:
  def __init__(self):
    self.trend = Trend.STEADY
    self.optimal = False        # the pack is in the OPTIMAL_LOW_C..OPTIMAL_HIGH_C charging range
    self._gap: FirstOrderFilter | None = None

  def reset(self) -> None:
    self.trend = Trend.STEADY
    self.optimal = False
    self._gap = None

  def _update_optimal(self, pack_c: float) -> None:
    lo, hi, h = OPTIMAL_LOW_C, OPTIMAL_HIGH_C, OPTIMAL_HYSTERESIS_C
    if self.optimal:
      self.optimal = lo - h <= pack_c <= hi + h
    else:
      self.optimal = lo <= pack_c <= hi

  def update(self, pack_c: float | None, coolant_c: float | None, dt: float) -> Trend:
    """One UI frame. pack_c / coolant_c are None when the car doesn't report them."""
    if pack_c is None or coolant_c is None:
      self.reset()
      return self.trend
    self._update_optimal(pack_c)
    raw = coolant_c - pack_c
    if self._gap is None:
      self._gap = FirstOrderFilter(0.0, FILTER_TAU_S, dt)    # starts at "no gap" so a first frame can't flash an arrow
    gap = self._gap.update(raw)

    if self.trend == Trend.STEADY:
      if gap >= ENTER_C:
        self.trend = Trend.HEATING
      elif gap <= -ENTER_C:
        self.trend = Trend.COOLING
    elif self.trend == Trend.HEATING:
      if gap <= -ENTER_C:
        self.trend = Trend.COOLING
      elif gap < EXIT_C:
        self.trend = Trend.STEADY
    else:
      if gap >= ENTER_C:
        self.trend = Trend.HEATING
      elif gap > -EXIT_C:
        self.trend = Trend.STEADY
    return self.trend


def temp_text(pack_c: float | None) -> str:
  """The number shown in the icon: whole degrees Celsius (always, whatever the units setting says)."""
  if pack_c is None:
    return "--"
  return str(round(pack_c))


def temp_color(pack_c: float | None) -> tuple[int, int, int]:
  if pack_c is None:
    return NO_DATA_COLOR
  for upper, color in COLOR_BANDS:
    if pack_c < upper:
      return color
  return COLOR_BANDS[-1][1]


def outline_color(optimal: bool) -> tuple[int, int, int]:
  return OPTIMAL_OUTLINE if optimal else NORMAL_OUTLINE


def fill_fraction(pack_c: float) -> float:
  """How full the battery body is drawn: the working range maps to 0..1 with a visible minimum."""
  lo, hi = FILL_RANGE_C
  return min(max((pack_c - lo) / (hi - lo), FILL_MIN), 1.0)


def telemetry_from(sm, started_frame: int) -> tuple[float, float] | None:
  """(pack_c, coolant_c) from a live carStateBP of the current drive, or None."""
  try:
    if not sm.alive.get("carStateBP", False) or not sm.valid.get("carStateBP", False):
      return None
    if sm.recv_frame.get("carStateBP", -1) < started_frame:
      return None
    hb = sm["carStateBP"].hybridBattery
    if not hb.thermalDataAvailable:
      return None
    return float(hb.packTempC), float(hb.coolantTempC)
  except (KeyError, AttributeError):
    return None


def should_show(enabled: bool, telemetry: tuple[float, float] | None, hud_busy: bool) -> bool:
  """The icon shows when the toggle is on, there is live telemetry, and the HUD isn't drawing the set-speed block over
  the same corner (the driver-monitoring ring yields to it the same way)."""
  return enabled and telemetry is not None and not hud_busy

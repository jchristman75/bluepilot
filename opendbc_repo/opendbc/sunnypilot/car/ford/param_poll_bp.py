"""
BluePilot: throttle for the Ford CarController's runtime param reads.

CarController.update() used to read about 24 Params from disk every frame (lateral curvature 10, angle tuning 7,
longitudinal 5, HUD 1, plus the BP lateral bypass) at 100 Hz, roughly 2,400 file reads a second inside `card`,
which runs on the core that the hardwared-stall notes show saturated. None of these is read-after-write critical:
they are UI toggles and tuning values, and nothing else mutates the attributes they set. Reading them at 2 Hz
cuts the disk work 50x; a changed setting takes effect within half a second.
"""

PARAM_POLL_FRAMES = 50   # 2 Hz at the 100 Hz control rate


class ParamPoll:
  def __init__(self, period: int = PARAM_POLL_FRAMES):
    self.period = period
    self.last: int | None = None

  def due(self, frame: int) -> bool:
    """True on the first call, once per period afterwards, and whenever the frame counter goes backwards."""
    if self.last is None or frame < self.last or frame - self.last >= self.period:
      self.last = frame
      return True
    return False

  def force(self) -> None:
    """Poll on the next call (after a known settings change)."""
    self.last = None

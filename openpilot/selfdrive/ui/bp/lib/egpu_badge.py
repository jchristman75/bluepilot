"""BluePilot: which eGPU badge the MICI onroad view shows (egpu_watts_icon.py). Pure, so it is unit-testable.

States are the values of ui_state.ChestnutState."""


def badge_mode(state: str, watts_enabled: bool) -> str | None:
  """'watts' (big model running), 'loading', 'failed' (the big model tried to load and failed, so the small model is
  driving), or None. A failure shows even with the wattage badge turned off."""
  if state == "failed":
    return "failed"
  if not watts_enabled:
    return None
  if state == "active":
    return "watts"
  if state == "loading":
    return "loading"
  return None

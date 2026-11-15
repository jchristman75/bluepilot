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


# the stock chestnut status code (system/hardware/chestnut/status.py) treats the supply as powered at 5 V and above
POWERED_MV = 5000
PCIE_L0 = 0x78          # LTSSM value for a trained PCIe link


def failure_reason(present: bool, supply_mv: int, supply_fault: bool, ltssm: int, state_alive: bool) -> str:
  """A short label for why the big model failed, from what the eGPU board reports (routes 574-582: a bridge that never
  enumerated, a board reading 0 V at its input with the car's 12 V fine, a powered board whose GPU link was down).
  Short enough to sit under the icon."""
  if not present:
    return "NO USB"
  if not state_alive:
    return "FAILED"
  if supply_fault or supply_mv < POWERED_MV:
    return "NO 12V"
  if ltssm != PCIE_L0:
    return "GPU LINK"
  return "MODEL"

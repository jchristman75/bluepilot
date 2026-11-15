"""Which eGPU badge the MICI onroad view shows: the watts while the big model runs on the eGPU, the pulsing logo while it
loads, and the orange failure logo (the one on the home screen) when the big model tried to load and failed."""
import pytest

from openpilot.selfdrive.ui.bp.lib import egpu_badge as eb


@pytest.mark.parametrize("state,watts_enabled,expected", [
  ("active", True, "watts"),
  ("active", False, None),               # the watts badge follows its own toggle
  ("loading", True, "loading"),
  ("loading", False, None),
  ("failed", True, "failed"),
  ("failed", False, "failed"),           # a failure is shown whatever the wattage toggle says: it means the small model is driving
  ("ready", True, None),
  ("uncompiled", True, None),
  ("disconnected", True, None),          # no eGPU connected: nothing to report
])
def test_the_badge_for_each_state(state, watts_enabled, expected):
  assert eb.badge_mode(state, watts_enabled) == expected


def test_failed_and_loading_never_show_the_watts():
  assert eb.badge_mode("failed", True) != "watts" and eb.badge_mode("loading", True) != "watts"


# ---- why it failed ----

PCIE_L0 = 0x78


@pytest.mark.parametrize("kw,expected", [
  (dict(present=False), "NO USB"),                                                       # the bridge isn't enumerated (routes 574-577)
  (dict(present=True, supply_mv=0, supply_fault=False, ltssm=0), "NO 12V"),              # bridge there, 0 V at the board (routes 57b-57e)
  (dict(present=True, supply_mv=14000, supply_fault=True, ltssm=PCIE_L0), "NO 12V"),     # a supply fault counts as power lost
  (dict(present=True, supply_mv=4999, supply_fault=False, ltssm=0), "NO 12V"),           # just under the stock "powered" threshold
  (dict(present=True, supply_mv=13000, supply_fault=False, ltssm=0), "GPU LINK"),        # powered but the PCIe link to the GPU is down
  (dict(present=True, supply_mv=13000, supply_fault=False, ltssm=PCIE_L0), "MODEL"),     # powered and linked: the model itself failed
])
def test_the_reason_for_a_failed_big_model(kw, expected):
  base = dict(present=True, supply_mv=13000, supply_fault=False, ltssm=PCIE_L0, state_alive=True)
  base.update(kw)
  assert eb.failure_reason(**base) == expected


def test_without_chestnut_state_data_the_reason_is_generic_not_a_guess():
  assert eb.failure_reason(present=True, supply_mv=0, supply_fault=False, ltssm=0, state_alive=False) == "FAILED"
  assert eb.failure_reason(present=False, supply_mv=0, supply_fault=False, ltssm=0, state_alive=False) == "NO USB"


def test_reasons_are_short_enough_for_the_badge():
  for r in ("NO USB", "NO 12V", "GPU LINK", "MODEL", "FAILED"):
    assert len(r) <= 8

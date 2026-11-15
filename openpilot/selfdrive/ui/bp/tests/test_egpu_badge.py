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

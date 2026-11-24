import math

import pytest

from openpilot.cereal import log
from openpilot.bluepilot.selfdrive.controls import follow_time_bp as F
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc

P = log.LongitudinalPersonality


def test_clamp_keeps_values_in_range():
  assert F.clamp_t_follow(1.25) == 1.25
  assert F.clamp_t_follow(0.2) == F.MIN_T_FOLLOW
  assert F.clamp_t_follow(9.0) == F.MAX_T_FOLLOW


@pytest.mark.parametrize("bad", [None, "", b"abc", math.nan, math.inf])
def test_clamp_falls_back_to_default_on_bad_values(bad):
  assert F.clamp_t_follow(bad) == F.DEFAULT_T_FOLLOW


def test_standard_follow_time_comes_from_the_module_value(monkeypatch):
  monkeypatch.setattr(long_mpc, "STANDARD_T_FOLLOW", 1.4)
  assert long_mpc.get_T_FOLLOW(P.standard) == 1.4
  assert long_mpc.get_T_FOLLOW(P.aggressive) == 1.25
  assert long_mpc.get_T_FOLLOW(P.relaxed) == 1.75

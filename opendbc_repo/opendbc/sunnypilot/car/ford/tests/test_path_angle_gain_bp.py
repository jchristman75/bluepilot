"""BluePilot: the Mach-E PSCM's own path-angle gain curve (path_angle_gain_bp.py)."""
from types import SimpleNamespace

import pytest

from opendbc.sunnypilot.car.ford.path_angle_gain_bp import MACHE_2021, gain_curve_for, pscm_strategy


def _fw(strategy: bytes, addr: int = 0x730, request: bytes = b"\x22\xf1\x88"):
  return SimpleNamespace(address=addr, brand="mazda", request=[request], fwVersion=strategy.ljust(24, b"\x00"))


def _cp(fp="FORD_MUSTANG_MACH_E_MK1", strategy=b"LJ9C-14D003-GA"):
  return SimpleNamespace(carFingerprint=fp, carFw=[_fw(strategy)])


class TestPathAngleGainCurve:
  def test_k2_is_4_over_d_on_the_table(self):
    # the module's K2 table is ~4/d between 10 and 150 m
    for d, k2 in zip(MACHE_2021.k2_distances, MACHE_2021.k2, strict=True):
      assert k2 == pytest.approx(4.0 / d, rel=3e-3)

  def test_gain_is_flat_across_town_and_highway_speeds(self):
    # T = 5 s, d = 0.28 * v_kph * T inside the distance clamps: K = 1/(K2 v) ~ T / 4 = 1.26
    for v in (7.0, 10.0, 15.0, 20.0, 25.0):
      assert MACHE_2021.curvature_factor(v) == pytest.approx(1.26, abs=0.06)  # the table is read straight between its points

  def test_low_speed_distance_clamp(self):
    # below 30 m the lower clamp holds d at 30 m: K2 = 0.1333, so path angle = 7.5 * kappa whatever the speed
    for v in (2.0, 4.0, 5.0):
      assert MACHE_2021.preview_distance(v, 5.0) == 30.0
      assert MACHE_2021.path_angle(0.01, v) == pytest.approx(0.075, rel=1e-3)

  def test_high_speed_upper_clamp(self):
    # past 150 m the upper clamp holds d: K2 = 0.0266
    assert MACHE_2021.preview_distance(35.0, 5.0) == 150.0
    assert MACHE_2021.k2_at(35.0) == pytest.approx(0.0266)

  def test_preview_time_shortens_with_c2_curvature(self):
    assert MACHE_2021.preview_time(0.0) == 5.0
    assert MACHE_2021.preview_time(0.004) == pytest.approx(4.0)
    assert MACHE_2021.preview_time(0.05) == 3.5  # floor

  def test_path_angle_inverts_the_module(self):
    for v in (6.0, 12.0, 22.0, 30.0):
      kappa = 0.004
      assert MACHE_2021.k2_at(v) * MACHE_2021.path_angle(kappa, v) == pytest.approx(kappa)


class TestGainCurveFor:
  def test_mache_2021_22_strategies(self):
    assert gain_curve_for(_cp(strategy=b"LJ9C-14D003-GA")) is MACHE_2021
    assert gain_curve_for(_cp(strategy=b"LJ9C-14D003-HA")) is MACHE_2021

  def test_other_strategy_keeps_the_hand_tuned_gain(self):
    assert gain_curve_for(_cp(strategy=b"LJ9C-14D003-FA")) is None  # 2023-24 Mach-E: no preview table
    assert gain_curve_for(_cp(strategy=b"")) is None

  def test_other_platform_keeps_the_hand_tuned_gain(self):
    assert gain_curve_for(_cp(fp="FORD_ESCAPE_MK4_5")) is None

  def test_only_f188_is_read(self):
    cp = SimpleNamespace(carFingerprint="FORD_MUSTANG_MACH_E_MK1",
                         carFw=[_fw(b"LJ9C-14D003-GA", request=b"\x22\xf1\x0a"), _fw(b"LJ9C-14D003-GA", addr=0x7e0)])
    assert pscm_strategy(cp.carFw) == ""
    assert gain_curve_for(cp) is None

"""Golden test: the Ford longitudinal path must keep sending the same ACCDATA bytes.

See long_harness.py for the scenarios. A failure here means a change altered what reaches
the car. If that was intended, regenerate the fixture and explain the change in the commit.
"""
import numpy as np
import pytest

from opendbc.sunnypilot.car.ford.tests import long_harness as H


@pytest.fixture(scope='module')
def golden():
  with np.load(H.GOLDEN_PATH) as z:
    return {k: z[k] for k in z.files}


@pytest.fixture(scope='module')
def current():
  return H.run_all()


def _first_diff(name, exp, got):
  rows = np.where((exp != got).any(axis=1))[0]
  i = int(rows[0])
  de, dg = H.decode(exp[i:i + 1]), H.decode(got[i:i + 1])
  sigs = {k: (float(de[k][0]), float(dg[k][0])) for k in de if de[k][0] != dg[k][0]}
  return f'{name}: {len(rows)} ACCDATA frames differ, first at frame {i}: (golden, now) {sigs}'


def test_scenarios_match_fixture(golden, current):
  assert set(current) == {k[:-5] for k in golden if k.endswith('__acc')}, 'scenario list changed: regenerate the fixture'


def test_accdata_bytes_unchanged(golden, current):
  diffs = []
  for name, (acc, _bp) in current.items():
    exp = golden[f'{name}__acc']
    assert acc.shape == exp.shape, f'{name}: {len(acc)} frames sent, golden has {len(exp)}'
    if not np.array_equal(acc, exp):
      diffs.append(_first_diff(name, exp, acc))
  assert not diffs, '\n'.join(diffs)


def test_scenarios_exercise_every_path(golden):
  """Guard the fixture itself: it must cover the paths the refactors touch."""
  acc = np.concatenate([golden[k] for k in golden if k.endswith('__acc')])
  bp = np.concatenate([golden[k] for k in golden if k.endswith('__bp')])
  d = H.decode(acc)
  brake = d['AccBrkDecel_B_Rq'] > 0.5
  inactive = d['AccPrpl_A_Rq'] < -4.9
  assert bp.sum() > 5000                           # BP long follow path
  assert brake.sum() > 5000 and (bp & brake).sum() > 500
  assert ((d['AccPrpl_A_Rq'] < 0) & ~inactive).sum() > 5000   # regen gas
  assert inactive.sum() > 5000                     # inactive gas
  assert (d['AccPrpl_A_Pred'] > -4.9).sum() > 5000  # mirrored pred (regen)


def _panda_accepts(acc_frames, regen_flag):
  """Run ACCDATA frames through the compiled panda safety (ford.h), CAN FD + long control."""
  from opendbc.car.ford.values import FordSafetyFlags
  from opendbc.car.structs import CarParams
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsBP
  safety = libsafety_py.libsafety
  safety.set_current_safety_param_sp(FordSafetyFlagsBP.REGEN_BLEND if regen_flag else 0)
  safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD)
  safety.init_tests()
  safety.set_controls_allowed(True)
  ok = np.array([safety.safety_tx_hook(libsafety_py.make_CANPacket(H.ACCDATA, 0, bytes(f))) for f in acc_frames])
  safety.set_current_safety_param_sp(0)
  return ok


def test_panda_accepts_every_frame_the_controller_sends(current):
  """Controller and panda agree on the gas floor: with the init flag, every ACCDATA frame of the
  deep-regen scenarios passes ford.h; the same frames without the flag are rejected exactly when
  they ask for regen below -0.5."""
  deep = np.concatenate([acc for name, (acc, _bp) in current.items() if name.endswith('-deep')])
  normal = np.concatenate([acc for name, (acc, _bp) in current.items() if name.endswith(('-dh0', '-dh1'))])
  assert _panda_accepts(deep, regen_flag=True).all()
  assert _panda_accepts(normal, regen_flag=False).all()
  ok_without_flag = _panda_accepts(deep, regen_flag=False)
  g = H.decode(deep)['AccPrpl_A_Rq']
  below_stock = (g < -0.5 - 1e-6) & (g > -4.9)
  assert below_stock.sum() > 1000
  assert np.array_equal(~ok_without_flag, below_stock)

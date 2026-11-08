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


def _scenario_speeds(name):
  sc = next(s for s in H.build_scenarios() if s.name == name)
  return np.array([f.v_ego for f in sc.frames])[1::2]  # ACCDATA is every other 100 Hz frame


def test_limited_regen_never_asks_for_more_than_the_pack_takes(current):
  """With the pack limit falling to 0, regen requests stay within limit x 0.8 / (mass x speed)."""
  from opendbc.sunnypilot.car.ford.longitudinal_ext import REGEN_LIMIT_MARGIN, REGEN_MIN_SPEED
  from opendbc.sunnypilot.car.ford.values_ext import FORD_REGEN_MASS_KG
  mass = FORD_REGEN_MASS_KG['FORD_MUSTANG_MACH_E_MK1']
  names = [n for n in current if n.endswith('-lim')]
  assert len(names) == 2 * len(H.LIMIT_SHAPES)
  for name in names:
    acc, _bp = current[name]
    d = H.decode(acc)
    gas, v = d['AccPrpl_A_Rq'], _scenario_speeds(name)[:len(acc)]
    limit = np.array([H._limit_ramp(2 * i + 1) for i in range(len(acc))])
    allowed = REGEN_LIMIT_MARGIN * limit * 1000 / (mass * np.maximum(v, 1.0))
    # regen coasting runs above REGEN_MIN_SPEED (below it, and while stopping, the stock path sends
    # the planner's small creep-speed requests as before)
    sc = next(x for x in H.build_scenarios() if x.name == name)
    stopping = np.array([f.stopping for f in sc.frames])[1::2][:len(acc)]
    regen = (gas < 0) & (gas > -4.9) & (v > REGEN_MIN_SPEED) & ~stopping
    assert np.all(-gas[regen] <= allowed[regen] + 0.011), name   # 0.01 CAN resolution
    assert regen.any()  # and regen is still used while the pack can take it


def test_silent_regen_shortfall_hands_off_to_friction(current):
  """The car reports plenty of regen but delivers none: the monitor puts the friction brakes on
  instead of coasting while regen is requested."""
  engaged = 0
  for name in (n for n in current if n.endswith('-noregen')):
    d = H.decode(current[name][0])
    gas, brake = d['AccPrpl_A_Rq'], d['AccBrkDecel_B_Rq'] > 0.5
    regen = (gas < -0.15) & (gas > -4.9)
    # longest unbroken run of regen-only frames (no friction) must stay short: 0.8 s hold + response lag
    longest, run = 0, 0
    for r in regen & ~brake:
      run = run + 1 if r else 0
      longest = max(longest, run)
    assert longest <= 50, f'{name}: regen requested without effect for {longest / 50:.1f} s'
    engaged += int(brake.any())
  assert engaged > 0  # the monitor did put the friction brakes on somewhere


def test_panda_accepts_limited_and_shortfall_scenarios(current):
  for name in (n for n in current if n.endswith(('-lim', '-noregen'))):
    deep = '-deep' in name
    assert _panda_accepts(current[name][0], regen_flag=deep).all(), name

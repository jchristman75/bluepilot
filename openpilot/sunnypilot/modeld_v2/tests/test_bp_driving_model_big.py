"""BluePilot: modeld_v2 must publish the big-model flag on drivingModelData as well as modelV2.

Stock modeld copies modelV2.big across in fill_driving_model_data(); modeld_v2 fills drivingModelData
by itself and had no such line (routes 000004d8/000004d9: modelV2.big True, drivingModelData.big False
on every frame). qlogs carry drivingModelData but not modelV2, so offline readers need this one.
main() cannot be run without a camera and a compiled model, so this pins the source instead.
"""
import ast
import inspect

from openpilot.common.test import OpenpilotTestCase
import openpilot.sunnypilot.modeld_v2.modeld as modeld_module


def _find_assignments(tree, target_src):
  return [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
          and any(ast.unparse(t) == target_src for t in n.targets)]


class TestBPDrivingModelBig(OpenpilotTestCase):
  def setup_method(self):
    self.src = inspect.getsource(modeld_module)
    self.tree = ast.parse(self.src)

  def test_big_flag_set_on_driving_model_data(self):
    assigns = _find_assignments(self.tree, 'drivingdata_send.drivingModelData.big')
    assert len(assigns) == 1, "drivingModelData.big must be assigned exactly once"
    # the value is the live model's flag, so a fallback to the small model reports False
    assert ast.unparse(assigns[0].value) == 'model.chestnut'

  def test_same_value_as_model_v2(self):
    v2 = _find_assignments(self.tree, 'modelv2_send.modelV2.big')
    ddm = _find_assignments(self.tree, 'drivingdata_send.drivingModelData.big')
    assert len(v2) == 1 and len(ddm) == 1
    assert ast.unparse(v2[0].value) == ast.unparse(ddm[0].value)

  def test_set_before_publish(self):
    ddm = _find_assignments(self.tree, 'drivingdata_send.drivingModelData.big')[0]
    sends = [n for n in ast.walk(self.tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == 'pm.send'
             and n.args and ast.unparse(n.args[0]) == "'drivingModelData'"]
    assert sends, "drivingModelData is no longer published from modeld_v2"
    assert all(ddm.lineno < s.lineno for s in sends)

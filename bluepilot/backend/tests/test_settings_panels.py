"""Portal settings panels and param metadata (params/settings_panels.py, params/params_manager.py).

The panels snapshot pins the whole /api/panels output for a fixed device (comma four, Mach-E,
op long). Regenerate it only for an intended change:
  python -m bluepilot.backend.tests.test_settings_panels --regen
"""
import json
import os
import sys

from bluepilot.backend.params import params_manager, settings_panels

SNAPSHOT = os.path.join(os.path.dirname(__file__), 'data', 'panels_snapshot.json')
CAPS = {'device_type': 'mici', 'platform': 'FORD_MUSTANG_MACH_E_MK1', 'brand': 'ford',
        'has_longitudinal_control': True, 'steer_control_type': 'angle', 'has_icbm': False,
        'icbm_available': False, 'pcm_cruise': False, 'is_sp_release': False, 'is_development': True,
        'alpha_long_available': True, 'has_bsm': True, 'torque_allowed': False, 'has_stop_and_go': True}


def build():
  panels = settings_panels.build_panels(caps=CAPS, param_types=params_manager._load_param_type_cache())
  return {'list': settings_panels.list_panels(panels), 'panels': panels}


def test_panels_match_snapshot():
  with open(SNAPSHOT) as f:
    expected = json.load(f)
  assert json.loads(json.dumps(build())) == expected


def test_bluepilot_params_are_the_ford_settings():
  keys = params_manager._load_bluepilot_params()
  assert len(keys) == 53
  assert {'BPLongRegenBlend', 'disable_ford_radar_UI', 'vbatt_pause_charging', 'mici_hide_onroad_fade'} <= keys
  assert params_manager.categorize_param('BPLongRegenBlend') == 'BluePilot'
  assert params_manager.categorize_param('IsOffroad') == 'System'


def test_param_metadata_comes_from_params_keys_h():
  types = params_manager._load_param_type_cache()
  attrs = params_manager._load_param_attributes_cache()
  assert len(types) > 300 and len(attrs) > 300
  assert types['BPLongRegenBlend'] == 'bool' and types['vbatt_pause_charging'] == 'float'
  assert attrs['BPLongRegenBlend'] == ['PERSISTENT', 'BACKUP']


if __name__ == '__main__' and '--regen' in sys.argv:
  with open(SNAPSHOT, 'w') as f:
    json.dump(build(), f, indent=1, sort_keys=True)
  print('wrote', SNAPSHOT)


def test_rules_fold_capabilities_and_negate():
  conv = settings_panels.convert_rule
  caps = {'device_type': 'mici'}
  assert conv({'type': 'capability', 'field': 'device_type', 'equals': 'mici'}, caps).always
  assert conv({'type': 'capability', 'field': 'device_type', 'equals': 'tici'}, caps).never
  assert conv({'type': 'not', 'condition': {'type': 'capability', 'field': 'device_type', 'equals': 'mici'}}, caps).never
  assert conv({'type': 'not', 'condition': {'type': 'param', 'key': 'X', 'equals': True}}, caps).cond == {'paramIsFalse': 'X'}
  # all(): a NEVER part makes the whole rule NEVER; ALWAYS parts drop out
  r = conv({'type': 'all', 'conditions': [{'type': 'offroad_only'},
                                          {'type': 'capability', 'field': 'device_type', 'equals': 'mici'}]}, caps)
  assert r.cond == {'isOffroad': True}
  # a rule the web conditions can't express never blocks
  assert conv({'type': 'param_compare', 'key': 'X', 'op': '>=', 'value': 1}, caps).always


def test_blocked_or_unavailable_item_is_disabled():
  item = {'key': 'X', 'widget': 'toggle', 'title': 'X', 'blocked': True}
  c = settings_panels.convert_item(item, {}, {})
  assert c['disabled'] is True and 'enableConditions' not in c

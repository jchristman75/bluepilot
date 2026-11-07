"""
Portal settings panels built from the Sunnylink settings schema.

The portal's Settings page (bluepilot/web SettingsView) renders PanelConfig
objects from /api/panels. They used to come from the Qt panel JSONs under
selfdrive/ui/bluepilot/menus, which no longer exist. The single source of truth
for settings is now openpilot/sunnypilot/sunnylink/settings_ui.json (also used
by Sunnylink and the device UI), so this module converts it to the panel shape
the web app expects (see bluepilot/web/src/types/panels.ts).

Rule mapping (Sunnylink -> web Conditions):
  offroad_only / not_engaged -> {isOffroad: true}   (not_engaged is approximated
                                                      as offroad: stricter, never looser)
  param equals bool          -> paramIsTrue / paramIsFalse
  param equals other         -> paramValueEquals
  param_compare > / <        -> paramValueGreaterThan / paramValueLessThan
  capability                 -> resolved on the device (sunnylink capabilities)
  not / any / all            -> combined; static parts folded in
"""

import json
import re
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[3]
SETTINGS_UI_PATH = _REPO_ROOT / "openpilot" / "sunnypilot" / "sunnylink" / "settings_ui.json"

FORD_PANEL_ID = "bp_vehicle_panel"
_IMPOSSIBLE = {"allConditionsTrue": [{"isOnroad": True}, {"isOffroad": True}]}
_CATEGORY_RE = re.compile(r"^\s*\[([^\]]+)\]\s*(.*)$")


def load_settings_ui(path: Path = SETTINGS_UI_PATH) -> dict:
  with open(path) as f:
    return json.load(f)


def load_capabilities() -> dict:
  try:
    from openpilot.sunnypilot.sunnylink.capabilities import generate_capabilities
    return generate_capabilities()
  except Exception:
    return {}


# --- rules -------------------------------------------------------------------

def _negate(cond):
  """Negate a rule result: bool, or a dynamic Conditions dict (None if it can't be expressed)."""
  if isinstance(cond, bool):
    return not cond
  if len(cond) == 1:
    (k, v), = cond.items()
    swaps = {"paramIsTrue": "paramIsFalse", "paramIsFalse": "paramIsTrue",
             "isOffroad": "isOnroad", "isOnroad": "isOffroad"}
    if k in swaps:
      return {swaps[k]: v}
  return None


def _combine(parts: list, mode: str):
  """Fold rule results with all/any semantics. Returns bool or Conditions."""
  dynamic = []
  for p in parts:
    if p is None:
      continue  # inexpressible: don't block on it
    if isinstance(p, bool):
      if mode == "all" and not p:
        return False
      if mode == "any" and p:
        return True
      continue
    dynamic.append(p)
  if not dynamic:
    return mode == "all"
  if len(dynamic) == 1:
    return dynamic[0]
  return {"allConditionsTrue" if mode == "all" else "anyConditionsTrue": dynamic}


def convert_rule(rule: dict, caps: dict):
  """One Sunnylink rule -> True/False (static) or a web Conditions dict (dynamic)."""
  t = rule.get("type")
  if t in ("offroad_only", "not_engaged"):
    return {"isOffroad": True}
  if t == "param":
    key, eq = rule.get("key"), rule.get("equals")
    if isinstance(eq, bool):
      return {"paramIsTrue": key} if eq else {"paramIsFalse": key}
    return {"paramValueEquals": {key: str(eq)}}
  if t == "param_compare":
    op, key, val = rule.get("op"), rule.get("key"), rule.get("value")
    if op == ">":
      return {"paramValueGreaterThan": {key: val}}
    if op == "<":
      return {"paramValueLessThan": {key: val}}
    if op in ("==", "="):
      return {"paramValueEquals": {key: str(val)}}
    return None
  if t == "capability":
    return caps.get(rule.get("field")) == rule.get("equals")
  if t == "not":
    return _negate(convert_rule(rule.get("condition", {}), caps))
  if t in ("any", "all"):
    return _combine([convert_rule(r, caps) for r in rule.get("conditions", [])], t)
  return None


def convert_rules(rules: list | None, caps: dict):
  return _combine([convert_rule(r, caps) for r in (rules or [])], "all")


# --- items -------------------------------------------------------------------

def _options(item: dict) -> list:
  return [{"name": str(o.get("label", o.get("value"))), "label": str(o.get("label", o.get("value"))),
           "value": str(o.get("value"))} for o in item.get("options", [])]


def convert_item(item: dict, caps: dict, param_types: dict, title: str | None = None) -> dict | None:
  """One Sunnylink item -> web PanelControl, or None when it is hidden on this device."""
  visible = convert_rules(item.get("visibility"), caps)
  if visible is False:
    return None
  key = item.get("key")
  widget = item.get("widget")
  control: dict[str, Any] = {"title": title or item.get("title", key), "desc": item.get("description", "")}

  if widget == "toggle":
    control.update(type="toggle", param=key)
  elif widget == "multiple_button":
    control.update(type="segmented_control", param=key, options=_options(item))
  elif widget == "option" and item.get("options"):
    control.update(type="selection", param=key, options=_options(item))
  elif widget == "option" and "min" in item:
    is_float = param_types.get(key) == "float" or any(isinstance(item.get(k), float) for k in ("min", "max", "step"))
    control.update(type="float" if is_float else "integer", param=key,
                   min=item["min"], max=item["max"], increment=item.get("step", 1))
    if item.get("unit"):
      control["unit"] = item["unit"]
  elif widget == "info":
    control.update(type="static_text")
  else:
    return None

  if item.get("needs_onroad_cycle"):
    control["requiresReboot"] = True
  if isinstance(visible, dict):
    control["visibleConditions"] = visible
  enabled = convert_rules(item.get("enablement"), caps)
  if item.get("blocked"):
    enabled = False
  if enabled is False:
    control["enableConditions"] = _IMPOSSIBLE
  elif isinstance(enabled, dict):
    control["enableConditions"] = enabled
  return control


def _group(name: str, title: str, items: list, caps: dict, param_types: dict, visible=True) -> dict | None:
  controls = []
  for it in items:
    c = convert_item(it, caps, param_types)
    if c:
      controls.append(c)
    for sub in it.get("sub_items", []) or []:
      sc = convert_item(sub, caps, param_types)
      if sc:
        controls.append(sc)
  if not controls or visible is False:
    return None
  group = {"groupName": name, "title": title, "controls": controls}
  if isinstance(visible, dict):
    for c in controls:  # sub-panel trigger (e.g. Mads on) gates every control in it
      c["visibleConditions"] = _combine([visible, c.get("visibleConditions", True)], "all")
  return group


# --- panels ------------------------------------------------------------------

def _panel_from_sp(panel: dict, caps: dict, param_types: dict) -> dict:
  groups = []
  for sec in panel.get("sections", []):
    g = _group(sec.get("id", ""), sec.get("title", ""), sec.get("items", []), caps, param_types)
    if g:
      groups.append(g)
    for sub in sec.get("sub_panels", []) or []:
      trigger = convert_rule(sub["trigger_condition"], caps) if sub.get("trigger_condition") else True
      g = _group(sub.get("id", ""), sub.get("label", ""), sub.get("items", []), caps, param_types, trigger)
      if g:
        groups.append(g)
  return {"menuName": panel.get("label", panel.get("id")), "menuDescription": panel.get("description", ""),
          "groups": groups}


def _panel_from_ford(ford: dict, caps: dict, param_types: dict) -> dict:
  """The Ford BluePilot settings, grouped by their '[Category]' title prefix."""
  groups: dict[str, list] = {}
  for it in ford.get("items", []):
    m = _CATEGORY_RE.match(it.get("title", ""))
    cat, title = (m.group(1), m.group(2)) if m else ("General", it.get("title", ""))
    c = convert_item(it, caps, param_types, title=title)
    if c:
      groups.setdefault(cat, []).append(c)
  return {"menuName": "BluePilot", "menuDescription": ford.get("description", ""),
          "groups": [{"groupName": re.sub(r"\W+", "_", cat.lower()), "title": cat, "controls": ctrls}
                     for cat, ctrls in groups.items()]}


def build_panels(settings: dict | None = None, caps: dict | None = None,
                 param_types: dict | None = None) -> dict[str, dict]:
  """All portal panels, keyed by panel id, in display order (BluePilot first)."""
  settings = settings if settings is not None else load_settings_ui()
  caps = caps if caps is not None else load_capabilities()
  param_types = param_types or {}
  panels: dict[str, dict] = {}
  ford = settings.get("vehicle_settings", {}).get("ford")
  if ford:
    panels[FORD_PANEL_ID] = _panel_from_ford(ford, caps, param_types)
  for p in sorted(settings.get("panels", []), key=lambda p: p.get("order", 99)):
    panel = _panel_from_sp(p, caps, param_types)
    if panel["groups"]:
      panels[f"bp_{p['id']}_panel"] = panel
  return panels


def list_panels(panels: dict[str, dict]) -> list[dict]:
  return [{"id": pid, "name": p["menuName"], "description": p["menuDescription"]} for pid, p in panels.items()]

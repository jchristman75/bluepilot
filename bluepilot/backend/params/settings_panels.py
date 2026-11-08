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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[3]
SETTINGS_UI_PATH = _REPO_ROOT / "openpilot" / "sunnypilot" / "sunnylink" / "settings_ui.json"

FORD_PANEL_ID = "bp_vehicle_panel"
_CATEGORY_RE = re.compile(r"^\s*\[([^\]]+)\]\s*(.*)$")


def load_settings_ui(path: Path = SETTINGS_UI_PATH) -> dict:
  with open(path) as f:
    return json.load(f)


def setting_keys(node) -> set:
  """Every param key ('key' field) anywhere under a settings_ui.json node."""
  if isinstance(node, dict):
    own = {node['key']} if isinstance(node.get('key'), str) else set()
    return own.union(*(setting_keys(v) for v in node.values()))
  if isinstance(node, list):
    return set().union(*(setting_keys(v) for v in node))
  return set()


def ford_setting_keys(settings: dict | None = None) -> set:
  """The BluePilot (Ford) settings: vehicle_settings.ford in settings_ui.json."""
  settings = settings if settings is not None else load_settings_ui()
  return setting_keys(settings.get('vehicle_settings', {}).get('ford', {}))


def load_capabilities() -> dict:
  try:
    from openpilot.sunnypilot.sunnylink.capabilities import generate_capabilities
    return generate_capabilities()
  except Exception:
    return {}


# --- rules -------------------------------------------------------------------

@dataclass(frozen=True)
class Rule:
  """A Sunnylink rule with the device capabilities folded in: always true, never true, or a
  condition the web app evaluates live (`cond`, in its Conditions shape)."""
  cond: dict | None = None
  static: bool = True  # the result when cond is None

  @property
  def always(self) -> bool:
    return self.cond is None and self.static

  @property
  def never(self) -> bool:
    return self.cond is None and not self.static


ALWAYS = Rule(static=True)
NEVER = Rule(static=False)


def when(cond: dict) -> Rule:
  return Rule(cond=cond)


# A rule the web Conditions can't express is treated as ALWAYS: it never blocks a control.
_INEXPRESSIBLE = ALWAYS
_NEGATIONS = {"paramIsTrue": "paramIsFalse", "paramIsFalse": "paramIsTrue", "isOffroad": "isOnroad", "isOnroad": "isOffroad"}


def _negate(rule: Rule) -> Rule:
  if rule.cond is None:
    return Rule(static=not rule.static)
  if len(rule.cond) == 1:
    (k, v), = rule.cond.items()
    if k in _NEGATIONS:
      return when({_NEGATIONS[k]: v})
  return _INEXPRESSIBLE


def _combine(rules: list[Rule], mode: str) -> Rule:
  """all/any of the rules, with the static ones folded in."""
  dynamic = []
  for r in rules:
    if r.cond is None:
      if mode == "all" and not r.static:
        return NEVER
      if mode == "any" and r.static:
        return ALWAYS
      continue
    dynamic.append(r.cond)
  if not dynamic:
    return ALWAYS if mode == "all" else NEVER
  if len(dynamic) == 1:
    return when(dynamic[0])
  return when({"allConditionsTrue" if mode == "all" else "anyConditionsTrue": dynamic})


def convert_rule(rule: dict, caps: dict) -> Rule:
  """One Sunnylink rule -> Rule."""
  t = rule.get("type")
  if t in ("offroad_only", "not_engaged"):
    return when({"isOffroad": True})
  if t == "param":
    key, eq = rule.get("key"), rule.get("equals")
    if isinstance(eq, bool):
      return when({"paramIsTrue": key} if eq else {"paramIsFalse": key})
    return when({"paramValueEquals": {key: str(eq)}})
  if t == "param_compare":
    op, key, val = rule.get("op"), rule.get("key"), rule.get("value")
    if op == ">":
      return when({"paramValueGreaterThan": {key: val}})
    if op == "<":
      return when({"paramValueLessThan": {key: val}})
    if op in ("==", "="):
      return when({"paramValueEquals": {key: str(val)}})
    return _INEXPRESSIBLE
  if t == "capability":
    return ALWAYS if caps.get(rule.get("field")) == rule.get("equals") else NEVER
  if t == "not":
    return _negate(convert_rule(rule.get("condition", {}), caps))
  if t in ("any", "all"):
    return _combine([convert_rule(r, caps) for r in rule.get("conditions", [])], t)
  return _INEXPRESSIBLE


def convert_rules(rules: list | None, caps: dict) -> Rule:
  return _combine([convert_rule(r, caps) for r in (rules or [])], "all")


# --- items -------------------------------------------------------------------

def _options(item: dict) -> list:
  return [{"name": str(o.get("label", o.get("value"))), "label": str(o.get("label", o.get("value"))),
           "value": str(o.get("value"))} for o in item.get("options", [])]


def convert_item(item: dict, caps: dict, param_types: dict, title: str | None = None) -> dict | None:
  """One Sunnylink item -> web PanelControl, or None when it is hidden on this device."""
  visible = convert_rules(item.get("visibility"), caps)
  if visible.never:
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
  if visible.cond is not None:
    control["visibleConditions"] = visible.cond
  enabled = NEVER if item.get("blocked") else convert_rules(item.get("enablement"), caps)
  if enabled.never:
    control["disabled"] = True
  elif enabled.cond is not None:
    control["enableConditions"] = enabled.cond
  return control


def _group(name: str, title: str, items: list, caps: dict, param_types: dict, visible: Rule = ALWAYS) -> dict | None:
  controls = []
  for it in items:
    c = convert_item(it, caps, param_types)
    if c:
      controls.append(c)
    for sub in it.get("sub_items", []) or []:
      sc = convert_item(sub, caps, param_types)
      if sc:
        controls.append(sc)
  if not controls or visible.never:
    return None
  group = {"groupName": name, "title": title, "controls": controls}
  if visible.cond is not None:
    for c in controls:  # sub-panel trigger (e.g. Mads on) gates every control in it
      own = when(c["visibleConditions"]) if "visibleConditions" in c else ALWAYS
      c["visibleConditions"] = _combine([visible, own], "all").cond
  return group


# --- panels ------------------------------------------------------------------

def _panel_from_sp(panel: dict, caps: dict, param_types: dict) -> dict:
  groups = []
  for sec in panel.get("sections", []):
    g = _group(sec.get("id", ""), sec.get("title", ""), sec.get("items", []), caps, param_types)
    if g:
      groups.append(g)
    for sub in sec.get("sub_panels", []) or []:
      trigger = convert_rule(sub["trigger_condition"], caps) if sub.get("trigger_condition") else ALWAYS
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


def portal_panels() -> dict[str, dict]:
  """The panels for this device, as /api/panels serves them."""
  from bluepilot.backend.params.params_manager import _load_param_type_cache
  return build_panels(param_types=_load_param_type_cache())


def list_panels(panels: dict[str, dict]) -> list[dict]:
  return [{"id": pid, "name": p["menuName"], "description": p["menuDescription"]} for pid, p in panels.items()]

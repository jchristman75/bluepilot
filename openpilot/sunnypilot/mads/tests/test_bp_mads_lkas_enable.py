"""BluePilot: MADS lkasEnable on a main-cruise press survives the sunnypilot merges.

Route 000004d0 seg 0 t=27.5-27.7 s: mainCruise pressed, cruiseState.available went False -> True, and
sunnypilot raised lkasEnable. The bp-gpu drives never pressed it (they engaged through pcmEnable), so
the qlog field audit could not tell whether the path still worked. This drives the real class through
that exact transition.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from opendbc.car import structs

EventNameSP = custom.OnroadEventSP.EventName


def _cs(available, buttons=()):
  return SimpleNamespace(
    cruiseState=SimpleNamespace(available=available, enabled=False, standstill=False),
    buttonEvents=list(buttons), gasPressed=False, brakePressed=False, regenBraking=False, standstill=False,
    vEgo=0.0, gearShifter=structs.CarState.GearShifter.drive, steeringPressed=False)


class _Params:
  def __init__(self, values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, *args, **kwargs):
    return self.values.get(key)


def _make_mads(main_cruise_allowed=True):
  CP = structs.CarParams()
  CP.brand = "ford"
  selfdrive = SimpleNamespace(
    CP=CP, CP_SP=structs.CarParamsSP(), params=_Params({"Mads": True, "MadsMainCruiseAllowed": main_cruise_allowed,
                                                         "MadsUnifiedEngagementMode": True}),
    events=Events(), events_sp=EventsSP(), enabled=False, enabled_prev=False, CS_prev=_cs(False),
    initialized=True, CS_sp=None, state_machine=MagicMock())
  return ModularAssistiveDrivingSystem(selfdrive), selfdrive


class TestBPMadsLkasEnable(OpenpilotTestCase):
  def test_main_cruise_available_raises_lkas_enable(self):
    mads, sd = _make_mads(main_cruise_allowed=True)
    mads.update_events(_cs(True))
    assert sd.events_sp.has(EventNameSP.lkasEnable)

  def test_no_event_when_main_cruise_not_allowed(self):
    mads, sd = _make_mads(main_cruise_allowed=False)
    mads.update_events(_cs(True))
    assert not sd.events_sp.has(EventNameSP.lkasEnable)

  def test_no_event_when_main_cruise_was_already_available(self):
    mads, sd = _make_mads(main_cruise_allowed=True)
    sd.CS_prev = _cs(True)
    mads.update_events(_cs(True))
    assert not sd.events_sp.has(EventNameSP.lkasEnable)

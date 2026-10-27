"""
BluePilot Ford interface parameter extensions.

Called from stock interface.py at the end of _get_params() and _get_params_sp()
to apply BluePilot-specific parameter overrides without modifying stock logic.

Includes:
  - HEV flag auto-detection from fingerprint CAN IDs
  - Alpha longitudinal availability policy (always True for all Ford platforms)
  - DELPHI_MRR_64 radar delay configuration
  - Tuning overrides (steerActuatorDelay, longitudinalTuning.kpV)
  - ICBM (Intelligent Cruise Button Management) availability
"""

from opendbc.car import Bus, structs
from opendbc.car.ford.values import CAR, DBC, FordFlags, RADAR, FordSafetyFlags


# BluePilot: platforms whose 0x7B pack-current signal has been checked against real data (see below).
HV_CURRENT_VERIFIED_CARS = frozenset({CAR.FORD_MUSTANG_MACH_E_MK1, CAR.FORD_F_150_LIGHTNING_MK1})


def apply_ford_ext_params(ret: structs.CarParams, CP, car_fw, fingerprint, alpha_long: bool) -> None:
  """
  Apply BluePilot parameter overrides to CarParams.

  Called at the end of CarInterface._get_params() after all stock parameter
  setup is complete.

  Args:
    ret: CarParams being built (modified in place)
    CP: CarParams reference (same as ret at this point)
    car_fw: List of CarFw from ECU queries
    fingerprint: Dict of CAN bus fingerprints {bus: {addr: len}}
  """
  from opendbc.car.ford.fordcan import CanBus
  CAN = CanBus(fingerprint=fingerprint)

  # BluePilot: tuning overrides
  ret.steerActuatorDelay = 0.22  # upstream: 0.2
  ret.longitudinalTuning.kpV = [0.]

  # BluePilot: DELPHI_MRR_64 radar support
  candidate = ret.carFingerprint
  if DBC[candidate][Bus.radar] == RADAR.DELPHI_MRR_64:
    ret.radarDelay = 0.1  # 20 Hz / 4 scan modes = 100 ms

  # BluePilot: alpha longitudinal always available for all Ford platforms.
  # This enables the developer toggle on both CAN and CANFD Ford vehicles.
  ret.alphaLongitudinalAvailable = True

  # BluePilot: make the alpha toggle authoritative for longitudinal mode.
  # True  -> openpilot longitudinal (alpha)
  # False -> Ford ACC (stock longitudinal)
  ret.openpilotLongitudinalControl = bool(alpha_long)
  if ret.openpilotLongitudinalControl:
    ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LONG_CONTROL.value
  else:
    ret.safetyConfigs[-1].safetyParam &= ~FordSafetyFlags.LONG_CONTROL.value

  # BluePilot: HEV flag auto-detection from CAN fingerprint
  # Cluster_HEV_Data2 (0x365) indicates hybrid cluster data is available
  if 0x365 in fingerprint[CAN.main]:
    ret.flags |= int(FordFlags.HEV_CLUSTER_DATA)

  # Battery_Traction_1 (0x07A), Battery_Traction_3 (0x24B), Battery_Traction_4 (0x24C)
  # All three must be present for full HEV battery telemetry
  if 0x07A in fingerprint[CAN.main] and 0x24B in fingerprint[CAN.main] and 0x24C in fingerprint[CAN.main]:
    ret.flags |= int(FordFlags.HEV_BATTERY_DATA)

  # BluePilot: real HV pack current (HV_Battery_Current_BP, 0x7B, 0.05 A/bit). Reverse-engineered on a
  # Mach-E SR: it matches Ford's own BattTrac_I_EstVsc (0x185) at r=0.9997 while driving, and integrates to
  # a constant 195-198 Ah per 100% SOC over drives and DC charges (96s3p pack). On the commaCarSegments
  # fleet it gives the two pack sizes cleanly on every car -- Mach-E ~195 / ~260 Ah, F-150 Lightning ~325 /
  # ~405 Ah -- so it is the same signal on both. No ICE/hybrid Ford in the fleet sends 0x7B. Platforms not
  # verified keep reporting the charge-power limit (see carstate_ext).
  if candidate in HV_CURRENT_VERIFIED_CARS and 0x7B in fingerprint[CAN.main]:
    ret.flags |= int(FordFlags.HV_CURRENT_DATA)

  # BluePilot: BEV/PHEV charging telemetry. MtrTrac_Data2 (0x442) has charge status; the power comes from
  # Battery_Traction_5 (0x24D, the pack's charge power limits) or from 0x7B above. The F-150 Lightning
  # sends 0x442 and 0x7B but no 0x24D.
  if 0x442 in fingerprint[CAN.main] and (0x24D in fingerprint[CAN.main] or ret.flags & FordFlags.HV_CURRENT_DATA):
    ret.flags |= int(FordFlags.CHARGING_DATA)


def apply_ford_ext_params_sp(ret: structs.CarParamsSP) -> None:
  """
  Apply BluePilot parameter overrides to CarParamsSP.

  Called at the end of CarInterface._get_params_sp() after all stock SP
  parameter setup is complete.

  Args:
    ret: CarParamsSP being built (modified in place)
  """
  # BluePilot: Enable ICBM (Intelligent Cruise Button Management) for all Ford vehicles.
  # ICBM allows openpilot to control cruise speed by emulating button presses.
  # Available when openpilotLongitudinalControl is False (using stock ACC).
  ret.intelligentCruiseButtonManagementAvailable = True

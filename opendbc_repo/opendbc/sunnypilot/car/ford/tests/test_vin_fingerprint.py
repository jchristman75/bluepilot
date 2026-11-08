"""BluePilot: VIN fallback fingerprinting for Ford.

VINs are real ones from comma's public car segments database (tools/car_porting/README.md),
with the serial digits already redacted to X -- X is a legal VIN character, so they still parse.
"""

from opendbc.car.ford.values import CAR, match_vin_to_car

# (vin, expected platform or None)
CASES = [
  ('1FM5K8GC7LGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Explorer 2020
  ('1FM5K7LC0MGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Explorer 2021
  ('1FM5K8GC7NGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Explorer 2022
  ('5LM5J7XC9LGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Lincoln Aviator 2020
  ('5LM5J7XC8MGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Lincoln Aviator 2021
  ('1FMCU9J94MUXXXXXX', CAR.FORD_ESCAPE_MK4),           # Escape 2021
  ('3FMCR9B69NRXXXXXX', CAR.FORD_BRONCO_SPORT_MK1),     # Bronco Sport 2022
  ('3FTTW8F98NRXXXXXX', CAR.FORD_MAVERICK_MK1),         # Maverick 2022
  ('3FTTW8E31PRXXXXXX', CAR.FORD_MAVERICK_MK1),         # Maverick 2023
  ('3FMTK3SU0MMXXXXXX', CAR.FORD_MUSTANG_MACH_E_MK1),   # Mach-E 2021
  ('1FTVW1EL4NWXXXXXX', CAR.FORD_F_150_LIGHTNING_MK1),  # position 8 = L -> electric
  ('1FTFW1E85MFXXXXXX', CAR.FORD_F_150_MK14),           # same body code, position 8 = E -> ICE

  # 2024-25 generation: body code 6, per-trim series codes W1B/W3L/W5L/W7L, SR/ER powertrain
  # codes K/S/7/M. This set is the best known mapping, not yet cross-checked against the Ford
  # Pro VIN guide.
  ('1FT6W3L78SWG05094', CAR.FORD_F_150_LIGHTNING_MK1),  # real VIN, 2025 Lightning Flash, ER
  ('1FT6W1BK8RGXXXXXX', CAR.FORD_F_150_LIGHTNING_MK1),  # 2024 Lightning Pro, SR
  ('1FT6W7LM8SGXXXXXX', CAR.FORD_F_150_LIGHTNING_MK1),  # 2025 Lightning Platinum, ER fleet

  ('1FT6W3LC8SWXXXXXX', None),                          # 2025 gas F-150, same series, non-electric position 8
  ('1FTBW3XKXPKB78450', None),                          # real 2023 E-Transit, series W3X, shares the old BEV code K
  ('1FT6W3XK8SWXXXXXX', None),                          # E-Transit-shaped 2024-25 VIN, series W3X, new BEV code K
  ('1FT6F4H78SWXXXXXX', None),                          # illustrative non-F-150 series (not a verified real code)

  # Constructed from NHTSA vPIC decodes (real VIN prefixes, filler serials) for the platforms
  # that have no VIN in comma's segment database
  ('2FMPK4J97NBXXXXXX', CAR.FORD_EDGE_MK2),             # Edge 2022
  ('1FMCU0J91MUXXXXXX', CAR.FORD_ESCAPE_MK4),           # Escape 2021 4x2
  ('1FMCU9J97PUXXXXXX', CAR.FORD_ESCAPE_MK4_5),         # Escape 2023 -- same code as MK4, later year
  ('1FMJU1JT8NEXXXXXX', CAR.FORD_EXPEDITION_MK4),       # Expedition 2022
  ('1FMJK1JT4PEXXXXXX', CAR.FORD_EXPEDITION_MK4),       # Expedition MAX 2023
  ('1FTER4LH8RLXXXXXX', CAR.FORD_RANGER_MK2),           # Ranger 2024

  ('1FTER4FH8KLXXXXXX', None),                          # Ranger 2019, same code, unsupported year
  ('1FMJU1JT8LEXXXXXX', None),                          # Expedition 2020, unsupported year
  ('WF0NXXGCHNJXXXXXX', None),                          # Focus, European VIN scheme
  ('00000000000000000', None),                          # VIN_UNKNOWN
  ('1FM5K8GC7LG', None),                                # malformed
  ('', None),                                           # no VIN at all
  ('1FM5K8GC7TGXXXXXX', None),                          # Explorer body code, model year outside 2020-24
]


def test_match_vin_to_car():
  for vin, expected in CASES:
    matches = match_vin_to_car(vin)
    if expected is None:
      assert matches == set(), f'{vin}: expected no match, got {matches}'
    else:
      assert matches == {str(expected)}, f'{vin}: expected {expected}, got {matches}'


def test_no_platform_matches_two_ways():
  """Any VIN the fallback accepts must resolve to exactly one platform, otherwise
  fingerprint() discards it anyway and the tables are wrong."""
  for vin, expected in CASES:
    if expected is not None:
      assert len(match_vin_to_car(vin)) == 1, vin


if __name__ == '__main__':
  test_match_vin_to_car()
  test_no_platform_matches_two_ways()
  print('ok')


def test_regen_blend_flag_set_at_init():
  from opendbc.car import structs
  from opendbc.sunnypilot.car.interfaces import _initialize_ford
  from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsBP, FordSafetyFlagsSP
  for on in (False, True):
    for pinion in (False, True):
      CP = structs.CarParams()
      CP.brand = 'ford'
      CP.carFingerprint = 'FORD_MUSTANG_MACH_E_MK1'
      CP_SP = structs.CarParamsSP()
      _initialize_ford(CP, CP_SP, {"BPLongRegenBlend": on, "FordPrefSteerAngleCurvature": pinion})
      assert bool(CP_SP.safetyParam & FordSafetyFlagsBP.REGEN_BLEND) == on
      assert bool(CP_SP.safetyParam & FordSafetyFlagsSP.STEER_ANGLE_CURVATURE) == pinion  # untouched


def test_ford_sp_and_bp_flags_fit_the_int16_safety_param():
  """CP_SP.safetyParam is an Int16 in custom.capnp: every flag combination must serialize (bit 15
  would make card fail to publish CarParamsSP), and SP (from bit 0) and BP (from bit 14) bits
  must not overlap."""
  from opendbc.car import structs
  from opendbc.sunnypilot.car.ford.values_ext import (FORD_PINION_GEOMETRY_INDEX, FORD_PINION_GEOMETRY_SHIFT,
                                                       FordSafetyFlagsBP, FordSafetyFlagsSP)
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  sp_bits = FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (max(FORD_PINION_GEOMETRY_INDEX.values()) << FORD_PINION_GEOMETRY_SHIFT)
  sp_mask = FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (0xF << FORD_PINION_GEOMETRY_SHIFT)
  bp_bits = FordSafetyFlagsBP.REGEN_BLEND
  assert sp_mask & bp_bits == 0
  for value in (sp_bits, bp_bits, sp_bits | bp_bits):
    CP_SP = structs.CarParamsSP()
    CP_SP.safetyParam = value
    assert convert_to_capnp(CP_SP).safetyParam == value

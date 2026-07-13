#!/usr/bin/env python3
"""
BluePilot: fake carStateBP.charging + hybridBattery publisher for visual testing
of the charging UI (tici sidebar icon/overlay, mici Charging screen) without a
real Mach-E connected.

Run this alongside a real `ui.py` (BluePilot build, i.e. is_bluepilot() true) on
PC, same as selfdrive/ui/tests/body.py does for the body UI:

  ./selfdrive/ui/ui.py &
  ./selfdrive/ui/tests/charging_mock.py

Simulates one full charge session using a piecewise curve shaped like a real
Mustang Mach-E Extended Range DC fast-charge session (quick ramp to a ~150kW
plateau through the low/mid SOC range, then a multi-stage taper as the battery
management system backs off approaching full), then a "Charge Complete" pause,
then restarts a fresh session at a new low SOC so the charge-session-reset logic
(selfdrive/ui/bp/charging/session.py) gets exercised too.
"""
import time
import cereal.messaging as messaging

PACK_KWH = 91.0  # approx Mach-E extended-range pack capacity
NOMINAL_VOLTAGE = 400.0
UPDATE_HZ = 5.0
DT = 1.0 / UPDATE_HZ
SIM_SPEEDUP = 10.0  # simulated time passes this many times faster than real time

# (soc_pct, power_kw) breakpoints approximating a Mach-E ER DC-fast-charge curve:
# fast ramp to peak, a wide plateau, then a progressively steeper multi-stage taper.
MACH_E_CURVE = [
  (0.0, 60.0),
  (5.0, 150.0),    # quick ramp to peak
  (40.0, 150.0),   # peak plateau holds through low/mid SOC
  (55.0, 110.0),   # taper begins
  (70.0, 80.0),
  (80.0, 55.0),
  (90.0, 30.0),
  (95.0, 18.0),
  (100.0, 8.0),    # trickle near full
]


def charge_power_kw(soc: float) -> float:
  """Piecewise-linear interpolation over MACH_E_CURVE."""
  soc = max(0.0, min(100.0, soc))
  for (soc0, kw0), (soc1, kw1) in zip(MACH_E_CURVE, MACH_E_CURVE[1:]):
    if soc0 <= soc <= soc1:
      frac = (soc - soc0) / (soc1 - soc0) if soc1 > soc0 else 0.0
      return kw0 + frac * (kw1 - kw0)
  return MACH_E_CURVE[-1][1]


if __name__ == "__main__":
  pm = messaging.PubMaster(['carStateBP'])

  while True:
    soc = 15.0 + 10.0 * (time.time() % 3)  # vary the starting SOC a bit between sessions

    # Active charge session: ramp soc up to 100%
    while soc < 100.0:
      power_kw = charge_power_kw(soc)
      amps = power_kw * 1000.0 / NOMINAL_VOLTAGE

      msg = messaging.new_message('carStateBP')
      msg.carStateBP.charging.dataAvailable = True
      msg.carStateBP.charging.chargingActive = True
      msg.carStateBP.charging.statusText = "Charging (Parked)"
      msg.carStateBP.charging.statusValue = 1
      msg.carStateBP.charging.powerKw = power_kw
      msg.carStateBP.charging.powerLimitKw = 150.0

      msg.carStateBP.hybridBattery.dataAvailable = True
      msg.carStateBP.hybridBattery.voltHighLimit = 410.0
      msg.carStateBP.hybridBattery.voltLowLimit = 320.0
      msg.carStateBP.hybridBattery.voltActual = NOMINAL_VOLTAGE
      msg.carStateBP.hybridBattery.ampsActual = amps
      msg.carStateBP.hybridBattery.socMinPerc = 0.0
      msg.carStateBP.hybridBattery.socMaxPerc = 100.0
      msg.carStateBP.hybridBattery.socActual = soc

      pm.send('carStateBP', msg)
      time.sleep(DT)

      # soc gain is directly proportional to instantaneous power (lower kW during
      # the taper => slower soc rise), just paced at SIM_SPEEDUP x real time
      sim_dt = DT * SIM_SPEEDUP
      soc += power_kw * (sim_dt / 3600.0) / PACK_KWH * 100.0

    # Charge complete: hold steady for a bit
    for _ in range(int(10.0 / DT)):
      msg = messaging.new_message('carStateBP')
      msg.carStateBP.charging.dataAvailable = True
      msg.carStateBP.charging.chargingActive = False
      msg.carStateBP.charging.statusText = "Charge Complete"
      msg.carStateBP.charging.statusValue = 4
      msg.carStateBP.charging.powerKw = 0.0
      msg.carStateBP.charging.powerLimitKw = 150.0

      msg.carStateBP.hybridBattery.dataAvailable = True
      msg.carStateBP.hybridBattery.voltHighLimit = 410.0
      msg.carStateBP.hybridBattery.voltLowLimit = 320.0
      msg.carStateBP.hybridBattery.voltActual = NOMINAL_VOLTAGE
      msg.carStateBP.hybridBattery.ampsActual = 0.0
      msg.carStateBP.hybridBattery.socMinPerc = 0.0
      msg.carStateBP.hybridBattery.socMaxPerc = 100.0
      msg.carStateBP.hybridBattery.socActual = 100.0

      pm.send('carStateBP', msg)
      time.sleep(DT)

    # Loop back around: "unplug/replug" to start a fresh session at a new low SOC
    time.sleep(1.0)

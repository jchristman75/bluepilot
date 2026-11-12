"""
Closed-loop simulation of the real LongitudinalPlanner (and its MPC solver) on a synthetic road, for the
BluePilot curve limiter and lane-change gap tests. Like selfdrive/test/longitudinal_maneuvers/plant.py, but
the model's planned path follows a road geometry (curves) and the car state can carry blinker, blind spot and
lane-change state. No sockets: messages are built in memory and passed to planner.update() as a dict.
"""

import numpy as np

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
T_IDXS = np.array(ModelConstants.T_IDXS)


class FakeParams:
  """Just enough Params for the BluePilot planner extensions (bool flags, JSON blobs)."""

  def __init__(self, **values):
    self.values = dict(values)

  def get_bool(self, key, block=False):
    return bool(self.values.get(key, False))

  def get(self, key, block=False, return_default=False):
    return self.values.get(key)

  def put(self, key, val, block=False):
    self.values[key] = val

  def get_default_value(self, key):
    return False


class Road:
  """A straight road that turns into constant-radius arcs: segments = [(length, curvature), ...]."""

  def __init__(self, segments, step=0.5):
    s, x, y, th, k = [0.0], [0.0], [0.0], [0.0], []
    for length, curv in segments:
      for _ in range(int(round(length / step))):
        th.append(th[-1] + curv * step)
        x.append(x[-1] + step * np.cos(th[-1]))
        y.append(y[-1] + step * np.sin(th[-1]))
        s.append(s[-1] + step)
        k.append(curv)
    k.append(k[-1] if k else 0.0)
    self.s, self.x, self.y, self.th, self.k = map(np.array, (s, x, y, th, k))

  @classmethod
  def curve(cls, start, radius, length=120.0, sign=1.0, lead_in=400.0):
    return cls([(start, 0.0), (length, sign / radius), (lead_in, 0.0)])

  def curvature_at(self, d):
    return float(np.interp(d, self.s, self.k))

  def plan_at(self, d, ds):
    """The planned path seen from distance d along the road, at path distances ds ahead: car-frame x, y (m)."""
    q = np.clip(d + ds, 0.0, self.s[-1])
    x0, y0, th0 = (float(np.interp(d, self.s, a)) for a in (self.x, self.y, self.th))
    dx, dy = np.interp(q, self.s, self.x) - x0, np.interp(q, self.s, self.y) - y0
    return dx * np.cos(th0) + dy * np.sin(th0), -dx * np.sin(th0) + dy * np.cos(th0)


class PlannerSim:
  def __init__(self, road: Road, speed: float, v_cruise: float | None = None, bp_params: dict | None = None,
               lead_gap: float | None = None, lead_speed: float = 0.0, personality: int = 0):
    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=speed)
    self.road = road
    self.d = 0.0
    self.speed = speed
    self.accel = 0.0
    self.v_cruise = speed if v_cruise is None else v_cruise
    self.personality = personality
    self.lead_gap, self.lead_speed = lead_gap, lead_speed
    self._v_lead_prev = lead_speed
    self.t = 0.0
    self.blinker = (False, False)
    self.blindspot = (False, False)
    self.lane_change = (LaneChangeState.off, LaneChangeDirection.none)
    self.params = FakeParams(**(bp_params or {}))
    self.planner.scc.vision.params = self.params
    self.planner.scc.vision.frame = 0          # first update reads the params
    if getattr(self.planner, 'lane_change_gap', None) is not None:
      self.planner.lane_change_gap.params = self.params
      self.planner.lane_change_gap.frame = 0
    self.log: list[dict] = []

  def _messages(self):
    d, v = self.d, self.speed
    radar = messaging.new_message('radarState')
    lead = log.RadarState.LeadData.new_message()
    if self.lead_gap is not None:
      lead.dRel = float(max(self.lead_gap, 0.0))
      lead.vRel = float(self.lead_speed - v)
      lead.vLead = lead.vLeadK = float(self.lead_speed)
      lead.aLeadK = float((self.lead_speed - self._v_lead_prev) / DT_MDL)    # radard reports the lead's accel
      self._v_lead_prev = self.lead_speed
      lead.aLeadTau = 1.5
      lead.present = True
      lead.modelProb = 1.0
      lead.radar = True
    radar.radarState.leadOne = lead
    radar.radarState.leadTwo = lead
    control = messaging.new_message('controlsState')
    control.controlsState.longControlState = LongCtrlState.pid
    control.controlsState.curvature = float(self.road.curvature_at(d))
    ss = messaging.new_message('selfdriveState')
    ss.selfdriveState.personality = self.personality
    cs = messaging.new_message('carState')
    c = cs.carState
    c.vEgo = float(v)
    c.aEgo = float(self.accel)
    c.standstill = bool(v < 0.01)
    c.vCruise = float(self.v_cruise * 3.6)
    c.vCruiseCluster = c.vCruise
    c.leftBlinker, c.rightBlinker = self.blinker
    c.leftBlindspot, c.rightBlindspot = self.blindspot
    cc = messaging.new_message('carControl')
    cc.carControl.enabled = True
    cc.carControl.longActive = True
    model = messaging.new_message('modelV2')
    ds = (v + 0.5) * T_IDXS
    px, py = self.road.plan_at(d, ds)
    pos = log.XYZTData.new_message()
    pos.x, pos.y, pos.z = [float(a) for a in px], [float(a) for a in py], [0.0] * len(T_IDXS)
    model.modelV2.position = pos
    vel = log.XYZTData.new_message()
    vel.x = [float(v + 0.5)] * len(T_IDXS)
    vel.x[0] = float(v)
    model.modelV2.velocity = vel
    acc = log.XYZTData.new_message()
    acc.x = [0.0] * len(T_IDXS)
    model.modelV2.acceleration = acc
    orate = log.XYZTData.new_message()
    orate.z = [0.0] * len(T_IDXS)      # the stock SCC-V reads the yaw-rate plan
    model.modelV2.orientationRate = orate
    model.modelV2.action.desiredAcceleration = float(self.accel + 0.5)   # the model wants to keep going (worst case)
    model.modelV2.meta.laneChangeState, model.modelV2.meta.laneChangeDirection = self.lane_change
    return {'radarState': radar.radarState, 'carState': cs.carState, 'carControl': cc.carControl, 'controlsState': control.controlsState,
            'selfdriveState': ss.selfdriveState, 'modelV2': model.modelV2,
            'carStateSP': messaging.new_message('carStateSP').carStateSP,
            'liveMapDataSP': messaging.new_message('liveMapDataSP').liveMapDataSP,
            'gpsLocation': messaging.new_message('gpsLocation').gpsLocation,
            'gpsLocationExternal': messaging.new_message('gpsLocationExternal').gpsLocationExternal}

  @property
  def t_follow_used(self) -> float:
    """The follow time the MPC used last frame: the lane-change gap's, or the personality's own."""
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
    gap = getattr(self.planner, 'lane_change_gap', None)
    return get_T_FOLLOW(self.personality) if gap is None or gap.last is None else gap.last

  def step(self, n: int = 1):
    for _ in range(n):
      sm = self._messages()
      self.planner.update(sm)
      a = float(self.planner.output_a_target)
      if self.planner.output_should_stop:
        a = min(-0.5, a)
      v_new = max(self.speed + a * DT_MDL, 0.0)
      a = (v_new - self.speed) / DT_MDL
      self.log.append({'t': self.t, 'd': self.d, 'v': self.speed, 'a': a, 'kappa': self.road.curvature_at(self.d),
                           'lat': self.speed ** 2 * abs(self.road.curvature_at(self.d)),
                           'curve_accel': self.planner.scc.curve_accel, 'lead_gap': self.lead_gap,
                           't_follow': self.t_follow_used})
      self.speed, self.accel = v_new, a
      self.d += self.speed * DT_MDL
      if self.lead_gap is not None:
        self.lead_gap += (self.lead_speed - self.speed) * DT_MDL
      self.t += DT_MDL

  def run_until(self, d_stop: float, max_s: float = 60.0):
    while self.d < d_stop and self.t < max_s:
      self.step()
    return np.array([[r['t'], r['d'], r['v'], r['a'], r['lat']] for r in self.log])


MPH = CV.MPH_TO_MS

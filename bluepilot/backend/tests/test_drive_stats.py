"""Characterization tests for the portal's drive stats (routes/processing.py).

Synthetic multi-segment logs go through a fake LogReader, so the tests pin the full
extract_drive_stats_from_segments() behavior: what carries across segments, what resets
per segment, rising-edge event counting, MADS engagement, saturation and loop timing.
"""
import openpilot.tools.lib.logreader as logreader_mod
import openpilot.cereal.messaging as messaging
import pytest

from bluepilot.backend.routes import processing

T0 = 1_000_000_000_000  # ns


def _msg(kind, t_s, **fields):
  m = messaging.new_message(kind)
  m.logMonoTime = int(T0 + t_s * 1e9)
  body = getattr(m, kind)
  for k, v in fields.items():
    setattr(body, k, v)
  return m


def _events(t_s, events):
  m = messaging.new_message('onroadEvents', len(events))
  m.logMonoTime = int(T0 + t_s * 1e9)
  for e, spec in zip(m.onroadEvents, events, strict=True):
    for k, v in spec.items():
      setattr(e, k, v)
  return m


def _controls(t_s, saturated):
  m = messaging.new_message('controlsState')
  m.logMonoTime = int(T0 + t_s * 1e9)
  m.controlsState.lateralControlState.init('angleState').saturated = saturated
  return m


def _mads(t_s, active):
  m = messaging.new_message('selfdriveStateSP')
  m.logMonoTime = int(T0 + t_s * 1e9)
  m.selfdriveStateSP.mads.active = active
  return m


def segment(t_start, seconds, *, state='enabled', mads=False, v=20.0, saturated_after=None, events=(),
            controls_gap_at=None):
  """One segment: carState + controlsState at 100 Hz, selfdriveState(SP) at 10 Hz, events as given."""
  msgs = []
  n = int(seconds * 100)
  for i in range(n):
    t = t_start + i * 0.01
    if controls_gap_at is not None and controls_gap_at <= t < controls_gap_at + 0.05:
      pass  # dropped controlsState frames: shows up as loop lag
    else:
      msgs.append(_controls(t, saturated_after is not None and t >= saturated_after))
    msgs.append(_msg('carState', t + 0.001, vEgo=v + 2.0 * (i / n)))
    if i % 10 == 0:
      msgs.append(_msg('selfdriveState', t + 0.002, state=state))
      msgs.append(_mads(t + 0.003, mads))
  for t, evs in events:
    msgs.append(_events(t, evs))
  msgs.sort(key=lambda m: m.logMonoTime)
  return [m.as_reader() for m in msgs]


@pytest.fixture
def run_stats(tmp_path, monkeypatch):
  """Write placeholder log files and serve the synthetic messages through a fake LogReader."""
  logs = {}

  def fake_logreader(path):
    return iter(logs[path])

  monkeypatch.setattr(logreader_mod, 'LogReader', fake_logreader)

  def run(segments):
    seg_dicts = []
    for i, (log_name, msgs) in enumerate(segments):
      d = tmp_path / f'seg{i}'
      d.mkdir()
      (d / log_name).write_bytes(b'')
      logs[str(d / log_name)] = msgs
      seg_dicts.append({'path': str(d)})
    return processing.extract_drive_stats_from_segments(seg_dicts)
  return run


USER_DISABLE = {'name': 'buttonCancel', 'userDisable': True}
FCW = {'name': 'fcw', 'warning': True}
LANE_CHANGE = {'name': 'laneChange'}
PRE_LC = {'name': 'preLaneChangeLeft'}
STEER_SAT = {'name': 'steerSaturated', 'warning': True}
SOFT = {'name': 'steerTempUnavailable', 'softDisable': True}


def three_segment_drive():
  return [
    ('rlog.zst', segment(0.0, 10.0, state='enabled', saturated_after=8.0, controls_gap_at=5.0, events=[
      (1.0, [PRE_LC]), (2.0, [PRE_LC, LANE_CHANGE]), (3.0, [LANE_CHANGE]),    # one lane change
      (4.0, [STEER_SAT]), (4.5, [STEER_SAT]), (6.0, []), (7.0, [STEER_SAT]),  # two warnings (re-armed)
      (9.0, [FCW]),
    ])),
    # MADS steering-only: selfdriveState disabled the whole segment
    ('rlog.zst', segment(10.0, 10.0, state='disabled', mads=True, v=25.0, events=[
      (12.0, [USER_DISABLE]), (12.5, [USER_DISABLE]),  # held: counted once
    ])),
    # qlog: no loop-timing measurement; a >1 s gap before it isn't counted as duration
    ('qlog.zst', segment(22.0, 5.0, state='disabled', mads=False, v=10.0, events=[
      (23.0, [SOFT]),
    ])),
  ]


EXPECTED = {
  'disengagementCount': 2,
  'disengagementReasons': ['userDisable', 'softDisable'],
  'alertCount': 3,
  'laneChanges': 1,
  'safetyEvents': {'fcw': 1, 'aeb': 0, 'stockFcw': 0, 'stockAeb': 0},
  'steeringSaturated': 200,
}


def test_event_counting(run_stats):
  s = run_stats(three_segment_drive())
  assert s['disengagementCount'] == EXPECTED['disengagementCount']
  assert [d['reason'] for d in s['disengagements']] == EXPECTED['disengagementReasons']
  assert s['alertCount'] == EXPECTED['alertCount']
  assert s['laneChanges'] == EXPECTED['laneChanges']
  assert s['safetyEvents'] == EXPECTED['safetyEvents']


def test_saturation_and_loop_timing(run_stats):
  s = run_stats(three_segment_drive())
  cm = s['controlMetrics']
  assert cm['steeringSaturated'] == EXPECTED['steeringSaturated']
  assert cm['maxControlLag'] == pytest.approx(50.0, abs=0.5)  # the 50 ms controlsState gap
  assert 0 < cm['avgControlLag'] < 1.0


def test_engagement_counts_mads(run_stats):
  s = run_stats(three_segment_drive())
  # enabled 10 s + MADS 10 s engaged, qlog segment 5 s not engaged; the 2 s gap is not duration
  assert s['duration'] == pytest.approx(25.0, abs=0.1)
  assert s['opEngagedTime'] == pytest.approx(20.0, abs=0.1)
  assert s['opEngagedPercent'] == pytest.approx(80.0, abs=0.5)


def test_speed(run_stats):
  s = run_stats(three_segment_drive())
  assert s['maxSpeed'] == pytest.approx(27.0 * 2.23694, abs=0.1)
  assert s['avgSpeed'] == pytest.approx(47.0, abs=0.1)  # samples: 21 m/s mean


def test_bad_segment_is_skipped(run_stats, monkeypatch):
  good = three_segment_drive()
  broken = [('rlog.zst', [None])]  # .which() on None raises inside the segment loop
  s = run_stats(good[:1] + broken)
  assert s is not None and s['laneChanges'] == 1


def test_no_segments():
  assert processing.extract_drive_stats_from_segments([]) is None


def test_distance_integrates_speed_between_car_states(run_stats):
  """Was speed x (time since the previous message of ANY type): 0.03 mi here, and 8x low on a
  real drive (50a: 0.13 mi reported vs 1.02 mi driven). Now integrated between carState samples."""
  s = run_stats(three_segment_drive())
  true_m = 21.0 * 10 + 26.0 * 10 + 11.0 * 5  # mean speed x time per segment; the gaps aren't driven
  assert s['distance'] == pytest.approx(true_m / 1609.34, abs=0.005)

#!/usr/bin/env python3
"""
Render the BluePilot MICI onroad view from a saved route into an mp4 and PNG screenshots, headless.

Based on openpilot/tools/clip/run.py, with three differences: it reads routes straight from the local route folders
(BP_ROUTES_DIR, default /mnt/j/CommaRoutes/c4), it renders MiciAugmentedRoadViewBP (the BluePilot HUD, not stock), and it
rebuilds carStateBP from the logged raw CAN with the real CarStateExt.update_car_state_bp, so routes recorded before a
carStateBP field existed (like the battery temperatures) still render it.

  python -m openpilot.bluepilot.tools.ui_clip.render 0000054c --start 500 --end 545 --out ~/clips/arrow_on --param BPShowBatteryTemp=1 \
      --shots 505,518,530,540 --caption battery

Screenshots are cut from the finished video at the given route times (seconds) and scaled up for viewing.
"""
import argparse
import glob
import logging
import os
import subprocess
import sys
from pathlib import Path


ROUTES_DIR = os.environ.get("BP_ROUTES_DIR", "/mnt/j/CommaRoutes/c4")
FPS = 20
logger = logging.getLogger("ui_clip")


def route_segments(route: str) -> dict[str, list[str | None]]:
  hits = glob.glob(f"{ROUTES_DIR}/*_{route}--*") + glob.glob(f"{ROUTES_DIR}/{route}--*")
  if not hits:
    raise SystemExit(f"route {route} not found under {ROUTES_DIR}")
  d = hits[0]
  segs = sorted(glob.glob(f"{d}/*--*"), key=lambda p: int(p.rsplit("--", 1)[1]))
  out: dict[str, list[str | None]] = {"rlog": [], "fcamera": [], "ecamera": []}
  for s in segs:
    for name, fn in (("rlog", "rlog.zst"), ("fcamera", "fcamera.hevc"), ("ecamera", "ecamera.hevc")):
      p = os.path.join(s, fn)
      out[name].append(p if os.path.exists(p) else None)
  return out


class _Msg:
  """A synthesized event shaped like the log events the clip replay feeds the SubMaster."""

  def __init__(self, builder, log_mono_time: int):
    self._builder = builder
    self.logMonoTime = log_mono_time

  def as_builder(self):
    return self._builder


class CarStateBPFromCan:
  """Feeds logged CAN frames to the Ford parsers and builds carStateBP with the real car-side code."""

  def __init__(self):
    from opendbc.can.parser import CANParser
    from opendbc.car import structs
    from opendbc.car.ford.values import FordFlags
    from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt
    cp = structs.CarParams()
    cp.flags = int(FordFlags.HEV_BATTERY_DATA | FordFlags.HV_CURRENT_DATA)
    msgs = [(m, float("nan")) for m in ("Battery_Traction_1_FD1", "Battery_Traction_3_FD1", "Battery_Traction_4_FD1", "HV_Battery_Current_BP")]
    self.parser = CANParser("ford_lincoln_base_pt", msgs, 0)
    self.ext = CarStateExt(cp, structs.CarParamsSP())

  def feed(self, can_msg) -> None:
    frames = [(c.address, bytes(c.dat), c.src) for c in can_msg.can]
    self.parser.update([(can_msg.logMonoTime, frames)])

  def build(self, log_mono_time: int):
    class _Empty:
      vl: dict = {}
    event = self.ext.update_car_state_bp(self.parser, _Empty())
    return _Msg(event, log_mono_time)


def load_chunks(rlog_paths: list[str | None], first_seg: int, fps: int = FPS) -> list[dict]:
  """One dict of the latest message per service per 1/fps, per segment, with carStateBP rebuilt from CAN."""
  from openpilot.selfdrive.test.process_replay.migration import migrate_all
  from openpilot.tools.clip.run import _download_segment
  from openpilot.tools.lib.logreader import _LogFileReader
  chunks: list[dict] = []
  cs = CarStateBPFromCan()
  for path in rlog_paths:
    if path is None:
      raise SystemExit("a needed segment has no rlog")
    logger.info("parsing %s", path)
    messages = migrate_all(list(_LogFileReader("", dat=_download_segment(path), sort_by_time=True)))
    dt_ns, current, next_time = 1e9 / fps, {}, messages[0].logMonoTime + 1e9 / fps
    for msg in messages:
      if msg.which() == "can":
        cs.feed(msg)
      if msg.logMonoTime >= next_time:
        current["carStateBP"] = cs.build(msg.logMonoTime)
        chunks.append(current)
        current, next_time = {}, next_time + dt_ns * ((msg.logMonoTime - next_time) // dt_ns + 1)
      current[msg.which()] = msg
    if current:
      current["carStateBP"] = cs.build(messages[-1].logMonoTime)
      chunks.append(current)
  return chunks


def seed_params(rlog_path: str, overrides: list[str]) -> None:
  from openpilot.common.params import Params
  from openpilot.tools.lib.logreader import LogReader
  init = LogReader(rlog_path).first("initData")
  params = Params()
  for entry in init.params.entries:
    try:
      params.put(entry.key, params.cpp2python(entry.key, entry.value), block=True)
    except Exception:       # a key this build doesn't know, or a JSON param logged as None
      pass
  for o in overrides:
    key, _, val = o.partition("=")
    params.put(key, params.cpp2python(key, val.encode()) if val else True, block=True)


def render(route: str, start: int, end: int, out: Path, overrides: list[str], caption: str | None, windowed: bool = False):
  os.environ.update({"RECORD": "1", "RECORD_OUTPUT": str(out.with_suffix(".mp4"))})
  if not windowed:
    os.environ["OFFSCREEN"] = "1"

  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.cereal.visionipc import VisionStreamType
  from msgq.visionipc import VisionIpcServer
  from openpilot.tools.clip.run import FrameQueue, patch_submaster, draw_text_box
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.system.ui.lib.application import gui_app, FontWeight

  segs = route_segments(route)
  seg_start, seg_end = start // 60, (end - 1) // 60 + 1
  chunks = load_chunks(segs["rlog"][seg_start:seg_end], seg_start)
  frame_start = (start - seg_start * 60) * FPS
  message_chunks = chunks[frame_start:frame_start + (end - start) * FPS]
  if not message_chunks:
    raise SystemExit("no messages in that window")

  if not windowed:
    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)

  with OpenpilotPrefix(shared_download_cache=True):
    seed_params(segs["rlog"][seg_start], overrides)
    from openpilot.selfdrive.ui.bp.mici.onroad.augmented_road_view_bp import MiciAugmentedRoadViewBP
    frames = FrameQueue(segs["fcamera"], start, end, fps=FPS)
    wide = FrameQueue(segs["ecamera"], start, end, fps=FPS) if any(segs["ecamera"][seg_start:seg_end]) else None

    vipc = VisionIpcServer("camerad")
    vipc.create_buffers(VisionStreamType.VISION_STREAM_NARROW_ROAD, 4, frames.frame_w, frames.frame_h)
    if wide:
      vipc.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 4, wide.frame_w, wide.frame_h)
    vipc.start_listener()

    patch_submaster(message_chunks, ui_state)
    gui_app.init_window("ui_clip", fps=FPS)
    view = MiciAugmentedRoadViewBP()
    view.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    font = gui_app.font(FontWeight.NORMAL)

    i = 0
    for should_render, _, _ in gui_app.render():
      if i >= len(message_chunks):
        break
      _, fb = frames.get()
      vipc.send(VisionStreamType.VISION_STREAM_NARROW_ROAD, fb, i, int(i * 5e7), int(i * 5e7))
      if wide:
        _, wb = wide.get()
        vipc.send(VisionStreamType.VISION_STREAM_WIDE_ROAD, wb, i, int(i * 5e7), int(i * 5e7))
      ui_state.update()
      if should_render:
        view.render()
        t = start + i / FPS
        draw_text_box(f"{int(t) // 60:02d}:{int(t) % 60:02d}", gui_app.width - 70, 0, 16, gui_app, font)
        if caption == "battery":
          hb = ui_state.sm["carStateBP"].hybridBattery
          if hb.thermalDataAvailable:
            draw_text_box(f"pack {hb.packTempC:.1f}C  coolant {hb.coolantTempC:.1f}C  gap {hb.coolantTempC - hb.packTempC:+.1f}C",
                          gui_app.width // 2 - 110, gui_app.height - 26, 14, gui_app, font)
      i += 1
    frames.stop()
    if wide:
      wide.stop()
    gui_app.close()


def cut_screenshots(video: Path, start: int, times: list[float], out_prefix: Path, scale: int = 3) -> list[Path]:
  paths = []
  for t in times:
    p = out_prefix.parent / f"{out_prefix.name}_{int(t) // 60:02d}m{int(t) % 60:02d}s.png"
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-ss", f"{t - start:.2f}", "-i", str(video), "-frames:v", "1",
                    "-vf", f"scale=iw*{scale}:ih*{scale}:flags=lanczos", str(p)], check=True)
    paths.append(p)
  return paths


def main():
  logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s\t%(message)s")
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", help="route id like 0000054c")
  ap.add_argument("--start", type=int, required=True)
  ap.add_argument("--end", type=int, required=True)
  ap.add_argument("--out", required=True, help="output path without extension")
  ap.add_argument("--param", action="append", default=[], help="KEY=VALUE param override (repeatable)")
  ap.add_argument("--caption", choices=["battery"], default=None)
  ap.add_argument("--shots", default="", help="comma-separated route times (s) to cut PNG screenshots at")
  ap.add_argument("--windowed", action="store_true")
  a = ap.parse_args()
  out = Path(a.out).expanduser()
  out.parent.mkdir(parents=True, exist_ok=True)
  render(a.route, a.start, a.end, out, a.param, a.caption, a.windowed)
  video = out.with_suffix(".mp4")
  logger.info("video: %s", video)
  times = [float(x) for x in a.shots.split(",") if x]
  for p in cut_screenshots(video, a.start, times, out):
    logger.info("screenshot: %s", p)


if __name__ == "__main__":
  sys.exit(main())

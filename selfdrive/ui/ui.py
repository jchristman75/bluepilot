#!/usr/bin/env python3
import os
import time

from cereal import messaging
from openpilot.system.hardware import TICI
from openpilot.common.realtime import Priority, config_realtime_process, set_core_affinity
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state
# BluePilot: charge-session watcher that opens the charging screen on its own
from openpilot.common.bluepilot import is_bluepilot
if is_bluepilot():
  from openpilot.selfdrive.ui.bp.charging.auto_open import charging_auto_open
# End BluePilot

BIG_UI = gui_app.big_ui()


def main():
  cores = {5, }
  # BluePilot: revert comma #37984 (5745909e9b43), which raised the UI to CTRL_HIGH (53, above
  # plannerd/radard). BP's overlay-heavy UI at that RT priority preempts the control processes and
  # starves them -> "process not communicating" (commIssue), worse with overlays on. Restore the
  # pre-#37984 priority (CTRL_LOW = 51) so the control loop always wins scheduling. If re-merging
  # upstream, keep this at CTRL_LOW for BP.
  config_realtime_process(0, Priority.CTRL_LOW)

  gui_app.init_window("UI")
  if BIG_UI:
    MainLayout()
  else:
    MiciMainLayout()

  pm = messaging.PubMaster(['uiDebug'])
  for should_render, frame_time, cpu_time in gui_app.render():
    extra_start = time.monotonic()
    ui_state.update()

    # BluePilot: poll the charge session here, not from a widget/nav tick -- those are
    # gated on rendering, and a session that starts with the screen off still has to
    # bring up the charging screen (see charging/auto_open.py).
    if is_bluepilot():
      charging_auto_open.update()
    # End BluePilot

    if should_render:
      # reaffine after power save offlines our core
      if TICI and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass

      extra_cpu = time.monotonic() - extra_start
      msg = messaging.new_message('uiDebug')
      msg.uiDebug.cpuTimeMillis = (cpu_time + extra_cpu) * 1000
      msg.uiDebug.frameTimeMillis = frame_time * 1000
      pm.send('uiDebug', msg)


if __name__ == "__main__":
  main()

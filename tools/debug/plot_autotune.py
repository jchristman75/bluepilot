#!/usr/bin/env python3
"""Parse autotune debug events from swaglog and plot the key signals.

Usage (on device or after pulling logs):
  python3 tools/debug/plot_autotune.py [/path/to/swaglog]

If no path is given, reads from /data/log/swaglog (device default).
Reads all swaglog.N rotation files in the same directory.
"""
import glob
import json
import sys
from pathlib import Path

try:
  import matplotlib.pyplot as plt
  import matplotlib.gridspec as gridspec
  HAS_PLOT = True
except ImportError:
  HAS_PLOT = False


def parse_swaglog(path: str):
  """Yield parsed JSON records from a swaglog file, skipping malformed lines."""
  try:
    with open(path) as f:
      for line in f:
        line = line.strip()
        if not line:
          continue
        try:
          yield json.loads(line)
        except json.JSONDecodeError:
          pass
  except OSError:
    pass


def extract_autotune(log_dir: str):
  states = []
  adjustments = []

  pattern = str(Path(log_dir) / "swaglog*")
  files = sorted(glob.glob(pattern))
  if not files:
    print(f"No swaglog files found in {log_dir}")
    return states, adjustments

  for path in files:
    for record in parse_swaglog(path):
      msg = record.get("msg$s") or record.get("msg", {})
      if not isinstance(msg, dict):
        continue
      event = msg.get("event$s") or msg.get("event")
      if event == "autotune_state":
        states.append({k.rstrip("$sfbi"): v for k, v in msg.items()})
      elif event == "autotune_adjust":
        adjustments.append({k.rstrip("$sfbi"): v for k, v in msg.items()})

  return states, adjustments


def plot(states, adjustments):
  if not states:
    print("No autotune_state entries found.")
    return

  idx = list(range(len(states)))
  get = lambda key: [s.get(key, 0) for s in states]

  fig = plt.figure(figsize=(14, 10))
  fig.suptitle("Autotune Debug", fontsize=13)
  gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.35)

  ax_kappa   = fig.add_subplot(gs[0, :])
  ax_error   = fig.add_subplot(gs[1, :])
  ax_int     = fig.add_subplot(gs[2, 0])
  ax_factor  = fig.add_subplot(gs[2, 1])
  ax_weight  = fig.add_subplot(gs[3, 0])
  ax_speed   = fig.add_subplot(gs[3, 1])

  # Curvature: commanded vs delayed vs actual
  ax_kappa.plot(idx, get("kappa_cmd"), label="kappa_cmd", lw=0.8, alpha=0.7)
  ax_kappa.plot(idx, get("delayed_kappa_cmd"), label="delayed_kappa_cmd", lw=0.8, alpha=0.7)
  ax_kappa.plot(idx, get("actual_kappa"), label="actual_kappa", lw=1.0)
  ax_kappa.set_title("Curvature (κ)")
  ax_kappa.set_ylabel("1/m")
  ax_kappa.legend(fontsize=7)
  ax_kappa.grid(True, alpha=0.3)

  # Error signals
  ax_error.plot(idx, get("raw_error"), label="raw_error", lw=0.7, alpha=0.6)
  ax_error.plot(idx, get("error_smooth"), label="error_smooth", lw=1.2)
  ax_error.axhline(0, color="gray", lw=0.5)
  ax_error.set_title("Curvature Error")
  ax_error.set_ylabel("1/m")
  ax_error.legend(fontsize=7)
  ax_error.grid(True, alpha=0.3)

  # Integrals
  ax_int.plot(idx, get("integral_low"), label="integral_low")
  ax_int.plot(idx, get("integral_high"), label="integral_high")
  ax_int.axhline(0, color="gray", lw=0.5)
  ax_int.set_title("Integrals")
  ax_int.legend(fontsize=7)
  ax_int.grid(True, alpha=0.3)

  # Factors
  ax_factor.plot(idx, get("low_factor"), label="low_factor")
  ax_factor.plot(idx, get("high_factor"), label="high_factor")
  # Mark adjustment events on the factor plot
  for adj in adjustments:
    fname = adj.get("factor", "")
    color = "blue" if "Low" in fname else "orange"
    ax_factor.axvline(x=len(states) - 1, color=color, lw=0.5, alpha=0.5)
  ax_factor.set_title("Curvature Factors")
  ax_factor.legend(fontsize=7)
  ax_factor.grid(True, alpha=0.3)

  # Blend weights
  ax_weight.plot(idx, get("w_low"), label="w_low")
  ax_weight.plot(idx, get("w_high"), label="w_high")
  ax_weight.set_title("Blend Weights vs Speed")
  ax_weight.set_ylabel("weight")
  ax_weight.legend(fontsize=7)
  ax_weight.grid(True, alpha=0.3)

  # Speed
  ax_speed.plot(idx, [s.get("v_ego", 0) * 2.237 for s in states], color="purple")
  ax_speed.set_title("Speed (mph)")
  ax_speed.set_ylabel("mph")
  ax_speed.grid(True, alpha=0.3)

  plt.show()


def print_adjustments(adjustments):
  if not adjustments:
    print("No autotune_adjust entries found.")
    return
  print(f"\n{'='*60}")
  print(f"{'Factor adjustments':^60}")
  print(f"{'='*60}")
  for adj in adjustments:
    factor = adj.get("factor", "?")
    old = adj.get("old_factor", "?")
    new = adj.get("new_factor", "?")
    step = adj.get("step", "?")
    osc = adj.get("oscillating", False)
    curves = adj.get("curve_count", "?")
    print(f"  {factor:<35} {old:.4f} -> {new:.4f}  step={step:.4f}  osc={osc}  curves={curves}")


def main():
  log_path = sys.argv[1] if len(sys.argv) > 1 else "/data/log"
  log_dir = str(Path(log_path).parent if Path(log_path).is_file() else log_path)

  print(f"Reading swaglogs from: {log_dir}")
  states, adjustments = extract_autotune(log_dir)
  print(f"Found {len(states)} state samples, {len(adjustments)} adjustments")

  print_adjustments(adjustments)

  if not HAS_PLOT:
    print("\nmatplotlib not available — install it to get plots.")
    return

  plot(states, adjustments)


if __name__ == "__main__":
  main()

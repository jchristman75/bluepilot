#!/usr/bin/env python3
"""Parse autotune debug events from swaglog and plot the key signals.

Local usage (point at a swaglog directory):
  python3 tools/debug/plot_autotune.py [/path/to/log/dir]

Extract from device over SSH (local WiFi) and plot immediately:
  python3 tools/debug/plot_autotune.py --ssh comma
  python3 tools/debug/plot_autotune.py --ssh 192.168.x.x

Extract from device and save for later, then plot saved data:
  python3 tools/debug/plot_autotune.py --ssh comma --save autotune.json
  python3 tools/debug/plot_autotune.py autotune.json

If no argument is given, reads from /data/log (device default path).
"""
import argparse
import glob
import json
import subprocess
import sys
from pathlib import Path

try:
  import matplotlib.pyplot as plt
  import matplotlib.gridspec as gridspec
  HAS_PLOT = True
except ImportError:
  HAS_PLOT = False

# Minimal extraction script run on the remote device via `python3 -c`.
# Prints one JSON object per autotune log line to stdout.
_REMOTE_EXTRACT = r"""
import glob, json, sys
for path in sorted(glob.glob('/data/log/swaglog*')):
  try:
    for line in open(path, errors='replace'):
      line = line.strip()
      if not line:
        continue
      try:
        r = json.loads(line)
        msg = r.get('msg', {})
        if not isinstance(msg, dict):
          continue
        ev = msg.get('event$s') or msg.get('event', '')
        if isinstance(ev, str) and ev.startswith('autotune'):
          sys.stdout.write(line + '\n')
      except Exception:
        pass
  except Exception:
    pass
"""


def _ssh_extract(host: str) -> list[dict]:
  """SSH to device, pipe extraction script via stdin, return parsed records."""
  # Pipe via stdin to avoid any shell quoting issues with the script body.
  cmd = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "BatchMode=yes",
         host, "python3 -"]
  print(f"Connecting to {host} ...", flush=True)
  try:
    result = subprocess.run(cmd, input=_REMOTE_EXTRACT, capture_output=True, text=True, timeout=60)
  except subprocess.TimeoutExpired:
    print("SSH timed out.", file=sys.stderr)
    sys.exit(1)
  except FileNotFoundError:
    print("ssh not found — is OpenSSH installed?", file=sys.stderr)
    sys.exit(1)

  if result.returncode != 0:
    print(f"SSH error (exit {result.returncode}):", file=sys.stderr)
    print(result.stderr, file=sys.stderr)
    sys.exit(1)

  records = []
  for line in result.stdout.splitlines():
    line = line.strip()
    if not line:
      continue
    try:
      records.append(json.loads(line))
    except json.JSONDecodeError:
      pass
  return records


def _parse_swaglog_file(path: str):
  try:
    with open(path, errors="replace") as f:
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


def _records_from_dir(log_dir: str) -> list[dict]:
  pattern = str(Path(log_dir) / "swaglog*")
  files = sorted(glob.glob(pattern))
  if not files:
    print(f"No swaglog files found in {log_dir}", file=sys.stderr)
    return []
  records = []
  for path in files:
    records.extend(_parse_swaglog_file(path))
  return records


def _records_from_json(path: str) -> list[dict]:
  records = []
  with open(path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        records.append(json.loads(line))
      except json.JSONDecodeError:
        pass
  return records


def _strip_type_suffix(key: str) -> str:
  for suffix in ("$s", "$f", "$b", "$i"):
    if key.endswith(suffix):
      return key[:-2]
  return key


def extract_autotune(records: list[dict]) -> tuple[list[dict], list[dict]]:
  states, adjustments = [], []
  for record in records:
    msg = record.get("msg$s") or record.get("msg", {})
    if not isinstance(msg, dict):
      continue
    event = msg.get("event$s") or msg.get("event")
    if event == "autotune_state":
      states.append({_strip_type_suffix(k): v for k, v in msg.items()})
    elif event == "autotune_adjust":
      adjustments.append({_strip_type_suffix(k): v for k, v in msg.items()})
  return states, adjustments


def plot(states: list[dict], adjustments: list[dict]) -> None:
  if not states:
    print("No autotune_state entries found — nothing to plot.")
    return

  idx = list(range(len(states)))
  get = lambda key: [s.get(key, 0) for s in states]

  fig = plt.figure(figsize=(14, 10))
  fig.suptitle("Autotune Debug", fontsize=13)
  gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.35)

  ax_kappa  = fig.add_subplot(gs[0, :])
  ax_error  = fig.add_subplot(gs[1, :])
  ax_int    = fig.add_subplot(gs[2, 0])
  ax_factor = fig.add_subplot(gs[2, 1])
  ax_weight = fig.add_subplot(gs[3, 0])
  ax_speed  = fig.add_subplot(gs[3, 1])

  ax_kappa.plot(idx, get("kappa_cmd"),         label="kappa_cmd",         lw=0.8, alpha=0.7)
  ax_kappa.plot(idx, get("delayed_kappa_cmd"), label="delayed_kappa_cmd", lw=0.8, alpha=0.7)
  ax_kappa.plot(idx, get("actual_kappa"),      label="actual_kappa",      lw=1.0)
  ax_kappa.set_title("Curvature (κ)")
  ax_kappa.set_ylabel("1/m")
  ax_kappa.legend(fontsize=7)
  ax_kappa.grid(True, alpha=0.3)

  ax_error.plot(idx, get("raw_error"),    label="raw_error",    lw=0.7, alpha=0.6)
  ax_error.plot(idx, get("error_smooth"), label="error_smooth", lw=1.2)
  ax_error.axhline(0, color="gray", lw=0.5)
  ax_error.set_title("Curvature Error")
  ax_error.set_ylabel("1/m")
  ax_error.legend(fontsize=7)
  ax_error.grid(True, alpha=0.3)

  ax_int.plot(idx, get("integral_low"),  label="integral_low")
  ax_int.plot(idx, get("integral_high"), label="integral_high")
  ax_int.axhline(0, color="gray", lw=0.5)
  ax_int.set_title("Integrals")
  ax_int.legend(fontsize=7)
  ax_int.grid(True, alpha=0.3)

  ax_factor.plot(idx, get("low_factor"),  label="low_factor")
  ax_factor.plot(idx, get("high_factor"), label="high_factor")
  # Vertical markers at each adjustment
  for i, adj in enumerate(adjustments):
    fname = adj.get("factor", "")
    color = "blue" if "Low" in fname else "orange"
    ax_factor.axvline(x=len(states) - 1 - i, color=color, lw=0.5, alpha=0.4)
  ax_factor.set_title("Curvature Factors")
  ax_factor.legend(fontsize=7)
  ax_factor.grid(True, alpha=0.3)

  ax_weight.plot(idx, get("w_low"),  label="w_low")
  ax_weight.plot(idx, get("w_high"), label="w_high")
  ax_weight.set_title("Blend Weights vs Speed")
  ax_weight.set_ylabel("weight")
  ax_weight.legend(fontsize=7)
  ax_weight.grid(True, alpha=0.3)

  ax_speed.plot(idx, [s.get("v_ego", 0) * 2.237 for s in states], color="purple")
  ax_speed.set_title("Speed (mph)")
  ax_speed.set_ylabel("mph")
  ax_speed.grid(True, alpha=0.3)

  plt.show()


def print_adjustments(adjustments: list[dict]) -> None:
  if not adjustments:
    print("No factor adjustments found.")
    return
  print(f"\n{'='*65}")
  print(f"{'Factor adjustments':^65}")
  print(f"{'='*65}")
  for adj in adjustments:
    factor = adj.get("factor", "?")
    old    = adj.get("old_factor", 0)
    new    = adj.get("new_factor", 0)
    step   = adj.get("step", 0)
    osc    = adj.get("oscillating", False)
    curves = adj.get("curve_count", "?")
    print(f"  {factor:<35} {old:.4f} -> {new:.4f}  step={step:+.4f}  osc={osc}  curves={curves}")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("path", nargs="?",
                      help="Local swaglog directory or saved .json file. "
                           "Defaults to /data/log when not provided.")
  parser.add_argument("--ssh", metavar="HOST",
                      help="Extract from device over SSH (e.g. 'comma' or '192.168.x.x') "
                           "and plot on this machine.")
  parser.add_argument("--save", metavar="FILE",
                      help="Save extracted records to a JSON file for later use.")
  args = parser.parse_args()

  if args.ssh:
    raw_records = _ssh_extract(args.ssh)
    print(f"Received {len(raw_records)} raw log records from device.")
  elif args.path and args.path.endswith(".json"):
    raw_records = _records_from_json(args.path)
    print(f"Loaded {len(raw_records)} records from {args.path}.")
  else:
    log_dir = args.path or "/data/log"
    print(f"Reading swaglogs from: {log_dir}")
    raw_records = _records_from_dir(log_dir)
    print(f"Read {len(raw_records)} raw log records.")

  if args.save:
    Path(args.save).write_text("\n".join(json.dumps(r) for r in raw_records) + "\n")
    print(f"Saved {len(raw_records)} records to {args.save}")

  states, adjustments = extract_autotune(raw_records)
  print(f"Found {len(states)} state samples, {len(adjustments)} adjustments.")

  print_adjustments(adjustments)

  if not states:
    return

  if not HAS_PLOT:
    print("\nmatplotlib not available — install it to get plots.")
    return

  plot(states, adjustments)


if __name__ == "__main__":
  main()

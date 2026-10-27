"""Right leader trigger check for the tool rotation (j6-0): range, noise, and whether the right hand can hold
the clutch button and set the trigger at the same time. Read-only: no RB5, no gripper, nothing written to
the leader arm (ping and read only).

Planned use (j6-1, docs/decisions.md j6-0): right trigger 0 (released) .. full = tool rotation 0 .. 90 deg
about the TCP, only while the clutch (right button) is held. This script shows that mapping live with the
measured range, so the operator can try it before any code moves the robot.

The right tool's raw value (teleop/leader.py): button = bits 8-15, trigger = bits 16-31.

Shown live (about 10 Hz): button, trigger raw, min / max so far, the angle it would give (0..90 deg with
--full, default 1000 as the left trigger), and the spread of the angle over the last 1 s (how steadily the
finger holds a partial angle).
Try, in this order:
  1. Hands off for a few seconds (rest value and noise).
  2. Pull the trigger fully and release a few times (full value).
  3. Hold the clutch button, then pull the trigger to the end and back, slowly (reach with the button held).
  4. Hold the clutch button and keep the trigger still at about half (45 deg) for 3-5 s, a few times (steadiness).
Summary at the end: rest value and noise, full value, the largest trigger with the button held, clutch drops
(button released while the trigger was pulled over 10 %), the steadiest 1 s at a partial angle, a suggested
trigger_full. Log: data/leader/right_trigger_<time>.csv (one row per read).

Usage:
    python scripts/36_right_trigger.py
    python scripts/36_right_trigger.py --full 1000 --range-deg 90
Stop with Ctrl+C.
"""
import argparse
import csv
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
import numpy as np

import config

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.leader import BUTTON_PRESSED, LeaderArm, read_latency_timer  # noqa: E402

OUT_DIR = ROOT / "data" / "leader"
PERIOD_S = 0.02                # 50 Hz, like the teleop
DISPLAY_EVERY = 5              # redraw every 5 reads (10 Hz)
WINDOW = 50                    # 1 s of reads for the spread
PULLED_FRAC = 0.10             # a button release with the trigger over 10 % counts as a clutch drop
PARTIAL = (0.2, 0.8)           # a 1 s window counts as "partial" when its mean is in 20..80 % of full
CLEAR_LINE = "\x1b[2K"


def split(raw):
    """(button, trigger) from the tool's raw value."""
    return (raw >> 8) & 0xFF, (raw >> 16) & 0xFFFF


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--full", type=float, default=1000.0, help="trigger value for the full angle (default 1000)")
    parser.add_argument("--range-deg", type=float, default=90.0, help="angle at --full (default 90)")
    args = parser.parse_args()

    lt = read_latency_timer(config.LEADER_PORT)
    if lt != 1:
        raise SystemExit(f"leader arm FTDI latency_timer {lt} ms (need 1; udev rule, docs/teleop_guide.md)")
    dxl_port_handler.LATENCY_TIMER = 1   # reply timeout follows this (12)
    leader = LeaderArm(config.LEADER_PORT)
    problems = leader.open()
    if problems:
        raise SystemExit("\n".join(problems))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"right_trigger_{datetime.now():%Y%m%d_%H%M%S}.csv"
    rows, window = [], deque(maxlen=WINDOW)
    deg = lambda trig: args.range_deg * min(trig / args.full, 1.0)   # noqa: E731
    lo = hi = held_max = None
    fails = drops = 0
    prev_button = 0
    best = None                     # (spread deg, mean deg, t) of the steadiest partial 1 s with the button held
    print("Reading the right tool (Ctrl+C to stop). Steps: rest, full pulls, pull with the button held, hold at half.")
    t0 = next_t = time.monotonic()
    n = 0
    try:
        while True:
            raw = leader.read_button_raw()
            t = time.monotonic() - t0
            if raw is None:
                fails += 1
                rows.append((f"{t:.3f}", "", "", ""))
            else:
                button, trig = split(raw)
                pressed = button == BUTTON_PRESSED
                lo = trig if lo is None else min(lo, trig)
                hi = trig if hi is None else max(hi, trig)
                if pressed:
                    held_max = trig if held_max is None else max(held_max, trig)
                if prev_button == BUTTON_PRESSED and not pressed and trig > PULLED_FRAC * args.full:
                    drops += 1
                prev_button = button
                window.append((pressed, deg(trig)))
                spread = mean = None
                if len(window) == WINDOW:
                    angles = np.array([a for _, a in window])
                    spread, mean = float(angles.max() - angles.min()), float(angles.mean())
                    if (all(p for p, _ in window) and PARTIAL[0] * args.range_deg <= mean <= PARTIAL[1] * args.range_deg
                            and (best is None or spread < best[0])):
                        best = (spread, mean, t)
                rows.append((f"{t:.3f}", raw, button, trig))
                if n % DISPLAY_EVERY == 0:
                    sys.stdout.write(
                        f"\r{CLEAR_LINE}button {'HELD    ' if pressed else 'released'}  trigger {trig:5d}  "
                        f"(min {lo}, max {hi}, max held {'-' if held_max is None else held_max})  "
                        f"angle {deg(trig):5.1f} deg  1 s spread {'-' if spread is None else f'{spread:4.1f}'} deg  "
                        f"drops {drops}  read fails {fails}")
                    sys.stdout.flush()
            n += 1
            next_t += PERIOD_S
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        leader.close()
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "raw", "button", "trigger"])
            w.writerows(rows)

    print("\n\nSummary")
    print(f"  reads {n}, failed {fails}, {n * PERIOD_S:.0f} s")
    trig = np.array([int(r[3]) for r in rows if r[3] != ""])
    if trig.size == 0:
        print("  no trigger values")
        return
    rest = trig[trig <= np.percentile(trig, 5)]
    print(f"  rest (lowest 5 %): {rest.min()}..{rest.max()}, noise {np.ptp(rest) / args.full * args.range_deg:.2f} deg")
    print(f"  full (max): {hi}  -> suggested trigger_full {hi}" + ("" if hi >= args.full else
          f" (below --full {args.full:.0f}: the full {args.range_deg:.0f} deg would not be reached)"))
    print(f"  max with the button held: {'-' if held_max is None else held_max}"
          + ("" if held_max is None else f" = {deg(held_max):.1f} deg with --full {args.full:.0f}"))
    print(f"  clutch drops (button released with the trigger over {PULLED_FRAC:.0%}): {drops}")
    print("  steadiest 1 s at a partial angle with the button held: "
          + ("none (step 4 not done)" if best is None else
             f"spread {best[0]:.1f} deg around {best[1]:.1f} deg (at {best[2]:.0f} s)"))
    print(f"  log: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

"""Measure state-read performance of the data channel. Read-only, no motion.

Calls CobotData.request_data() N times in a tight loop and times each call.
Only the data channel (port 5001) is opened; no command is sent to the robot.

Prints achieved Hz and per-call latency statistics (mean / min / max / std / p99),
saves raw latencies to docs/measurements/read_rate_<time>.csv and a histogram PNG.

Usage:
    python scripts/09_measure_read_rate.py
    python scripts/09_measure_read_rate.py --count 5000
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # write PNG files, no window
import matplotlib.pyplot as plt
import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

WARMUP_CALLS = 10          # not measured; keeps connection setup out of the statistics
TIMEOUT_S = 1.0            # request_data() returns None after this long
OUT_DIR = ROOT / "docs" / "measurements"

# Chart colors (reference palette from the dataviz guide, light mode)
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
SERIES = "#2a78d6"


def measure(data_channel, count):
    """Return a list of (start time from first call [s], latency [s], ok) per call."""
    for _ in range(WARMUP_CALLS):
        data_channel.request_data(TIMEOUT_S)

    rows = []
    t_first = time.perf_counter()
    for _ in range(count):
        t0 = time.perf_counter()
        data = data_channel.request_data(TIMEOUT_S)
        t1 = time.perf_counter()
        rows.append((t0 - t_first, t1 - t0, data is not None))
    total_s = time.perf_counter() - t_first
    return rows, total_s


def save_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "t_start_s", "latency_ms", "ok"])
        for i, (t_start, latency, ok) in enumerate(rows):
            writer.writerow([i, f"{t_start:.6f}", f"{latency * 1000:.4f}", int(ok)])


def save_histogram(lat_ms, mean_ms, p99_ms, path):
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    ax.hist(lat_ms, bins=60, color=SERIES, edgecolor=SURFACE, linewidth=1.0, zorder=2)
    # mean label to the left of its line, p99 label to the right and one row lower, so they never overlap
    for value, label, style, dx, dy, ha in ((mean_ms, "mean", "--", -4, -4, "right"),
                                            (p99_ms, "p99", ":", 4, -18, "left")):
        ax.axvline(value, color=TEXT_SECONDARY, linestyle=style, linewidth=1.5, zorder=3)
        ax.annotate(f"{label} {value:.2f} ms", xy=(value, 1), xycoords=("data", "axes fraction"),
                    xytext=(dx, dy), textcoords="offset points", ha=ha, va="top",
                    color=TEXT_PRIMARY, fontsize=9)
    # A single slow call can be too short a bar to see; label where the maximum is
    max_ms = lat_ms.max()
    ax.annotate(f"max {max_ms:.2f} ms", xy=(max_ms, 0), xytext=(0, 14), textcoords="offset points",
                ha="right", va="bottom", color=TEXT_SECONDARY, fontsize=9,
                arrowprops=dict(arrowstyle="-", color=TEXT_SECONDARY, linewidth=0.8))

    ax.set_title(f"request_data() latency, n = {len(lat_ms)}", color=TEXT_PRIMARY, loc="left", fontsize=11)
    ax.set_xlabel("latency per call (ms)", color=TEXT_SECONDARY)
    ax.set_ylabel("calls", color=TEXT_SECONDARY)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)

    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=1000, help="number of measured calls")
    args = parser.parse_args()

    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001) only, read-only
    print(f"Measuring {args.count} request_data() calls on {config.ROBOT_IP} (read-only)...")
    rows, total_s = measure(data_channel, args.count)

    ok = np.array([r[2] for r in rows])
    lat_ms = np.array([r[1] for r in rows if r[2]]) * 1000
    if len(lat_ms) == 0:
        raise SystemExit("No successful reads.")
    mean_ms, p99_ms = lat_ms.mean(), np.percentile(lat_ms, 99)

    print(f"\nCalls: {len(rows)}   failed (timeout): {np.sum(~ok)}   total time: {total_s:.3f} s")
    print(f"Achieved rate: {len(rows) / total_s:.1f} Hz")
    print(f"Latency (ms):  mean {mean_ms:.3f}   min {lat_ms.min():.3f}   max {lat_ms.max():.3f}   "
          f"std {lat_ms.std():.3f}   p99 {p99_ms:.3f}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stem = f"read_rate_{datetime.now():%Y%m%d_%H%M%S}"
    csv_path, png_path = OUT_DIR / f"{stem}.csv", OUT_DIR / f"{stem}.png"
    save_csv(rows, csv_path)
    save_histogram(lat_ms, mean_ms, p99_ms, png_path)
    print(f"\nSaved: {csv_path.relative_to(ROOT)}")
    print(f"Saved: {png_path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

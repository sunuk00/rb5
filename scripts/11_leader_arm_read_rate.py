"""Measure how fast the RB-Y1 leader arm motors can be read (read-only).

The leader arm is a Dynamixel bus (Protocol 2.0, 2 Mbps) behind a USB-serial adapter
(refs/rby1-sdk: dynamixel_bus.h, leader_arm.cpp). Only ping and read instructions are sent;
nothing is written to the motors (no torque, mode, ID or baud changes). The RB5 is not used.

Stages:
    0  ping every leader-arm ID (0..13, 128, 129) and print the model number
    1  read Present Position (address 132, 4 bytes) one motor at a time, as a baseline
    2  one read cycle as rby1-sdk does it (main result): GroupSyncRead of address 132 for the
       joint motors, then read4ByteTxRx for each tool (the tools do not answer Sync Read;
       rby1-sdk leader_arm.cpp also reads them one by one with ReadButtonStatus)

The FTDI latency_timer (sysfs) strongly affects the result, so it is read and saved with
every run. Results go to docs/measurements/leader_read_lt<latency>_<ids>_<time>.csv / .png

Usage:
    python scripts/11_leader_arm_read_rate.py              # joints 0..6 + tool 128 (right side)
    python scripts/11_leader_arm_read_rate.py --all        # joints 0..13 + tools 128, 129 (reference)
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # write PNG files, no window
import matplotlib.pyplot as plt
import numpy as np
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

ROOT = Path(__file__).resolve().parent.parent

PROTOCOL_VERSION = 2.0     # rby1-sdk dynamixel_bus.h: kProtocolVersion
BAUDRATE = 2000000         # rby1-sdk dynamixel_bus.h: kDefaultBaudrate
ADDR_PRESENT_POSITION = 132  # rby1-sdk: kAddrPresentPosition (tools read button state at the same address)
LEN_PRESENT_POSITION = 4
TOOL_ID_MIN = 0x80          # IDs from here are tools (handles), not joint motors
ALL_IDS = list(range(14)) + [0x80, 0x81]       # 14 joints + right / left tool
DEFAULT_IDS = list(range(7)) + [0x80]          # right arm joints + right tool
WARMUP_CALLS = 10
OUT_DIR = ROOT / "docs" / "measurements"

# Chart colors (reference palette from the dataviz guide, light mode)
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
SERIES = "#2a78d6"


def read_latency_timer(port_name):
    """FTDI latency_timer in ms from sysfs, or None if it cannot be read."""
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


def stats_line(ms):
    return (f"mean {ms.mean():7.3f}   min {ms.min():7.3f}   max {ms.max():7.3f}   "
            f"std {ms.std():6.3f}   p99 {np.percentile(ms, 99):7.3f} ms")


def stage0_ping(port, ph):
    print("\nStage 0: ping")
    ok_ids = []
    for dxl_id in ALL_IDS:
        model, result, error = ph.ping(port, dxl_id)
        if result == COMM_SUCCESS:
            ok_ids.append(dxl_id)
            print(f"  ID {dxl_id:3d}: model number {model}" + (f"  (error byte {error})" if error else ""))
        else:
            print(f"  ID {dxl_id:3d}: no response ({ph.getTxRxResult(result)})")
    print(f"  {len(ok_ids)} / {len(ALL_IDS)} devices responded")
    return ok_ids


def stage1_sequential(port, ph, ids, cycles):
    """Read each ID one after another. Returns per-cycle latency (s) and failure count."""
    for _ in range(WARMUP_CALLS):
        ph.read4ByteTxRx(port, ids[0], ADDR_PRESENT_POSITION)
    cycle_lat, per_call, fails = [], [], 0
    for _ in range(cycles):
        t_cycle = time.perf_counter()
        for dxl_id in ids:
            t0 = time.perf_counter()
            _, result, _ = ph.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            per_call.append(time.perf_counter() - t0)
            fails += result != COMM_SUCCESS
        cycle_lat.append(time.perf_counter() - t_cycle)
    return np.array(cycle_lat), np.array(per_call), fails


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def stage2_cycle(port, ph, ids, count):
    """Read cycle = GroupSyncRead of the joints + one read4ByteTxRx per tool.

    Returns rows (t_start, cycle, sync part, tool part, sync result, failed tool reads, ok),
    the total time, and the last values (signed raw).
    """
    joints = [i for i in ids if i < TOOL_ID_MIN]
    tools = [i for i in ids if i >= TOOL_ID_MIN]
    sync = GroupSyncRead(port, ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    for dxl_id in joints:
        sync.addParam(dxl_id)
    for _ in range(WARMUP_CALLS):
        sync.txRxPacket()

    rows, values = [], {}
    t_first = time.perf_counter()
    for _ in range(count):
        t0 = time.perf_counter()
        result = sync.txRxPacket()
        t1 = time.perf_counter()
        # rxPacket stops at the first ID that fails, so success means every joint answered
        sync_ok = result == COMM_SUCCESS and all(
            sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in joints)
        tool_values, tool_fails = {}, 0
        for dxl_id in tools:
            value, tool_result, _ = ph.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            if tool_result == COMM_SUCCESS:
                tool_values[dxl_id] = value
            else:
                tool_fails += 1
        t2 = time.perf_counter()
        ok = sync_ok and tool_fails == 0
        rows.append((t0 - t_first, t2 - t0, t1 - t0, t2 - t1, result, tool_fails, ok))
        if ok:
            values = {i: to_signed32(sync.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) for i in joints}
            values.update({i: to_signed32(v) for i, v in tool_values.items()})
    total_s = time.perf_counter() - t_first
    return rows, total_s, values


def save_csv(rows, path, meta):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        for key, value in meta.items():
            writer.writerow([f"# {key}", value])
        writer.writerow(["index", "t_start_s", "cycle_ms", "sync_read_ms", "tool_reads_ms",
                         "sync_comm_result", "tool_fails", "ok"])
        for i, (t_start, cycle, sync_part, tool_part, result, tool_fails, ok) in enumerate(rows):
            writer.writerow([i, f"{t_start:.6f}", f"{cycle * 1000:.4f}", f"{sync_part * 1000:.4f}",
                             f"{tool_part * 1000:.4f}", result, tool_fails, int(ok)])


def save_histogram(lat_ms, title, path):
    mean_ms, p99_ms, max_ms = lat_ms.mean(), np.percentile(lat_ms, 99), lat_ms.max()
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
    # Label the maximum only when it is a separate outlier; next to p99 it would overlap that label
    if max_ms - p99_ms > 0.15 * (max_ms - lat_ms.min()):
        ax.annotate(f"max {max_ms:.2f} ms", xy=(max_ms, 0), xytext=(0, 14), textcoords="offset points",
                    ha="right", va="bottom", color=TEXT_SECONDARY, fontsize=9,
                    arrowprops=dict(arrowstyle="-", color=TEXT_SECONDARY, linewidth=0.8))
    ax.set_title(title, color=TEXT_PRIMARY, loc="left", fontsize=11)
    ax.set_xlabel("time per read cycle (ms)", color=TEXT_SECONDARY)
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
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USB-serial device of the leader arm")
    parser.add_argument("--all", action="store_true", help="sync-read all 16 devices instead of 0..6 + 128")
    parser.add_argument("--count", type=int, default=1000, help="number of GroupSyncRead calls")
    parser.add_argument("--seq-cycles", type=int, default=200, help="number of one-by-one read cycles")
    args = parser.parse_args()
    ids = ALL_IDS if args.all else DEFAULT_IDS

    latency_timer = read_latency_timer(args.port)
    print(f"Port {args.port}   baud {BAUDRATE}   protocol {PROTOCOL_VERSION}   "
          f"FTDI latency_timer {latency_timer} ms   (read-only: ping / read only)")

    port = PortHandler(args.port)
    ph = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise SystemExit(f"Failed to open {args.port}")
    try:
        if not port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")

        ok_ids = stage0_ping(port, ph)
        missing = [i for i in ids if i not in ok_ids]
        if missing:
            raise SystemExit(f"IDs {missing} did not answer the ping; stopping before the rate test.")

        print(f"\nStage 1: one-by-one read of IDs {ids}, {args.seq_cycles} cycles")
        cycle_lat, per_call, fails = stage1_sequential(port, ph, ids, args.seq_cycles)
        print(f"  per motor read : {stats_line(per_call * 1000)}")
        print(f"  all {len(ids)} motors : {stats_line(cycle_lat * 1000)}   -> {1 / cycle_lat.mean():.1f} Hz"
              f"   failed reads {fails}")

        joints = [i for i in ids if i < TOOL_ID_MIN]
        tools = [i for i in ids if i >= TOOL_ID_MIN]
        print(f"\nStage 2: read cycle = GroupSyncRead of joints {joints} + read4ByteTxRx of tools {tools}, "
              f"{args.count} cycles")
        rows, total_s, values = stage2_cycle(port, ph, ids, args.count)
    finally:
        port.closePort()

    ok = np.array([r[6] for r in rows])
    good = [r for r in rows if r[6]]
    print(f"  cycles {len(rows)}   failed {np.sum(~ok)}   total {total_s:.3f} s   -> {len(rows) / total_s:.1f} Hz")
    if not good:
        raise SystemExit("No successful read cycle.")
    lat_ms = np.array([r[1] for r in good]) * 1000
    print(f"  whole cycle    : {stats_line(lat_ms)}")
    print(f"  sync read part : {stats_line(np.array([r[2] for r in good]) * 1000)}")
    print(f"  tool read part : {stats_line(np.array([r[3] for r in good]) * 1000)}")
    if (~ok).any():
        sync_fail = sum(1 for r in rows if r[4] != COMM_SUCCESS)
        tool_fail = sum(r[5] for r in rows)
        print(f"  failures       : sync read failed in {sync_fail} cycles, tool reads failed {tool_fail} times")
    print("  last raw values (signed): " + "  ".join(f"{i}:{v}" for i, v in values.items()))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    tag = "all16" if args.all else "r7tool"   # all 16 devices / right arm 7 joints + right tool
    stem = f"leader_read_lt{latency_timer}_{tag}_{datetime.now():%Y%m%d_%H%M%S}"
    csv_path, png_path = OUT_DIR / f"{stem}.csv", OUT_DIR / f"{stem}.png"
    save_csv(rows, csv_path, {"port": args.port, "latency_timer_ms": latency_timer, "baudrate": BAUDRATE,
                              "ids": " ".join(map(str, ids)), "address": ADDR_PRESENT_POSITION,
                              "length": LEN_PRESENT_POSITION,
                              "stage1_cycle_mean_ms": f"{cycle_lat.mean() * 1000:.3f}"})
    save_histogram(lat_ms, f"Leader arm read cycle ({len(joints)} joints sync + {len(tools)} tool), "
                           f"latency_timer {latency_timer} ms, n = {len(lat_ms)}", png_path)
    print(f"\nSaved: {csv_path.relative_to(ROOT)}")
    print(f"Saved: {png_path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

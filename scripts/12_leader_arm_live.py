"""Live view of the RB-Y1 leader arm right side (read-only).

Reads at 100 Hz: joints ID 0..6 with GroupSyncRead, the right tool (ID 128) with a
separate read (the tools do not answer Sync Read; see 11_leader_arm_read_rate.py).
A failed read keeps the last value and increases a failure counter. The screen is redrawn
in place at about 10 Hz. Only ping and read instructions are sent; nothing is written.

Angle conversion follows rby1-sdk (src/base/dynamixel_bus.cpp): rad = raw / 4096 * 2 * pi.
Tool raw value: button = bits 8..15, trigger = bits 16..31 (ReadButtonStatus in the same file).

Usage:
    python scripts/12_leader_arm_live.py
    python scripts/12_leader_arm_live.py --port /dev/ttyUSB0
Stop with Ctrl+C.
"""
import argparse
import os
import sys
import time
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

PROTOCOL_VERSION = 2.0     # rby1-sdk dynamixel_bus.h: kProtocolVersion
BAUDRATE = 2000000         # rby1-sdk dynamixel_bus.h: kDefaultBaudrate
ADDR_PRESENT_POSITION = 132  # tools return their button state at the same address
LEN_PRESENT_POSITION = 4
TICKS_PER_REV = 4096

JOINT_IDS = list(range(7))
JOINT_NAMES = ["J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R",
               "J4_Wrist_Yaw1_R", "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R"]   # rby1-sdk leader_arm.cpp
TOOL_ID = 0x80             # right tool (handle)

LOOP_HZ = 100
PERIOD_S = 1.0 / LOOP_HZ
DISPLAY_EVERY = 10         # redraw every this many cycles (about 10 Hz)

# Terminal control (ANSI escape codes)
HIDE_CURSOR, SHOW_CURSOR = "\x1b[?25l", "\x1b[?25h"
CLEAR_LINE = "\x1b[2K"
ALERT, RESET = "\x1b[1;97;41m", "\x1b[0m"   # bold white on red, for non-zero failure counts


def read_latency_timer(port_name):
    """FTDI latency_timer in ms from sysfs, or None if it cannot be read."""
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def fail_text(label, count):
    """Failure counter; highlighted when it is not zero so it stands out."""
    text = f" {label} {count} "
    return f"{ALERT}{text}{RESET}" if count else text


def ago(t_ok, now):
    return "never" if t_ok is None else f"{now - t_ok:5.2f} s ago"


def render(state, now):
    rate = state["window_cycles"] / max(now - state["window_start"], 1e-9)
    lines = [
        f"loop {rate:6.1f} Hz (target {LOOP_HZ})   cycle mean {state['busy_sum'] / max(state['cycles'], 1) * 1000:5.2f} ms"
        f"   overruns {state['overruns']}",
        "failures:" + fail_text("joint sync read", state["sync_fails"]) + fail_text("tool read", state["tool_fails"]),
        f"{'ID':>4}  {'joint':<20} {'raw':>8} {'deg':>9}     joints last OK {ago(state['joint_ok'], now)}",
    ]
    for dxl_id, name in zip(JOINT_IDS, JOINT_NAMES):
        raw = state["joint_raw"][dxl_id]
        if raw is None:
            lines.append(f"{dxl_id:>4}  {name:<20} {'-':>8} {'-':>9}")
        else:
            lines.append(f"{dxl_id:>4}  {name:<20} {raw:>8d} {raw * 360.0 / TICKS_PER_REV:>9.2f}")
    raw = state["tool_raw"]
    if raw is None:
        tool_txt = "raw -"
    else:
        button = (raw >> 8) & 0xFF
        trigger = (raw >> 16) & 0xFFFF
        tool_txt = f"raw {to_signed32(raw):>11d} (0x{raw:08X})   button {button:3d}   trigger {trigger:5d}"
    lines.append(f"{TOOL_ID:>4}  {'tool (right)':<20} {tool_txt}     last OK {ago(state['tool_ok'], now)}")
    return lines


def draw(lines, first):
    out = [] if first else [f"\x1b[{len(lines)}F"]   # move the cursor back to the first line of the block
    out += [CLEAR_LINE + line + "\n" for line in lines]
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USB-serial device of the leader arm")
    args = parser.parse_args()

    latency_timer = read_latency_timer(args.port)
    print(f"Leader arm live (read-only)   {args.port}   baud {BAUDRATE}   FTDI latency_timer {latency_timer} ms")
    if latency_timer is None:
        print("WARNING: could not read the FTDI latency_timer from sysfs.")
    elif latency_timer == 16:
        print("WARNING: latency_timer is 16 ms. Each read takes about 16 ms, so 100 Hz is not possible.")
        print(f"         To set 1 ms: echo 1 | sudo tee /sys/bus/usb-serial/devices/"
              f"{os.path.basename(os.path.realpath(args.port))}/latency_timer")
    if latency_timer == 1:
        # dynamixel_sdk computes its reply timeout from this module constant (16 by default),
        # so a failed read would wait about 34 ms. Match it to the real setting.
        dxl_port_handler.LATENCY_TIMER = 1
    print(f"dynamixel_sdk timeout constant LATENCY_TIMER = {dxl_port_handler.LATENCY_TIMER}")

    port = PortHandler(args.port)
    ph = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise SystemExit(f"Failed to open {args.port}")

    state = {"joint_raw": {i: None for i in JOINT_IDS}, "tool_raw": None, "joint_ok": None, "tool_ok": None,
             "sync_fails": 0, "tool_fails": 0, "overruns": 0, "cycles": 0, "busy_sum": 0.0,
             "window_cycles": 0, "window_start": time.monotonic()}
    t_start = time.monotonic()
    try:
        if not port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")

        missing = [i for i in JOINT_IDS if ph.ping(port, i)[1] != COMM_SUCCESS]
        if missing:
            raise SystemExit(f"Joint IDs {missing} did not answer the ping.")
        if ph.ping(port, TOOL_ID)[1] != COMM_SUCCESS:
            print(f"WARNING: tool ID {TOOL_ID} did not answer the ping; it will show as failed reads.")
        print("Ctrl+C to stop.\n")

        sync = GroupSyncRead(port, ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for dxl_id in JOINT_IDS:
            sync.addParam(dxl_id)

        sys.stdout.write(HIDE_CURSOR)
        first = True
        t_start = next_t = time.monotonic()
        while True:
            t0 = time.monotonic()
            result = sync.txRxPacket()
            if result == COMM_SUCCESS and all(
                    sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in JOINT_IDS):
                for dxl_id in JOINT_IDS:
                    state["joint_raw"][dxl_id] = to_signed32(
                        sync.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
                state["joint_ok"] = t0
            else:
                state["sync_fails"] += 1   # keep the last joint values

            value, tool_result, _ = ph.read4ByteTxRx(port, TOOL_ID, ADDR_PRESENT_POSITION)
            if tool_result == COMM_SUCCESS:
                state["tool_raw"] = value
                state["tool_ok"] = time.monotonic()
            else:
                state["tool_fails"] += 1   # keep the last tool value

            now = time.monotonic()
            state["busy_sum"] += now - t0
            state["cycles"] += 1
            state["window_cycles"] += 1
            if state["cycles"] % DISPLAY_EVERY == 0:
                draw(render(state, now), first)
                first = False
                state["window_cycles"], state["window_start"] = 0, now

            next_t += PERIOD_S
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                state["overruns"] += 1
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW_CURSOR)
        sys.stdout.flush()
        port.closePort()

    elapsed = time.monotonic() - t_start
    if state["cycles"]:
        print(f"\nStopped. {state['cycles']} cycles in {elapsed:.1f} s ({state['cycles'] / elapsed:.1f} Hz), "
              f"overruns {state['overruns']}, joint sync read failures {state['sync_fails']}, "
              f"tool read failures {state['tool_fails']}")


if __name__ == "__main__":
    main()

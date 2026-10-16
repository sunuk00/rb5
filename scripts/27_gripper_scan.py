"""Read-only scan of the gripper bus (grip-1): Dynamixel MX-106T on its own U2D2. No robot, no leader arm.

Only ping, broadcast ping and read are sent: nothing is written (no torque, no setting change).

1. FTDI latency_timer of the port (should be 1 ms, like the leader arm).
2. Find the motor: for each baud rate, Protocol 2.0 broadcast ping, then Protocol 1.0 ping of IDs 0-252
   (Protocol 1.0 has no broadcast ping). Stops at the first baud rate where a motor answers (--all: all).
3. Read the control table items that tell what the motor is and how it is set: model number (320 =
   MX-106 Protocol 1.0 firmware, 321 = MX-106(2.0)), firmware version, ID, baud rate, return delay,
   operating mode / protocol type / limits / hardware error (2.0), present voltage and temperature.
4. Time 200 reads of the block the gripper thread would read (2.0: present current, velocity and
   position, 126-135; 1.0: present position, speed and load, 36-41).

Addresses: ROBOTIS e-Manual, MX-106T/R(2.0) and MX-106T/R (Protocol 1.0) control tables.
The port must not be the leader arm's (refused).
Usage:
    python scripts/27_gripper_scan.py --port /dev/serial/by-id/usb-FTDI_..._<U2D2 serial>-if00-port0
    python scripts/27_gripper_scan.py --port /dev/ttyUSB1 --all
"""
import argparse
import os
import time
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
import numpy as np

import config

BAUDS = [57600, 1000000, 2000000, 3000000, 4000000, 115200, 9600]   # 57600 = factory default. 4.5M: not in dynamixel_sdk
P1_BAUDS = {9600, 57600, 115200, 1000000, 2000000, 3000000}   # 2,000,000 / (value + 1) in Protocol 1.0
MODEL = {320: "MX-106 (Protocol 1.0 firmware)", 321: "MX-106(2.0) (Protocol 2.0 firmware)"}

# (name, address, size) per protocol, e-Manual control tables
P2_ITEMS = [("model number", 0, 2), ("firmware version", 6, 1), ("ID", 7, 1), ("baud rate", 8, 1),
            ("return delay time", 9, 1), ("drive mode", 10, 1), ("operating mode", 11, 1),
            ("protocol type", 13, 1), ("temperature limit", 31, 1), ("max voltage limit", 32, 2),
            ("min voltage limit", 34, 2), ("current limit", 38, 2), ("max position limit", 48, 4),
            ("min position limit", 52, 4), ("shutdown", 63, 1), ("torque enable", 64, 1),
            ("hardware error status", 70, 1), ("present current", 126, 2), ("present position", 132, 4),
            ("present input voltage", 144, 2), ("present temperature", 146, 1)]
P1_ITEMS = [("model number", 0, 2), ("firmware version", 2, 1), ("ID", 3, 1), ("baud rate", 4, 1),
            ("return delay time", 5, 1), ("shutdown", 18, 1), ("torque enable", 24, 1),
            ("present position", 36, 2), ("present voltage", 42, 1), ("present temperature", 43, 1),
            ("torque ctrl mode enable", 70, 1)]
P2_MODES = {0: "current", 1: "velocity", 3: "position", 4: "extended position",
            5: "current-based position", 16: "PWM"}
P2_BAUD = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000, 5: 3000000, 6: 4000000, 7: 4500000}
ERROR_BITS = {0: "input voltage", 2: "overheating", 3: "motor encoder", 4: "electrical shock", 5: "overload"}
CURRENT_UNIT_MA = 3.36   # 2.0: Current Limit / Goal Current unit
TIMING_BLOCK = {2: (126, 10), 1: (36, 6)}


def read_latency_timer(port_name):
    """FTDI latency_timer in ms from sysfs, or None if it cannot be read."""
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


def signed(value, size):
    bits = 8 * size
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


def read_item(ph, port, dxl_id, addr, size):
    fn = {1: ph.read1ByteTxRx, 2: ph.read2ByteTxRx, 4: ph.read4ByteTxRx}[size]
    value, result, error = fn(port, dxl_id, addr)
    return value if result == COMM_SUCCESS else None, error


def explain(protocol, name, v):
    """Value with its meaning where it helps."""
    if v is None:
        return "no answer"
    if name == "model number":
        return f"{v} = {MODEL.get(v, 'not an MX-106')}"
    if protocol == 2:
        if name == "baud rate":
            return f"{v} = {P2_BAUD.get(v, '?')} bps"
        if name == "operating mode":
            return f"{v} = {P2_MODES.get(v, '?')}"
        if name == "protocol type":
            return f"{v} = Protocol {v}.0"
        if name in ("max voltage limit", "min voltage limit", "present input voltage"):
            return f"{v} = {v / 10:.1f} V"
        if name == "current limit":
            return f"{v} = {v * CURRENT_UNIT_MA / 1000:.2f} A"
        if name == "present current":
            return f"{signed(v, 2)} = {signed(v, 2) * CURRENT_UNIT_MA:.0f} mA"
        if name == "present position":
            return f"{signed(v, 4)} = {signed(v, 4) * 360 / 4096:.1f} deg"
    else:
        if name == "baud rate":
            return f"{v} = {2000000 / (v + 1):.0f} bps"
        if name == "present voltage":
            return f"{v} = {v / 10:.1f} V"
        if name == "present position":
            return f"{v} = {v * 360 / 4096:.1f} deg"
    if name in ("hardware error status", "shutdown"):
        bits = [n for b, n in ERROR_BITS.items() if v >> b & 1]
        return f"{v} ({', '.join(bits) if bits else 'none'})"
    if name == "return delay time":
        return f"{v} = {2 * v} us"
    if name in ("temperature limit", "present temperature"):
        return f"{v} C"
    return str(v)


def find(port, all_bauds):
    """List of (baud, protocol, id, model)."""
    found = []
    for baud in BAUDS:
        if not port.setBaudRate(baud):
            print(f"  {baud} bps: cannot set on the adapter")
            continue
        here = []
        ph2 = PacketHandler(2.0)
        data, result = ph2.broadcastPing(port)
        if result == COMM_SUCCESS:
            here += [(baud, 2, i, v[0]) for i, v in sorted(data.items())]
        if baud in P1_BAUDS:
            ph1 = PacketHandler(1.0)
            for i in range(253):
                model, result, _ = ph1.ping(port, i)
                if result == COMM_SUCCESS:
                    here.append((baud, 1, i, model))
        print(f"  {baud} bps: " + (", ".join(f"Protocol {p}.0 ID {i} model {m}" for _, p, i, m in here) or "nothing"))
        found += here
        if here and not all_bauds:
            break
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=config.GRIPPER_PORT, help="U2D2 USB-serial device of the gripper")
    parser.add_argument("--all", action="store_true", help="scan every baud rate (default: stop at the first hit)")
    args = parser.parse_args()
    if not args.port:
        raise SystemExit("give --port (the U2D2: ls -l /dev/serial/by-id/)")
    if not os.path.exists(args.port):
        raise SystemExit(f"{args.port} does not exist")
    if os.path.realpath(args.port) == os.path.realpath(config.LEADER_PORT):
        raise SystemExit(f"{args.port} is the leader arm's adapter ({config.LEADER_PORT}), not the gripper's")

    latency = read_latency_timer(args.port)
    print(f"Gripper bus scan (read-only)   {args.port} -> {os.path.realpath(args.port)}")
    print(f"FTDI latency_timer {latency} ms" + ("" if latency == 1 else "   (need 1: udev rule or "
          f"echo 1 | sudo tee /sys/bus/usb-serial/devices/{os.path.basename(os.path.realpath(args.port))}/latency_timer)"))
    if latency is not None:
        dxl_port_handler.LATENCY_TIMER = latency   # reply timeout follows this (12)

    port = PortHandler(args.port)
    if not port.openPort():
        raise SystemExit(f"cannot open {args.port}")
    try:
        print("\nsearch:")
        found = find(port, args.all)
        if not found:
            print("\nno motor answered: check the 12 V supply, the TTL cable (3-pin) and the hub wiring")
            return
        for baud, protocol, dxl_id, model in found:
            port.setBaudRate(baud)
            ph = PacketHandler(float(protocol))
            print(f"\nID {dxl_id}, Protocol {protocol}.0, {baud} bps:")
            values = {}
            for name, addr, size in (P2_ITEMS if protocol == 2 else P1_ITEMS):
                v, _ = read_item(ph, port, dxl_id, addr, size)
                values[name] = v
                print(f"  {name:24s} [{addr:3d}] {explain(protocol, name, v)}")

            addr, length = TIMING_BLOCK[protocol]
            times, fails = [], 0
            for _ in range(200):
                t0 = time.perf_counter()
                _, result, _ = ph.readTxRx(port, dxl_id, addr, length)
                times.append((time.perf_counter() - t0) * 1000.0)
                fails += result != COMM_SUCCESS
            t = np.array(times)
            print(f"  read {length} bytes at {addr}, 200 times: p50 {np.percentile(t, 50):.2f} / p95 "
                  f"{np.percentile(t, 95):.2f} / max {t.max():.2f} ms, {fails} failed")

            fw = values.get("firmware version")
            if model == 321:
                print(f"  -> Protocol 2.0 firmware (v{fw}): current-based position control (mode 5) is supported")
            elif model == 320:
                print(f"  -> Protocol 1.0 firmware (v{fw}): no mode 5 (only wheel / joint / multi-turn and torque "
                      "control). Mode 5 needs the MX(2.0) firmware (DYNAMIXEL Wizard 2.0): ask before updating")
            else:
                print(f"  -> model {model} is not an MX-106")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()

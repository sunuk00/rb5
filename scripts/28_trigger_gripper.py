"""Leader arm trigger -> gripper (grip-2): the gripper alone, no RB5. The leader arm is only read.

The LEFT trigger by default (--trigger right for the right one). Trigger 0 (released) = closed, 1000
(fully pulled) = fully open (user, 2026-10-17); in between the gripper opens in proportion:
    goal = closed + trigger / 1000 * (open - closed)
The motor (MX-106(2.0), ID 2, 1 Mbps, script 27) runs in current-based position control (operating
mode 5): it goes to the goal position, but its current (the gripping force) is capped by Goal Current.
In mode 5 the motor's own position limits (48, 52) are NOT used (e-Manual), so the open / closed
positions measured here are the only position limits: every goal is clipped to them.

1. Checks (read only): both FTDI latency timers 1 ms, leader arm ping, gripper model 321, no hardware
   error, input voltage >= 10 V, torque off.
2. Open / closed positions: move the gripper by hand (torque off) and press Enter at each, or give
   --closed / --open.
3. Writes, after typed confirmation: torque off, operating mode 5, profile velocity, profile
   acceleration, goal current, goal position = present position, torque on. The position is read
   again after the mode change and must not have moved.
4. Catch: the gripper holds its position until the trigger points within 5 % of the range of where the
   gripper is, or passes over that point between two reads (a fast trigger skips the 5 % window: first
   run, 2026-10-17); no jump at the start, like the teleop clutch. The needed trigger value is shown.
5. Loop at 50 Hz until Ctrl+C: read the trigger, write the goal position when it changes by 2 ticks or
   more, read the gripper state (current, position, voltage, temperature, hardware error). Stops by
   itself on a hardware error, input voltage < 10 V, temperature > 70 C, 3 failed gripper or trigger
   reads in a row, or any error (e.g. a USB adapter disconnected).
6. Always at the end: torque off, operating mode back to the value it had at the start (3), the log saved.

Addresses and units: ROBOTIS e-Manual, MX-106T/R(2.0) control table.
Output: data/gripper/trigger_gripper_<time>.csv (one row per cycle), a summary.
Usage:
    python scripts/28_trigger_gripper.py                       # measure open / closed by hand
    python scripts/28_trigger_gripper.py --closed 3968 --open 2196 --current 150
    python scripts/28_trigger_gripper.py --trigger right
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
import numpy as np

import config

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.leader import LeaderArm, read_latency_timer  # noqa: E402

OUT_DIR = ROOT / "data" / "gripper"
GRIPPER_ID = 2                 # script 27
GRIPPER_BAUD = 1000000
MODEL_MX106_2 = 321
PERIOD_S = 0.02                # 50 Hz, like the teleop
TRIGGER_FULL = 1000.0          # trigger 0 = closed, 1000 = fully open
GOAL_DEADBAND = 2              # ticks: write the goal only when it changes by this much
PROFILE_VELOCITY = 100         # x 0.229 rpm = 22.9 rpm (no-load maximum at 12 V: 45 rpm)
PROFILE_ACCELERATION = 50      # x 214.577 rev/min^2
CURRENT_DEFAULT = 100          # x 3.36 mA = 0.34 A (stall 5.2 A at 12 V)
CURRENT_MAX = 300              # 1.0 A: more needs its own decision
MIN_VOLTAGE = 10.0             # e-Manual operating range 10.0-14.8 V
MAX_TEMPERATURE = 70           # C (motor shutdown at 80)
MIN_RANGE = 50                 # ticks between open and closed (4.4 deg)
CATCH_FRAC = 0.05              # follow the trigger once it points within 5 % of the range of the position
MAX_MOVE_ON_MODE_CHANGE = 20   # ticks: the position must not change when the mode changes
FAIL_STOP = 3
CURRENT_UNIT_MA = 3.36
TICK_DEG = 360.0 / 4096

# MX-106(2.0) control table (address, size)
TORQUE_ENABLE = (64, 1)
OPERATING_MODE = (11, 1)
MODEL_NUMBER = (0, 2)
HARDWARE_ERROR = (70, 1)
GOAL_CURRENT = (102, 2)
PROFILE_ACC = (108, 4)
PROFILE_VEL = (112, 4)
GOAL_POSITION = (116, 4)
PRESENT_POSITION = (132, 4)
STATE_BLOCK = (126, 21)        # 126 current (2), 128 velocity (4), 132 position (4), 136 / 140 trajectory, 144 voltage (2), 146 temperature (1)
MODE_CURRENT_POSITION = 5

HEADER = ["t_s", "trigger_raw", "trigger", "open_frac", "following", "goal", "goal_written", "position", "current_mA",
          "velocity_rpm", "voltage_V", "temperature_C", "hardware_error", "work_ms"]


def signed(value, size):
    bits = 8 * size
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


class Gripper:
    """One MX-106(2.0) on its own U2D2."""

    def __init__(self, port_name):
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(2.0)

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"cannot open {self.port.getPortName()}")
        if not self.port.setBaudRate(GRIPPER_BAUD):
            raise SystemExit(f"cannot set {GRIPPER_BAUD} bps")

    def read(self, item):
        addr, size = item
        fn = {1: self.ph.read1ByteTxRx, 2: self.ph.read2ByteTxRx, 4: self.ph.read4ByteTxRx}[size]
        value, result, error = fn(self.port, GRIPPER_ID, addr)
        return signed(value, size) if result == COMM_SUCCESS else None

    def write(self, item, value):
        """Write and check the reply. Returns an error text or None."""
        addr, size = item
        fn = {1: self.ph.write1ByteTxRx, 2: self.ph.write2ByteTxRx, 4: self.ph.write4ByteTxRx}[size]
        result, error = fn(self.port, GRIPPER_ID, addr, value & ((1 << (8 * size)) - 1))
        if result != COMM_SUCCESS:
            return f"write {value} to address {addr}: {self.ph.getTxRxResult(result)}"
        if error & 0x7F:
            return f"write {value} to address {addr}: {self.ph.getRxPacketError(error)}"
        return None

    def state(self):
        """(current mA, velocity rpm, position ticks, voltage V, temperature C) or None."""
        data, result, _ = self.ph.readTxRx(self.port, GRIPPER_ID, *STATE_BLOCK)
        if result != COMM_SUCCESS or len(data) < STATE_BLOCK[1]:
            return None
        word = lambda o, n: signed(int.from_bytes(bytes(data[o:o + n]), "little"), n)
        return (word(0, 2) * CURRENT_UNIT_MA, word(2, 4) * 0.229, word(6, 4), word(18, 2) / 10.0, data[20])

    def close(self):
        self.port.closePort()


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def check_start(gripper):
    problems = []
    model = gripper.read(MODEL_NUMBER)
    if model != MODEL_MX106_2:
        problems.append(f"gripper ID {GRIPPER_ID}: model {model} (need {MODEL_MX106_2} = MX-106(2.0)); run script 27")
        return problems
    err = gripper.read(HARDWARE_ERROR)
    s = gripper.state()
    torque = gripper.read(TORQUE_ENABLE)
    if err != 0:
        problems.append(f"hardware error status {err} (bit 0 input voltage, 2 overheating, 3 encoder, 4 shock, 5 overload)")
    if s is None:
        problems.append("cannot read the gripper state")
    elif s[3] < MIN_VOLTAGE:
        problems.append(f"input voltage {s[3]:.1f} V < {MIN_VOLTAGE} V (12 V recommended)")
    if torque != 0:
        problems.append("gripper torque is on: turn it off (power cycle or DYNAMIXEL Wizard) so it can be moved by hand")
    return problems


def measure_positions(gripper):
    print("\nMove the gripper by hand (torque is off).")
    ask("  1) CLOSE the gripper fully, then press Enter: ")
    closed = gripper.read(PRESENT_POSITION)
    print(f"     closed = {closed}")
    ask("  2) OPEN the gripper fully, then press Enter: ")
    opened = gripper.read(PRESENT_POSITION)
    print(f"     open   = {opened}")
    if closed is None or opened is None:
        raise SystemExit("position read failed")
    return closed, opened


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--closed", type=int, help="closed position [ticks] (default: measure by hand)")
    parser.add_argument("--open", type=int, help="fully open position [ticks] (default: measure by hand)")
    parser.add_argument("--trigger", choices=("left", "right"), default="left", help="which leader trigger (default left)")
    parser.add_argument("--current", type=int, default=CURRENT_DEFAULT,
                        help=f"goal current = force cap, x 3.36 mA (default {CURRENT_DEFAULT}, max {CURRENT_MAX})")
    args = parser.parse_args()
    if not 1 <= args.current <= CURRENT_MAX:
        raise SystemExit(f"--current must be 1..{CURRENT_MAX} ({CURRENT_MAX * CURRENT_UNIT_MA / 1000:.2f} A)")
    if (args.closed is None) != (args.open is None):
        raise SystemExit("give both --closed and --open, or neither")

    problems = []
    for name, port in (("leader arm", config.LEADER_PORT), ("gripper", config.GRIPPER_PORT)):
        lt = read_latency_timer(port)
        if lt != 1:
            problems.append(f"{name} FTDI latency_timer {lt} ms (need 1; udev rule, docs/teleop_guide.md)")
    if problems:
        raise SystemExit("\n".join(problems))
    dxl_port_handler.LATENCY_TIMER = 1   # reply timeout follows this (12)

    leader = LeaderArm(config.LEADER_PORT)
    problems = leader.open(left_tool=args.trigger == "left")
    if problems:
        raise SystemExit("\n".join(problems))
    read_trigger_raw = leader.read_left_button_raw if args.trigger == "left" else leader.read_button_raw
    gripper = Gripper(config.GRIPPER_PORT)
    gripper.open()
    mode_at_start = None
    changed = False
    rows, stop_reason = [], "Ctrl+C"
    try:
        problems = check_start(gripper)
        if problems:
            raise SystemExit("\n".join(problems))
        mode_at_start = gripper.read(OPERATING_MODE)
        s = gripper.state()
        print(f"gripper: MX-106(2.0) ID {GRIPPER_ID}, mode {mode_at_start}, position {s[2]}, {s[3]:.1f} V, {s[4]} C")

        if args.closed is None:
            closed, opened = measure_positions(gripper)
        else:
            closed, opened = args.closed, args.open
        if abs(opened - closed) < MIN_RANGE:
            raise SystemExit(f"open {opened} and closed {closed} are only {abs(opened - closed)} ticks apart (min {MIN_RANGE})")
        lo, hi = min(closed, opened), max(closed, opened)
        pos = gripper.read(PRESENT_POSITION)
        print(f"\nrange: closed {closed}, open {opened} ({abs(opened - closed) * TICK_DEG:.1f} deg); now {pos}")
        if not lo - MIN_RANGE <= pos <= hi + MIN_RANGE:
            raise SystemExit(f"the gripper is at {pos}, outside the range {lo}..{hi}: put it inside first")

        print("\nwrites to the gripper (current -> new):")
        print(f"  torque enable          {gripper.read(TORQUE_ENABLE)} -> 0")
        print(f"  operating mode         {mode_at_start} -> {MODE_CURRENT_POSITION} (current-based position; back to {mode_at_start} at the end)")
        print(f"  profile velocity       {gripper.read(PROFILE_VEL)} -> {PROFILE_VELOCITY} ({PROFILE_VELOCITY * 0.229:.1f} rpm)")
        print(f"  profile acceleration   {gripper.read(PROFILE_ACC)} -> {PROFILE_ACCELERATION}")
        print(f"  goal current           {gripper.read(GOAL_CURRENT)} -> {args.current} ({args.current * CURRENT_UNIT_MA:.0f} mA, force cap)")
        print(f"  goal position          -> {pos} (present position, no jump)")
        print("  torque enable          -> 1")
        print("Keep fingers and objects out of the gripper.")
        if ask("Type yes to write and start: ") != "yes":
            raise SystemExit("not confirmed; nothing written")

        changed = True
        for item, value in ((TORQUE_ENABLE, 0), (OPERATING_MODE, MODE_CURRENT_POSITION)):
            e = gripper.write(item, value)
            if e:
                raise SystemExit(e)
        if gripper.read(OPERATING_MODE) != MODE_CURRENT_POSITION:
            raise SystemExit("operating mode did not change to 5")
        pos2 = gripper.read(PRESENT_POSITION)
        if pos2 is None or abs(pos2 - pos) > MAX_MOVE_ON_MODE_CHANGE:
            raise SystemExit(f"position changed from {pos} to {pos2} with the mode change: the open / closed values no longer match")
        for item, value in ((PROFILE_VEL, PROFILE_VELOCITY), (PROFILE_ACC, PROFILE_ACCELERATION),
                            (GOAL_CURRENT, args.current), (GOAL_POSITION, pos2), (TORQUE_ENABLE, 1)):
            e = gripper.write(item, value)
            if e:
                raise SystemExit(e)
        need = (pos2 - closed) / (opened - closed) * TRIGGER_FULL
        print(f"\nrunning: {args.trigger} trigger 0 = closed, 1000 = open. Ctrl+C to stop.")
        print(f"catch: holding at {pos2}; move the {args.trigger} trigger to about {need:.0f} to start following\n")

        last_goal, trigger, fails, trigger_fails, following = pos2, None, 0, 0, False
        prev_side = None   # sign of (trigger goal - held position) at the last read, for the catch
        t0 = time.monotonic()
        next_t = t0
        while True:
            t_start = time.monotonic()
            raw = read_trigger_raw()
            if raw is not None:
                trigger = (raw >> 16) & 0xFFFF
                trigger_fails = 0
            else:
                trigger_fails += 1
            if trigger is None:
                goal, frac = last_goal, None
            else:
                frac = min(max(trigger / TRIGGER_FULL, 0.0), 1.0)
                goal = int(round(closed + frac * (opened - closed)))
                goal = min(max(goal, lo), hi)
                if not following:
                    side = (goal > last_goal) - (goal < last_goal)
                    crossed = prev_side is not None and side != prev_side
                    if abs(goal - last_goal) <= CATCH_FRAC * (hi - lo) or crossed:
                        following = True
                        print(f"\rcaught at trigger {trigger}: following" + " " * 60)
                    prev_side = side
                if not following:
                    goal = last_goal
            written = abs(goal - last_goal) >= GOAL_DEADBAND
            if written:
                e = gripper.write(GOAL_POSITION, goal)
                if e:
                    print(f"\n{e}")
                else:
                    last_goal = goal
            s = gripper.state()
            err = gripper.read(HARDWARE_ERROR)
            work_ms = (time.monotonic() - t_start) * 1000.0
            if s is None:
                fails += 1
            else:
                fails = 0
            rows.append([f"{t_start - t0:.4f}", raw if raw is not None else "", trigger if trigger is not None else "",
                         f"{frac:.4f}" if frac is not None else "", int(following), goal, int(written)]
                        + ([s[2], f"{s[0]:.0f}", f"{s[1]:.2f}", f"{s[3]:.1f}", s[4]] if s else ["", "", "", "", ""])
                        + [err if err is not None else "", f"{work_ms:.2f}"])
            if len(rows) % 10 == 0 and s:
                mode_txt = "follow" if following else f"catch: trigger -> {need:.0f}"
                print(f"\r{mode_txt}  trigger {trigger if trigger is not None else '-':>5}  goal {goal:5d}  position {s[2]:5d}  "
                      f"current {s[0]:6.0f} mA  {s[3]:4.1f} V  {s[4]} C  work {work_ms:4.1f} ms   ", end="", flush=True)
            if err:
                stop_reason = f"hardware error status {err}"
                break
            if s and s[3] < MIN_VOLTAGE:
                stop_reason = f"input voltage {s[3]:.1f} V < {MIN_VOLTAGE} V"
                break
            if s and s[4] > MAX_TEMPERATURE:
                stop_reason = f"temperature {s[4]} C > {MAX_TEMPERATURE} C"
                break
            if fails >= FAIL_STOP:
                stop_reason = f"{FAIL_STOP} failed gripper reads in a row"
                break
            if trigger_fails >= FAIL_STOP:
                stop_reason = f"{FAIL_STOP} failed {args.trigger} trigger reads in a row"
                break
            next_t += PERIOD_S
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    except Exception as e:   # e.g. termios.error when a USB adapter disconnects
        stop_reason = f"error: {e!r}"
    finally:
        if changed:
            for item, value in ((TORQUE_ENABLE, 0), (OPERATING_MODE, mode_at_start)):
                try:
                    e = gripper.write(item, value)
                except Exception as ex:
                    e = repr(ex)
                if e:
                    print(f"\nWARNING at the end: {e} (check with script 27 / DYNAMIXEL Wizard)")
            try:
                mode_now = gripper.read(OPERATING_MODE)
            except Exception:
                mode_now = "?"
            print(f"\n\nstopped: {stop_reason}. torque off, operating mode {mode_now}")
        for port in (gripper, leader):
            try:
                port.close()
            except Exception:
                pass

    if rows:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        path = OUT_DIR / f"trigger_gripper_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(path, "w", newline="") as f:
            csv.writer(f).writerows([HEADER] + rows)
        col = {name: i for i, name in enumerate(HEADER)}
        good = [r for r in rows if r[col["position"]] != ""]
        work = np.array([float(r[col["work_ms"]]) for r in rows])
        cur = np.array([abs(float(r[col["current_mA"]])) for r in good])
        volt = np.array([float(r[col["voltage_V"]]) for r in good])
        trig = [int(r[col["trigger"]]) for r in rows if r[col["trigger"]] != ""]
        print(f"saved {path.relative_to(ROOT)} ({len(rows)} cycles, {len(rows) * PERIOD_S:.1f} s)")
        if trig:
            print(f"trigger min / max {min(trig)} / {max(trig)}")
        if len(good):
            print(f"|current| p50 / max {np.percentile(cur, 50):.0f} / {cur.max():.0f} mA; voltage min / max "
                  f"{volt.min():.1f} / {volt.max():.1f} V")
        print(f"work per cycle p50 / p99 / max {np.percentile(work, 50):.1f} / {np.percentile(work, 99):.1f} / "
              f"{work.max():.1f} ms; failed gripper reads {len(rows) - len(good)}")


if __name__ == "__main__":
    main()

"""Leader arm trigger -> gripper (grip-2): the gripper alone, no RB5. The leader arm is only read.
The motor code, the catch and the gripper thread are in teleop/gripper.py (grip-3a), shared with the teleop.

The LEFT trigger by default (--trigger right for the right one). Trigger 0 (released) = closed, 1000
(fully pulled) = fully open (user, 2026-10-17); in between the gripper opens in proportion:
    goal = closed + trigger / 1000 * (open - closed)
The motor (MX-106(2.0), ID 2, 1 Mbps, script 27) runs in current-based position control (operating
mode 5): it goes to the goal position, but its current (the gripping force) is capped by Goal Current.
In mode 5 the motor's own position limits (48, 52) are NOT used (e-Manual), so the open / closed
positions measured here are the only position limits: every goal is clipped to them.

1. Checks (read only): both FTDI latency timers 1 ms, leader arm ping, gripper model 321, no hardware
   error, input voltage >= 10 V. Torque on or mode 5 (left by a run that did not finish) is shown and
   turned off with the confirmed writes (before measuring by hand: its own confirmation).
2. Open / closed positions: move the gripper by hand (torque off) and press Enter at each, or give
   --closed / --open.
3. Writes, after typed confirmation: torque off, operating mode 5, profile velocity, profile
   acceleration, goal current, goal position = present position, torque on. The position is read
   again after the mode change and must not have moved.
4. Catch: the gripper holds its position until the trigger points within 5 % of the range of where the
   gripper is, or passes over that point between two reads (a fast trigger skips the 5 % window: first
   run, 2026-10-17); no jump at the start, like the teleop clutch. The needed trigger value is shown.
5. Until Ctrl+C: this loop reads the trigger at 50 Hz and hands it to the gripper thread
   (GripperWorker, its own 50 Hz), which writes the goal position when it changes by 2 ticks or more and
   reads the gripper state (current, position, voltage, temperature, hardware error). Stops by itself on
   a hardware error, input voltage < 10 V, temperature > 70 C, 3 failed gripper or trigger reads in a
   row, or any error (e.g. a USB adapter disconnected).
6. Always at the end (also on SIGTERM / SIGHUP; Ctrl+C is ignored while cleaning up): torque off,
   operating mode 3 (position, as before grip-2; user), the log saved.

Addresses and units: ROBOTIS e-Manual, MX-106T/R(2.0) control table.
Output: data/gripper/trigger_gripper_<time>.csv (one row per gripper cycle, GripperWorker.ROW_HEADER), a summary.
Usage:
    python scripts/28_trigger_gripper.py                       # measure open / closed by hand
    python scripts/28_trigger_gripper.py --closed 3968 --open 2196 --current 130
    python scripts/28_trigger_gripper.py --trigger right
"""
import argparse
import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
import numpy as np

import config

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.gripper import (CURRENT_UNIT_MA, GOAL_CURRENT, MODE_AT_END, MODE_CURRENT_POSITION, OPERATING_MODE,  # noqa: E402
                            PRESENT_POSITION, PROFILE_ACC, PROFILE_VEL, TICK_DEG, TORQUE_ENABLE, VELOCITY_LIMIT,
                            GripCommand, GripperWorker, Motor, check_motor, start_motor)
from teleop.leader import LeaderArm, read_latency_timer  # noqa: E402

OUT_DIR = ROOT / "data" / "gripper"
PERIOD_S = 0.02                # 50 Hz, like the teleop (trigger reads here, gripper cycles in its thread)
TRIGGER_FULL = 1000.0          # trigger 0 = closed, 1000 = fully open
GOAL_DEADBAND = 2              # ticks: write the goal only when it changes by this much
PROFILE_VELOCITY = 210         # x 0.229 rpm = 48.1 rpm = the motor's Velocity Limit (same as configs/*.yaml; was 200, 100)
PROFILE_ACCELERATION = 200     # x 214.577 rev/min^2: 0 -> 48 rpm in about 0.07 s (same as configs/*.yaml; was 100, 50)
CURRENT_DEFAULT = 200          # x 3.36 mA = 0.67 A (stall 5.2 A at 12 V), same as configs/*.yaml (2026-10-19; was 150, 100).
                               # It also caps the acceleration: at 150 the start was at the cap (teleop 084105)
CURRENT_MAX = 300              # 1.0 A: more needs its own decision
MIN_VOLTAGE = 10.0             # e-Manual operating range 10.0-14.8 V
MAX_TEMPERATURE = 70           # C (motor shutdown at 80)
MIN_RANGE = 50                 # ticks between open and closed (4.4 deg)
CATCH_FRAC = 0.05              # follow the trigger once it points within 5 % of the range of the position
FAIL_STOP = 3


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def to_keyboard_interrupt(signum, frame):
    """SIGTERM / SIGHUP (terminal closed, kill) end the run like Ctrl+C, so the cleanup runs."""
    raise KeyboardInterrupt


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
    gripper = Motor(config.GRIPPER_PORT)
    problems = gripper.open()
    if problems:
        leader.close()
        raise SystemExit("\n".join(problems))
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, to_keyboard_interrupt)
    mode_at_start = None
    changed = False
    worker, stop_reason = None, "Ctrl+C"
    try:
        problems = check_motor(gripper, MIN_VOLTAGE)
        if problems:
            raise SystemExit("\n".join(problems))
        mode_at_start = gripper.read(OPERATING_MODE)
        s = gripper.state()
        vlimit = gripper.read(VELOCITY_LIMIT)
        if vlimit is None or PROFILE_VELOCITY > vlimit:
            raise SystemExit(f"profile velocity {PROFILE_VELOCITY} is above the motor's velocity limit {vlimit}")
        torque = gripper.read(TORQUE_ENABLE)
        print(f"gripper: MX-106(2.0) ID {gripper.id}, mode {mode_at_start}, torque {torque}, position {s[2]}, "
              f"{s[3]:.1f} V, {s[4]} C")
        if torque != 0 or mode_at_start != MODE_AT_END:
            print(f"  left by an earlier run that did not finish (expected torque 0, mode {MODE_AT_END})")

        if args.closed is None and torque != 0:
            print("Measuring by hand needs torque off: the gripper becomes free (it may open or close by itself).")
            if ask("Type yes to turn the torque off: ") != "yes":
                raise SystemExit("not confirmed; nothing written")
            changed = True   # from here the cleanup also puts the mode back
            e = gripper.write(TORQUE_ENABLE, 0)
            if e:
                raise SystemExit(e)
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
        print(f"  operating mode         {mode_at_start} -> {MODE_CURRENT_POSITION} (current-based position; {MODE_AT_END} at the end)")
        print(f"  profile velocity       {gripper.read(PROFILE_VEL)} -> {PROFILE_VELOCITY} ({PROFILE_VELOCITY * 0.229:.1f} rpm)")
        print(f"  profile acceleration   {gripper.read(PROFILE_ACC)} -> {PROFILE_ACCELERATION}")
        print(f"  goal current           {gripper.read(GOAL_CURRENT)} -> {args.current} ({args.current * CURRENT_UNIT_MA:.0f} mA, force cap)")
        print(f"  goal position          -> {pos} (present position, no jump)")
        print("  torque enable          -> 1")
        print("Keep fingers and objects out of the gripper.")
        if ask("Type yes to write and start: ") != "yes":
            raise SystemExit("not confirmed; nothing written")

        changed = True
        held, e = start_motor(gripper, pos, args.current, PROFILE_VELOCITY, PROFILE_ACCELERATION)
        if e:
            raise SystemExit(e)
        command = GripCommand(closed, opened, held, CATCH_FRAC)
        worker = GripperWorker(gripper, command, PERIOD_S, GOAL_DEADBAND, MIN_VOLTAGE, MAX_TEMPERATURE, FAIL_STOP,
                               low_voltage_stops=True)
        need = command.frac_of(held) * TRIGGER_FULL
        print(f"\nrunning: {args.trigger} trigger 0 = closed, 1000 = open. Ctrl+C to stop.")
        print(f"catch: holding at {held}; move the {args.trigger} trigger to about {need:.0f} to start following\n")
        worker.start()

        trigger, trigger_fails, n = None, 0, 0
        next_t = time.monotonic()
        while True:
            raw = read_trigger_raw()
            if raw is not None:
                trigger = (raw >> 16) & 0xFFFF
                trigger_fails = 0
            else:
                trigger_fails += 1
            worker.set(None if raw is None else trigger / TRIGGER_FULL)
            for event in worker.events():
                print(f"\r{event}" + " " * 60)
            if not worker.is_alive():
                stop_reason = worker.stop_reason or "gripper thread ended"
                break
            if trigger_fails >= FAIL_STOP:
                stop_reason = f"{FAIL_STOP} failed {args.trigger} trigger reads in a row"
                break
            n += 1
            r = worker.latest()
            if n % 10 == 0 and r and r["position"] is not None:
                mode_txt = "follow" if r["following"] else f"catch: trigger -> {need:.0f}"
                print(f"\r{mode_txt}  trigger {trigger if trigger is not None else '-':>5}  goal {r['goal']:5d}  "
                      f"position {r['position']:5d}  current {r['current_mA']:6.0f} mA  {r['voltage_V']:4.1f} V  "
                      f"{r['temperature_C']} C  work {r['work_ms']:4.1f} ms   ", end="", flush=True)
            next_t += PERIOD_S
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    except Exception as e:   # e.g. termios.error when a USB adapter disconnects
        stop_reason = f"error: {e!r}"
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl+C must not skip the cleanup
        if changed:
            torque_now, mode_now = worker.finish() if worker else gripper.end_state()
            print(f"\n\nstopped: {stop_reason}. read back: torque {torque_now}, operating mode {mode_now}")
            if torque_now != 0 or mode_now != MODE_AT_END:
                print(f"WARNING: the gripper is NOT in torque 0 / mode {MODE_AT_END}: run this script again (it turns "
                      "them off with the confirmed writes) or use DYNAMIXEL Wizard")
        for port in (gripper, leader):
            try:
                port.close()
            except Exception:
                pass

    rows = worker.rows if worker else []
    if rows:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        path = OUT_DIR / f"trigger_gripper_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(path, "w", newline="") as f:
            csv.writer(f).writerows([GripperWorker.ROW_HEADER] + worker.csv_rows())
        good = [r for r in rows if r["position"] is not None]
        work = np.array([r["work_ms"] for r in rows])
        fracs = [r["open_frac"] for r in rows if r["open_frac"] is not None]
        age = np.array([r["frac_age_ms"] for r in rows if r["frac_age_ms"] is not None])
        print(f"saved {path.relative_to(ROOT)} ({len(rows)} gripper cycles, {rows[-1]['t_s']:.1f} s)")
        if fracs:
            print(f"trigger min / max {min(fracs) * TRIGGER_FULL:.0f} / {max(fracs) * TRIGGER_FULL:.0f}; "
                  f"trigger value age p50 / max {np.percentile(age, 50):.1f} / {age.max():.1f} ms")
        if good:
            cur = np.array([abs(r["current_mA"]) for r in good])
            volt = np.array([r["voltage_V"] for r in good])
            print(f"|current| p50 / max {np.percentile(cur, 50):.0f} / {cur.max():.0f} mA; voltage min / max "
                  f"{volt.min():.1f} / {volt.max():.1f} V")
        print(f"gripper work per cycle p50 / p99 / max {np.percentile(work, 50):.1f} / {np.percentile(work, 99):.1f} / "
              f"{work.max():.1f} ms; failed gripper reads {len(rows) - len(good)}")


if __name__ == "__main__":
    main()

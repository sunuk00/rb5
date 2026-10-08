"""Servo keyboard jog: stream move_servo_j targets in a fixed 20 Hz loop.

Same keys as 05_key_input.py, but instead of short move_j steps, the script keeps a
target pose and sends it with move_servo_j every cycle. Holding Left / Right moves the
target at a fixed speed; releasing the key freezes the target, so the robot stops.

Keys:
    Up / Down     select joint J1..J6 (one step per press, no wrap-around)
    Left / Right  hold to jog the selected joint -/+ (robot modes only)
    d o n e       type "done" to return to HOME and exit (robot modes), or just exit
    Ctrl+C        stop the robot and exit

Usage:
    python scripts/07_servo_jog.py          # keyboard only, no robot connection
    python scripts/07_servo_jog.py --sim    # Simulation mode
    python scripts/07_servo_jog.py --real   # Real mode (asks for typed confirmation)
    python scripts/07_servo_jog.py --device /dev/input/event3

move_servo_j parameters come from observation in 06_servo_j_probe.py, not from a manual:
t1 should match the send period (50 ms here); a longer gap than t1 made the robot jump.
The speed bar does not seem to limit servo motion, so the jog speed is limited in code.

Requires read access to /dev/input/event* (user in the 'input' group).
Note: evdev reads the keyboard device directly, so keys pressed while another
window has focus are read too.
"""
import argparse
import csv
import sys
import termios
import time
from datetime import datetime
from pathlib import Path

import evdev
from evdev import ecodes
import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

KEYBOARD_NAME = "AT Translated Set 2 keyboard"   # laptop keyboard
LOOP_HZ = 20
PERIOD_S = 1.0 / LOOP_HZ

HOME = [87.0, -2.5, -145.0, 137.0, 3.5, 14.0]    # deg, J1..J6

# move_servo_j parameters (observed in 06, meaning not confirmed by a manual)
SERVO_T1 = PERIOD_S        # matches the send period
SERVO_T2 = 0.1
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0

JOG_SPEED_SIM = 5.0        # deg/s, target speed while a key is held (Simulation)
JOG_SPEED_REAL = 2.0       # deg/s, lower for Real mode (collision detection is off)
MAX_LEAD_DEG = 1.0         # do not move the target further than this ahead of the robot
MAX_GAP_S = 1.5 * SERVO_T1 # a send gap longer than this freezes the target for that cycle
MAX_LATE_CYCLES = 3        # this many late cycles in a row -> stop and exit
SETTLED_DEG = 0.01         # robot counts as stopped when within this of the target
HOLD_ON_EXIT_S = 0.3       # keep sending the frozen target this long before ending servo mode

JOINT_SPEED = 70.0         # move_j speed for the return to HOME (deg/s), scaled by the speed bar
JOINT_ACC = 20.0           # move_j acceleration (deg/s^2)
HOME_TIMEOUT_S = 300.0     # give up waiting for the return-home move after this long
DATA_DIR = ROOT / "data"

ARROW_KEYS = [ecodes.KEY_LEFT, ecodes.KEY_RIGHT, ecodes.KEY_UP, ecodes.KEY_DOWN]
DONE_KEYS = [ecodes.KEY_D, ecodes.KEY_O, ecodes.KEY_N, ecodes.KEY_E]
KEY_LABELS = {ecodes.KEY_LEFT: "←", ecodes.KEY_RIGHT: "→", ecodes.KEY_UP: "↑", ecodes.KEY_DOWN: "↓"}


def find_keyboard(path=None):
    if path:
        return evdev.InputDevice(path)
    candidates = []
    for p in evdev.list_devices():
        dev = evdev.InputDevice(p)
        keys = dev.capabilities().get(ecodes.EV_KEY, [])
        if all(k in keys for k in ARROW_KEYS + DONE_KEYS):
            candidates.append(dev)
        else:
            dev.close()
    if not candidates:
        raise SystemExit("No keyboard found. Is this user in the 'input' group (log out and in after adding)?")
    chosen = next((d for d in candidates if d.name == KEYBOARD_NAME), candidates[0])
    for d in candidates:
        if d is not chosen:
            d.close()
    return chosen


def disable_echo():
    """Hide arrow-key escape codes (^[[D) in the terminal. Returns the old settings."""
    if not sys.stdin.isatty():
        return None
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new)
    return old


def restore_terminal(old):
    if old is None:
        return
    fd = sys.stdin.fileno()
    termios.tcflush(fd, termios.TCIFLUSH)   # drop keys typed during the loop so the shell does not get them
    termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_state(data_channel, simulation):
    # jnt_ang (measured) does not change in Simulation mode; use jnt_ref (commanded) there.
    sdata = data_channel.request_data(1.0).sdata
    q = np.array(sdata.jnt_ref if simulation else sdata.jnt_ang, dtype=float)
    return q, sdata.robot_state


def wait_until_stopped(robot, rc, timeout_s):
    """Poll get_robot_state() until the robot is no longer moving."""
    t0 = time.monotonic()
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        if time.monotonic() - t0 > timeout_s:
            raise SystemExit(f"Robot still moving after {timeout_s} s.")
        time.sleep(0.01)


def send_servo(robot, rc, target):
    robot.move_servo_j(rc, target, SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)


def stop_streaming(robot, rc, target):
    """Normal end of servo mode: hold the target, then finish as in examples/move_servo_j.cpp."""
    next_t = time.monotonic()
    t_end = next_t + HOLD_ON_EXIT_S
    while time.monotonic() < t_end:
        send_servo(robot, rc, target)
        next_t += PERIOD_S
        time.sleep(max(0.0, next_t - time.monotonic()))
    robot.move_speed_j(rc, np.zeros(6), SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
    robot.enable_waiting_ack(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc, 10.0)
    rc.clear()


def emergency_stop(robot, rc):
    """End servo mode after an error: no more servo commands, task_stop and poll."""
    robot.enable_waiting_ack(rc)
    robot.task_stop(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc, 10.0)
    rc.clear()


def run_loop(kbd, robot, rc, data_channel, simulation, jog_speed, ctx):
    """Fixed-rate loop. Returns True if "done" was typed.

    ctx["target"] holds the servo target; ctx["cycles"] gets one row per cycle:
    (cycle, time since start, loop interval, servo send interval), intervals in seconds or None.
    """
    selected = 0               # joint index 0..5 -> J1..J6
    left = right = False       # held state of the Left / Right keys
    press_cycle = {}           # cycle at which Left / Right was pressed, to report hold time
    recent = []                # last pressed keys, to detect "done"
    done = False
    step = jog_speed * PERIOD_S   # target change per cycle while a key is held
    moving = False             # target moved since the robot last settled
    blocked = False            # target is waiting for the robot (lead limit)
    t_release = None           # time the jog keys were released, to report stop latency
    last_send = None
    late_cycles = 0

    q_start = None
    if robot is not None:
        q_start = read_state(data_channel, simulation)[0]
        ctx["target"] = q_start.copy()
    target = ctx["target"]

    t_start = time.monotonic()
    next_t = t_start
    cycle = 0
    prev_cycle_t = None
    while not done:
        t_cycle = time.monotonic()
        loop_interval = None if prev_cycle_t is None else t_cycle - prev_cycle_t
        prev_cycle_t = t_cycle
        send_interval = None
        events = []            # what happened this cycle; a line is printed only if not empty

        # 1. Process every key event queued since the last cycle, without waiting
        try:
            for ev in kbd.read():
                if ev.type != ecodes.EV_KEY or ev.value == 2:   # 2 = auto-repeat, ignored
                    continue
                if ev.code in (ecodes.KEY_LEFT, ecodes.KEY_RIGHT):
                    label = KEY_LABELS[ev.code]
                    if ev.value == 1:
                        press_cycle[ev.code] = cycle
                        events.append(f"{label} press")
                    else:
                        held = cycle - press_cycle.get(ev.code, cycle)
                        events.append(f"{label} release (held {held} cycles = {held * PERIOD_S:.2f} s)")
                    if ev.code == ecodes.KEY_LEFT:
                        left = ev.value == 1
                    else:
                        right = ev.value == 1
                elif ev.code in (ecodes.KEY_UP, ecodes.KEY_DOWN) and ev.value == 1:
                    step_sel = 1 if ev.code == ecodes.KEY_UP else -1
                    selected = min(max(selected + step_sel, 0), 5)
                    events.append(f"{KEY_LABELS[ev.code]} press -> J{selected + 1}")
                if ev.value == 1:
                    recent = (recent + [ev.code])[-len(DONE_KEYS):]
                    if recent == DONE_KEYS:
                        done = True
                        events.append("'done' typed")
        except BlockingIOError:
            pass   # no events queued

        # 2. Servo: move the target while a key is held, and send it every cycle
        q = None
        if robot is not None:
            # ACK waiting is off, so read queued controller messages here and stop on any error
            robot.flush(rc)
            rc.error().throw_if_not_empty()
            rc.clear()

            q, _ = read_state(data_channel, simulation)
            now = time.monotonic()
            late = last_send is not None and now - last_send > MAX_GAP_S
            if late:
                late_cycles += 1
                events.append(f"late: {(now - last_send) * 1000:.0f} ms since last send, target frozen")
                if late_cycles >= MAX_LATE_CYCLES:
                    raise RuntimeError(f"{MAX_LATE_CYCLES} late cycles in a row; loop too slow for servo")
            else:
                late_cycles = 0

            direction = 0 if done else int(right) - int(left)   # both held -> 0 -> no motion
            if direction != 0 and not late:
                new = target[selected] + direction * step
                if abs(new - q[selected]) <= MAX_LEAD_DEG:
                    target[selected] = new
                    moving = True
                    t_release = None
                    blocked = False
                elif not blocked:
                    blocked = True
                    events.append(f"target waits for robot (lead limit {MAX_LEAD_DEG} deg)")
            elif direction == 0 and moving and t_release is None:
                t_release = now

            send_servo(robot, rc, target)
            t_send = time.monotonic()
            if last_send is not None:
                send_interval = t_send - last_send
            last_send = t_send

            if moving and direction == 0 and np.max(np.abs(target - q)) < SETTLED_DEG:
                moving = False
                blocked = False
                events.append(f"stopped ({(now - t_release) * 1000:.0f} ms after release)")

        # 3. Print only when something happened
        if events:
            if q is None:
                joint_txt = "no robot"
            else:
                dq = q[selected] - q_start[selected]
                lead = target[selected] - q[selected]
                joint_txt = (f"J{selected + 1} = {q[selected]:8.2f} deg ({dq:+6.2f} from start, "
                             f"lead {lead:+.2f})")
            print(f"[{cycle:5d} | {time.monotonic() - t_start:6.2f} s]  selected J{selected + 1}  |  "
                  f"{joint_txt}  |  {', '.join(events)}")

        ctx["cycles"].append((cycle, t_cycle - t_start, loop_interval, send_interval))

        # 4. Sleep until the next cycle (absolute schedule, so timing errors do not accumulate)
        cycle += 1
        next_t += PERIOD_S
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            # Overran: wait a full period from now instead of starting the next cycle at once,
            # so two servo commands are never sent back to back
            next_t = time.monotonic() + PERIOD_S
            time.sleep(PERIOD_S)
    return done


def report_intervals(rows, mode_name):
    """Print loop / send interval statistics and save every cycle's intervals to CSV."""
    if len(rows) < 2:
        return
    limit_ms = MAX_GAP_S * 1000
    print()
    for name, col in (("Loop interval", 2), ("Servo send interval", 3)):
        ms = np.array([r[col] for r in rows if r[col] is not None]) * 1000
        if len(ms) == 0:
            continue
        print(f"{name:<20}: mean {ms.mean():6.2f} ms   min {ms.min():6.2f} ms   max {ms.max():6.2f} ms   "
              f"(n={len(ms)}, over {limit_ms:.0f} ms: {np.sum(ms > limit_ms)})")

    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"07_servo_jog_{mode_name}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["cycle", "time_s", "loop_interval_ms", "send_interval_ms"])
        for cycle, t, loop_iv, send_iv in rows:
            writer.writerow([cycle, f"{t:.4f}",
                             "" if loop_iv is None else f"{loop_iv * 1000:.3f}",
                             "" if send_iv is None else f"{send_iv * 1000:.3f}"])
    print(f"Saved: {path.relative_to(ROOT)}")


def return_home(robot, rc, data_channel, simulation):
    q, _ = read_state(data_channel, simulation)
    q_home = np.array(HOME)
    print("\nReturn to HOME")
    print("Current:", q)
    print("Home:   ", q_home)
    print("Delta:  ", q_home - q)
    if not simulation:
        if input("REAL mode: the robot will move to HOME. Type 'yes' to continue: ") != "yes":
            print("Cancelled. Robot stays where it is.")
            return
    try:
        robot.flush(rc)
        robot.move_j(rc, q_home, JOINT_SPEED, JOINT_ACC)
        if robot.wait_for_move_started(rc, 0.5).type() != rb.ReturnType.Success:
            print("warning: move start was not detected within 0.5 s")
        wait_until_stopped(robot, rc, HOME_TIMEOUT_S)
        rc.error().throw_if_not_empty()
    except KeyboardInterrupt:
        print("\nInterrupted. Stopping.")
        robot.task_stop(rc)
        wait_until_stopped(robot, rc, 10.0)
        return
    q_after, _ = read_state(data_channel, simulation)
    print("Reached:", q_after)
    print("Error:  ", q_after - q_home)


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="connect in Simulation mode")
    mode.add_argument("--real", action="store_true", help="connect in Real mode")
    parser.add_argument("--device", help="evdev device path, e.g. /dev/input/event3")
    args = parser.parse_args()
    simulation = not args.real
    jog_speed = JOG_SPEED_SIM if simulation else JOG_SPEED_REAL
    np.set_printoptions(precision=2, suppress=True)

    kbd = find_keyboard(args.device)
    print(f"Keyboard: {kbd.name} ({kbd.path})")

    robot = rc = data_channel = None
    if args.sim or args.real:
        robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
        rc = rb.ResponseCollector()
        data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001)
        robot.set_operation_mode(rc, rb.OperationMode.Simulation if simulation else rb.OperationMode.Real)
        robot.set_speed_bar(rc, config.SPEED)
        rc.error().throw_if_not_empty()
        rc.clear()

        q, _ = read_state(data_channel, simulation)
        print(f"Mode: {'Simulation' if simulation else 'Real'}   jog speed {jog_speed} deg/s "
              f"({jog_speed * PERIOD_S:.3f} deg per cycle)   servo t1={SERVO_T1} t2={SERVO_T2} "
              f"gain={SERVO_GAIN} alpha={SERVO_ALPHA}")
        print("Current:", q)
        print("Home:   ", np.array(HOME))
        if args.real:
            if input("REAL mode: arrow keys will move the robot. Type 'yes' to continue: ") != "yes":
                raise SystemExit("Cancelled.")
    else:
        print("Mode: keyboard only (no robot connection)")
    print("Up/Down: select joint   Left/Right: hold to jog   type 'done': return HOME and exit   Ctrl+C: stop")
    print("A line is printed only when a key is pressed/released or the robot stops.")
    print("[cycle | time]  selected joint  |  its current angle (lead = target - angle)  |  events\n")

    old_term = disable_echo()
    ctx = {"target": None, "cycles": []}
    done = False
    error = None
    if robot is not None:
        robot.disable_waiting_ack(rc)   # servo commands must not wait for an ACK each cycle
    try:
        done = run_loop(kbd, robot, rc, data_channel, simulation, jog_speed, ctx)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:   # controller error or late loop: stop without more servo commands
        error = e
        print(f"\nError: {e}")
    finally:
        if robot is not None:
            if error is None and ctx["target"] is not None:
                stop_streaming(robot, rc, ctx["target"])
            else:
                emergency_stop(robot, rc)
        restore_terminal(old_term)
        kbd.close()

    mode_name = "sim" if args.sim else "real" if args.real else "keyboard"
    report_intervals(ctx["cycles"], mode_name)

    if done and robot is not None and error is None:
        return_home(robot, rc, data_channel, simulation)
    print("Exit.")


if __name__ == "__main__":
    main()

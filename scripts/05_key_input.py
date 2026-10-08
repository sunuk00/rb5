"""Keyboard jog: read the laptop keyboard with evdev in a fixed 20 Hz loop.

Keys:
    Up / Down     select joint J1..J6 (one step per press, no wrap-around)
    Left / Right  hold to jog the selected joint -/+ (robot modes only)
    d o n e       type "done" to return to HOME and exit (robot modes), or just exit
    Ctrl+C        stop the robot and exit

Usage:
    python scripts/05_key_input.py          # keyboard only, no robot connection
    python scripts/05_key_input.py --sim    # Simulation mode
    python scripts/05_key_input.py --real   # Real mode (asks for typed confirmation)
    python scripts/05_key_input.py --device /dev/input/event3

Requires read access to /dev/input/event* (user in the 'input' group).
Note: evdev reads the keyboard device directly, so keys pressed while another
window has focus are read too.
"""
import argparse
import sys
import termios
import time
from pathlib import Path

import evdev
from evdev import ecodes
import numpy as np
import rbpodo as rb

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config  # noqa: E402

KEYBOARD_NAME = "AT Translated Set 2 keyboard"   # laptop keyboard
LOOP_HZ = 20
PERIOD_S = 1.0 / LOOP_HZ

HOME = [87.0, -2.5, -145.0, 137.0, 3.5, 14.0]    # deg, J1..J6
JOG_STEP_DEG = 0.5         # each jog command moves the selected joint this much
JOINT_SPEED = 70.0         # move_j speed (deg/s), scaled by the speed bar
JOINT_ACC = 20.0           # move_j acceleration (deg/s^2)
START_TIMEOUT_S = 0.5      # stop waiting for a jog command to show up as "moving"
HOME_TIMEOUT_S = 300.0     # give up waiting for the return-home move after this long
STATE_IDLE = 1             # sdata.robot_state: 1 = idle, 3 = executing motion
STATE_MOVING = 3

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


def stop_robot(robot, rc):
    robot.task_stop(rc)
    wait_until_stopped(robot, rc, 10.0)
    robot.flush(rc)
    rc.clear()


def run_loop(kbd, robot, rc, data_channel, simulation):
    """Fixed-rate loop. Returns True if "done" was typed."""
    selected = 0               # joint index 0..5 -> J1..J6
    left = right = False       # held state of the Left / Right keys
    press_cycle = {}           # cycle at which Left / Right was pressed, to report hold time
    recent = []                # last pressed keys, to detect "done"
    done = False
    pending_since = None       # time a jog command was sent but not yet seen as moving
    prev_state = None
    q_start = read_state(data_channel, simulation)[0] if robot else None

    t_start = time.monotonic()
    next_t = t_start
    cycle = 0
    while not done:
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
                    step = 1 if ev.code == ecodes.KEY_UP else -1
                    selected = min(max(selected + step, 0), 5)
                    events.append(f"{KEY_LABELS[ev.code]} press -> J{selected + 1}")
                if ev.value == 1:
                    recent = (recent + [ev.code])[-len(DONE_KEYS):]
                    if recent == DONE_KEYS:
                        done = True
                        events.append("'done' typed")
        except BlockingIOError:
            pass   # no events queued

        # 2. Jog: one small move_j at a time, only while a key is held and the robot is idle
        q = None
        if robot is not None:
            q, state = read_state(data_channel, simulation)
            now = time.monotonic()
            if pending_since is not None and (state == STATE_MOVING or now - pending_since > START_TIMEOUT_S):
                pending_since = None
            direction = int(right) - int(left)   # both held -> 0 -> no motion
            if direction != 0 and not done and state == STATE_IDLE and pending_since is None:
                target = q.copy()
                target[selected] += direction * JOG_STEP_DEG
                robot.flush(rc)
                robot.move_j(rc, target, JOINT_SPEED, JOINT_ACC)
                rc.error().throw_if_not_empty()
                rc.clear()
                pending_since = time.monotonic()
                events.append(f"move_j {direction * JOG_STEP_DEG:+.1f} deg")
            if prev_state == STATE_MOVING and state == STATE_IDLE and direction == 0:
                events.append("stopped")
            prev_state = state

        # 3. Print only when something happened
        if events:
            if q is None:
                joint_txt = "no robot"
            else:
                dq = q[selected] - q_start[selected]
                joint_txt = f"J{selected + 1} = {q[selected]:8.2f} deg ({dq:+6.2f} from start)"
            print(f"[{cycle:5d} | {time.monotonic() - t_start:6.2f} s]  selected J{selected + 1}  |  "
                  f"{joint_txt}  |  {', '.join(events)}")

        # 3. Sleep until the next cycle (absolute schedule, so timing errors do not accumulate)
        cycle += 1
        next_t += PERIOD_S
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            print(f"warning: cycle overran by {-delay * 1000:.1f} ms")
            next_t = time.monotonic()
    return done


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
        stop_robot(robot, rc)
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
        print(f"Mode: {'Simulation' if simulation else 'Real'}   speed bar {config.SPEED}   "
              f"jog step {JOG_STEP_DEG} deg   move_j {JOINT_SPEED} deg/s")
        print("Current:", q)
        print("Home:   ", np.array(HOME))
        if args.real:
            if input("REAL mode: arrow keys will move the robot. Type 'yes' to continue: ") != "yes":
                raise SystemExit("Cancelled.")
    else:
        print("Mode: keyboard only (no robot connection)")
    print("Up/Down: select joint   Left/Right: hold to jog   type 'done': return HOME and exit   Ctrl+C: stop")
    print("A line is printed only when a key is pressed/released or the robot starts/stops a jog step.")
    print("[cycle | time]  selected joint  |  its current angle  |  events\n")

    old_term = disable_echo()
    done = False
    try:
        done = run_loop(kbd, robot, rc, data_channel, simulation)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        restore_terminal(old_term)
        if robot is not None:
            stop_robot(robot, rc)
        kbd.close()

    if done and robot is not None:
        return_home(robot, rc, data_channel, simulation)
    print("Exit.")


if __name__ == "__main__":
    main()

"""Faster servo keyboard jog: 07 with a background state reader and a shorter period.

Same behavior as 07_servo_jog.py, with two changes:
  1. A background thread reads the data channel continuously and keeps the latest state,
     so the control loop never blocks on request_data() (about 10 ms per call).
  2. The loop period is selectable (--period 0.05 / 0.02 / 0.01, default 0.02 s);
     move_servo_j t1 is set to the same value, as tested in 06_servo_j_probe.py.

Keys:
    Up / Down     select joint J1..J6 (one step per press, no wrap-around)
    Left / Right  hold to jog the selected joint -/+ (robot modes only)
    d o n e       type "done" to return to HOME and exit (robot modes), or just exit
    Ctrl+C        stop the robot and exit

Usage:
    python scripts/08_servo_jog_fast.py                   # keyboard only, no robot connection
    python scripts/08_servo_jog_fast.py --sim             # Simulation mode, 20 ms period
    python scripts/08_servo_jog_fast.py --sim --period 0.01
    python scripts/08_servo_jog_fast.py --real            # Real mode (asks for typed confirmation)
    python scripts/08_servo_jog_fast.py --device /dev/input/event3

move_servo_j parameters come from observation in 06_servo_j_probe.py, not from a manual:
t1 should match the send period; a longer gap than t1 made the robot jump.
The speed bar does not seem to limit servo motion, so the jog speed is limited in code.

Requires read access to /dev/input/event* (user in the 'input' group).
Note: evdev reads the keyboard device directly, so keys pressed while another
window has focus are read too.
"""
import argparse
import csv
import sys
import termios
import threading
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
PERIOD_CHOICES = (0.05, 0.02, 0.01)   # periods tested in 06 with t1 = period
PERIOD_S = 0.02            # loop period; changed by set_period() from --period

HOME = [87.0, -2.5, -145.0, 137.0, 3.5, 14.0]    # deg, J1..J6

# move_servo_j parameters (observed in 06, meaning not confirmed by a manual)
SERVO_T1 = PERIOD_S        # matches the send period (set_period keeps them equal)
SERVO_T2 = 0.1
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0

JOG_SPEED_SIM = 5.0        # deg/s, target speed while a key is held (Simulation)
JOG_SPEED_REAL = 2.0       # deg/s, lower for Real mode (collision detection is off)
MAX_LEAD_DEG = 1.0         # do not move the target further than this ahead of the robot
MAX_GAP_S = 1.5 * SERVO_T1 # a send gap longer than this freezes the target for that cycle
MAX_LATE_CYCLES = 3        # this many late cycles in a row -> stop and exit
MAX_STATE_AGE_S = 0.1      # a state sample older than this freezes the target for that cycle
MAX_STALE_CYCLES = 3       # this many stale-state cycles in a row -> stop and exit
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


def set_period(period):
    """Set the loop period; t1 and the late-send limit follow it."""
    global PERIOD_S, SERVO_T1, MAX_GAP_S
    PERIOD_S = period
    SERVO_T1 = period
    MAX_GAP_S = 1.5 * SERVO_T1


class StateReader(threading.Thread):
    """Reads the data channel continuously on its own socket and keeps only the latest sample."""

    def __init__(self, simulation):
        super().__init__(daemon=True)
        self.data_channel = rb.CobotData(config.ROBOT_IP)
        self.simulation = simulation
        self.lock = threading.Lock()
        self.sample = None     # (q, robot_state, receive time)
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            data = self.data_channel.request_data(1.0)
            if data is None:
                continue
            s = data.sdata
            # jnt_ang (measured) does not change in Simulation mode; use jnt_ref (commanded) there.
            q = np.array(s.jnt_ref if self.simulation else s.jnt_ang, dtype=float)
            with self.lock:
                self.sample = (q, s.robot_state, time.monotonic())

    def wait_first(self, timeout_s=2.0):
        t0 = time.monotonic()
        while self.sample is None:
            if time.monotonic() - t0 > timeout_s:
                raise SystemExit("State reader got no data from the robot.")
            time.sleep(0.01)

    def latest(self):
        """Return (q, robot_state, age in seconds) of the newest sample, without waiting."""
        with self.lock:
            q, state, t = self.sample
        return q.copy(), state, time.monotonic() - t

    def stop(self):
        self.stop_event.set()
        self.join(2.0)


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


def run_loop(kbd, robot, rc, reader, jog_speed, ctx):
    """Fixed-rate loop. Returns True if "done" was typed.

    ctx["target"] holds the servo target; ctx["cycles"] gets one row per cycle:
    (cycle, time since start, loop interval, servo send interval, state age), in seconds or None.
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
    stale_cycles = 0

    q_start = None
    if robot is not None:
        q_start = reader.latest()[0]
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
        state_age = None
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

            q, _, state_age = reader.latest()   # newest sample from the reader thread, no waiting
            now = time.monotonic()
            late = last_send is not None and now - last_send > MAX_GAP_S
            if late:
                late_cycles += 1
                events.append(f"late: {(now - last_send) * 1000:.0f} ms since last send, target frozen")
                if late_cycles >= MAX_LATE_CYCLES:
                    raise RuntimeError(f"{MAX_LATE_CYCLES} late cycles in a row; loop too slow for servo")
            else:
                late_cycles = 0
            stale = state_age > MAX_STATE_AGE_S
            if stale:
                stale_cycles += 1
                events.append(f"stale state: {state_age * 1000:.0f} ms old, target frozen")
                if stale_cycles >= MAX_STALE_CYCLES:
                    raise RuntimeError(f"{MAX_STALE_CYCLES} stale-state cycles in a row; state reader stopped?")
            else:
                stale_cycles = 0

            direction = 0 if done else int(right) - int(left)   # both held -> 0 -> no motion
            if direction != 0 and not late and not stale:
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

        ctx["cycles"].append((cycle, t_cycle - t_start, loop_interval, send_interval, state_age))

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
    for name, col, lim in (("Loop interval", 2, limit_ms), ("Servo send interval", 3, limit_ms),
                           ("State age", 4, MAX_STATE_AGE_S * 1000)):
        ms = np.array([r[col] for r in rows if r[col] is not None]) * 1000
        if len(ms) == 0:
            continue
        print(f"{name:<20}: mean {ms.mean():6.2f} ms   min {ms.min():6.2f} ms   max {ms.max():6.2f} ms   "
              f"(n={len(ms)}, over {lim:.0f} ms: {np.sum(ms > lim)})")

    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"08_servo_jog_fast_{mode_name}_p{PERIOD_S * 1000:g}ms_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["cycle", "time_s", "loop_interval_ms", "send_interval_ms", "state_age_ms"])
        for cycle, t, *intervals in rows:
            writer.writerow([cycle, f"{t:.4f}"] + ["" if v is None else f"{v * 1000:.3f}" for v in intervals])
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
    parser.add_argument("--period", type=float, default=PERIOD_S, choices=PERIOD_CHOICES,
                        help="loop period in seconds; move_servo_j t1 uses the same value")
    args = parser.parse_args()
    set_period(args.period)
    simulation = not args.real
    jog_speed = JOG_SPEED_SIM if simulation else JOG_SPEED_REAL
    np.set_printoptions(precision=2, suppress=True)

    kbd = find_keyboard(args.device)
    print(f"Keyboard: {kbd.name} ({kbd.path})")

    robot = rc = data_channel = reader = None
    if args.sim or args.real:
        robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
        rc = rb.ResponseCollector()
        data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001)
        robot.set_operation_mode(rc, rb.OperationMode.Simulation if simulation else rb.OperationMode.Real)
        robot.set_speed_bar(rc, config.SPEED)
        rc.error().throw_if_not_empty()
        rc.clear()

        q, _ = read_state(data_channel, simulation)
        print(f"Mode: {'Simulation' if simulation else 'Real'}   period {PERIOD_S * 1000:g} ms   "
              f"jog speed {jog_speed} deg/s "
              f"({jog_speed * PERIOD_S:.3f} deg per cycle)   servo t1={SERVO_T1} t2={SERVO_T2} "
              f"gain={SERVO_GAIN} alpha={SERVO_ALPHA}")
        print("Current:", q)
        print("Home:   ", np.array(HOME))
        if args.real:
            if input("REAL mode: arrow keys will move the robot. Type 'yes' to continue: ") != "yes":
                raise SystemExit("Cancelled.")
        reader = StateReader(simulation)
        reader.start()
        reader.wait_first()
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
        done = run_loop(kbd, robot, rc, reader, jog_speed, ctx)
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
            reader.stop()
        restore_terminal(old_term)
        kbd.close()

    mode_name = "sim" if args.sim else "real" if args.real else "keyboard"
    report_intervals(ctx["cycles"], mode_name)

    if done and robot is not None and error is None:
        return_home(robot, rc, data_channel, simulation)
    print("Exit.")


if __name__ == "__main__":
    main()

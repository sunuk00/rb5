"""Leader arm (both arms, 14 joints) -> RB-Y1 simulator arms, with a clutch per arm.

RB-Y1 SIMULATOR ONLY (model M, default address localhost:50051). Run in the `rby1` conda env.

Leader arm (read-only: ping / read only, nothing is written to the Dynamixel bus):
    joints 0..13 with GroupSyncRead (address 132), tools 128 / 129 with separate reads + 100 us pause
    (rby1-sdk dynamixel_bus.cpp ReadButtonStatus). Angle: deg = raw * 360 / 4096.
Clutch per arm (128 -> right arm, 129 -> left arm), same debounce as 13:
    button = (raw >> 8) & 0xff; 1 = pressed, 0 = released, anything else = failed read.
    A press counts after 2 consecutive pressed reads; 3 failed reads in a row count as released.
    Starts released; a button held at start must be released once first.
Mapping per arm, per joint (13 "B" policy): target = robot start + (leader now - leader start),
    limited to MAX_STEP_DEG per cycle (the excess is dropped by shifting the leader start), then to
    start +/- RANGE_DEG and the model joint limits. Released arm = frozen target.
RB-Y1: power_on / servo_on / enable_control_manager (simulator only), then move to READY_POSE with
    move_j-style send_command (examples/python/35_leader_arm_teleop_with_monitor.py), then stream both
    arm targets every cycle with minimum_time = 1.01 * period and control_hold_time = 0.5 s.

Usage:
    python scripts/14_leader_rby1_teleop.py
Stop with Ctrl+C. Each run is logged to data/14_leader_rby1_<time>.csv
"""
import argparse
import csv
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
import numpy as np
import rby1_sdk as rby

ROOT = Path(__file__).resolve().parent.parent

# --- Loop and RB-Y1 command stream
PERIOD_S = 0.02            # 50 Hz. Example 35 uses 100 Hz, but one cycle here takes about 11.6 ms
                           # (leader 7.2 ms + send_command 4.2 ms, measured), so 10 ms does not fit
MINIMUM_TIME_S = PERIOD_S * 1.01   # example 35: max(minimum_time, leader_arm_loop_period * 1.01)
CONTROL_HOLD_TIME_S = 0.5  # chosen: a stopped stream does not keep a command alive for long
READY_MINIMUM_TIME_S = 5.0 # example 35: move_j(robot, READY_POSE[model.model_name], 5)
# example 35 READY_POSE["M"] (degrees)
READY_TORSO_DEG = [0.0, 45.0, -90.0, 45.0, 0.0, 0.0]
READY_RIGHT_DEG = [0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]
READY_LEFT_DEG = [0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]
POWER_PATTERN = ".*"                                  # example 35 --power default
SERVO_PATTERN = "torso_.*|right_arm_.*|left_arm_.*"   # example 35 --servo default

# --- Safety
MAX_STEP_DEG = 0.6         # per joint per cycle (30 deg/s at 50 Hz, same as 13 Simulation)
RANGE_DEG = 45.0           # per joint, around the pose after the READY move
MAX_GAP_S = 1.5 * PERIOD_S # a send gap longer than this freezes the targets for that cycle
MAX_LATE_CYCLES = 3
MAX_STATE_AGE_S = 0.1      # RB-Y1 state older than this freezes the targets
MAX_STALE_CYCLES = 3
MAX_LEADER_FAILS = 3       # leader joint sync read failures in a row -> stop
HOLD_ON_EXIT_S = 0.3

# --- Leader arm (rby1-sdk dynamixel_bus.h, leader_arm.cpp)
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
JOINT_IDS = list(range(14))   # 0..6 right arm, 7..13 left arm
TOOL_IDS = {"right": 0x80, "left": 0x81}
TOOL_READ_PAUSE_S = 100e-6
TICKS_PER_REV = 4096
BUTTON_PRESSED = 1
BUTTON_RELEASED = 0
PRESS_CONFIRM_READS = 2
FAILS_TO_RELEASE = 3

ARMS = ("right", "left")
LEADER_SLICE = {"right": slice(0, 7), "left": slice(7, 14)}
STATUS_EVERY = 25          # status line every this many cycles (2 Hz)
DATA_DIR = ROOT / "data"
CLEAR_LINE = "\r\x1b[2K"


class StopRequest(Exception):
    """Controlled stop: hold the targets briefly, then stop streaming."""


def event(msg):
    sys.stdout.write(CLEAR_LINE + msg + "\n")
    sys.stdout.flush()


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def read_latency_timer(port_name):
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


class LeaderArm:
    """Read-only access to the 14 leader arm joints and both tool buttons."""

    def __init__(self, port_name):
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        self.sync = None

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"Failed to open {self.port.getPortName()}")
        if not self.port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")
        missing = [i for i in JOINT_IDS + list(TOOL_IDS.values()) if self.ph.ping(self.port, i)[1] != COMM_SUCCESS]
        if missing:
            raise SystemExit(f"Leader arm IDs {missing} did not answer the ping.")
        self.sync = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for dxl_id in JOINT_IDS:
            self.sync.addParam(dxl_id)

    def read_joints_deg(self):
        """Return 14 joint angles (deg), or None if the sync read failed."""
        result = self.sync.txRxPacket()
        if result != COMM_SUCCESS or not all(
                self.sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in JOINT_IDS):
            return None
        raw = [to_signed32(self.sync.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) for i in JOINT_IDS]
        return np.array(raw, dtype=float) * 360.0 / TICKS_PER_REV

    def read_button_raw(self, tool_id):
        value, result, _ = self.ph.read4ByteTxRx(self.port, tool_id, ADDR_PRESENT_POSITION)
        time.sleep(TOOL_READ_PAUSE_S)
        return value if result == COMM_SUCCESS else None

    def close(self):
        self.port.closePort()


class Clutch:
    """Debounced clutch from a tool button (same rules as 13)."""

    def __init__(self):
        self.engaged = False
        self.armed = False
        self.press_count = 0
        self.fail_count = 0
        self.reads = 0
        self.failures = 0

    def update(self, raw):
        self.reads += 1
        button = None if raw is None else (raw >> 8) & 0xFF
        if button == BUTTON_PRESSED:
            self.fail_count = 0
            if self.armed:
                self.press_count += 1
                if not self.engaged and self.press_count >= PRESS_CONFIRM_READS:
                    self.engaged = True
                    return button, "engaged"
            return button, None
        if button == BUTTON_RELEASED:
            self.fail_count = 0
            self.press_count = 0
            if not self.armed:
                self.armed = True
                return button, "armed (button released, press to engage)"
            if self.engaged:
                self.engaged = False
                return button, "released"
            return button, None
        self.failures += 1
        self.fail_count += 1
        self.press_count = 0
        if self.engaged and self.fail_count >= FAILS_TO_RELEASE:
            self.engaged = False
            return button, f"released ({FAILS_TO_RELEASE} failed button reads in a row)"
        return button, None


class RobotState:
    """Latest RB-Y1 joint positions from start_state_update (latest-value pattern from 08)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.sample = None

    def callback(self, state):
        q = np.degrees(np.array(state.position, dtype=float))
        with self.lock:
            self.sample = (q, time.monotonic())

    def latest(self):
        with self.lock:
            q, t = self.sample
        return q.copy(), time.monotonic() - t


def arm_command(q_deg, minimum_time, hold_time, qdot_max, qddot_max):
    """JointPositionCommandBuilder as in example 35 (position mode)."""
    return (rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(hold_time))
            .set_position(np.radians(q_deg))
            .set_velocity_limit(qdot_max)
            .set_acceleration_limit(qddot_max * 30)
            .set_minimum_time(minimum_time))


def ready_command():
    """example 35 joint_position_command_builder(READY_POSE, minimum_time=5), position mode."""
    def jp(deg):
        return (rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(0))
                .set_position(np.radians(deg))
                .set_minimum_time(READY_MINIMUM_TIME_S))
    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(jp(READY_TORSO_DEG))
            .set_right_arm_command(jp(READY_RIGHT_DEG))
            .set_left_arm_command(jp(READY_LEFT_DEG))))


def prepare_simulator(robot):
    """Power, servo and control manager, following example 35 (simulator only)."""
    robot.reset_fault_control_manager()
    if not robot.is_power_on(POWER_PATTERN):
        print(f"power_on({POWER_PATTERN!r}) ...")
        if not robot.power_on(POWER_PATTERN):
            raise SystemExit("power_on failed")
        time.sleep(1.0)
    if not robot.is_servo_on(SERVO_PATTERN):
        print(f"servo_on({SERVO_PATTERN!r}) ...")
        if not robot.servo_on(SERVO_PATTERN):
            raise SystemExit("servo_on failed")
        time.sleep(0.3)
    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        raise SystemExit("enable_control_manager failed")


def run_loop(dev, stream, rstate, idx, lim, ctx):
    """Fixed-period loop. ctx["target"][arm] holds the 7-joint targets (deg), ctx["rows"] the CSV rows."""
    q0, _ = rstate.latest()
    base = {arm: q0[idx[arm]] for arm in ARMS}     # range is fixed around the pose after the READY move
    low = {arm: np.maximum(base[arm] - RANGE_DEG, lim["q_lo"][idx[arm]]) for arm in ARMS}
    high = {arm: np.minimum(base[arm] + RANGE_DEG, lim["q_hi"][idx[arm]]) for arm in ARMS}
    target = {arm: base[arm].copy() for arm in ARMS}
    ctx["target"] = target
    for arm in ARMS:
        event(f"{arm} arm allowed range (deg): low {np.round(low[arm], 1)}  high {np.round(high[arm], 1)}")

    clutch = {arm: Clutch() for arm in ARMS}
    ctx["clutch"] = clutch
    robot_start = {arm: None for arm in ARMS}
    leader_start = {arm: None for arm in ARMS}
    leader_last = None
    leader_fails = leader_fail_total = leader_reads = 0
    flags_prev = {(arm, k): False for arm in ARMS for k in ("rate", "range")}
    last_send = None
    late_cycles = stale_cycles = 0
    prev_cycle_t = None

    t_start = next_t = time.monotonic()
    cycle = 0
    while True:
        t_cycle = time.monotonic()
        interval = None if prev_cycle_t is None else t_cycle - prev_cycle_t
        prev_cycle_t = t_cycle

        # 1. Leader arm: 14 joints, then both buttons
        leader_reads += 1
        leader = dev.read_joints_deg()
        if leader is None:
            leader_fails += 1
            leader_fail_total += 1
            if leader_fails >= MAX_LEADER_FAILS:
                raise StopRequest(f"{MAX_LEADER_FAILS} leader joint read failures in a row")
        else:
            leader_fails = 0
            leader_last = leader
        raws = {arm: dev.read_button_raw(TOOL_IDS[arm]) for arm in ARMS}

        # 2. Clutches
        buttons = {}
        for arm in ARMS:
            buttons[arm], ev = clutch[arm].update(raws[arm])
            if ev == "engaged":
                if leader_last is None:
                    clutch[arm].engaged = False
                    ev = "press ignored (no leader reading yet)"
                else:
                    robot_start[arm] = target[arm].copy()
                    leader_start[arm] = leader_last[LEADER_SLICE[arm]].copy()
                    ev = "engaged"
            if ev:
                event(f"[{time.monotonic() - t_start:7.2f} s] {arm} clutch {ev}")

        # 3. RB-Y1 state and timing checks
        q, state_age = rstate.latest()
        now = time.monotonic()
        late = last_send is not None and now - last_send > MAX_GAP_S
        late_cycles = late_cycles + 1 if late else 0
        if late_cycles >= MAX_LATE_CYCLES:
            raise RuntimeError(f"{MAX_LATE_CYCLES} late cycles in a row")
        stale = state_age > MAX_STATE_AGE_S
        stale_cycles = stale_cycles + 1 if stale else 0
        if stale_cycles >= MAX_STALE_CYCLES:
            raise RuntimeError(f"{MAX_STALE_CYCLES} stale RB-Y1 state cycles in a row")
        if late:
            event(f"late: {(now - last_send) * 1000:.0f} ms since last send, targets frozen")
        if stale:
            event(f"stale RB-Y1 state: {state_age * 1000:.0f} ms old, targets frozen")

        # 4. Targets per arm, per joint (B policy: drop the excess of the speed limit)
        flags = {(arm, k): False for arm in ARMS for k in ("rate", "range")}
        for arm in ARMS:
            if not (clutch[arm].engaged and leader is not None and not late and not stale):
                continue
            desired = robot_start[arm] + (leader[LEADER_SLICE[arm]] - leader_start[arm])
            step = desired - target[arm]
            limited = np.clip(step, -MAX_STEP_DEG, MAX_STEP_DEG)
            if np.any(limited != step):
                leader_start[arm] += step - limited
                flags[(arm, "rate")] = True
            new = target[arm] + limited
            clipped = np.clip(new, low[arm], high[arm])
            if np.any(clipped != new):
                flags[(arm, "range")] = True
            target[arm] = clipped
        for key, on in flags.items():
            if on and not flags_prev[key]:
                arm, kind = key
                event(f"[{time.monotonic() - t_start:7.2f} s] {arm} arm "
                      + (f"speed limit ({MAX_STEP_DEG} deg/cycle)" if kind == "rate" else "range limit"))
        flags_prev = flags

        # 5. Send both arms every cycle (a released arm sends its frozen target)
        body = rby.BodyComponentBasedCommandBuilder()
        body.set_right_arm_command(arm_command(target["right"], MINIMUM_TIME_S, CONTROL_HOLD_TIME_S,
                                               lim["qdot"][idx["right"]], lim["qddot"][idx["right"]]))
        body.set_left_arm_command(arm_command(target["left"], MINIMUM_TIME_S, CONTROL_HOLD_TIME_S,
                                              lim["qdot"][idx["left"]], lim["qddot"][idx["left"]]))
        stream.send_command(rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(body)))
        last_send = time.monotonic()

        row = [f"{t_cycle - t_start:.4f}", "" if interval is None else f"{interval * 1000:.3f}",
               f"{state_age * 1000:.2f}", int(late), int(stale)]
        for arm in ARMS:
            row += ["" if raws[arm] is None else raws[arm], int(clutch[arm].engaged),
                    int(flags[(arm, "rate")]), int(flags[(arm, "range")])]
        row += ["" if leader is None else f"{v:.3f}" for v in (leader if leader is not None else [None] * 14)]
        row += [f"{v:.3f}" for arm in ARMS for v in target[arm]]
        row += [f"{v:.3f}" for arm in ARMS for v in q[idx[arm]]]
        ctx["rows"].append(row)

        # 6. Status line
        if cycle % STATUS_EVERY == 0:
            rate = {arm: 100.0 * clutch[arm].failures / max(clutch[arm].reads, 1) for arm in ARMS}
            sys.stdout.write(
                f"{CLEAR_LINE}R {'ENGAGED' if clutch['right'].engaged else 'free   '}  "
                f"L {'ENGAGED' if clutch['left'].engaged else 'free   '}  |  button read fail "
                f"R {rate['right']:4.1f}% L {rate['left']:4.1f}%  leader joint read fail "
                f"{100.0 * leader_fail_total / max(leader_reads, 1):4.1f}%")
            sys.stdout.flush()

        # 7. Sleep until the next cycle; after an overrun wait a full period (08)
        cycle += 1
        next_t += PERIOD_S
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic() + PERIOD_S
            time.sleep(PERIOD_S)


def hold_and_stop(stream, target, idx, lim):
    """Send the frozen targets for HOLD_ON_EXIT_S, then stop streaming (commands expire after hold time)."""
    t_end = time.monotonic() + HOLD_ON_EXIT_S
    while time.monotonic() < t_end:
        body = rby.BodyComponentBasedCommandBuilder()
        body.set_right_arm_command(arm_command(target["right"], MINIMUM_TIME_S, CONTROL_HOLD_TIME_S,
                                               lim["qdot"][idx["right"]], lim["qddot"][idx["right"]]))
        body.set_left_arm_command(arm_command(target["left"], MINIMUM_TIME_S, CONTROL_HOLD_TIME_S,
                                              lim["qdot"][idx["left"]], lim["qddot"][idx["left"]]))
        stream.send_command(rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(body)))
        time.sleep(PERIOD_S)


CSV_HEADER = (["t_s", "cycle_interval_ms", "state_age_ms", "late", "stale"]
              + [f"{arm}_{k}" for arm in ARMS for k in ("button_raw", "clutch", "speed_limited", "range_limited")]
              + [f"leader_{i}_deg" for i in JOINT_IDS]
              + [f"target_{arm}_{j}_deg" for arm in ARMS for j in range(7)]
              + [f"rby1_{arm}_{j}_deg" for arm in ARMS for j in range(7)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="localhost:50051", help="RB-Y1 simulator address")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="leader arm USB-serial device")
    args = parser.parse_args()
    np.set_printoptions(precision=1, suppress=True)

    # Leader arm (read-only)
    latency_timer = read_latency_timer(args.port)
    if latency_timer != 1:
        raise SystemExit(f"FTDI latency_timer is {latency_timer} (need 1). Set it with:\n"
                         f"  echo 1 | sudo tee /sys/bus/usb-serial/devices/"
                         f"{os.path.basename(os.path.realpath(args.port))}/latency_timer")
    dxl_port_handler.LATENCY_TIMER = 1
    leader = LeaderArm(args.port)
    leader.open()

    # RB-Y1 simulator
    robot = rby.create_robot(args.address, "m")
    if not robot.connect():
        raise SystemExit(f"Cannot connect to {args.address}")
    model = robot.model()
    if model.model_name != "M":
        raise SystemExit(f"Expected model M, got {model.model_name}")
    idx = {"right": list(model.right_arm_idx), "left": list(model.left_arm_idx)}
    dyn = robot.get_dynamics()
    dyn_state = dyn.make_state([], model.robot_joint_names)
    # Convert only the arm joint limits to degrees (wheel limits are huge and overflow np.degrees)
    arm_idx = idx["right"] + idx["left"]
    q_lo, q_hi = np.full(model.robot_dof, np.nan), np.full(model.robot_dof, np.nan)
    q_lo[arm_idx] = np.degrees(np.asarray(dyn.get_limit_q_lower(dyn_state))[arm_idx])
    q_hi[arm_idx] = np.degrees(np.asarray(dyn.get_limit_q_upper(dyn_state))[arm_idx])
    lim = {"q_lo": q_lo, "q_hi": q_hi,
           "qdot": dyn.get_limit_qdot_upper(dyn_state), "qddot": dyn.get_limit_qddot_upper(dyn_state)}

    print(f"RB-Y1 SIMULATOR {args.address} (model M)   leader {args.port}   loop {PERIOD_S * 1000:.0f} ms   "
          f"max {MAX_STEP_DEG} deg/cycle   range +/-{RANGE_DEG:.0f} deg   hold {CONTROL_HOLD_TIME_S} s")
    prepare_simulator(robot)

    q_now = np.degrees(np.array(robot.get_state().position))
    print("Moving to READY_POSE (example 35) with minimum_time 5 s:")
    for name, i, ready in (("torso", list(model.torso_idx), READY_TORSO_DEG),
                           ("right", idx["right"], READY_RIGHT_DEG), ("left", idx["left"], READY_LEFT_DEG)):
        print(f"  {name:<5} now {q_now[i]}  ->  {np.array(ready)}")
    feedback = robot.send_command(ready_command()).get()
    if feedback.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        raise SystemExit(f"READY move did not finish: {feedback.finish_code}")
    print("READY pose reached.")

    rstate = RobotState()
    robot.start_state_update(rstate.callback, 1 / PERIOD_S)
    t0 = time.monotonic()
    while rstate.sample is None:
        if time.monotonic() - t0 > 2.0:
            raise SystemExit("No RB-Y1 state update received.")
        time.sleep(0.01)

    stream = robot.create_command_stream(priority=1)
    ctx = {"target": None, "rows": [], "clutch": None}
    print("Clutch: right handle -> right arm, left handle -> left arm (hold to follow). Ctrl+C to stop.\n")
    try:
        run_loop(leader, stream, rstate, idx, lim, ctx)
    except KeyboardInterrupt:
        event("Interrupted.")
    except StopRequest as e:
        event(f"Stopping: {e}")
    except Exception as e:
        event(f"Error: {e}")
    finally:
        if ctx["target"] is not None:
            try:
                hold_and_stop(stream, ctx["target"], idx, lim)
            except Exception as e:
                event(f"hold on exit failed: {e}")
        robot.stop_state_update()
        leader.close()

    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"14_leader_rby1_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        writer.writerows(ctx["rows"])
    if ctx["clutch"]:
        for arm in ARMS:
            c = ctx["clutch"][arm]
            print(f"{arm} button reads {c.reads}, failed or unexpected {c.failures} "
                  f"({100.0 * c.failures / max(c.reads, 1):.1f}%)")
    print(f"Saved: {path.relative_to(ROOT)} ({len(ctx['rows'])} cycles)")


if __name__ == "__main__":
    main()

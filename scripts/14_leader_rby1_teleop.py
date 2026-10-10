"""Leader arm (both arms, 14 joints) -> RB-Y1 simulator arms, with a clutch per arm.

RB-Y1 SIMULATOR ONLY (model M, default address localhost:50051). Run in the `rby1` conda env.

Leader arm, one dynamixel_sdk PortHandler for the whole run:
    Startup alignment (writes, before teleop): torque off -> operating mode 5 (current-based position)
    -> torque on, then cosine interpolation to the RB-Y1 READY arm pose in ALIGN_DURATION_S with goal
    current = ALIGN_TORQUE_NM (example 35 move_leader_arm_to_pose; conversions from rby1-sdk
    dynamixel_bus.cpp). The arm then holds that pose with the same weak torque.
    During teleop the ONLY write is torque off: once per arm, at that arm's first clutch press,
    and for all joints when the script ends.
    Reads: joints 0..13 with GroupSyncRead (address 132), tools 128 / 129 with separate reads +
    100 us pause (rby1-sdk ReadButtonStatus). Angle: deg = raw * 360 / 4096.
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
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
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

# --- Leader arm alignment (writes before teleop only). Addresses: rby1-sdk dynamixel_bus.h, confirmed in
# dynamixel_sdk's ROBOTIS control tables xm430_w210.model / xm540_w150.model
ADDR_OPERATING_MODE = 11   # 1 byte
ADDR_TORQUE_ENABLE = 64    # 1 byte
ADDR_GOAL_CURRENT = 102    # 2 bytes
ADDR_GOAL_POSITION = 116   # 4 bytes
CURRENT_BASED_POSITION_MODE = 5   # rby1-sdk kCurrentBasedPositionControlMode
CURRENT_UNIT_MA = 2.69     # rby1-sdk: raw = torque / torque_constant * 1000 / 2.69
# Torque constant (Nm/A) by model number read with ping. Rainbow's leader_arm.cpp uses
# {1.6591 x3, 1.3043 x4} per arm by ID; here it is chosen per ID from the actual model (docs/decisions.md).
# 1130 = XM540-W150, 1030 = XM430-W210 (dynamixel_easy_sdk/control_table/dynamixel.model)
TORQUE_CONSTANT_BY_MODEL = {1130: 1.6591, 1030: 1.3043}
ALIGN_DURATION_S = 6.0
ALIGN_TORQUE_NM = 0.5      # example 35: goal_torque=0.5
ALIGN_WARN_DEG = 90.0      # warn and ask again when a joint must move more than this

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
    """Leader arm on one dynamixel_sdk PortHandler: reads, startup alignment, and torque off."""

    def __init__(self, port_name):
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        self.sync = None
        self.models = {}           # ID -> model number from ping
        self.torque_on = False     # True after the alignment enabled torque

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"Failed to open {self.port.getPortName()}")
        if not self.port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")
        missing = []
        for dxl_id in JOINT_IDS + list(TOOL_IDS.values()):
            model, result, _ = self.ph.ping(self.port, dxl_id)
            if result == COMM_SUCCESS:
                self.models[dxl_id] = model
            else:
                missing.append(dxl_id)
        if missing:
            raise SystemExit(f"Leader arm IDs {missing} did not answer the ping.")
        unknown = {i: self.models[i] for i in JOINT_IDS if self.models[i] not in TORQUE_CONSTANT_BY_MODEL}
        if unknown:
            raise SystemExit(f"No torque constant for joint models {unknown}; not writing to the leader arm.")
        self.sync = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for dxl_id in JOINT_IDS:
            self.sync.addParam(dxl_id)

    # ---- reads
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

    # ---- writes
    def _sync_write(self, address, size, id_bytes):
        """One GroupSyncWrite; id_bytes maps ID -> list of `size` bytes (little endian)."""
        writer = GroupSyncWrite(self.port, self.ph, address, size)
        for dxl_id, data in id_bytes.items():
            writer.addParam(dxl_id, data)
        result = writer.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"sync write to address {address} failed: {self.ph.getTxRxResult(result)}")

    def torque_off(self, ids):
        """The only write allowed during teleop."""
        self._sync_write(ADDR_TORQUE_ENABLE, 1, {i: [0] for i in ids})

    def align_to(self, target_deg, duration_s, torque_nm):
        """Startup only: move the joints to target_deg (14 values) like example 35 move_leader_arm_to_pose."""
        start_deg = self.read_joints_deg()
        if start_deg is None:
            raise SystemExit("Could not read the leader arm before alignment.")
        # Same order as rby1-sdk / example 35: torque off -> operating mode -> torque on
        self._sync_write(ADDR_TORQUE_ENABLE, 1, {i: [0] for i in JOINT_IDS})
        self._sync_write(ADDR_OPERATING_MODE, 1, {i: [CURRENT_BASED_POSITION_MODE] for i in JOINT_IDS})
        self._sync_write(ADDR_TORQUE_ENABLE, 1, {i: [1] for i in JOINT_IDS})
        self.torque_on = True
        # Goal current from torque: rby1-sdk dynamixel_bus.cpp:501, torque constant by actual model
        current = {}
        for i in JOINT_IDS:
            raw = int(torque_nm / TORQUE_CONSTANT_BY_MODEL[self.models[i]] * 1000.0 / CURRENT_UNIT_MA)
            current[i] = list(raw.to_bytes(2, "little", signed=True))
        t0 = time.monotonic()
        next_t = t0
        while True:
            ratio = min((time.monotonic() - t0) / duration_s, 1.0)
            s_curve = 0.5 * (1.0 - np.cos(np.pi * ratio))          # example 35 cosine S-curve
            cmd_deg = start_deg + s_curve * (target_deg - start_deg)
            # Goal position: rby1-sdk GroupSyncWriteSendPosition, (int)(rad * 4096 / 2 / 3.141592)
            pos = {i: list(int(np.radians(cmd_deg[i]) * 4096.0 / 2.0 / 3.141592).to_bytes(4, "little", signed=True))
                   for i in JOINT_IDS}
            self._sync_write(ADDR_GOAL_CURRENT, 2, current)
            self._sync_write(ADDR_GOAL_POSITION, 4, pos)
            if ratio >= 1.0:
                break
            next_t += PERIOD_S
            time.sleep(max(0.0, next_t - time.monotonic()))

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
    torque_released = {arm: False for arm in ARMS}   # leader torque switched off at the first press
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
                    if not torque_released[arm]:
                        # Only allowed teleop write: torque off for this arm's 7 joints
                        dev.torque_off(JOINT_IDS[LEADER_SLICE[arm]])
                        torque_released[arm] = True
                        event(f"[{time.monotonic() - t_start:7.2f} s] {arm} leader arm torque off (free to move)")
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
    try:
        if not align_leader(leader):
            return
        teleop(args, leader)
    finally:
        # Whatever happened, leave the leader arm without torque
        if leader.torque_on:
            try:
                leader.torque_off(JOINT_IDS)
                print("Leader arm torque off (all joints).")
            except Exception as e:
                print(f"WARNING: leader arm torque off failed: {e}")
        leader.close()


MODEL_NAMES = {1130: "XM540-W150", 1030: "XM430-W210"}


def align_leader(leader):
    """Move the leader arm to the RB-Y1 READY arm pose (startup only). Returns False if cancelled."""
    q = leader.read_joints_deg()
    if q is None:
        raise SystemExit("Could not read the leader arm.")
    target = np.array(READY_RIGHT_DEG + READY_LEFT_DEG)
    delta = target - q
    print(f"Leader arm alignment to the RB-Y1 READY arm pose: {ALIGN_DURATION_S:.0f} s, "
          f"goal current from {ALIGN_TORQUE_NM} Nm")
    print(f"{'ID':>3} {'model':<11} {'now':>8} {'target':>8} {'move':>8}")
    for i in JOINT_IDS:
        mark = "   <-- more than %.0f deg" % ALIGN_WARN_DEG if abs(delta[i]) > ALIGN_WARN_DEG else ""
        print(f"{i:>3} {MODEL_NAMES[leader.models[i]]:<11} {q[i]:8.1f} {target[i]:8.1f} {delta[i]:+8.1f}{mark}")
    big = [i for i in JOINT_IDS if abs(delta[i]) > ALIGN_WARN_DEG]
    if big:
        print(f"WARNING: joints {big} move more than {ALIGN_WARN_DEG:.0f} deg (check cables, e.g. wrist rotation).")
        if input("Type 'yes' to accept these large moves: ") != "yes":
            print("Cancelled. Turn those joints closer to the target by hand and run again.")
            return False
    print("Keep hands away from the leader arm path.")
    if input("The REAL leader arm will move. Type 'yes' to continue: ") != "yes":
        print("Cancelled.")
        return False
    leader.align_to(target, ALIGN_DURATION_S, ALIGN_TORQUE_NM)
    time.sleep(0.5)
    q_after = leader.read_joints_deg()
    print(f"Alignment done. {'ID':>3} {'target':>8} {'actual':>8} {'diff':>7}")
    for i in JOINT_IDS:
        print(f"                {i:>3} {target[i]:8.2f} {q_after[i]:8.2f} {q_after[i] - target[i]:+7.2f}")
    print("The leader arm now holds this pose with weak torque; its torque turns off at that arm's first "
          "clutch press.\nHold the arm when you press the button: it can drop once the torque is off.")
    return True


def teleop(args, leader):
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

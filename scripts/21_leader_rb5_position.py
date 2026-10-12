"""Leader arm wrist center -> RB5 TCP position with a clutch, move_servo_l, Simulation mode only.

Each 20 ms cycle:
  1. read leader right arm joints ID 0..6 (Sync Read) and the right tool (ID 128) button; read-only
  2. clutch (same debounce as 13): press = 2 pressed reads, 3 failed reads in a row = released
  3. while engaged: desired = RB5 start + scale * Rz(yaw) * (leader now - leader start)   [mm]
     leader position = Link_6R origin (wrist center) from the leader URDF (numpy chain, same result
     as rby1_sdk in 20). The orientation stays at the TCP orientation at script start.
  4. limits, in this order:
     - speed: at most MAX_STEP_MM per cycle, scaled down near the wrist singularity (below)
     - joint speed: the DH model predicts the joint motion dq = J^-1 dx; if max |dq| is over
       MAX_DQ_DEG per cycle the step is scaled down
     - the cut part of the step is dropped (B: the leader start is shifted), so the robot stops soon
       after the hand stops
     - range: +/-RANGE_MM per axis around the TCP at script start (not dropped)
     - lead: the target is not moved while it is more than MAX_LEAD_MM ahead of tcp_ref (not dropped)
  5. wrist singularity watch (DH model): effective J5 = asin(|sin J5|) of the predicted joint angles.
     Below 20 deg: warning and the speed limit falls linearly to 0.25x at 10 deg. Below 10 deg: stop.
  6. send move_servo_l(target x, y, z, fixed rx, ry, rz) every cycle (released clutch = frozen target)

Coordinate alignment: leader Base and RB5 base are taken as the same directions (x forward, y left,
z up for the leader, confirmed in 20), rotated about z by --yaw-deg (default 0).

Usage:
    python scripts/21_leader_rb5_position.py
    python scripts/21_leader_rb5_position.py --yaw-deg 90 --scale 1.0
Stop with Ctrl+C. Each run is logged to data/21_leader_pos_sim_<time>.csv
"""
import argparse
import csv
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

# --- RB5 servo streaming (same values as 13 / 18)
PERIOD_S = 0.02
SERVO_T1 = PERIOD_S        # matches the send period (06 / 10)
SERVO_T2 = 0.1
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0

# --- Limits
MAX_STEP_MM = 1.0          # per cycle (50 mm/s)
MAX_DQ_DEG = 1.0           # predicted joint motion per cycle (50 deg/s)
RANGE_MM = 150.0           # per axis, around the TCP at script start
MAX_LEAD_MM = 2.0          # target may be at most this far ahead of tcp_ref
J5_SLOW_DEG = 20.0         # effective J5 below this: warning and slower
J5_STOP_DEG = 10.0         # effective J5 below this: stop
J5_MIN_SCALE = 0.25        # speed scale at J5_STOP_DEG
MAX_FK_ERROR_MM = 5.0      # DH model vs tcp_ref at start
MAX_GAP_S = 1.5 * SERVO_T1 # a send gap longer than this freezes the target for that cycle (08)
MAX_LATE_CYCLES = 3
MAX_STATE_AGE_S = 0.1
MAX_STALE_CYCLES = 3
MAX_LEADER_FAILS = 3       # leader joint read failures in a row -> stop
HOLD_ON_EXIT_S = 0.3

# --- Leader arm (rby1-sdk: dynamixel_bus.h, leader_arm.cpp)
LEADER_PORT = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
ADDR_PRESENT_POSITION = 132  # the tool returns its button state at the same address
LEN_PRESENT_POSITION = 4
LEADER_IDS = list(range(7))
LEADER_JOINTS = ["J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R",
                 "J4_Wrist_Yaw1_R", "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R"]
TOOL_ID = 0x80             # right tool
TOOL_READ_PAUSE_S = 100e-6 # rby1-sdk ReadButtonStatus sleeps 100 us after the read
TICKS_PER_REV = 4096
URDF_PATH = ROOT / "refs" / "rby1-sdk" / "models" / "leader_arm" / "model.urdf"

# --- Clutch (13)
BUTTON_PRESSED = 1
BUTTON_RELEASED = 0
PRESS_CONFIRM_READS = 2
FAILS_TO_RELEASE = 3

# --- RB5-850 standard DH (rb_cobot_docs technical_docs/dh_parameters), mm / deg
D1, D2, D3, D4, D5, D6, A1, A2 = 169.2, 148.4, 148.4, 110.7, 110.7, 96.7, 425.0, 392.0

STATUS_EVERY = 25
DATA_DIR = ROOT / "data"
CLEAR_LINE = "\r\x1b[2K"


class StopRequest(Exception):
    """Controlled stop: end servo mode normally (hold target, move_speed_j(0), poll)."""


# ---------------------------------------------------------------- math
def rot_zyx(rx, ry, rz):
    """Rotation matrix of a ZY'X'' Euler pose (deg): R = Rz(rz) Ry(ry) Rx(rx)."""
    a, b, c = np.radians([rx, ry, rz])
    rx_m = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    ry_m = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    rz_m = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def rot_z(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def axis_rotation(axis, angle):
    """Rodrigues formula: rotation by angle [rad] about a (normalized) axis."""
    k = np.asarray(axis, dtype=float)
    k /= np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def clip_norm(v, limit):
    n = np.linalg.norm(v)
    return v if n <= limit else v * (limit / n)


def effective_j5(j5_deg):
    """Distance of J5 from the singular angles 0 and +/-180 deg."""
    return np.degrees(np.arcsin(min(abs(np.sin(np.radians(j5_deg))), 1.0)))


def j5_speed_scale(j5_eff):
    if j5_eff >= J5_SLOW_DEG:
        return 1.0
    if j5_eff <= J5_STOP_DEG:
        return J5_MIN_SCALE
    return J5_MIN_SCALE + (1.0 - J5_MIN_SCALE) * (j5_eff - J5_STOP_DEG) / (J5_SLOW_DEG - J5_STOP_DEG)


class LeaderModel:
    """Right arm wrist center (Link_6R origin) from the leader URDF; all joint origins have rpy 0 (checked)."""

    def __init__(self, urdf_path):
        joints = {j.get("name"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
        self.chain = []
        for name in LEADER_JOINTS:
            j = joints[name]
            origin = j.find("origin")
            if any(float(v) != 0.0 for v in origin.get("rpy", "0 0 0").split()):
                raise SystemExit(f"{name}: origin rpy is not zero; this FK does not handle it.")
            self.chain.append((np.array([float(v) for v in origin.get("xyz").split()]),
                               np.array([float(v) for v in j.find("axis").get("xyz").split()])))

    def wrist_center_mm(self, q_rad):
        r, p = np.eye(3), np.zeros(3)
        for (xyz, axis), angle in zip(self.chain, q_rad):
            p = p + r @ xyz
            r = r @ axis_rotation(axis, angle)
        return p * 1000.0


class RB5Model:
    """RB5-850 standard DH model plus the TCP offset (flange frame, mm). Validated against 17 / 18."""

    def __init__(self, tcp_offset_mm):
        self.offset = np.asarray(tcp_offset_mm, dtype=float)

    @staticmethod
    def _dh(theta, d, a, alpha):
        th, al = np.radians(theta), np.radians(alpha)
        ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                         [0, sa, ca, d], [0, 0, 0, 1]])

    def flange(self, q):
        t1, t2, t3, t4, t5, t6 = q
        t = np.eye(4)
        for row in ((t1, D1, 0, -90), (t2 - 90, -D2, 0, 0), (0, 0, A1, 0), (t3, D3, 0, 0), (0, 0, A2, 0),
                    (t4 + 90, -D4, 0, 0), (0, 0, 0, 90), (t5, D5, 0, -90), (t6, -D6, 0, 90)):
            t = t @ self._dh(*row)
        return t

    def tcp(self, q):
        t = self.flange(q)
        return t[:3, 3] + t[:3, :3] @ self.offset, t[:3, :3]

    def jacobian(self, q, h_deg=1e-4):
        """6x6: rows = TCP velocity (mm) and angular velocity, per rad of each joint (finite differences)."""
        p0, r0 = self.tcp(q)
        jac = np.zeros((6, 6))
        for i in range(6):
            dq = np.array(q, dtype=float)
            dq[i] += h_deg
            p, r = self.tcp(dq)
            dr = r @ r0.T
            jac[:3, i] = (p - p0) / np.radians(h_deg)
            jac[3:, i] = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2 / np.radians(h_deg)
        return jac

    def predict(self, q, dp_mm):
        """Joint motion [deg] for a TCP position change dp (orientation unchanged), from the current q."""
        return np.degrees(np.linalg.solve(self.jacobian(q), np.r_[dp_mm, 0, 0, 0]))


def plan_step(target, desired, tcp_pos, q, model, low, high):
    """One cycle of the limits. Returns (new target, dropped vector, flags, predicted eff. J5, max |dq|, stop reason)."""
    flags = dict.fromkeys(("speed", "j5slow", "jspeed", "range", "lead"), False)
    step = desired - target

    # Speed limit, scaled by the predicted J5 of a full-size step
    j5_eff = effective_j5(q[4] + model.predict(q, target + clip_norm(step, MAX_STEP_MM) - tcp_pos)[4])
    scale = j5_speed_scale(j5_eff)
    flags["j5slow"] = scale < 1.0
    limited = clip_norm(step, MAX_STEP_MM * scale)
    flags["speed"] = np.linalg.norm(limited) < np.linalg.norm(step) - 1e-9

    # Joint speed limit (the joint motion is nearly linear in a step this small)
    max_dq = np.abs(model.predict(q, target + limited - tcp_pos)).max()
    lead_dq = np.abs(model.predict(q, target - tcp_pos)).max()   # part that is only the robot catching up
    if max_dq > MAX_DQ_DEG and max_dq > lead_dq:
        limited = limited * max(0.0, (MAX_DQ_DEG - lead_dq) / (max_dq - lead_dq))
        flags["jspeed"] = True
    dropped = step - limited   # B: dropped, not caught up later

    new = target + limited
    clamped = np.clip(new, low, high)
    flags["range"] = bool(np.any(clamped != new))
    if np.linalg.norm(clamped - tcp_pos) > MAX_LEAD_MM:
        clamped = target.copy()
        flags["lead"] = True

    dq = model.predict(q, clamped - tcp_pos)
    j5_final = effective_j5(q[4] + dq[4])
    stop = None
    if j5_final < J5_STOP_DEG and not np.array_equal(clamped, target):
        stop = f"predicted J5 {j5_final:.1f} deg (from the singular angle) < {J5_STOP_DEG} deg"
    return clamped, dropped, flags, j5_final, np.abs(dq).max(), stop


# ---------------------------------------------------------------- devices
def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def read_latency_timer(port_name):
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


class LeaderArm:
    """Read-only access to the right arm joints and the right tool button. Only ping and read are used."""

    def __init__(self, port_name):
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        self.sync = None

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"Failed to open {self.port.getPortName()}")
        if not self.port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")
        missing = [i for i in LEADER_IDS + [TOOL_ID] if self.ph.ping(self.port, i)[1] != COMM_SUCCESS]
        if missing:
            raise SystemExit(f"Leader arm IDs {missing} did not answer the ping.")
        self.sync = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for i in LEADER_IDS:
            self.sync.addParam(i)

    def read_joints_rad(self):
        """Return the 7 joint angles [rad], or None if the read failed."""
        if self.sync.txRxPacket() != COMM_SUCCESS or not all(
                self.sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in LEADER_IDS):
            return None
        raw = [to_signed32(self.sync.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) for i in LEADER_IDS]
        return np.array(raw, dtype=float) / TICKS_PER_REV * 2 * np.pi

    def read_button_raw(self):
        value, result, _ = self.ph.read4ByteTxRx(self.port, TOOL_ID, ADDR_PRESENT_POSITION)
        time.sleep(TOOL_READ_PAUSE_S)
        return value if result == COMM_SUCCESS else None

    def close(self):
        self.port.closePort()


class Clutch:
    """Debounced clutch from the tool button (13)."""

    def __init__(self):
        self.engaged = False       # always start released
        self.armed = False         # a press is accepted only after one release has been seen
        self.press_count = 0
        self.fail_count = 0
        self.reads = 0
        self.failures = 0

    def update(self, raw):
        """Feed one tool read (raw value or None). Returns (button or None, event or None)."""
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


class StateReader(threading.Thread):
    """Reads the RB5 data channel continuously and keeps the latest tcp_ref / jnt_ref (08 / 18)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.data_channel = rb.CobotData(config.ROBOT_IP)
        self.lock = threading.Lock()
        self.sample = None
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            data = self.data_channel.request_data(1.0)
            if data is None:
                continue
            s = data.sdata
            # Simulation: jnt_ang does not follow the motion, so use the reference values
            sample = (np.array(s.tcp_ref, dtype=float), np.array(s.jnt_ref, dtype=float), time.monotonic())
            with self.lock:
                self.sample = sample

    def wait_first(self, timeout_s=2.0):
        t0 = time.monotonic()
        while self.sample is None:
            if time.monotonic() - t0 > timeout_s:
                raise SystemExit("State reader got no data from the robot.")
            time.sleep(0.01)

    def latest(self):
        with self.lock:
            tcp, q, t = self.sample
        return tcp.copy(), q.copy(), time.monotonic() - t

    def stop(self):
        self.stop_event.set()
        self.join(2.0)


def wait_until_stopped(robot, rc, timeout_s=10.0):
    t0 = time.monotonic()
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        if time.monotonic() - t0 > timeout_s:
            raise SystemExit(f"Robot still moving after {timeout_s} s.")
        time.sleep(0.01)


def send_servo(robot, rc, target, rpy):
    robot.move_servo_l(rc, np.r_[target, rpy], SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)


def stop_streaming(robot, rc, target, rpy):
    """Normal end of servo mode: hold the target, then move_speed_j(0) and poll (13)."""
    next_t = time.monotonic()
    t_end = next_t + HOLD_ON_EXIT_S
    while time.monotonic() < t_end:
        send_servo(robot, rc, target, rpy)
        next_t += PERIOD_S
        time.sleep(max(0.0, next_t - time.monotonic()))
    robot.move_speed_j(rc, np.zeros(6), SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
    robot.enable_waiting_ack(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def emergency_stop(robot, rc):
    """End servo mode after an error: no more servo commands, task_stop and poll (13)."""
    robot.enable_waiting_ack(rc)
    robot.task_stop(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def event(msg):
    sys.stdout.write(CLEAR_LINE + msg + "\n")
    sys.stdout.flush()


# ---------------------------------------------------------------- loop
def run_loop(leader, leader_model, robot, rc, reader, rb5_model, args, ctx):
    """20 ms loop. ctx["target"] holds the TCP position target [mm], ctx["rows"] the CSV rows."""
    tcp0, _, _ = reader.latest()
    ctx["target"] = tcp0[:3].copy()
    target = ctx["target"]
    rpy = ctx["rpy"]
    low, high = tcp0[:3] - RANGE_MM, tcp0[:3] + RANGE_MM
    yaw = rot_z(args.yaw_deg)

    clutch = Clutch()
    ctx["clutch"] = clutch
    leader_q = leader_p = None
    leader_fails = leader_fail_total = leader_reads = 0
    rb5_start = leader_start = None
    flags_prev = {}
    last_send = prev_cycle_t = None
    late_cycles = stale_cycles = 0
    j5_warned = False

    t_start = next_t = time.monotonic()
    cycle = 0
    while True:
        t_cycle = time.monotonic()
        interval = None if prev_cycle_t is None else t_cycle - prev_cycle_t
        prev_cycle_t = t_cycle

        # 1. Leader arm joints -> wrist center, then the button
        leader_reads += 1
        q_l = leader.read_joints_rad()
        if q_l is None:
            leader_fails += 1
            leader_fail_total += 1
            if leader_fails >= MAX_LEADER_FAILS:
                raise StopRequest(f"{MAX_LEADER_FAILS} leader joint read failures in a row")
        else:
            leader_fails = 0
            leader_q, leader_p = q_l, leader_model.wrist_center_mm(q_l)
        button_raw = leader.read_button_raw()

        # 2. Clutch
        button, clutch_event = clutch.update(button_raw)
        if clutch_event == "engaged":
            if leader_p is None:
                clutch.engaged = False
                clutch_event = "press ignored (no leader reading yet)"
            else:
                rb5_start, leader_start = target.copy(), leader_p.copy()
                clutch_event = f"engaged: RB5 start {np.round(rb5_start, 1)} mm, leader start {np.round(leader_start, 1)} mm"
        if clutch_event:
            event(f"[{time.monotonic() - t_start:7.2f} s] clutch {clutch_event}")

        # 3. RB5 state and the 08 safety checks
        robot.flush(rc)
        rc.error().throw_if_not_empty()
        rc.clear()
        tcp, q, state_age = reader.latest()
        now = time.monotonic()
        late = last_send is not None and now - last_send > MAX_GAP_S
        late_cycles = late_cycles + 1 if late else 0
        if late_cycles >= MAX_LATE_CYCLES:
            raise RuntimeError(f"{MAX_LATE_CYCLES} late cycles in a row; loop too slow for servo")
        stale = state_age > MAX_STATE_AGE_S
        stale_cycles = stale_cycles + 1 if stale else 0
        if stale_cycles >= MAX_STALE_CYCLES:
            raise RuntimeError(f"{MAX_STALE_CYCLES} stale-state cycles in a row; state reader stopped?")
        if late:
            event(f"late: {(now - last_send) * 1000:.0f} ms since last send, target frozen")
        if stale:
            event(f"stale RB5 state: {state_age * 1000:.0f} ms old, target frozen")

        # 4. Target: follow the leader only while engaged and every check passed
        desired = None
        flags = {}
        j5_pred, max_dq = effective_j5(q[4]), 0.0
        if clutch.engaged and q_l is not None and not late and not stale:
            desired = rb5_start + args.scale * (yaw @ (leader_p - leader_start))
            new, dropped, flags, j5_pred, max_dq, stop = plan_step(target, desired, tcp[:3], q, rb5_model, low, high)
            if stop:
                raise StopRequest(stop)
            # B: shift the leader start so that the dropped part is not caught up later
            leader_start += yaw.T @ dropped / args.scale
            target[:] = new
        labels = {"speed": f"speed limit ({MAX_STEP_MM} mm/cycle)",
                  "j5slow": f"J5 below {J5_SLOW_DEG:.0f} deg: slower",
                  "jspeed": f"joint speed limit ({MAX_DQ_DEG} deg/cycle predicted)",
                  "range": f"range limit (+/-{RANGE_MM:.0f} mm)",
                  "lead": f"target waits for robot (lead limit {MAX_LEAD_MM} mm)"}
        for key, on in flags.items():
            if on and not flags_prev.get(key, False):
                event(f"[{time.monotonic() - t_start:7.2f} s] {labels[key]}")
        flags_prev = flags
        if flags.get("j5slow") and not j5_warned:
            event(f"WARNING: J5 {j5_pred:.1f} deg from the singular angle (stop below {J5_STOP_DEG:.0f})")
        j5_warned = flags.get("j5slow", False)

        # 5. Send every cycle (a released clutch sends the frozen target)
        send_servo(robot, rc, target, rpy)
        last_send = time.monotonic()

        f = lambda v, fmt="{:.3f}": [fmt.format(x) for x in v]   # noqa: E731
        empty3 = [""] * 3
        ctx["rows"].append(
            [f"{t_cycle - t_start:.4f}", "" if interval is None else f"{interval * 1000:.3f}",
             "" if button_raw is None else button_raw, int(clutch.engaged)]
            + (f(np.degrees(leader_q)) if leader_q is not None else [""] * 7)
            + (f(leader_p) if leader_p is not None else empty3)
            + (f(desired) if desired is not None else empty3) + f(target) + f(tcp[:3]) + f(q)
            + [f"{j5_pred:.2f}", f"{max_dq:.3f}"]
            + [int(flags.get(k, False)) for k in ("speed", "j5slow", "jspeed", "range", "lead")]
            + [int(late), int(stale), f"{state_age * 1000:.2f}"])

        # 6. Status line
        if cycle % STATUS_EVERY == 0:
            tool_rate = 100.0 * clutch.failures / max(clutch.reads, 1)
            leader_rate = 100.0 * leader_fail_total / max(leader_reads, 1)
            sys.stdout.write(
                f"{CLEAR_LINE}clutch {'ENGAGED ' if clutch.engaged else 'released'}  "
                f"target-start {np.round(target - tcp0[:3], 1)} mm  J5 {q[4]:6.1f} (eff {effective_j5(q[4]):5.1f})  |  "
                f"button fail {tool_rate:4.1f}%  leader fail {leader_rate:4.1f}%")
            sys.stdout.flush()

        # 7. Sleep until the next cycle; after an overrun wait a full period (08, as in 13)
        cycle += 1
        next_t += PERIOD_S
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic() + PERIOD_S
            time.sleep(PERIOD_S)


CSV_HEADER = (["t_s", "cycle_interval_ms", "button_raw", "clutch_engaged"]
              + [f"leader_q{i}_deg" for i in range(7)] + [f"leader_{a}_mm" for a in "xyz"]
              + [f"desired_{a}" for a in "xyz"] + [f"target_{a}" for a in "xyz"] + [f"tcp_ref_{a}" for a in "xyz"]
              + [f"jnt_ref_{i}" for i in range(6)] + ["j5_eff_pred_deg", "max_dq_pred_deg"]
              + ["speed_limited", "j5_slow", "joint_speed_limited", "range_limited", "lead_limited",
                 "late", "stale", "state_age_ms"])


def save_csv(rows):
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"21_leader_pos_sim_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)
    return path


def read_tcp_offset(robot, rc):
    """TCP offset in the flange frame from get_tcp_info / get_tfc_info (17)."""
    res_t, tcp = robot.get_tcp_info(rc, timeout=1.0)
    res_f, tfc = robot.get_tfc_info(rc, timeout=1.0)
    if not (res_t.is_success() and res_f.is_success()):
        raise SystemExit("get_tcp_info / get_tfc_info failed.")
    tcp, tfc = np.asarray(tcp, dtype=float).reshape(6), np.asarray(tfc, dtype=float).reshape(6)
    if np.abs(tcp[3:] - tfc[3:]).max() > 0.01:
        raise SystemExit("The TCP has an orientation offset; this script assumes none (17: none).")
    return rot_zyx(*tfc[3:]).T @ (tcp[:3] - tfc[:3])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="rotation of the leader frame about z [deg]")
    parser.add_argument("--scale", type=float, default=1.0, help="RB5 motion / leader motion")
    parser.add_argument("--port", default=LEADER_PORT, help="leader arm USB-serial device")
    args = parser.parse_args()
    if not 0.1 <= args.scale <= 2.0:
        raise SystemExit("--scale must be between 0.1 and 2.0")
    np.set_printoptions(precision=2, suppress=True)

    latency_timer = read_latency_timer(args.port)
    if latency_timer != 1:
        raise SystemExit(f"FTDI latency_timer is {latency_timer} (need 1). Set it with:\n"
                         f"  echo 1 | sudo tee /sys/bus/usb-serial/devices/"
                         f"{os.path.basename(os.path.realpath(args.port))}/latency_timer")
    dxl_port_handler.LATENCY_TIMER = 1
    leader_model = LeaderModel(URDF_PATH)
    leader = LeaderArm(args.port)
    leader.open()

    robot = rb.Cobot(config.ROBOT_IP)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)
    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()
    if data_channel.request_data(1.0).sdata.real_vs_simulation_mode != 1:
        leader.close()
        raise SystemExit("The controller does not report Simulation mode. Not starting.")
    if robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        leader.close()
        raise SystemExit("The robot is moving. Not starting.")
    rb5_model = RB5Model(read_tcp_offset(robot, rc))

    reader = StateReader()
    reader.start()
    reader.wait_first()
    tcp0, q0, _ = reader.latest()
    fk_error = np.linalg.norm(rb5_model.tcp(q0)[0] - tcp0[:3])
    print(f"Mode: Simulation   loop {PERIOD_S * 1000:.0f} ms   move_servo_l t1={SERVO_T1} t2={SERVO_T2}   "
          f"yaw {args.yaw_deg} deg   scale {args.scale}")
    print(f"RB5 tcp_ref {tcp0}   jnt_ref {q0}")
    print(f"TCP offset (flange frame) {rb5_model.offset} mm   DH model vs tcp_ref {fk_error:.2f} mm")
    print(f"Limits: {MAX_STEP_MM} mm/cycle ({MAX_STEP_MM / PERIOD_S:.0f} mm/s), {MAX_DQ_DEG} deg/cycle joints, "
          f"range +/-{RANGE_MM:.0f} mm, lead {MAX_LEAD_MM} mm, J5 slow < {J5_SLOW_DEG:.0f} / stop < {J5_STOP_DEG:.0f} deg")
    problems = []
    if fk_error > MAX_FK_ERROR_MM:
        problems.append(f"DH model differs from tcp_ref by {fk_error:.2f} mm (> {MAX_FK_ERROR_MM})")
    if effective_j5(q0[4]) < J5_SLOW_DEG:
        problems.append(f"J5 is {effective_j5(q0[4]):.1f} deg from the singular angle (need >= {J5_SLOW_DEG:.0f}); "
                        f"move it first, e.g. scripts/19_move_joint_sim.py --joint 5 --to 45")
    if problems:
        reader.stop()
        leader.close()
        raise SystemExit("Not starting:\n  " + "\n  ".join(problems))
    print("Clutch: right handle button (press and hold to follow, release to freeze). Starts released.")

    ctx = {"target": None, "rows": [], "clutch": None, "rpy": tcp0[3:].copy()}
    error = None
    robot.disable_waiting_ack(rc)   # servo commands must not wait for an ACK each cycle
    print("Ctrl+C to stop.\n")
    try:
        run_loop(leader, leader_model, robot, rc, reader, rb5_model, args, ctx)
    except KeyboardInterrupt:
        event("Interrupted.")
    except StopRequest as e:
        event(f"Stopping: {e}")
    except Exception as e:   # controller error, late loop, stale state: stop without more servo commands
        error = e
        event(f"Error: {e}")
    finally:
        if error is None and ctx["target"] is not None:
            stop_streaming(robot, rc, ctx["target"], ctx["rpy"])
        else:
            emergency_stop(robot, rc)
        reader.stop()
        leader.close()

    path = save_csv(ctx["rows"])
    clutch = ctx["clutch"]
    if clutch is not None and clutch.reads:
        print(f"Button reads {clutch.reads}, failed or unexpected {clutch.failures} "
              f"({100.0 * clutch.failures / clutch.reads:.1f}%)")
    print(f"Saved: {path.relative_to(ROOT)} ({len(ctx['rows'])} cycles)")


if __name__ == "__main__":
    main()

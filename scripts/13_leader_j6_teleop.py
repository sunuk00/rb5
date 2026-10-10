"""Leader arm J6 -> RB5 J6 teleoperation with a clutch (Simulation by default).

Based on 08_servo_jog_fast.py. Each 20 ms cycle:
  1. read leader arm ID 6 (J6_Wrist_Yaw2_R) and the right tool (ID 128) button, read-only
  2. clutch: button = (raw >> 8) & 0xff; 1 = pressed, 0 = released, anything else = failed read
  3. while the clutch is engaged: target = RB5 start + (leader now - leader start),
     limited per cycle and to a range around the RB5 J6 angle at script start.
     When the per-cycle limit cuts a step, the excess is dropped (the leader start is shifted),
     so the robot does not keep catching up after the hand stops.
  4. send move_servo_j(target) every cycle; released clutch = frozen target = robot stops

Clutch debounce: a press counts after 2 consecutive pressed reads. 1-2 failed reads keep the last
state; 3 failed reads in a row count as released. The clutch always starts released, and if the
button is held at start it must be released once before a press is accepted.

Usage:
    python scripts/13_leader_j6_teleop.py            # Simulation
    python scripts/13_leader_j6_teleop.py --real     # Real mode (asks for typed confirmation)
Stop with Ctrl+C. Each run is logged to data/13_leader_j6_<mode>_<time>.csv
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
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

# --- RB5 servo streaming (same values as 08 with --period 0.02)
PERIOD_S = 0.02
SERVO_T1 = PERIOD_S        # matches the send period (observed in 06 / 10)
SERVO_T2 = 0.1
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0
RB5_JOINT = 5              # RB5 J6 (index 5)

# --- Safety limits
MAX_STEP_SIM = 0.6         # deg per cycle (30 deg/s)
MAX_STEP_REAL = 0.2        # deg per cycle (10 deg/s)
RANGE_SIM = 90.0           # deg around the RB5 J6 angle at script start
RANGE_REAL = 30.0
MAX_LEAD_DEG = 1.0         # do not move the target further than this ahead of the robot (08)
MAX_GAP_S = 1.5 * SERVO_T1 # a send gap longer than this freezes the target for that cycle (08)
MAX_LATE_CYCLES = 3
MAX_STATE_AGE_S = 0.1      # RB5 state sample older than this freezes the target (08)
MAX_STALE_CYCLES = 3
MAX_LEADER_FAILS = 3       # leader J6 read failures in a row -> stop
HOLD_ON_EXIT_S = 0.3

# --- Leader arm (rby1-sdk: dynamixel_bus.h, leader_arm.cpp)
LEADER_PORT = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
ADDR_PRESENT_POSITION = 132  # the tool returns its button state at the same address
LEADER_JOINT_ID = 6        # J6_Wrist_Yaw2_R
TOOL_ID = 0x80             # right tool
TOOL_READ_PAUSE_S = 100e-6 # rby1-sdk ReadButtonStatus sleeps 100 us after the read
TICKS_PER_REV = 4096       # rad = raw / 4096 * 2 * pi (rby1-sdk dynamixel_bus.cpp)

# --- Clutch
BUTTON_PRESSED = 1         # raw 0x00000100 when held (confirmed with 12)
BUTTON_RELEASED = 0
PRESS_CONFIRM_READS = 2
FAILS_TO_RELEASE = 3

STATUS_EVERY = 25          # redraw the status line every this many cycles (about 2 Hz)
DATA_DIR = ROOT / "data"
CLEAR_LINE = "\r\x1b[2K"


class StopRequest(Exception):
    """Controlled stop: end servo mode normally (hold target, move_speed_j(0), poll)."""


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def read_latency_timer(port_name):
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


class LeaderArm:
    """Read-only access to leader arm J6 and the right tool button. Only ping and read are used."""

    def __init__(self, port_name):
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"Failed to open {self.port.getPortName()}")
        if not self.port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")
        for dxl_id in (LEADER_JOINT_ID, TOOL_ID):
            if self.ph.ping(self.port, dxl_id)[1] != COMM_SUCCESS:
                raise SystemExit(f"Leader arm ID {dxl_id} did not answer the ping.")

    def read_joint_deg(self):
        """Return (raw, deg), or None if the read failed."""
        value, result, _ = self.ph.read4ByteTxRx(self.port, LEADER_JOINT_ID, ADDR_PRESENT_POSITION)
        if result != COMM_SUCCESS:
            return None
        raw = to_signed32(value)
        return raw, raw * 360.0 / TICKS_PER_REV

    def read_button_raw(self):
        """Return the tool's raw value, or None if the read failed."""
        value, result, _ = self.ph.read4ByteTxRx(self.port, TOOL_ID, ADDR_PRESENT_POSITION)
        time.sleep(TOOL_READ_PAUSE_S)
        return value if result == COMM_SUCCESS else None

    def close(self):
        self.port.closePort()


class Clutch:
    """Debounced clutch from the tool button."""

    def __init__(self):
        self.engaged = False       # always start released
        self.armed = False         # a press is accepted only after one release has been seen
        self.press_count = 0
        self.fail_count = 0
        self.reads = 0
        self.failures = 0          # failed reads + unexpected values

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
        # Failed read or a value that is neither pressed nor released
        self.failures += 1
        self.fail_count += 1
        self.press_count = 0
        if self.engaged and self.fail_count >= FAILS_TO_RELEASE:
            self.engaged = False
            return button, f"released ({FAILS_TO_RELEASE} failed button reads in a row)"
        return button, None


class StateReader(threading.Thread):
    """Reads the RB5 data channel continuously and keeps only the latest sample (from 08)."""

    def __init__(self, simulation):
        super().__init__(daemon=True)
        self.data_channel = rb.CobotData(config.ROBOT_IP)
        self.simulation = simulation
        self.lock = threading.Lock()
        self.sample = None
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
                self.sample = (q, time.monotonic())

    def wait_first(self, timeout_s=2.0):
        t0 = time.monotonic()
        while self.sample is None:
            if time.monotonic() - t0 > timeout_s:
                raise SystemExit("State reader got no data from the robot.")
            time.sleep(0.01)

    def latest(self):
        with self.lock:
            q, t = self.sample
        return q.copy(), time.monotonic() - t

    def stop(self):
        self.stop_event.set()
        self.join(2.0)


def wait_until_stopped(robot, rc, timeout_s=10.0):
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
    wait_until_stopped(robot, rc)
    rc.clear()


def emergency_stop(robot, rc):
    """End servo mode after an error: no more servo commands, task_stop and poll."""
    robot.enable_waiting_ack(rc)
    robot.task_stop(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def event(msg):
    sys.stdout.write(CLEAR_LINE + msg + "\n")
    sys.stdout.flush()


def run_loop(leader, robot, rc, reader, limits, ctx):
    """20 ms teleop loop. ctx["target"] holds the servo target, ctx["rows"] the CSV rows."""
    max_step, range_deg = limits
    q0, _ = reader.latest()
    ctx["target"] = q0.copy()
    target = ctx["target"]
    base = q0[RB5_JOINT]                          # range is fixed around the J6 angle at start
    low, high = base - range_deg, base + range_deg

    clutch = Clutch()
    ctx["clutch"] = clutch
    leader_last = None                            # last good (raw, deg)
    leader_fails = leader_fail_total = leader_reads = 0
    rb5_start = leader_start = None
    flags_prev = {"rate": False, "range": False, "lead": False}
    last_send = None
    late_cycles = stale_cycles = 0
    prev_cycle_t = None

    t_start = next_t = time.monotonic()
    cycle = 0
    while True:
        t_cycle = time.monotonic()
        interval = None if prev_cycle_t is None else t_cycle - prev_cycle_t
        prev_cycle_t = t_cycle

        # 1. Leader arm: J6 angle, then the tool button (with the SDK's 100 us pause)
        leader_reads += 1
        reading = leader.read_joint_deg()
        if reading is None:
            leader_fails += 1
            leader_fail_total += 1
            if leader_fails >= MAX_LEADER_FAILS:
                raise StopRequest(f"{MAX_LEADER_FAILS} leader J6 read failures in a row")
        else:
            leader_fails = 0
            leader_last = reading
        button_raw = leader.read_button_raw()

        # 2. Clutch
        button, clutch_event = clutch.update(button_raw)
        if clutch_event == "engaged":
            if leader_last is None:
                clutch.engaged = False            # no leader angle yet; ignore this press
                clutch_event = "press ignored (no leader J6 reading yet)"
            else:
                rb5_start, leader_start = target[RB5_JOINT], leader_last[1]
                clutch_event = (f"engaged: RB5 start {rb5_start:.2f} deg, leader start {leader_start:.2f} deg")
        if clutch_event:
            event(f"[{time.monotonic() - t_start:7.2f} s] clutch {clutch_event}")

        # 3. RB5 state and the 08 safety checks
        robot.flush(rc)
        rc.error().throw_if_not_empty()
        rc.clear()
        q, state_age = reader.latest()
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
        flags = {"rate": False, "range": False, "lead": False}
        if clutch.engaged and reading is not None and not late and not stale:
            desired = rb5_start + (reading[1] - leader_start)
            step = desired - target[RB5_JOINT]
            if abs(step) > max_step:
                limited = np.sign(step) * max_step
                # Drop the excess instead of catching up later: shift the leader start so that,
                # for this leader angle, the desired value equals where the target actually goes.
                # The robot then stops soon after the hand stops (hand-robot offset changes instead).
                leader_start += step - limited
                step = limited
                flags["rate"] = True
            new = target[RB5_JOINT] + step
            if new < low or new > high:
                new = min(max(new, low), high)
                flags["range"] = True
            if abs(new - q[RB5_JOINT]) <= MAX_LEAD_DEG:
                target[RB5_JOINT] = new
            else:
                flags["lead"] = True
        labels = {"rate": f"speed limit ({max_step} deg/cycle)",
                  "range": f"range limit ({low:.1f} .. {high:.1f} deg)",
                  "lead": f"target waits for robot (lead limit {MAX_LEAD_DEG} deg)"}
        for key, on in flags.items():
            if on and not flags_prev[key]:
                event(f"[{time.monotonic() - t_start:7.2f} s] {labels[key]}")
        flags_prev = flags

        # 5. Send every cycle (a released clutch sends the frozen target)
        send_servo(robot, rc, target)
        last_send = time.monotonic()

        ctx["rows"].append([
            f"{t_cycle - t_start:.4f}", "" if interval is None else f"{interval * 1000:.3f}",
            "" if button_raw is None else button_raw, "" if button is None else button, int(clutch.engaged),
            "" if reading is None else reading[0], "" if reading is None else f"{reading[1]:.3f}",
            "" if desired is None else f"{desired:.3f}", f"{target[RB5_JOINT]:.3f}", f"{q[RB5_JOINT]:.3f}",
            f"{state_age * 1000:.2f}", int(flags["rate"]), int(flags["range"]), int(flags["lead"]),
            int(late), int(stale)])

        # 6. Status line, redrawn in place
        if cycle % STATUS_EVERY == 0:
            tool_rate = 100.0 * clutch.failures / max(clutch.reads, 1)
            leader_rate = 100.0 * leader_fail_total / max(leader_reads, 1)
            leader_txt = "-" if leader_last is None else f"{leader_last[1]:7.2f}"
            sys.stdout.write(
                f"{CLEAR_LINE}clutch {'ENGAGED ' if clutch.engaged else 'released'}  leader J6 {leader_txt}  "
                f"target {target[RB5_JOINT]:7.2f}  RB5 J6 {q[RB5_JOINT]:7.2f}  |  "
                f"button read fail {tool_rate:4.1f}%  leader J6 read fail {leader_rate:4.1f}%")
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


CSV_HEADER = ["t_s", "cycle_interval_ms", "button_raw", "button", "clutch_engaged", "leader_raw",
              "leader_deg", "desired_deg", "target_deg", "rb5_j6_deg", "state_age_ms",
              "speed_limited", "range_limited", "lead_limited", "late", "stale"]


def save_csv(rows, mode_name):
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"13_leader_j6_{mode_name}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)
    return path


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="Simulation mode (default)")
    mode.add_argument("--real", action="store_true", help="Real mode (asks for typed confirmation)")
    parser.add_argument("--port", default=LEADER_PORT, help="leader arm USB-serial device")
    args = parser.parse_args()
    simulation = not args.real
    max_step, range_deg = (MAX_STEP_SIM, RANGE_SIM) if simulation else (MAX_STEP_REAL, RANGE_REAL)
    np.set_printoptions(precision=2, suppress=True)

    # Leader arm: needs latency_timer 1 ms, otherwise two reads take about 32 ms (> 20 ms period)
    latency_timer = read_latency_timer(args.port)
    if latency_timer != 1:
        raise SystemExit(f"FTDI latency_timer is {latency_timer} (need 1). Set it with:\n"
                         f"  echo 1 | sudo tee /sys/bus/usb-serial/devices/"
                         f"{os.path.basename(os.path.realpath(args.port))}/latency_timer")
    dxl_port_handler.LATENCY_TIMER = 1   # dynamixel_sdk reply timeout follows this constant (12)
    leader = LeaderArm(args.port)
    leader.open()

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001), used before the reader starts
    robot.set_operation_mode(rc, rb.OperationMode.Simulation if simulation else rb.OperationMode.Real)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()
    sdata = data_channel.request_data(1.0).sdata
    q = np.array(sdata.jnt_ref if simulation else sdata.jnt_ang, dtype=float)

    j6 = q[RB5_JOINT]
    print(f"Mode: {'Simulation' if simulation else 'REAL'}   leader {args.port} (latency_timer 1 ms)   "
          f"loop {PERIOD_S * 1000:.0f} ms   servo t1={SERVO_T1} t2={SERVO_T2}")
    print("RB5 current:", q)
    print(f"RB5 J6 now {j6:.2f} deg -> allowed range {j6 - range_deg:.2f} .. {j6 + range_deg:.2f} deg "
          f"(start +/-{range_deg:.0f}), max {max_step} deg/cycle ({max_step / PERIOD_S:.0f} deg/s)")
    print("Clutch: right handle button (press and hold to follow, release to freeze). Starts released.")
    if args.real:
        if input("REAL mode: the RB5 J6 will follow the leader arm. Type 'yes' to continue: ") != "yes":
            leader.close()
            raise SystemExit("Cancelled.")

    reader = StateReader(simulation)
    reader.start()
    reader.wait_first()

    ctx = {"target": None, "rows": [], "clutch": None}
    error = None
    robot.disable_waiting_ack(rc)   # servo commands must not wait for an ACK each cycle
    print("Ctrl+C to stop.\n")
    try:
        run_loop(leader, robot, rc, reader, (max_step, range_deg), ctx)
    except KeyboardInterrupt:
        event("Interrupted.")
    except StopRequest as e:
        event(f"Stopping: {e}")
    except Exception as e:   # controller error, late loop, stale state: stop without more servo commands
        error = e
        event(f"Error: {e}")
    finally:
        if error is None and ctx["target"] is not None:
            stop_streaming(robot, rc, ctx["target"])
        else:
            emergency_stop(robot, rc)
        reader.stop()
        leader.close()

    path = save_csv(ctx["rows"], "real" if args.real else "sim")
    clutch = ctx["clutch"]
    if clutch is not None and clutch.reads:
        print(f"Button reads {clutch.reads}, failed or unexpected {clutch.failures} "
              f"({100.0 * clutch.failures / clutch.reads:.1f}%)")
    print(f"Saved: {path.relative_to(ROOT)} ({len(ctx['rows'])} cycles)")


if __name__ == "__main__":
    main()

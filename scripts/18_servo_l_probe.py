"""Probe move_servo_l in Simulation mode: frame, axis and rotation convention. One experiment per run.

  --exp hold : target = tcp_ref at start (expect no motion)
  --exp z    : target = start with z + 1.0 mm (step)
  --exp rz   : target = start with rz + 1.0 deg (step; ZY'X'' Euler, so a rotation about base Z)

The same target is sent every 20 ms for 1.0 s with move_servo_l(target, t1=0.02, t2=0.1, gain=1,
alpha=1), then servo mode ends as in 13 (move_speed_j(0), flush, poll get_robot_state).
A StateReader thread (from 08/13) keeps the latest tcp_ref / jnt_ref.

Safety (Simulation only, there is no --real): stop at once if the TCP moves more than 3 mm or
rotates more than 2 deg from the start pose, a joint moves more than 3 deg, a send gap is over
30 ms, or the state sample is older than 0.1 s.

Usage:
    python scripts/18_servo_l_probe.py --exp hold
    python scripts/18_servo_l_probe.py --exp z
    python scripts/18_servo_l_probe.py --exp rz
Each run is logged to data/18_servo_l_<exp>_<time>.csv
"""
import argparse
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

# --- Servo streaming (same values as 13)
PERIOD_S = 0.02
SERVO_T1 = PERIOD_S        # matches the send period (observed in 06 / 10)
SERVO_T2 = 0.1
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0
SEND_CYCLES = 50           # 1.0 s of the same target

# --- Experiments: offset added to the start pose (x, y, z [mm], rx, ry, rz [deg])
OFFSETS = {
    "hold": np.zeros(6),
    "z": np.array([0, 0, 1.0, 0, 0, 0]),
    "rz": np.array([0, 0, 0, 0, 0, 1.0]),
}

# --- Safety limits
MAX_POS_MM = 3.0           # TCP position change from the start pose
MAX_ROT_DEG = 2.0          # TCP rotation angle from the start pose
MAX_JOINT_DEG = 3.0        # any joint change from the start
MAX_GAP_S = 0.03           # send gap
MAX_STATE_AGE_S = 0.1

# --- "Reached" thresholds for the summary
REACH_MM = 0.01
REACH_DEG = 0.01

DATA_DIR = ROOT / "data"


class SafetyStop(Exception):
    """A safety limit was hit: stop sending servo commands at once."""


def rot_zyx(rx, ry, rz):
    """Rotation matrix of a ZY'X'' Euler pose (deg): R = Rz(rz) Ry(ry) Rx(rx)."""
    a, b, c = np.radians([rx, ry, rz])
    rx_m = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    ry_m = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    rz_m = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def rot_angle_deg(p, q):
    """Angle of the rotation between the orientations of poses p and q (deg)."""
    r = rot_zyx(*p[3:]).T @ rot_zyx(*q[3:])
    return np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))


def wrap_deg(v):
    return (v + 180.0) % 360.0 - 180.0


class StateReader(threading.Thread):
    """Reads the data channel continuously and keeps only the latest tcp_ref / jnt_ref (from 08/13)."""

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
            # Simulation: jnt_ang does not change, so use the reference values (tcp_ref, jnt_ref)
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


def end_servo(robot, rc):
    """End servo mode as in 13 / examples/move_servo_j.cpp: speed 0, then poll."""
    robot.move_speed_j(rc, np.zeros(6), SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
    robot.enable_waiting_ack(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)


def check_safety(tcp, q, age, p0, q0, gap):
    if age > MAX_STATE_AGE_S:
        raise SafetyStop(f"state sample is {age * 1000:.0f} ms old")
    if gap is not None and gap > MAX_GAP_S:
        raise SafetyStop(f"send gap {gap * 1000:.1f} ms > {MAX_GAP_S * 1000:.0f} ms")
    pos = np.linalg.norm(tcp[:3] - p0[:3])
    if pos > MAX_POS_MM:
        raise SafetyStop(f"TCP moved {pos:.3f} mm > {MAX_POS_MM} mm")
    rot = rot_angle_deg(p0, tcp)
    if rot > MAX_ROT_DEG:
        raise SafetyStop(f"TCP rotated {rot:.3f} deg > {MAX_ROT_DEG} deg")
    dq = np.max(np.abs(wrap_deg(q - q0)))
    if dq > MAX_JOINT_DEG:
        raise SafetyStop(f"a joint moved {dq:.3f} deg > {MAX_JOINT_DEG} deg")


def run(robot, rc, reader, target, p0, q0, rows):
    """Send the same target every period. Each row: t, gap, target, tcp_ref, jnt_ref, state age."""
    t_start = next_t = time.monotonic()
    last_send = None
    for _ in range(SEND_CYCLES):
        tcp, q, age = reader.latest()
        now = time.monotonic()
        gap = None if last_send is None else now - last_send
        rows.append([now - t_start, gap, *target, *tcp, *q, age])
        check_safety(tcp, q, age, p0, q0, gap)
        robot.move_servo_l(rc, target, SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
        last_send = now
        next_t += PERIOD_S
        time.sleep(max(0.0, next_t - time.monotonic()))


CSV_HEADER = (["t_s", "send_gap_ms"] + [f"target_{s}" for s in ("x", "y", "z", "rx", "ry", "rz")]
              + [f"tcp_ref_{s}" for s in ("x", "y", "z", "rx", "ry", "rz")]
              + [f"jnt_ref_{i}" for i in range(6)] + ["state_age_ms"])


def save_csv(rows, exp):
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"18_servo_l_{exp}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        for r in rows:
            t, gap = r[0], r[1]
            writer.writerow([f"{t:.4f}", "" if gap is None else f"{gap * 1000:.2f}"]
                            + [f"{v:.4f}" for v in r[2:-1]] + [f"{r[-1] * 1000:.1f}"])
    return path


def summarize(rows, exp, p0, q0, final_tcp, final_q):
    """Print what the experiment shows. Rotation deltas are shown as plain number differences."""
    np.set_printoptions(precision=4, suppress=True)
    target = p0 + OFFSETS[exp]
    d = final_tcp - p0
    d[3:] = wrap_deg(d[3:])
    print(f"\nStart tcp_ref      {p0}")
    print(f"Target             {target}")
    print(f"Final tcp_ref      {final_tcp}")
    print(f"Final - start      {d}   (x, y, z [mm], rx, ry, rz [deg])")
    print(f"Rotation from start {rot_angle_deg(p0, final_tcp):.4f} deg, "
          f"max joint change {np.max(np.abs(wrap_deg(final_q - q0))):.4f} deg")
    print(f"Joint change       {wrap_deg(final_q - q0)}")
    if exp == "z":
        tool_z = rot_zyx(*p0[3:])[:, 2]
        print(f"Expected if base Z: [0, 0, 1.000]   if tool Z: {tool_z}")
    if exp == "rz":
        print("Expected if rotation about base Z through the TCP: drz = +1.000, dx dy dz about 0")
    # First time the measured pose is within the reach thresholds of the target
    for r in rows:
        tcp = np.array(r[8:14])
        err = tcp - target
        err[3:] = wrap_deg(err[3:])
        if np.all(np.abs(err[:3]) < REACH_MM) and np.all(np.abs(err[3:]) < REACH_DEG):
            print(f"Within {REACH_MM} mm / {REACH_DEG} deg of the target at t = {r[0] * 1000:.0f} ms")
            break
    else:
        print(f"Never within {REACH_MM} mm / {REACH_DEG} deg of the target during the {SEND_CYCLES} cycles")
    gaps = [r[1] for r in rows if r[1] is not None]
    if gaps:
        print(f"Send gap mean {np.mean(gaps) * 1000:.2f} ms, max {np.max(gaps) * 1000:.2f} ms")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--exp", required=True, choices=OFFSETS.keys())
    args = parser.parse_args()
    np.set_printoptions(precision=3, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001), used before the reader starts
    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()
    sdata = data_channel.request_data(1.0).sdata
    if sdata.real_vs_simulation_mode != 1:
        raise SystemExit("The controller does not report Simulation mode. Not starting.")
    if robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        raise SystemExit("The robot is moving. Not starting.")

    reader = StateReader()
    reader.start()
    reader.wait_first()
    p0, q0, _ = reader.latest()
    target = p0 + OFFSETS[args.exp]
    print(f"Mode: Simulation   experiment: {args.exp}   {SEND_CYCLES} x {PERIOD_S * 1000:.0f} ms   "
          f"move_servo_l t1={SERVO_T1} t2={SERVO_T2} gain={SERVO_GAIN} alpha={SERVO_ALPHA}")
    print(f"Start tcp_ref {p0}")
    print(f"Target        {target}")
    print(f"Start jnt_ref {q0}")

    rows = []
    error = None
    robot.disable_waiting_ack(rc)   # servo commands must not wait for an ACK each cycle
    try:
        run(robot, rc, reader, target, p0, q0, rows)
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as e:          # SafetyStop or a controller error
        error = e
        print(f"Stopping: {e}")
    finally:
        end_servo(robot, rc)
        time.sleep(0.2)             # let the reader pick up the pose after stopping
        final_tcp, final_q, _ = reader.latest()
        reader.stop()

    try:
        rc.error().throw_if_not_empty()
        print("No error messages from the controller.")
    except Exception as e:
        print(f"Controller error message(s): {e}")
    rc.clear()

    path = save_csv(rows, args.exp)
    summarize(rows, args.exp, p0, q0, final_tcp, final_q)
    print(f"Saved: {path.relative_to(ROOT)} ({len(rows)} cycles)" + (" - stopped early" if error else ""))


if __name__ == "__main__":
    main()

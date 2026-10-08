"""Observe how move_servo_j behaves (Simulation mode only).

The meaning of move_servo_j(joint, t1, t2, gain, alpha) is not documented in refs/rbpodo,
so this script streams J6 targets with given parameters and records how jnt_ref follows.

Phases (J6 only, targets relative to the current jnt_ref):
    1 ramp   2.0 s   target += RAMP_SPEED * t, sent every --period (default 10 ms)
    2 hold   1.0 s   same target, still sent every --period
    3 ramp   1.0 s   ramp again
    4 cut    2.0 s   stop sending (like releasing a jog key)
    5 finish         move_speed_j(zeros), enable ACK, poll until stopped (as in examples/move_servo_j.cpp)

Usage:
    python scripts/06_servo_j_probe.py                      # example values: 0.01 0.1 1.0 1.0
    python scripts/06_servo_j_probe.py --t1 0.05            # change one parameter at a time
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

JOINT_INDEX = 5            # J6 (wrist 3)
RAMP_SPEED = 5.0           # deg/s
PHASES = [(1, "ramp", 2.0), (2, "hold", 1.0), (3, "ramp", 1.0), (4, "cut", 2.0)]
STOP_TIMEOUT_S = 10.0
DATA_DIR = ROOT / "data"


class Logger(threading.Thread):
    """Reads the data channel as fast as possible on its own socket."""

    def __init__(self, t0, shared):
        super().__init__(daemon=True)
        self.data_channel = rb.CobotData(config.ROBOT_IP)
        self.t0 = t0
        self.shared = shared   # phase / target written by the command thread
        self.rows = []
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            data = self.data_channel.request_data(1.0)
            if data is None:
                continue
            s = data.sdata
            self.rows.append((time.perf_counter() - self.t0, self.shared["phase"], self.shared["target"],
                              s.jnt_ref[JOINT_INDEX], s.robot_state))


def wait_for_simulation(data_channel):
    t0 = time.monotonic()
    while True:
        sdata = data_channel.request_data(1.0).sdata
        if sdata.real_vs_simulation_mode == 1:
            return sdata
        if time.monotonic() - t0 > 2.0:
            raise SystemExit("Controller is not in Simulation mode. Stopping.")
        time.sleep(0.05)


def analyze(arr, phase_start, q0):
    t, phase, target, ref = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    print(f"\nJ6 start {q0:.2f} deg, total change {ref[-1] - q0:+.2f} deg")
    if np.max(np.abs(ref - q0)) < 1e-3:
        print("No motion observed in jnt_ref.")
        return

    # Phase 1: tracking error and time lag (skip the first 0.5 s while it settles)
    m = (phase == 1) & (t > phase_start[1] + 0.5)
    if m.any():
        err = target[m] - ref[m]
        print(f"Phase 1 ramp : tracking error mean {err.mean():+.3f} deg, max {np.abs(err).max():.3f} deg "
              f"-> lag about {err.mean() / RAMP_SPEED * 1000:.0f} ms")

    # Phase 2: does it reach the held target, and overshoot?
    m = phase == 2
    if m.any():
        hold = target[m][-1]
        off = np.nonzero(np.abs(ref[m] - hold) > 1e-3)[0]   # samples not yet within 0.001 deg
        settle = t[m][min(off[-1] + 1, m.sum() - 1)] - phase_start[2] if len(off) else 0.0
        print(f"Phase 2 hold : settled after {settle * 1000:.0f} ms, error at end {ref[m][-1] - hold:+.3f} deg, "
              f"max beyond target {np.max(ref[m] - hold):+.3f} deg")

    # Phase 4: after the last command, how long and how far does it keep moving?
    m4 = phase == 4
    if m4.any():
        t_cut = phase_start[4]
        last_target = target[m4][0]
        ref_cut = ref[m4][0]
        r4, t4 = ref[m4], t[m4]
        changed = np.nonzero(np.diff(r4) != 0)[0]
        t_last_change = t4[changed[-1] + 1] if len(changed) else t_cut
        print(f"Phase 4 cut  : last target {last_target:.3f}, jnt_ref at cut {ref_cut:.3f}, "
              f"final {r4[-1]:.3f} (final - last target {r4[-1] - last_target:+.3f} deg)")
        print(f"               kept moving for {(t_last_change - t_cut) * 1000:.0f} ms, "
              f"{r4[-1] - ref_cut:+.3f} deg after the cut")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--t1", type=float, default=0.01)
    parser.add_argument("--t2", type=float, default=0.1)
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--alpha", type=float, default=1.0)
    parser.add_argument("--period", type=float, default=0.01, help="servo command period (s)")
    args = parser.parse_args()
    params = (args.t1, args.t2, args.gain, args.alpha)
    np.set_printoptions(precision=3, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001)

    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()
    sdata = wait_for_simulation(data_channel)

    q_start = np.array(sdata.jnt_ref, dtype=float)
    total = RAMP_SPEED * sum(d for _, kind, d in PHASES if kind == "ramp")
    print("Mode: Simulation   speed bar", config.SPEED)
    print(f"move_servo_j params: t1={args.t1} t2={args.t2} gain={args.gain} alpha={args.alpha}")
    print("Start (jnt_ref):", q_start)
    print(f"J6 will be streamed {RAMP_SPEED} deg/s up to {total:+.1f} deg, every {args.period * 1000:.0f} ms")

    shared = {"phase": 0, "target": q_start[JOINT_INDEX]}
    t0 = time.perf_counter()
    logger = Logger(t0, shared)
    logger.start()
    time.sleep(0.2)   # a little baseline before the first command

    phase_start = {}
    target = q_start.copy()
    sent = 0
    robot.disable_waiting_ack(rc)
    try:
        for phase, kind, duration in PHASES:
            shared["phase"] = phase
            phase_start[phase] = time.perf_counter() - t0
            base = target[JOINT_INDEX]
            t_phase = time.perf_counter()
            next_t = t_phase
            while time.perf_counter() - t_phase < duration:
                if kind == "ramp":
                    target[JOINT_INDEX] = base + RAMP_SPEED * (time.perf_counter() - t_phase)
                if kind != "cut":
                    shared["target"] = target[JOINT_INDEX]
                    robot.move_servo_j(rc, target, *params)
                    sent += 1
                next_t += args.period
                time.sleep(max(0.0, next_t - time.perf_counter()))

        # Finish the same way as examples/move_servo_j.cpp, but poll instead of an event wait
        shared["phase"] = 5
        phase_start[5] = time.perf_counter() - t0
        robot.move_speed_j(rc, np.zeros(6), *params)
    finally:
        robot.enable_waiting_ack(rc)
        robot.flush(rc)
        t_stop = time.monotonic()
        while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
            if time.monotonic() - t_stop > STOP_TIMEOUT_S:
                print(f"warning: still moving after {STOP_TIMEOUT_S} s, sending task_stop")
                robot.task_stop(rc)
                break
            time.sleep(0.01)
        time.sleep(0.5)
        logger.stop_event.set()
        logger.join()

    # Controller messages collected while ACK waiting was off (errors, warnings)
    errors = rc.error()
    try:
        errors.throw_if_not_empty()
        print(f"\nCommands sent: {sent}. No error messages from the controller.")
    except Exception as e:
        print(f"\nCommands sent: {sent}. Controller error message(s): {e}")

    # Save CSV
    DATA_DIR.mkdir(exist_ok=True)
    tag = "_".join(f"{p:g}" for p in params) + f"_p{args.period * 1000:g}ms"
    path = DATA_DIR / f"06_servo_{tag}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["pc_time_s", "phase", "target_j6", "jnt_ref_j6", "robot_state"])
        writer.writerows(logger.rows)
    print(f"Saved: {path.relative_to(ROOT)} ({len(logger.rows)} samples)")

    arr = np.array(logger.rows, dtype=float)
    states = {p: sorted(set(arr[arr[:, 1] == p, 4].astype(int))) for p in range(6)}
    print("robot_state seen per phase:", {p: s for p, s in states.items() if s})
    analyze(arr, phase_start, q_start[JOINT_INDEX])


if __name__ == "__main__":
    main()

"""Move one RB5 joint to an absolute angle with move_j, Simulation mode only.

Used to put J5 away from the wrist singularity (J5 = 0) before rerunning 18 --exp rz.
The target is the current jnt_ref with only the chosen joint changed.

Checks:
  before: controller reports Simulation, robot not moving, |change| <= 60 deg.
          Prints current / target joints and the controller's FK (calc_fk_tcp) of both.
  during: move_j(10 deg/s, 20 deg/s^2) with speed bar 0.1. The controller scales the move_j speed by
          the speed bar (seen in the first run: about 1 deg/s), so the expected time uses
          speed x speed bar; task_stop if not finished in 1.5 x expected + 5 s.
  after:  the joint within 0.05 deg of the target, other joints changed less than 0.05 deg,
          no controller error messages.

Usage:
    python scripts/19_move_joint_sim.py --joint 5 --to 45
(--joint is 1-based: 5 = J5 = index 4)
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

SPEED_DEG_S = 10.0         # same as 02
ACC_DEG_S2 = 20.0
MAX_CHANGE_DEG = 60.0
TIMEOUT_FACTOR = 1.5       # move timeout = factor x expected time + margin
TIMEOUT_MARGIN_S = 5.0
TOL_DEG = 0.05
TIMEOUT_S = 1.0


def read_jnt_ref(data_channel):
    data = data_channel.request_data(TIMEOUT_S)
    if data is None:
        raise SystemExit("request_data() timed out")
    # Simulation: jnt_ang does not change, so use the reference joints
    return data.sdata, np.array(data.sdata.jnt_ref, dtype=float)


def fk(robot, rc, q):
    res, pnt = robot.calc_fk_tcp(rc, j0=q[0], j1=q[1], j2=q[2], j3=q[3], j4=q[4], j5=q[5], timeout=TIMEOUT_S)
    return np.array(pnt, dtype=float).reshape(6) if res.is_success() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint", type=int, required=True, choices=range(1, 7), help="joint number 1-6")
    parser.add_argument("--to", type=float, required=True, help="target angle [deg]")
    args = parser.parse_args()
    idx = args.joint - 1
    np.set_printoptions(precision=3, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001)
    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()

    sdata, q_now = read_jnt_ref(data_channel)
    if sdata.real_vs_simulation_mode != 1:
        raise SystemExit("The controller does not report Simulation mode. Not starting.")
    if robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        raise SystemExit("The robot is moving. Not starting.")

    q_target = q_now.copy()
    q_target[idx] = args.to
    change = q_target[idx] - q_now[idx]
    if abs(change) > MAX_CHANGE_DEG:
        raise SystemExit(f"J{args.joint} change {change:.2f} deg is over {MAX_CHANGE_DEG} deg. Not starting.")

    # Trapezoid profile: distance / speed + speed / acceleration. Acceleration is assumed to be scaled
    # by the speed bar too (not confirmed); that gives the longer, safer estimate.
    speed = SPEED_DEG_S * config.SPEED
    expected_s = abs(change) / speed + speed / (ACC_DEG_S2 * config.SPEED)
    timeout_s = TIMEOUT_FACTOR * expected_s + TIMEOUT_MARGIN_S
    print(f"Mode: Simulation   move_j J{args.joint}: {q_now[idx]:.3f} -> {args.to:.3f} deg "
          f"({change:+.3f} deg at {SPEED_DEG_S} deg/s x speed bar {config.SPEED} = {speed:.2f} deg/s, "
          f"about {expected_s:.1f} s, timeout {timeout_s:.1f} s)")
    print(f"Current joints {q_now}")
    print(f"Target joints  {q_target}")
    print(f"Current TCP (calc_fk_tcp) {fk(robot, rc, q_now)}")
    print(f"Target TCP  (calc_fk_tcp) {fk(robot, rc, q_target)}")

    robot.flush(rc)
    robot.move_j(rc, q_target, SPEED_DEG_S, ACC_DEG_S2)
    if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
        t0 = time.monotonic()
        while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
            if time.monotonic() - t0 > timeout_s:
                robot.task_stop(rc)
                raise SystemExit(f"Move not finished in {timeout_s:.1f} s; task_stop sent.")
        print(f"Move took {time.monotonic() - t0:.1f} s (expected about {expected_s:.1f} s)")
            time.sleep(0.01)
    else:
        print("Warning: the move did not report a start within 0.5 s.")

    try:
        rc.error().throw_if_not_empty()
        print("No error messages from the controller.")
    except Exception as e:
        print(f"Controller error message(s): {e}")
    rc.clear()

    time.sleep(0.1)
    sdata, q_after = read_jnt_ref(data_channel)
    diff = q_after - q_target
    others = np.delete(np.abs(q_after - q_now), idx)
    print(f"\nAfter joints   {q_after}")
    print(f"After - target {diff}")
    print(f"After TCP (tcp_ref) {np.array(sdata.tcp_ref, dtype=float)}")
    ok_joint = abs(diff[idx]) < TOL_DEG
    ok_others = np.all(others < TOL_DEG)
    print(f"J{args.joint} within {TOL_DEG} deg of target: {'OK' if ok_joint else 'NO'}   "
          f"other joints changed < {TOL_DEG} deg: {'OK' if ok_others else 'NO'} (max {others.max():.4f})")


if __name__ == "__main__":
    main()

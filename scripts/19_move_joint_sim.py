"""Move the RB5 with move_j, Simulation mode only: one joint, or all six joints to a pose.

  --joint N --to DEG : only joint N changes (used to put J5 away from the wrist singularity)
  --pose J1 .. J6    : all joints. For each joint the target is replaced by the equivalent angle
                       (target + k x 360) inside the joint limit that is closest to the current angle,
                       so the move is the shortest (e.g. J4 -165 -> 195 if that is closer and allowed).

Joint limits (rb_cobot_docs manual 1.6 Robot Operating Range: J1, J2, J4, J5, J6 +/-360 deg,
J3 +/-165 / +/-150 deg; appendix System Specifications: "Joint Range +/-360 (Elbow: +/-165)" for the
E series). This robot is an RB5-850 (N), not E, so J3 uses the smaller +/-150 here. The controller's
own (setup) joint limits may be narrower and are not read by this script.

Checks:
  before: controller reports Simulation, robot not moving, change per joint <= 60 deg (--joint) or
          <= 180 deg (--pose). Prints current / target joints and the controller's FK (calc_fk_tcp).
  during: move_j(speed, 2 x speed deg/s^2) with speed bar 0.1. The controller scales the move_j speed
          by the speed bar (19 first run), so the expected time uses speed x speed bar;
          task_stop if not finished in 1.5 x expected + 5 s.
  after:  every joint within 0.05 deg of the target, no controller error messages.

Usage:
    python scripts/19_move_joint_sim.py --joint 5 --to 45
    python scripts/19_move_joint_sim.py --pose 180 -15 -90 -165 90 0 --speed 30
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

DEFAULT_SPEED_DEG_S = 10.0  # same as 02
MAX_SPEED_DEG_S = 30.0      # largest --speed (Simulation only); scaled by the speed bar
ACC_PER_SPEED = 2.0         # acceleration = 2 x speed (10 -> 20 deg/s^2 as before)
JOINT_LIMIT_DEG = np.array([360.0, 360.0, 150.0, 360.0, 360.0, 360.0])
MAX_CHANGE_JOINT_DEG = 60.0
MAX_CHANGE_POSE_DEG = 180.0
TIMEOUT_FACTOR = 1.5        # move timeout = factor x expected time + margin
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


def nearest_equivalent(target, current, limit):
    """target + k x 360 inside +/-limit that is closest to current, or None if none is inside."""
    candidates = [target + 360.0 * k for k in range(-2, 3) if abs(target + 360.0 * k) <= limit]
    return min(candidates, key=lambda a: abs(a - current)) if candidates else None


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--joint", type=int, choices=range(1, 7), help="joint number 1-6 (with --to)")
    mode.add_argument("--pose", type=float, nargs=6, metavar="J", help="target joint angles J1..J6 [deg]")
    parser.add_argument("--to", type=float, help="target angle [deg] for --joint")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_DEG_S,
                        help=f"move_j speed argument [deg/s], at most {MAX_SPEED_DEG_S:.0f}; scaled by the speed bar")
    args = parser.parse_args()
    if args.joint is not None and args.to is None:
        parser.error("--joint needs --to")
    if not 0 < args.speed <= MAX_SPEED_DEG_S:
        parser.error(f"--speed must be in (0, {MAX_SPEED_DEG_S:.0f}]")
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
    if args.joint is not None:
        q_target[args.joint - 1] = args.to
        max_change = MAX_CHANGE_JOINT_DEG
    else:
        for i, (t, c) in enumerate(zip(args.pose, q_now)):
            a = nearest_equivalent(t, c, JOINT_LIMIT_DEG[i])
            if a is None:
                raise SystemExit(f"J{i + 1} target {t} has no equivalent inside +/-{JOINT_LIMIT_DEG[i]:.0f} deg.")
            q_target[i] = a
        max_change = MAX_CHANGE_POSE_DEG
    if np.any(np.abs(q_target) > JOINT_LIMIT_DEG):
        raise SystemExit(f"Target {q_target} is outside the joint limits +/-{JOINT_LIMIT_DEG}. Not starting.")
    change = q_target - q_now
    if np.abs(change).max() > max_change:
        raise SystemExit(f"Joint change {change} is over {max_change:.0f} deg. Not starting.")

    # Trapezoid profile of the largest change: distance / speed + speed / acceleration. Acceleration is
    # assumed to be scaled by the speed bar too (not confirmed); that gives the longer, safer estimate.
    acc = ACC_PER_SPEED * args.speed
    speed = args.speed * config.SPEED
    expected_s = np.abs(change).max() / speed + speed / (acc * config.SPEED)
    timeout_s = TIMEOUT_FACTOR * expected_s + TIMEOUT_MARGIN_S
    print(f"Mode: Simulation   move_j {args.speed} deg/s, {acc} deg/s^2 x speed bar {config.SPEED} "
          f"= {speed:.2f} deg/s; largest change {np.abs(change).max():.2f} deg, "
          f"about {expected_s:.1f} s, timeout {timeout_s:.1f} s")
    if args.pose is not None:
        print(f"Requested pose {np.array(args.pose)}")
    print(f"Current joints {q_now}")
    print(f"Target joints  {q_target}")
    print(f"Change         {change}")
    print(f"Current TCP (calc_fk_tcp) {fk(robot, rc, q_now)}")
    print(f"Target TCP  (calc_fk_tcp) {fk(robot, rc, q_target)}")

    if np.abs(change).max() < TOL_DEG:
        print(f"Already within {TOL_DEG} deg of the target; no move_j sent.")
        return

    robot.flush(rc)
    robot.move_j(rc, q_target, args.speed, acc)
    if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
        t0 = time.monotonic()
        while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
            if time.monotonic() - t0 > timeout_s:
                robot.task_stop(rc)
                raise SystemExit(f"Move not finished in {timeout_s:.1f} s; task_stop sent.")
            time.sleep(0.01)
        print(f"Move took {time.monotonic() - t0:.1f} s (expected about {expected_s:.1f} s)")
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
    print(f"\nAfter joints   {q_after}")
    print(f"After - target {diff}")
    print(f"After TCP (tcp_ref) {np.array(sdata.tcp_ref, dtype=float)}")
    print(f"All joints within {TOL_DEG} deg of target: {'OK' if np.all(np.abs(diff) < TOL_DEG) else 'NO'} "
          f"(max {np.abs(diff).max():.4f})")


if __name__ == "__main__":
    main()

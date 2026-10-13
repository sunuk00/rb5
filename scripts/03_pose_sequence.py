"""Pose sequence demo: home -> pose_1 -> home using move_j.

Usage:
    python scripts/03_pose_sequence.py          # Simulation mode (default)
    python scripts/03_pose_sequence.py --real   # Real mode (asks for typed confirmation)
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rbpodo as rb

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config  # noqa: E402

# Named poses: joint angles in degrees, J1..J6 (base -> wrist3)
POSES = {
    "home":   [ 87, -2.5, -145, 137, 3.5, 14 ],
    "pose_1": [ 45, -2, -145, 137, 3, 14],
}
# Order of poses to visit
SEQUENCE = ["home", "pose_1", "home"]

HOME_TOLERANCE_DEG = 2.0   # every joint must be within this of home before starting
JOINT_SPEED = 20.0         # move_j speed (deg/s), scaled by the speed bar
JOINT_ACC = 20.0           # move_j acceleration (deg/s^2)
MOVE_TIMEOUT_S = 120.0     # give up waiting for a single move after this long


def validate_poses():
    for name, q in POSES.items():
        if len(q) != 6:
            raise SystemExit(f"Pose '{name}' has {len(q)} values, expected 6.")
    for name in SEQUENCE:
        if name not in POSES:
            raise SystemExit(f"Sequence uses unknown pose '{name}'.")


def read_joints(data_channel, simulation):
    # jnt_ang (measured) does not change in Simulation mode; use jnt_ref (commanded) there.
    sdata = data_channel.request_data().sdata
    return np.array(sdata.jnt_ref if simulation else sdata.jnt_ang, dtype=float)


def wait_until_stopped(robot, rc):
    """Poll get_robot_state() until the robot is no longer moving. Returns elapsed seconds."""
    t0 = time.monotonic()
    if robot.wait_for_move_started(rc, 0.5).type() != rb.ReturnType.Success:
        print("  warning: move start was not detected within 0.5 s")
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        if time.monotonic() - t0 > MOVE_TIMEOUT_S:
            raise SystemExit(f"Move did not finish within {MOVE_TIMEOUT_S} s.")
        time.sleep(0.01)
    return time.monotonic() - t0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", action="store_true", help="run on the real robot")
    args = parser.parse_args()
    simulation = not args.real
    mode_name = "Simulation" if simulation else "Real"

    validate_poses()
    np.set_printoptions(precision=3, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001)

    # Switch mode first so the near-home check reads the right field.
    robot.set_operation_mode(rc, rb.OperationMode.Simulation if simulation else rb.OperationMode.Real)
    rc.error().throw_if_not_empty()

    # 1. Near-home check
    q_now = read_joints(data_channel, simulation)
    q_home = np.array(POSES["home"])
    diff = q_now - q_home
    print(f"Mode: {mode_name}")
    print("Current:", q_now)
    print("Home:   ", q_home)
    print("Diff:   ", diff)
    if np.any(np.abs(diff) > HOME_TOLERANCE_DEG):
        raise SystemExit(f"Robot is not within {HOME_TOLERANCE_DEG} deg of home on every joint. Stopping.")

    # 2. Show the plan
    print("\nPlan (speed bar %.2f, %.1f deg/s, %.1f deg/s^2):" % (config.SPEED, JOINT_SPEED, JOINT_ACC))
    q_prev = q_now
    for i, name in enumerate(SEQUENCE, 1):
        q_target = np.array(POSES[name])
        print(f"  {i}. {name:<8} target {q_target}  delta {q_target - q_prev}")
        q_prev = q_target

    # 3. Typed confirmation in Real mode
    if not simulation:
        if input("\nREAL mode: the robot will move. Type 'yes' to continue: ") != "yes":
            raise SystemExit("Cancelled.")

    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()

    # 4. Run the sequence
    for i, name in enumerate(SEQUENCE, 1):
        q_target = np.array(POSES[name])
        print(f"\n[{i}/{len(SEQUENCE)}] move_j -> {name}")
        robot.flush(rc)
        robot.move_j(rc, q_target, JOINT_SPEED, JOINT_ACC)
        elapsed = wait_until_stopped(robot, rc)
        rc.error().throw_if_not_empty()

        q_after = read_joints(data_channel, simulation)
        print(f"  elapsed: {elapsed:.2f} s")
        print("  reached:", q_after)
        print("  error:  ", q_after - q_target)

    print("\nDone.")


if __name__ == "__main__":
    main()

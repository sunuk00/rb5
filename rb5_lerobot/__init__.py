"""Demonstrations -> LeRobot dataset -> ACT training -> policy runs on the RB5 (env lerobot, p2).

Run from the repository root:
    python -m rb5_lerobot.convert <sessions ...>     sessions (python -m teleop --record) -> dataset
    python -m rb5_lerobot.check <checkpoint>         a trained policy offline (no robot)
    python -m rb5_lerobot.run <checkpoint>           a trained policy on the RB5 (Simulation only)

Not named "lerobot": a folder of that name at the repository root would hide the installed LeRobot package.
Hardware code (cameras, gripper, robot link, follower) stays in teleop/ and is shared with the teleop.
"""

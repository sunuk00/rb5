"""Robot IP and speed bar for the scripts (import config), read from configs/robot.yaml (the teleop
reads the same file). Found because Python puts the script folder (scripts/) on the import path."""
from pathlib import Path

import yaml

with open(Path(__file__).resolve().parent.parent / "configs" / "robot.yaml") as _f:
    _robot = yaml.safe_load(_f)

ROBOT_IP = _robot["ip"]
SPEED = _robot["speed_bar"]     # speed bar

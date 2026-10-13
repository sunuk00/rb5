"""Shared settings for the scripts, read from configs/robot.yaml (the teleop reads the same file)."""
from pathlib import Path

import yaml

with open(Path(__file__).resolve().parent / "configs" / "robot.yaml") as _f:
    _robot = yaml.safe_load(_f)

ROBOT_IP = _robot["ip"]
SPEED = _robot["speed_bar"]     # speed bar

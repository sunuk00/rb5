"""Robot IP and speed bar for the scripts (import config), read from configs/robot.yaml (the teleop
reads the same file), and the USB-serial devices. Found because Python puts the script folder (scripts/)
on the import path."""
from pathlib import Path

import yaml

with open(Path(__file__).resolve().parent.parent / "configs" / "robot.yaml") as _f:
    _robot = yaml.safe_load(_f)

ROBOT_IP = _robot["ip"]
SPEED = _robot["speed_bar"]     # speed bar

# USB-serial devices by adapter serial number (/dev/serial/by-id): /dev/ttyUSB0 / 1 depend on the plug
# order once two FTDI adapters are connected. The teleop uses the same leader path (configs/*.yaml).
LEADER_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT62AJW7-if00-port0"   # FT62AJW7
GRIPPER_PORT = None   # U2D2 of the gripper: set when its serial is known (grip-1)

"""RB-Y1 leader arm (right side): read-only joint and tool access, URDF FK, clutch and gripper signal.

Moved from scripts/21_leader_rb5_position.py. Only ping and read instructions are sent to the leader
arm. Protocol, addresses and conversions follow rby1-sdk (dynamixel_bus.h / .cpp, leader_arm.cpp):
2 Mbps, Protocol 2.0, present position at 132, rad = raw / 4096 * 2 pi, tool raw value at 132 with
button = bits 8-15 and trigger = bits 16-31.
"""
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .kinematics import axis_rotation

PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
JOINT_IDS = list(range(7))
JOINT_NAMES = ["J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R",
               "J4_Wrist_Yaw1_R", "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R"]
TOOL_ID = 0x80             # right tool (handle)
TOOL_ID_LEFT = 0x81        # left tool: its button switches the vertical mode (mapping.vertical_button)
TOOL_READ_PAUSE_S = 100e-6 # rby1-sdk ReadButtonStatus sleeps 100 us after the read
TICKS_PER_REV = 4096
BUTTON_PRESSED = 1         # raw 0x00000100 when held (13)
BUTTON_RELEASED = 0


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def read_latency_timer(port_name):
    """FTDI latency_timer in ms from sysfs, or None if it cannot be read."""
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


class LeaderArm:
    """Read-only access to the right arm joints and the right tool. Only ping and read are used."""

    def __init__(self, port_name):
        from dynamixel_sdk import PacketHandler, PortHandler   # imported here so tests need no hardware
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        self.sync = None

    def open(self, left_tool=False):
        """Open the port and ping every joint and the right tool (and the left tool if left_tool).
        Returns a list of problems (empty = OK)."""
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead
        if not os.path.exists(self.port.getPortName()):
            return [f"port {self.port.getPortName()} does not exist (USB cable? udev link? see leader.port in the settings)"]
        try:
            opened = self.port.openPort()
        except Exception as e:   # pyserial raises when the device cannot be opened
            return [f"failed to open {self.port.getPortName()}: {e}"]
        if not opened:
            return [f"failed to open {self.port.getPortName()}"]
        if not self.port.setBaudRate(BAUDRATE):
            return [f"failed to set baud rate {BAUDRATE}"]
        ids = JOINT_IDS + [TOOL_ID] + ([TOOL_ID_LEFT] if left_tool else [])
        missing = [i for i in ids if self.ph.ping(self.port, i)[1] != COMM_SUCCESS]
        if missing:
            return [f"leader arm IDs {missing} did not answer the ping"]
        self.sync = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for i in JOINT_IDS:
            self.sync.addParam(i)
        return []

    def read_joints_rad(self):
        """Return the 7 joint angles [rad], or None if the read failed."""
        from dynamixel_sdk import COMM_SUCCESS
        if self.sync.txRxPacket() != COMM_SUCCESS or not all(
                self.sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in JOINT_IDS):
            return None
        raw = [to_signed32(self.sync.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) for i in JOINT_IDS]
        return np.array(raw, dtype=float) / TICKS_PER_REV * 2 * np.pi

    def read_button_raw(self):
        """Return the tool's raw 32-bit value, or None if the read failed."""
        from dynamixel_sdk import COMM_SUCCESS
        value, result, _ = self.ph.read4ByteTxRx(self.port, TOOL_ID, ADDR_PRESENT_POSITION)
        time.sleep(TOOL_READ_PAUSE_S)
        return value if result == COMM_SUCCESS else None

    def read_left_button_raw(self):
        """Return the left tool's raw 32-bit value (button = bits 8-15), or None if the read failed."""
        from dynamixel_sdk import COMM_SUCCESS
        value, result, _ = self.ph.read4ByteTxRx(self.port, TOOL_ID_LEFT, ADDR_PRESENT_POSITION)
        time.sleep(TOOL_READ_PAUSE_S)
        return value if result == COMM_SUCCESS else None

    def close(self):
        self.port.closePort()


class LeaderModel:
    """Right arm wrist center (Link_6R origin) from the leader URDF; all joint origins have rpy 0 (checked).
    Same result as rby1_sdk.dynamics (20: max difference 6e-8)."""

    def __init__(self, urdf_path):
        joints = {j.get("name"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
        self.chain = []
        for name in JOINT_NAMES:
            j = joints[name]
            origin = j.find("origin")
            if any(float(v) != 0.0 for v in origin.get("rpy", "0 0 0").split()):
                raise ValueError(f"{name}: origin rpy is not zero; this FK does not handle it.")
            self.chain.append((np.array([float(v) for v in origin.get("xyz").split()]),
                               np.array([float(v) for v in j.find("axis").get("xyz").split()])))

    def wrist_center_mm(self, q_rad):
        r, p = np.eye(3), np.zeros(3)
        for (xyz, axis), angle in zip(self.chain, q_rad):
            p = p + r @ xyz
            r = r @ axis_rotation(axis, angle)
        return p * 1000.0


class LeaderFilter:
    """Hysteresis on the leader readings: a value is taken only when it moved at least the deadband
    from the last taken value, otherwise the last taken value is kept. This removes the encoder
    flicker (one tick = 0.088 deg back and forth, run 141108) without adding a delay to real motion.

    The wrist center then goes through a first-order low-pass filter with time constant tau_s
    (0 = off): one encoder tick moves the wrist center by up to about 0.5 mm, so during motion the step
    per cycle jumps and shook the RB5 at 200 mm/s (run 20261016_021514). Adds a delay of about tau_s."""

    def __init__(self, deadband_mm, deadband_deg, tau_s=0.0, period_s=0.02):
        self.deadband_mm, self.deadband_deg = deadband_mm, deadband_deg
        self.alpha = period_s / (tau_s + period_s) if tau_s > 0 else 1.0
        self.p = self.j6 = self.smooth = None

    def update(self, p_mm, j6_deg):
        """Return (wrist center [mm] after the hysteresis and the low-pass, J6 [deg] after the hysteresis)."""
        p_mm = np.asarray(p_mm, dtype=float)
        if self.p is None or np.linalg.norm(p_mm - self.p) >= self.deadband_mm:
            self.p = p_mm.copy()
        self.smooth = self.p.copy() if self.smooth is None else self.smooth + self.alpha * (self.p - self.smooth)
        d = (j6_deg - self.j6 + 180.0) % 360.0 - 180.0 if self.j6 is not None else None
        if d is None or abs(d) >= self.deadband_deg:
            self.j6 = float(j6_deg)
        return self.smooth.copy(), self.j6


class Clutch:
    """Debounced clutch from the tool button (13): a press counts after `press_reads` pressed reads,
    `fails_to_release` failed reads in a row count as released (motion stops). Starts released, and a
    button held at start must be released once before a press is accepted."""

    def __init__(self, press_reads=2, fails_to_release=3):
        self.press_reads, self.fails_to_release = press_reads, fails_to_release
        self.engaged = False
        self.armed = False
        self.press_count = 0
        self.fail_count = 0
        self.reads = 0
        self.failures = 0

    def update(self, raw):
        """Feed one tool read (raw value or None). Returns (button or None, event or None)."""
        self.reads += 1
        button = None if raw is None else (raw >> 8) & 0xFF
        if button == BUTTON_PRESSED:
            self.fail_count = 0
            if self.armed:
                self.press_count += 1
                if not self.engaged and self.press_count >= self.press_reads:
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
        self.failures += 1
        self.fail_count += 1
        self.press_count = 0
        if self.engaged and self.fail_count >= self.fails_to_release:
            self.engaged = False
            return button, f"released ({self.fails_to_release} failed button reads in a row)"
        return button, None


class GripSignal:
    """Gripper closing from the tool trigger. The trigger reads 0 when pulled closed and trigger_full
    (1000) when fully open / released (user, 2026-10-12), so grip_close = 1 - trigger / trigger_full:
    pulled = closed, released = open. State closed above close_above, open below open_below, kept in
    between. A failed read keeps the previous value (no drop), unlike the clutch. Independent of the
    clutch."""

    def __init__(self, g):
        self.g = g
        self.trigger = None        # last good raw trigger value
        self.close = None          # 0 = open .. 1 = closed
        self.closed = None         # True closed / False open (hysteresis)
        self.fail_count = 0

    def update(self, raw):
        """Feed one tool read (raw value or None). Returns an event string or None."""
        if raw is None:
            self.fail_count += 1
            if self.fail_count == self.g.fail_warn:
                return f"gripper: {self.g.fail_warn} failed tool reads in a row, keeping the last value"
            return None
        self.fail_count = 0
        self.trigger = (raw >> 16) & 0xFFFF
        self.close = 1.0 - min(max(self.trigger / self.g.trigger_full, 0.0), 1.0)
        previous = self.closed
        if self.closed is None:
            self.closed = self.close >= 0.5
        elif self.close > self.g.close_above:
            self.closed = True
        elif self.close < self.g.open_below:
            self.closed = False
        if self.closed != previous:
            return f"gripper {'CLOSED' if self.closed else 'OPEN'} (trigger {self.trigger})"
        return None

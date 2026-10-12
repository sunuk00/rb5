"""Leader arm wrist center -> RB5 TCP position with a clutch, move_servo_l, Simulation mode only.

Each 20 ms cycle:
  1. read leader right arm joints ID 0..6 (Sync Read) and the right tool (ID 128) button; read-only
  2. clutch (same debounce as 13): press = 2 pressed reads, 3 failed reads in a row = released
  3. while engaged: desired = RB5 start + scale * Rz(yaw) * (leader now - leader start)   [mm]
     leader position = Link_6R origin (wrist center) from the leader URDF (numpy chain, same result
     as rby1_sdk in 20). The orientation stays at the TCP orientation at script start.
  4. limits, in this order:
     - speed: at most MAX_STEP_MM per cycle, scaled down near the wrist singularity (below)
     - joint speed: the DH model predicts the joint motion dq = J^-1 dx; if max |dq| is over
       MAX_DQ_DEG per cycle for that joint (wrist J4-J6 2, others 1 deg/cycle) the step is scaled down
     - the cut part of the step is dropped (B: the leader start is shifted), so the robot stops soon
       after the hand stops; with --max-lag-mm (C) only the lag beyond that length is dropped
     - range: a cylinder piece about the base z axis (not dropped): radius CYL_R_MIN..CYL_R_MAX,
       z = start z +/- CYL_Z_HALF, azimuth = start azimuth +/- CYL_AZ_HALF_DEG. A target outside
       is clamped in r, z and azimuth separately.
     - lead: the target is not moved while it is more than MAX_LEAD_MM ahead of tcp_ref (not dropped)
  5. singularity watch on the predicted joint angles (DH model):
     - wrist: effective J5 = asin(|sin J5|). Below 20 deg: warning and the speed limit falls linearly
       to 0.25x at 10 deg. Below 10 deg: stop.
     - shoulder: margin = horizontal radius of the wrist point P05 - d4 (J1 has no unique solution
       at margin 0; Rainbow IK th1 = atan2(...) - acos(d4 / r)). Below 50 mm: warning and slower
       (0.25x at 20 mm). Below 20 mm: stop.
     - elbow: J3 distance from 0 / 180 deg (arm stretched). Below 20 deg: warning and slower (0.25x at
       10 deg). Below 10 deg: stop.
     - joint limits: margin = limit - |joint| (J3 +/-150, others +/-360, as in 19). Below 20 deg:
       warning and slower (0.25x at 10 deg). Below 10 deg: stop.
  C (catch-up up to a lag): with --max-lag-mm L > 0 the part cut by the speed / joint / singularity
     limits is kept and caught up later while the lag (desired - target) is at most L; only the part
     beyond L is dropped. L = 0 is B (drop all of the cut part). Release drops the lag.
  yaw: RB5 yaw (rotation about base z, TCP fixed) = start + (leader J6 now - leader J6 at press);
     the tool stays pointing down (rx, ry fixed). 90 deg/s, +/-90 deg around the start rz, C lag 15 deg
     (B with --max-lag-mm 0). The joint predictions include the yaw part. --no-yaw turns it off.
  6. send move_servo_l(target x, y, z, rx, ry, start rz + yaw) every cycle (released clutch = frozen target).
     Just before sending, the gap since the previous send is checked; over MAX_GAP_S the cycle is
     "late" and the target is frozen for that cycle (the previous check measured only up to the
     start of the next cycle and missed 40-50 ms gaps after an overrun).
  Every cycle logs the time of the leader read, the planning, the send call and the real send interval.
  gripper signal (logged only, no gripper output yet): trigger = bits 16-31 of the tool raw value,
     continuous 0 (fully closed) .. 1000 (fully open); grip_open = trigger / 1000; state open above
     0.6, closed below 0.4, kept in between. Independent of the clutch. A failed tool read keeps the
     previous gripper value (no drop), while the clutch releases after 3 failed reads (motion stops).

Coordinate alignment: leader Base and RB5 base are taken as the same directions (x forward, y left,
z up for the leader, confirmed in 20), rotated about z by --yaw-deg (default 0).

Usage:
    python scripts/21_leader_rb5_position.py
    python scripts/21_leader_rb5_position.py --yaw-deg 90 --scale 1.0
    python scripts/21_leader_rb5_position.py --speed-mm-s 200
    python scripts/21_leader_rb5_position.py --speed-mm-s 200 --max-lag-mm 0     # B
    python scripts/21_leader_rb5_position.py --speed-mm-s 200 --scale 0.5
Stop with Ctrl+C. Each run is logged to data/21_leader_pos_sim_<time>.csv
"""
import argparse
import csv
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

# --- RB5 servo streaming (same values as 13 / 18)
PERIOD_S = 0.02
SERVO_T1 = PERIOD_S        # matches the send period (06 / 10)
SERVO_T2 = 0.1
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0

# --- Limits
MAX_SPEED_MM_S = 200.0     # largest allowed --speed-mm-s (Simulation)
MAX_STEP_MM = 1.0          # per cycle (50 mm/s); set from --speed-mm-s in main()
# Predicted joint motion per cycle, per joint (Simulation): J1-J3 1 deg (50 deg/s), wrist J4-J6 2 deg
# (100 deg/s). Real values to be set later from the RB5 specification (decisions 2026-10-12).
MAX_DQ_DEG = np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])
CYL_R_MIN = 450.0          # TCP horizontal radius from the base z axis [mm]; J3 limit at low z inside
CYL_R_MAX = 550.0          # ready pose at r 500 (object on the table, z 0); see decisions 2026-10-12
CYL_Z_HALF = 150.0         # z range around the TCP z at script start [mm] (Real: set from table + gripper)
CYL_AZ_HALF_DEG = 90.0     # azimuth range around the TCP azimuth at script start [deg]
MAX_LEAD_MM = 2.0          # target may be at most this far ahead of tcp_ref; set in main()
LEAD_PER_STEP = 2.0        # lead limit = this x MAX_STEP_MM (2 mm at 1 mm/cycle as first tested)
J5_SLOW_DEG = 20.0         # effective J5 below this: warning and slower
J5_STOP_DEG = 10.0         # effective J5 below this: stop
J5_MIN_SCALE = 0.25        # speed scale at J5_STOP_DEG (also used at SHOULDER_STOP_MM)
SHOULDER_SLOW_MM = 50.0    # shoulder margin (P05 radius - d4) below this: warning and slower
SHOULDER_STOP_MM = 20.0    # shoulder margin below this: stop
ELBOW_SLOW_DEG = 20.0      # J3 distance from 0 / 180 deg below this: warning and slower
ELBOW_STOP_DEG = 10.0      # below this: stop
JOINT_LIMIT_DEG = np.array([360.0, 360.0, 150.0, 360.0, 360.0, 360.0])   # J3 +/-150 until checked (19)
JOINT_LIMIT_SLOW_DEG = 20.0   # margin to the joint limit below this: warning and slower
JOINT_LIMIT_STOP_DEG = 10.0   # below this: stop
MAX_LAG_MM = 30.0          # C: lag kept for catch-up [mm]; 0 = B. Set from --max-lag-mm in main() (default 30)
MAX_LAG_LIMIT_MM = 50.0    # largest allowed --max-lag-mm

# --- Yaw (rotation about base z): RB5 yaw change = leader J6 change (decisions 2026-10-12, method B)
YAW_SPEED_DEG_S = 90.0
MAX_YAW_STEP_DEG = YAW_SPEED_DEG_S * PERIOD_S   # 1.8 deg per cycle
YAW_RANGE_DEG = 90.0       # around the TCP rz at script start
MAX_YAW_LEAD_DEG = 2.0 * MAX_YAW_STEP_DEG       # same rule as the position lead (2 x step)
MAX_YAW_LAG_DEG = 15.0     # C: yaw lag kept for catch-up (0 when --max-lag-mm 0, i.e. B)
MAX_FK_ERROR_MM = 5.0      # DH model vs tcp_ref at start
MAX_GAP_S = 1.5 * SERVO_T1 # a send gap longer than this freezes the target for that cycle (08)
MAX_LATE_CYCLES = 3
MAX_STATE_AGE_S = 0.1
MAX_STALE_CYCLES = 3
MAX_LEADER_FAILS = 3       # leader joint read failures in a row -> stop
HOLD_ON_EXIT_S = 0.3

# --- Leader arm (rby1-sdk: dynamixel_bus.h, leader_arm.cpp)
LEADER_PORT = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
ADDR_PRESENT_POSITION = 132  # the tool returns its button state at the same address
LEN_PRESENT_POSITION = 4
LEADER_IDS = list(range(7))
LEADER_JOINTS = ["J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R",
                 "J4_Wrist_Yaw1_R", "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R"]
TOOL_ID = 0x80             # right tool
TOOL_READ_PAUSE_S = 100e-6 # rby1-sdk ReadButtonStatus sleeps 100 us after the read
TICKS_PER_REV = 4096
URDF_PATH = ROOT / "refs" / "rby1-sdk" / "models" / "leader_arm" / "model.urdf"

# --- Gripper signal from the tool trigger (decisions 2026-10-12)
TRIGGER_FULL = 1000.0      # trigger 0 = fully closed, 1000 = fully open (user measurement)
GRIP_OPEN_ABOVE = 0.6      # grip_open above this: state open
GRIP_CLOSE_BELOW = 0.4     # grip_open below this: state closed; in between the state is kept
GRIP_FAIL_WARN = 3         # warn after this many failed tool reads in a row (value is kept)

# --- Clutch (13)
BUTTON_PRESSED = 1
BUTTON_RELEASED = 0
PRESS_CONFIRM_READS = 2
FAILS_TO_RELEASE = 3

# --- RB5-850 standard DH (rb_cobot_docs technical_docs/dh_parameters), mm / deg
D1, D2, D3, D4, D5, D6, A1, A2 = 169.2, 148.4, 148.4, 110.7, 110.7, 96.7, 425.0, 392.0

STATUS_EVERY = 25
DATA_DIR = ROOT / "data"
CLEAR_LINE = "\r\x1b[2K"


class StopRequest(Exception):
    """Controlled stop: end servo mode normally (hold target, move_speed_j(0), poll)."""


# ---------------------------------------------------------------- math
def rot_zyx(rx, ry, rz):
    """Rotation matrix of a ZY'X'' Euler pose (deg): R = Rz(rz) Ry(ry) Rx(rx)."""
    a, b, c = np.radians([rx, ry, rz])
    rx_m = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    ry_m = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    rz_m = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def rot_z(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def axis_rotation(axis, angle):
    """Rodrigues formula: rotation by angle [rad] about a (normalized) axis."""
    k = np.asarray(axis, dtype=float)
    k /= np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def clip_norm(v, limit):
    n = np.linalg.norm(v)
    return v if n <= limit else v * (limit / n)


def wrap_deg(a):
    """Angle wrapped to [-180, 180) deg."""
    return (a + 180.0) % 360.0 - 180.0


def singular_distance(angle_deg):
    """Distance of a joint angle from 0 and +/-180 deg (J5: wrist singularity, J3: elbow straight)."""
    return np.degrees(np.arcsin(min(abs(np.sin(np.radians(angle_deg))), 1.0)))


def ramp_scale(value, slow, stop):
    """1.0 at or above slow, falling linearly to J5_MIN_SCALE at stop (and below)."""
    if value >= slow:
        return 1.0
    if value <= stop:
        return J5_MIN_SCALE
    return J5_MIN_SCALE + (1.0 - J5_MIN_SCALE) * (value - stop) / (slow - stop)


def joint_limit_margin(q):
    """Smallest margin to the joint limits [deg] and the joint number (1-6) that has it."""
    margin = JOINT_LIMIT_DEG - np.abs(q)
    i = int(np.argmin(margin))
    return margin[i], i + 1


def j5_speed_scale(j5_eff):
    return ramp_scale(j5_eff, J5_SLOW_DEG, J5_STOP_DEG)


class LeaderModel:
    """Right arm wrist center (Link_6R origin) from the leader URDF; all joint origins have rpy 0 (checked)."""

    def __init__(self, urdf_path):
        joints = {j.get("name"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
        self.chain = []
        for name in LEADER_JOINTS:
            j = joints[name]
            origin = j.find("origin")
            if any(float(v) != 0.0 for v in origin.get("rpy", "0 0 0").split()):
                raise SystemExit(f"{name}: origin rpy is not zero; this FK does not handle it.")
            self.chain.append((np.array([float(v) for v in origin.get("xyz").split()]),
                               np.array([float(v) for v in j.find("axis").get("xyz").split()])))

    def wrist_center_mm(self, q_rad):
        r, p = np.eye(3), np.zeros(3)
        for (xyz, axis), angle in zip(self.chain, q_rad):
            p = p + r @ xyz
            r = r @ axis_rotation(axis, angle)
        return p * 1000.0


class RB5Model:
    """RB5-850 standard DH model plus the TCP offset (flange frame, mm). Validated against 17 / 18."""

    def __init__(self, tcp_offset_mm):
        self.offset = np.asarray(tcp_offset_mm, dtype=float)

    @staticmethod
    def _dh(theta, d, a, alpha):
        th, al = np.radians(theta), np.radians(alpha)
        ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                         [0, sa, ca, d], [0, 0, 0, 1]])

    def flange(self, q):
        t1, t2, t3, t4, t5, t6 = q
        t = np.eye(4)
        for row in ((t1, D1, 0, -90), (t2 - 90, -D2, 0, 0), (0, 0, A1, 0), (t3, D3, 0, 0), (0, 0, A2, 0),
                    (t4 + 90, -D4, 0, 0), (0, 0, 0, 90), (t5, D5, 0, -90), (t6, -D6, 0, 90)):
            t = t @ self._dh(*row)
        return t

    def shoulder_margin(self, q):
        """Horizontal radius of the wrist point P05 minus d4 [mm] (P05 = flange + d6 * flange Y, Rainbow IK)."""
        t = self.flange(q)
        p05 = t[:3, 3] + D6 * t[:3, 1]
        return np.hypot(p05[0], p05[1]) - D4

    def tcp(self, q):
        t = self.flange(q)
        return t[:3, 3] + t[:3, :3] @ self.offset, t[:3, :3]

    def jacobian(self, q, h_deg=1e-4):
        """6x6: rows = TCP velocity (mm) and angular velocity, per rad of each joint (finite differences)."""
        p0, r0 = self.tcp(q)
        jac = np.zeros((6, 6))
        for i in range(6):
            dq = np.array(q, dtype=float)
            dq[i] += h_deg
            p, r = self.tcp(dq)
            dr = r @ r0.T
            jac[:3, i] = (p - p0) / np.radians(h_deg)
            jac[3:, i] = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2 / np.radians(h_deg)
        return jac

    def predict(self, q, dp_mm, jac=None, dyaw_deg=0.0):
        """Joint motion [deg] for a TCP position change dp and a rotation dyaw about base z (TCP fixed),
        from the current q. Pass jac = jacobian(q) to reuse it (one Jacobian per cycle)."""
        return np.degrees(np.linalg.solve(self.jacobian(q) if jac is None else jac,
                                          np.r_[dp_mm, 0, 0, np.radians(dyaw_deg)]))


class CylinderRange:
    """Cylinder piece about the base z axis, fixed at script start."""

    def __init__(self, tcp0):
        self.z0 = tcp0[2]
        self.az0 = np.degrees(np.arctan2(tcp0[1], tcp0[0]))

    def clamp(self, p):
        """Return the point clamped into the range (r, z and azimuth separately)."""
        r = np.hypot(p[0], p[1])
        daz = (np.degrees(np.arctan2(p[1], p[0])) - self.az0 + 180.0) % 360.0 - 180.0
        r_c = np.clip(r, CYL_R_MIN, CYL_R_MAX)
        az_c = np.radians(self.az0 + np.clip(daz, -CYL_AZ_HALF_DEG, CYL_AZ_HALF_DEG))
        z_c = np.clip(p[2], self.z0 - CYL_Z_HALF, self.z0 + CYL_Z_HALF)
        out = np.array([r_c * np.cos(az_c), r_c * np.sin(az_c), z_c])
        return out if np.linalg.norm(out - p) > 1e-9 else np.array(p, dtype=float)

    def contains(self, p):
        return np.linalg.norm(self.clamp(p) - p) < 1e-6


def plan_step(target, desired, tcp_pos, q, model, cyl, yaw):
    """One cycle of the limits for the position and the yaw.

    yaw: dict with "target", "desired", "tcp" = yaw about base z relative to the start rz [deg]
    (None or a dict with equal values when yaw mapping is off).
    Returns a SimpleNamespace: target (new position), yaw (new yaw), dropped (position vector),
    dropped_yaw, flags, margins (J5, shoulder, elbow, joint limit), max_dq, lag, lag_yaw, stop.
    """
    flags = dict.fromkeys(("speed", "j5slow", "shslow", "elslow", "jlslow", "jspeed", "range", "lead",
                           "yspeed", "yrange", "ylead"), False)
    # Range first: the part of desired outside the range is kept (not dropped). Clamping only the new
    # target would let desired - target grow outside the range, and the speed limit would then drop it.
    desired_in = cyl.clamp(desired)
    flags["range"] = bool(np.any(desired_in != desired))
    ydes_in = float(np.clip(yaw["desired"], -YAW_RANGE_DEG, YAW_RANGE_DEG))
    flags["yrange"] = ydes_in != yaw["desired"]
    step = desired_in - target
    ystep = ydes_in - yaw["target"]
    jac = model.jacobian(q)   # all predictions below are from the same q: compute it once
    lead_p, lead_y = target - tcp_pos, yaw["target"] - yaw["tcp"]   # parts that are only the robot catching up

    # Speed limits, scaled by the predicted singularity / joint limit margins of a full-size step
    q_full = q + model.predict(q, lead_p + clip_norm(step, MAX_STEP_MM), jac,
                               lead_y + np.clip(ystep, -MAX_YAW_STEP_DEG, MAX_YAW_STEP_DEG))
    j5_scale = j5_speed_scale(singular_distance(q_full[4]))
    sh_scale = ramp_scale(model.shoulder_margin(q_full), SHOULDER_SLOW_MM, SHOULDER_STOP_MM)
    el_scale = ramp_scale(singular_distance(q_full[2]), ELBOW_SLOW_DEG, ELBOW_STOP_DEG)
    jl_scale = ramp_scale(joint_limit_margin(q_full)[0], JOINT_LIMIT_SLOW_DEG, JOINT_LIMIT_STOP_DEG)
    flags["j5slow"] = j5_scale < 1.0
    flags["shslow"] = sh_scale < 1.0
    flags["elslow"] = el_scale < 1.0
    flags["jlslow"] = jl_scale < 1.0
    scale = min(j5_scale, sh_scale, el_scale, jl_scale)
    limited = clip_norm(step, MAX_STEP_MM * scale)
    flags["speed"] = np.linalg.norm(limited) < np.linalg.norm(step) - 1e-9
    ylimited = float(np.clip(ystep, -MAX_YAW_STEP_DEG * scale, MAX_YAW_STEP_DEG * scale))
    flags["yspeed"] = abs(ylimited) < abs(ystep) - 1e-9

    # Joint speed limit on the combined motion (nearly linear in a step this small); both parts are
    # scaled by the same factor
    dq_step = np.abs(model.predict(q, lead_p + limited, jac, lead_y + ylimited))
    dq_lead = np.abs(model.predict(q, lead_p, jac, lead_y))
    over = (dq_step > MAX_DQ_DEG) & (dq_step > dq_lead)
    if np.any(over):
        f = max(0.0, np.min((MAX_DQ_DEG[over] - dq_lead[over]) / (dq_step[over] - dq_lead[over])))
        limited, ylimited = limited * f, ylimited * f
        flags["jspeed"] = True
    new = target + limited
    clamped = cyl.clamp(new)   # a straight step between two inside points can cut the inner radius
    flags["range"] = flags["range"] or bool(np.any(clamped != new))
    if np.linalg.norm(clamped - tcp_pos) > MAX_LEAD_MM:
        clamped = target.copy()
        flags["lead"] = True
    ynew = float(np.clip(yaw["target"] + ylimited, -YAW_RANGE_DEG, YAW_RANGE_DEG))
    if abs(ynew - yaw["tcp"]) > MAX_YAW_LEAD_DEG:
        ynew = yaw["target"]
        flags["ylead"] = True

    # What is dropped (the leader start is shifted by it): B drops all of the cut part of this step;
    # C keeps the lag up to MAX_LAG_MM / MAX_YAW_LAG_DEG for catch-up and drops only the part beyond it.
    if MAX_LAG_MM <= 0.0:
        dropped, ydropped = step - limited, ystep - ylimited
    else:
        lag = desired_in - clamped
        n = np.linalg.norm(lag)
        dropped = lag * (1.0 - MAX_LAG_MM / n) if n > MAX_LAG_MM else np.zeros(3)
        ylag = ydes_in - ynew
        ydropped = float(np.sign(ylag) * (abs(ylag) - MAX_YAW_LAG_DEG)) if abs(ylag) > MAX_YAW_LAG_DEG else 0.0
    lag_after = np.linalg.norm(desired_in - dropped - clamped)
    ylag_after = abs(ydes_in - ydropped - ynew)

    dq = model.predict(q, clamped - tcp_pos, jac, ynew - yaw["tcp"])
    j5_final = singular_distance(q[4] + dq[4])
    sh_final = model.shoulder_margin(q + dq)
    el_final = singular_distance(q[2] + dq[2])
    jl_final, jl_joint = joint_limit_margin(q + dq)
    stop = None
    if not (np.array_equal(clamped, target) and ynew == yaw["target"]):
        if j5_final < J5_STOP_DEG:
            stop = f"predicted J5 {j5_final:.1f} deg (from the singular angle) < {J5_STOP_DEG} deg"
        elif sh_final < SHOULDER_STOP_MM:
            stop = f"predicted shoulder margin {sh_final:.1f} mm < {SHOULDER_STOP_MM} mm"
        elif el_final < ELBOW_STOP_DEG:
            stop = f"predicted J3 {el_final:.1f} deg from straight < {ELBOW_STOP_DEG} deg"
        elif jl_final < JOINT_LIMIT_STOP_DEG:
            stop = f"predicted J{jl_joint} {jl_final:.1f} deg from its limit < {JOINT_LIMIT_STOP_DEG} deg"
    return SimpleNamespace(target=clamped, yaw=ynew, dropped=dropped, dropped_yaw=ydropped, flags=flags,
                           margins=(j5_final, sh_final, el_final, jl_final), max_dq=np.abs(dq).max(),
                           lag=lag_after, lag_yaw=ylag_after, stop=stop)


# ---------------------------------------------------------------- devices
def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def read_latency_timer(port_name):
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


class LeaderArm:
    """Read-only access to the right arm joints and the right tool button. Only ping and read are used."""

    def __init__(self, port_name):
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        self.sync = None

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"Failed to open {self.port.getPortName()}")
        if not self.port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")
        missing = [i for i in LEADER_IDS + [TOOL_ID] if self.ph.ping(self.port, i)[1] != COMM_SUCCESS]
        if missing:
            raise SystemExit(f"Leader arm IDs {missing} did not answer the ping.")
        self.sync = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for i in LEADER_IDS:
            self.sync.addParam(i)

    def read_joints_rad(self):
        """Return the 7 joint angles [rad], or None if the read failed."""
        if self.sync.txRxPacket() != COMM_SUCCESS or not all(
                self.sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in LEADER_IDS):
            return None
        raw = [to_signed32(self.sync.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) for i in LEADER_IDS]
        return np.array(raw, dtype=float) / TICKS_PER_REV * 2 * np.pi

    def read_button_raw(self):
        value, result, _ = self.ph.read4ByteTxRx(self.port, TOOL_ID, ADDR_PRESENT_POSITION)
        time.sleep(TOOL_READ_PAUSE_S)
        return value if result == COMM_SUCCESS else None

    def close(self):
        self.port.closePort()


class Clutch:
    """Debounced clutch from the tool button (13)."""

    def __init__(self):
        self.engaged = False       # always start released
        self.armed = False         # a press is accepted only after one release has been seen
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
                if not self.engaged and self.press_count >= PRESS_CONFIRM_READS:
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
        if self.engaged and self.fail_count >= FAILS_TO_RELEASE:
            self.engaged = False
            return button, f"released ({FAILS_TO_RELEASE} failed button reads in a row)"
        return button, None


class GripSignal:
    """Gripper opening from the tool trigger. A failed read keeps the previous value."""

    def __init__(self):
        self.trigger = None        # last good raw trigger value
        self.open = None           # 0 = closed .. 1 = open
        self.state_open = None     # True open / False closed (hysteresis)
        self.fail_count = 0

    def update(self, raw):
        """Feed one tool read (raw value or None). Returns an event string or None."""
        if raw is None:
            self.fail_count += 1
            if self.fail_count == GRIP_FAIL_WARN:
                return f"gripper: {GRIP_FAIL_WARN} failed tool reads in a row, keeping the last value"
            return None
        self.fail_count = 0
        self.trigger = (raw >> 16) & 0xFFFF
        self.open = min(max(self.trigger / TRIGGER_FULL, 0.0), 1.0)
        previous = self.state_open
        if self.state_open is None:
            self.state_open = self.open >= 0.5
        elif self.open > GRIP_OPEN_ABOVE:
            self.state_open = True
        elif self.open < GRIP_CLOSE_BELOW:
            self.state_open = False
        if self.state_open != previous:
            return f"gripper {'OPEN' if self.state_open else 'CLOSED'} (trigger {self.trigger})"
        return None


class StateReader(threading.Thread):
    """Reads the RB5 data channel continuously and keeps the latest tcp_ref / jnt_ref (08 / 18)."""

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
            # Simulation: jnt_ang does not follow the motion, so use the reference values
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


def send_servo(robot, rc, target, rpy):
    robot.move_servo_l(rc, np.r_[target, rpy], SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)


def stop_streaming(robot, rc, target, rpy):
    """Normal end of servo mode: hold the target, then move_speed_j(0) and poll (13)."""
    next_t = time.monotonic()
    t_end = next_t + HOLD_ON_EXIT_S
    while time.monotonic() < t_end:
        send_servo(robot, rc, target, rpy)
        next_t += PERIOD_S
        time.sleep(max(0.0, next_t - time.monotonic()))
    robot.move_speed_j(rc, np.zeros(6), SERVO_T1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
    robot.enable_waiting_ack(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def emergency_stop(robot, rc):
    """End servo mode after an error: no more servo commands, task_stop and poll (13)."""
    robot.enable_waiting_ack(rc)
    robot.task_stop(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def event(msg):
    sys.stdout.write(CLEAR_LINE + msg + "\n")
    sys.stdout.flush()


# ---------------------------------------------------------------- loop
def run_loop(leader, leader_model, robot, rc, reader, rb5_model, args, ctx):
    """20 ms loop. ctx["target"] holds the TCP position target [mm], ctx["rows"] the CSV rows."""
    tcp0, _, _ = reader.latest()
    ctx["target"] = tcp0[:3].copy()
    target = ctx["target"]
    rpy0 = ctx["rpy"].copy()
    rpy = ctx["rpy"]
    yaw_target = 0.0          # RB5 yaw relative to the start rz [deg]
    cyl = CylinderRange(tcp0[:3])
    yaw = rot_z(args.yaw_deg)

    clutch = Clutch()
    ctx["clutch"] = clutch
    grip = GripSignal()
    leader_q = leader_p = None
    leader_fails = leader_fail_total = leader_reads = 0
    rb5_start = leader_start = None
    yaw_rb5_start = yaw_leader_start = None
    flags_prev = {}
    last_send = prev_cycle_t = None
    late_cycles = stale_cycles = 0
    warned = {"j5slow": False, "shslow": False, "elslow": False, "jlslow": False}

    t_start = next_t = time.monotonic()
    cycle = 0
    while True:
        t_cycle = time.monotonic()
        interval = None if prev_cycle_t is None else t_cycle - prev_cycle_t
        prev_cycle_t = t_cycle

        # 1. Leader arm joints -> wrist center, then the button
        t_leader0 = time.monotonic()
        leader_reads += 1
        q_l = leader.read_joints_rad()
        if q_l is None:
            leader_fails += 1
            leader_fail_total += 1
            if leader_fails >= MAX_LEADER_FAILS:
                raise StopRequest(f"{MAX_LEADER_FAILS} leader joint read failures in a row")
        else:
            leader_fails = 0
            leader_q, leader_p = q_l, leader_model.wrist_center_mm(q_l)
        button_raw = leader.read_button_raw()
        t_leader = time.monotonic() - t_leader0
        grip_event = grip.update(button_raw)   # independent of the clutch
        if grip_event:
            event(f"[{time.monotonic() - t_start:7.2f} s] {grip_event}")

        # 2. Clutch
        button, clutch_event = clutch.update(button_raw)
        if clutch_event == "engaged":
            if leader_p is None:
                clutch.engaged = False
                clutch_event = "press ignored (no leader reading yet)"
            else:
                rb5_start, leader_start = target.copy(), leader_p.copy()
                yaw_rb5_start, yaw_leader_start = yaw_target, np.degrees(leader_q[6])
                clutch_event = f"engaged: RB5 start {np.round(rb5_start, 1)} mm, leader start {np.round(leader_start, 1)} mm"
        if clutch_event:
            event(f"[{time.monotonic() - t_start:7.2f} s] clutch {clutch_event}")

        # 3. RB5 state and the 08 safety checks
        robot.flush(rc)
        rc.error().throw_if_not_empty()
        rc.clear()
        tcp, q, state_age = reader.latest()
        now = time.monotonic()
        late_early = last_send is not None and now - last_send > MAX_GAP_S   # already late before planning
        stale = state_age > MAX_STATE_AGE_S
        stale_cycles = stale_cycles + 1 if stale else 0
        if stale_cycles >= MAX_STALE_CYCLES:
            raise RuntimeError(f"{MAX_STALE_CYCLES} stale-state cycles in a row; state reader stopped?")
        if stale:
            event(f"stale RB5 state: {state_age * 1000:.0f} ms old, target frozen")

        # 4. Target: follow the leader only while engaged and every check passed.
        #    The new target and the B shift are applied only after the gap check right before sending.
        t_plan0 = time.monotonic()
        desired = new = None
        flags = {}
        j5_pred, sh_pred, el_pred = singular_distance(q[4]), rb5_model.shoulder_margin(q), singular_distance(q[2])
        jl_pred = joint_limit_margin(q)[0]
        max_dq = lag = yaw_lag = 0.0
        dropped = np.zeros(3)
        yaw_desired = None
        plan = None
        yaw_tcp = wrap_deg(tcp[5] - rpy0[2])
        if clutch.engaged and q_l is not None and not late_early and not stale:
            desired = rb5_start + args.scale * (yaw @ (leader_p - leader_start))
            yaw_desired = (yaw_rb5_start + wrap_deg(np.degrees(leader_q[6]) - yaw_leader_start)) if not args.no_yaw else 0.0
            plan = plan_step(target, desired, tcp[:3], q, rb5_model, cyl,
                             {"target": yaw_target, "desired": yaw_desired, "tcp": yaw_tcp})
            new, dropped, flags = plan.target, plan.dropped, plan.flags
            j5_pred, sh_pred, el_pred, jl_pred = plan.margins
            max_dq, lag, yaw_lag = plan.max_dq, plan.lag, plan.lag_yaw
            if plan.stop:
                raise StopRequest(plan.stop)
        t_plan = time.monotonic() - t_plan0

        # 5. Late check on the real send gap, then apply the new target
        gap = None if last_send is None else time.monotonic() - last_send
        late = late_early or (gap is not None and gap > MAX_GAP_S)
        late_cycles = late_cycles + 1 if late else 0
        if late_cycles >= MAX_LATE_CYCLES:
            raise RuntimeError(f"{MAX_LATE_CYCLES} late cycles in a row; loop too slow for servo")
        if late:
            event(f"[{time.monotonic() - t_start:7.2f} s] late: {gap * 1000:.0f} ms since last send, target frozen")
        elif new is not None:
            # B / C: shift the leader start so that the dropped part is not caught up later
            leader_start += yaw.T @ dropped / args.scale
            target[:] = new
            yaw_leader_start += plan.dropped_yaw
            yaw_target = plan.yaw
            rpy[2] = wrap_deg(rpy0[2] + yaw_target)
        labels = {"speed": f"speed limit ({MAX_STEP_MM} mm/cycle)",
                  "j5slow": f"J5 below {J5_SLOW_DEG:.0f} deg: slower",
                  "shslow": f"shoulder margin below {SHOULDER_SLOW_MM:.0f} mm: slower",
                  "elslow": f"J3 within {ELBOW_SLOW_DEG:.0f} deg of straight: slower",
                  "jlslow": f"a joint within {JOINT_LIMIT_SLOW_DEG:.0f} deg of its limit: slower",
                  "jspeed": f"joint speed limit ({MAX_DQ_DEG.tolist()} deg/cycle predicted)",
                  "range": f"range limit (cylinder r {CYL_R_MIN:.0f}..{CYL_R_MAX:.0f} mm, "
                           f"z +/-{CYL_Z_HALF:.0f} mm, azimuth +/-{CYL_AZ_HALF_DEG:.0f} deg)",
                  "lead": f"target waits for robot (lead limit {MAX_LEAD_MM} mm)",
                  "yspeed": f"yaw speed limit ({MAX_YAW_STEP_DEG} deg/cycle)",
                  "yrange": f"yaw range limit (+/-{YAW_RANGE_DEG:.0f} deg)",
                  "ylead": f"yaw waits for robot (lead limit {MAX_YAW_LEAD_DEG} deg)"}
        for key, on in flags.items():
            if on and not flags_prev.get(key, False):
                event(f"[{time.monotonic() - t_start:7.2f} s] {labels[key]}")
        flags_prev = flags
        if flags.get("j5slow") and not warned["j5slow"]:
            event(f"WARNING: J5 {j5_pred:.1f} deg from the singular angle (stop below {J5_STOP_DEG:.0f})")
        if flags.get("shslow") and not warned["shslow"]:
            event(f"WARNING: shoulder margin {sh_pred:.1f} mm (stop below {SHOULDER_STOP_MM:.0f})")
        if flags.get("elslow") and not warned["elslow"]:
            event(f"WARNING: J3 {el_pred:.1f} deg from straight (stop below {ELBOW_STOP_DEG:.0f})")
        if flags.get("jlslow") and not warned["jlslow"]:
            event(f"WARNING: joint limit margin {jl_pred:.1f} deg (stop below {JOINT_LIMIT_STOP_DEG:.0f})")
        warned = {k: flags.get(k, False) for k in warned}

        # 6. Send every cycle (a released clutch sends the frozen target)
        t_send0 = time.monotonic()
        send_servo(robot, rc, target, rpy)
        t_send_end = time.monotonic()
        send_interval = None if last_send is None else t_send0 - last_send
        last_send = t_send0

        f = lambda v, fmt="{:.3f}": [fmt.format(x) for x in v]   # noqa: E731
        empty3 = [""] * 3
        ctx["rows"].append(
            [f"{t_cycle - t_start:.4f}", "" if interval is None else f"{interval * 1000:.3f}",
             "" if button_raw is None else button_raw, int(clutch.engaged)]
            + (f(np.degrees(leader_q)) if leader_q is not None else [""] * 7)
            + (f(leader_p) if leader_p is not None else empty3)
            + (f(desired) if desired is not None else empty3) + f(target) + f(tcp[:3]) + f(q)
            + [f"{j5_pred:.2f}", f"{sh_pred:.1f}", f"{max_dq:.3f}"]
            + [int(flags.get(k, False)) for k in ("speed", "j5slow", "shslow", "jspeed", "range", "lead")]
            + [int(late), int(late_early), int(stale), f"{state_age * 1000:.2f}"]
            + [f"{t_leader * 1000:.2f}", f"{t_plan * 1000:.2f}", f"{(t_send_end - t_send0) * 1000:.2f}",
               f"{(t_send_end - t_cycle) * 1000:.2f}", "" if send_interval is None else f"{send_interval * 1000:.2f}"]
            + [f"{el_pred:.2f}", int(flags.get("elslow", False)), f"{lag:.3f}",
               f"{np.linalg.norm(dropped) if not late else 0.0:.3f}"]
            + [f"{jl_pred:.2f}", int(flags.get("jlslow", False))]
            + ["" if yaw_desired is None else f"{yaw_desired:.3f}", f"{yaw_target:.3f}", f"{yaw_tcp:.3f}"]
            + [int(flags.get(k, False)) for k in ("yspeed", "yrange", "ylead")]
            + [f"{yaw_lag:.3f}", f"{abs(plan.dropped_yaw) if plan is not None and not late else 0.0:.3f}"]
            + ["" if grip.trigger is None else grip.trigger, "" if grip.open is None else f"{grip.open:.3f}",
               "" if grip.state_open is None else int(grip.state_open), int(button_raw is None)])

        # 7. Status line
        if cycle % STATUS_EVERY == 0:
            tool_rate = 100.0 * clutch.failures / max(clutch.reads, 1)
            leader_rate = 100.0 * leader_fail_total / max(leader_reads, 1)
            sys.stdout.write(
                f"{CLEAR_LINE}clutch {'ENGAGED ' if clutch.engaged else 'released'}  "
                f"target-start {np.round(target - tcp0[:3], 1)} mm  J5 {q[4]:6.1f} (eff {singular_distance(q[4]):5.1f})  "
                f"shoulder {rb5_model.shoulder_margin(q):5.0f} mm  yaw {yaw_target:6.1f}  "
                f"grip {'-' if grip.open is None else f'{grip.open:.2f}'}"
                f"{'' if grip.state_open is None else (' OPEN' if grip.state_open else ' CLOSED')}  |  "
                f"button fail {tool_rate:4.1f}%  leader fail {leader_rate:4.1f}%")
            sys.stdout.flush()

        # 8. Sleep until the next cycle; after an overrun wait a full period (08, as in 13)
        cycle += 1
        next_t += PERIOD_S
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic() + PERIOD_S
            time.sleep(PERIOD_S)


CSV_HEADER = (["t_s", "cycle_interval_ms", "button_raw", "clutch_engaged"]
              + [f"leader_q{i}_deg" for i in range(7)] + [f"leader_{a}_mm" for a in "xyz"]
              + [f"desired_{a}" for a in "xyz"] + [f"target_{a}" for a in "xyz"] + [f"tcp_ref_{a}" for a in "xyz"]
              + [f"jnt_ref_{i}" for i in range(6)] + ["j5_eff_pred_deg", "shoulder_margin_pred_mm", "max_dq_pred_deg"]
              + ["speed_limited", "j5_slow", "shoulder_slow", "joint_speed_limited", "range_limited", "lead_limited",
                 "late", "late_before_plan", "stale", "state_age_ms"]
              + ["leader_read_ms", "plan_ms", "send_call_ms", "work_ms", "send_interval_ms"]
              + ["elbow_from_straight_pred_deg", "elbow_slow", "lag_mm", "dropped_mm"]
              + ["joint_limit_margin_pred_deg", "joint_limit_slow"]
              + ["yaw_desired_deg", "yaw_target_deg", "yaw_tcp_deg", "yaw_speed_limited", "yaw_range_limited",
                 "yaw_lead_limited", "yaw_lag_deg", "yaw_dropped_deg"]
              + ["trigger_raw", "grip_open", "grip_state_open", "tool_read_failed"])


def save_csv(rows):
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"21_leader_pos_sim_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)
    return path


def read_tcp_offset(robot, rc):
    """TCP offset in the flange frame from get_tcp_info / get_tfc_info (17)."""
    res_t, tcp = robot.get_tcp_info(rc, timeout=1.0)
    res_f, tfc = robot.get_tfc_info(rc, timeout=1.0)
    if not (res_t.is_success() and res_f.is_success()):
        raise SystemExit("get_tcp_info / get_tfc_info failed.")
    tcp, tfc = np.asarray(tcp, dtype=float).reshape(6), np.asarray(tfc, dtype=float).reshape(6)
    if np.abs(tcp[3:] - tfc[3:]).max() > 0.01:
        raise SystemExit("The TCP has an orientation offset; this script assumes none (17: none).")
    return rot_zyx(*tfc[3:]).T @ (tcp[:3] - tfc[:3])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="rotation of the leader frame about z [deg]")
    parser.add_argument("--scale", type=float, default=1.0, help="RB5 motion / leader motion")
    parser.add_argument("--speed-mm-s", type=float, default=50.0,
                        help=f"TCP speed limit [mm/s], at most {MAX_SPEED_MM_S:.0f} (Simulation)")
    parser.add_argument("--max-lag-mm", type=float, default=30.0,
                        help=f"C: lag kept for catch-up [mm], at most {MAX_LAG_LIMIT_MM:.0f} (default 30); 0 = B")
    parser.add_argument("--no-yaw", action="store_true", help="keep the start orientation (no yaw mapping)")
    parser.add_argument("--port", default=LEADER_PORT, help="leader arm USB-serial device")
    args = parser.parse_args()
    if not 0 <= args.max_lag_mm <= MAX_LAG_LIMIT_MM:
        raise SystemExit(f"--max-lag-mm must be in [0, {MAX_LAG_LIMIT_MM:.0f}]")
    if not 0.1 <= args.scale <= 2.0:
        raise SystemExit("--scale must be between 0.1 and 2.0")
    if not 0 < args.speed_mm_s <= MAX_SPEED_MM_S:
        raise SystemExit(f"--speed-mm-s must be in (0, {MAX_SPEED_MM_S:.0f}]")
    # The lead limit scales with the step: with a fixed 2 mm, a 4 mm step would always wait for the robot
    global MAX_STEP_MM, MAX_LEAD_MM, MAX_LAG_MM, MAX_YAW_LAG_DEG
    MAX_LAG_MM = args.max_lag_mm
    if MAX_LAG_MM <= 0:
        MAX_YAW_LAG_DEG = 0.0
    MAX_STEP_MM = args.speed_mm_s * PERIOD_S
    MAX_LEAD_MM = LEAD_PER_STEP * MAX_STEP_MM
    np.set_printoptions(precision=2, suppress=True)

    latency_timer = read_latency_timer(args.port)
    if latency_timer != 1:
        raise SystemExit(f"FTDI latency_timer is {latency_timer} (need 1). Set it with:\n"
                         f"  echo 1 | sudo tee /sys/bus/usb-serial/devices/"
                         f"{os.path.basename(os.path.realpath(args.port))}/latency_timer")
    dxl_port_handler.LATENCY_TIMER = 1
    leader_model = LeaderModel(URDF_PATH)
    leader = LeaderArm(args.port)
    leader.open()

    robot = rb.Cobot(config.ROBOT_IP)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)
    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()
    if data_channel.request_data(1.0).sdata.real_vs_simulation_mode != 1:
        leader.close()
        raise SystemExit("The controller does not report Simulation mode. Not starting.")
    if robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        leader.close()
        raise SystemExit("The robot is moving. Not starting.")
    rb5_model = RB5Model(read_tcp_offset(robot, rc))

    reader = StateReader()
    reader.start()
    reader.wait_first()
    tcp0, q0, _ = reader.latest()
    fk_error = np.linalg.norm(rb5_model.tcp(q0)[0] - tcp0[:3])
    print(f"Mode: Simulation   loop {PERIOD_S * 1000:.0f} ms   move_servo_l t1={SERVO_T1} t2={SERVO_T2}   "
          f"yaw {args.yaw_deg} deg   scale {args.scale}   "
          f"{'B (drop the cut part)' if MAX_LAG_MM <= 0 else f'C (catch up to {MAX_LAG_MM:.0f} mm / {MAX_YAW_LAG_DEG:.0f} deg lag)'}   "
          f"yaw {'off' if args.no_yaw else f'leader J6 -> RB5 rz, {YAW_SPEED_DEG_S:.0f} deg/s, +/-{YAW_RANGE_DEG:.0f} deg'}")
    print(f"RB5 tcp_ref {tcp0}   jnt_ref {q0}")
    print(f"TCP offset (flange frame) {rb5_model.offset} mm   DH model vs tcp_ref {fk_error:.2f} mm   "
          f"J5 {singular_distance(q0[4]):.1f} deg from singular, shoulder margin {rb5_model.shoulder_margin(q0):.1f} mm, "
          f"J3 {singular_distance(q0[2]):.1f} deg from straight, joint limit margin {joint_limit_margin(q0)[0]:.1f} deg "
          f"(J{joint_limit_margin(q0)[1]})")
    print(f"Limits: {MAX_STEP_MM} mm/cycle ({MAX_STEP_MM / PERIOD_S:.0f} mm/s), {MAX_DQ_DEG.tolist()} deg/cycle joints, "
          f"cylinder r {CYL_R_MIN:.0f}..{CYL_R_MAX:.0f} mm / z +/-{CYL_Z_HALF:.0f} mm / "
          f"azimuth +/-{CYL_AZ_HALF_DEG:.0f} deg, lead {MAX_LEAD_MM} mm, J5 slow < {J5_SLOW_DEG:.0f} / stop < {J5_STOP_DEG:.0f} deg, "
          f"shoulder slow < {SHOULDER_SLOW_MM:.0f} / stop < {SHOULDER_STOP_MM:.0f} mm, "
          f"elbow slow < {ELBOW_SLOW_DEG:.0f} / stop < {ELBOW_STOP_DEG:.0f} deg, "
          f"joint limit slow < {JOINT_LIMIT_SLOW_DEG:.0f} / stop < {JOINT_LIMIT_STOP_DEG:.0f} deg (J3 +/-{JOINT_LIMIT_DEG[2]:.0f})")
    problems = []
    if fk_error > MAX_FK_ERROR_MM:
        problems.append(f"DH model differs from tcp_ref by {fk_error:.2f} mm (> {MAX_FK_ERROR_MM})")
    r0 = np.hypot(tcp0[0], tcp0[1])
    if not CYL_R_MIN <= r0 <= CYL_R_MAX:
        problems.append(f"TCP radius {r0:.1f} mm is outside the cylinder range {CYL_R_MIN:.0f}..{CYL_R_MAX:.0f} mm "
                        f"(ready pose: scripts/19_move_joint_sim.py --pose 182 -11.0 -110.1 31.1 -90 -2 --speed 30)")
    if rb5_model.shoulder_margin(q0) < SHOULDER_SLOW_MM:
        problems.append(f"shoulder margin is {rb5_model.shoulder_margin(q0):.1f} mm (need >= {SHOULDER_SLOW_MM:.0f})")
    if joint_limit_margin(q0)[0] < JOINT_LIMIT_SLOW_DEG:
        problems.append(f"J{joint_limit_margin(q0)[1]} is {joint_limit_margin(q0)[0]:.1f} deg from its limit "
                        f"(need >= {JOINT_LIMIT_SLOW_DEG:.0f})")
    if singular_distance(q0[2]) < ELBOW_SLOW_DEG:
        problems.append(f"J3 is {singular_distance(q0[2]):.1f} deg from straight (need >= {ELBOW_SLOW_DEG:.0f})")
    if singular_distance(q0[4]) < J5_SLOW_DEG:
        problems.append(f"J5 is {singular_distance(q0[4]):.1f} deg from the singular angle (need >= {J5_SLOW_DEG:.0f}); "
                        f"move it first, e.g. scripts/19_move_joint_sim.py --joint 5 --to 45")
    if problems:
        reader.stop()
        leader.close()
        raise SystemExit("Not starting:\n  " + "\n  ".join(problems))
    print("Clutch: right handle button (press and hold to follow, release to freeze). Starts released.")

    ctx = {"target": None, "rows": [], "clutch": None, "rpy": tcp0[3:].copy()}
    error = None
    robot.disable_waiting_ack(rc)   # servo commands must not wait for an ACK each cycle
    print("Ctrl+C to stop.\n")
    try:
        run_loop(leader, leader_model, robot, rc, reader, rb5_model, args, ctx)
    except KeyboardInterrupt:
        event("Interrupted.")
    except StopRequest as e:
        event(f"Stopping: {e}")
    except Exception as e:   # controller error, late loop, stale state: stop without more servo commands
        error = e
        event(f"Error: {e}")
    finally:
        if error is None and ctx["target"] is not None:
            stop_streaming(robot, rc, ctx["target"], ctx["rpy"])
        else:
            emergency_stop(robot, rc)
        reader.stop()
        leader.close()

    path = save_csv(ctx["rows"])
    clutch = ctx["clutch"]
    if clutch is not None and clutch.reads:
        print(f"Button reads {clutch.reads}, failed or unexpected {clutch.failures} "
              f"({100.0 * clutch.failures / clutch.reads:.1f}%)")
    print(f"Saved: {path.relative_to(ROOT)} ({len(ctx['rows'])} cycles)")


if __name__ == "__main__":
    main()

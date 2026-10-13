"""Live forward kinematics of the RB-Y1 leader arm (read-only).

Reads joints ID 0..13 with GroupSyncRead (ping and read only; torque is not touched) and computes
the pose of Link_6R / Link_6L from the leader URDF with rby1_sdk.dynamics (as example 35 does).

"Tip" here = the Link_6 origin = the point where the three wrist axes (J4, J5, J6) meet, which is
the center of the J5 (wrist pitch) motor axis from the outside. The URDF has no handle tip frame;
a handle offset can be measured and added later (like the RB5 TCP offset).

Shown at about 10 Hz:
  - joint angles [deg] (rad = raw / 4096 * 2 pi, raw 0 = URDF 0)
  - wrist center position [mm] in the leader Base frame
  - orientation as ZY'X'' Euler [deg] (same convention as the RB5), and the Link_6 z axis with its
    tilt from vertical
  - right - left wrist center distance [mm]
  - after pressing b: right arm change from the saved baseline (dx, dy, dz, distance [mm], rotation [deg])

At startup the rby1_sdk result is compared with a direct numpy evaluation of the URDF; the script
stops if they differ (rby1_sdk silently ignored J3..J6 when make_state got only the 7 right joints).

Keys: b = save baseline (right arm), q = quit.
Usage:
    python scripts/20_leader_fk_live.py
    python scripts/20_leader_fk_live.py --port /dev/ttyUSB0
"""
import argparse
import os
import select
import sys
import termios
import time
import tty
import xml.etree.ElementTree as ET
from pathlib import Path

import dynamixel_sdk.port_handler as dxl_port_handler
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
import numpy as np
import rby1_sdk as rby

ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = ROOT / "assets" / "leader_arm" / "model.urdf"   # copy of rby1-sdk v0.10.0 (was refs/rby1-sdk)

PROTOCOL_VERSION = 2.0     # rby1-sdk dynamixel_bus.h
BAUDRATE = 2000000
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
TICKS_PER_REV = 4096

# Same link / joint lists as example 35 (all 14 joints are required, see the docstring)
LINKS = ["Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R",
         "Link_0L", "Link_1L", "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"]
JOINTS = ["J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R",
          "J4_Wrist_Yaw1_R", "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R",
          "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L", "J10_Elbow_L",
          "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"]
JOINT_IDS = list(range(14))           # ID i = JOINTS[i]
BASE_LINK, RIGHT_LINK, LEFT_LINK = 0, 7, 14   # indices in LINKS
ARMS = (("right", RIGHT_LINK, slice(0, 7)), ("left", LEFT_LINK, slice(7, 14)))

PERIOD_S = 0.02            # read at 50 Hz
DISPLAY_EVERY = 5          # redraw at about 10 Hz
SELF_TEST_TOL = 1e-6
STALE_S = 0.5              # joint values older than this are flagged

HIDE_CURSOR, SHOW_CURSOR = "\x1b[?25l", "\x1b[?25h"
CLEAR_LINE = "\x1b[2K"


def read_latency_timer(port_name):
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


class LeaderFK:
    """rby1_sdk FK of the leader URDF, plus a direct numpy evaluation used as a self-test."""

    def __init__(self, urdf_path):
        self.robot = rby.dynamics.Robot(rby.dynamics.load_robot_from_urdf(str(urdf_path), "Base"))
        self.state = self.robot.make_state(LINKS, JOINTS)
        self.urdf_joints = {j.get("name"): j for j in ET.parse(urdf_path).getroot().findall("joint")}

    def poses(self, q):
        """q: 14 joint angles [rad]. Returns {"right": T, "left": T} (4x4, meters)."""
        self.state.set_q(q)
        self.robot.compute_forward_kinematics(self.state)
        return {arm: self.robot.compute_transformation(self.state, BASE_LINK, link) for arm, link, _ in ARMS}

    def numpy_pose(self, q_arm, names):
        """Chain of URDF joints: T = prod(Trans(origin xyz) * Rot(axis, q)). All origins have rpy 0."""
        t = np.eye(4)
        for name, angle in zip(names, q_arm):
            j = self.urdf_joints[name]
            origin = j.find("origin")
            if any(float(v) != 0.0 for v in origin.get("rpy", "0 0 0").split()):
                raise SystemExit(f"{name}: origin rpy is not zero; the numpy self-test does not handle it.")
            step = np.eye(4)
            step[:3, 3] = [float(v) for v in origin.get("xyz").split()]
            step[:3, :3] = axis_rotation([float(v) for v in j.find("axis").get("xyz").split()], angle)
            t = t @ step
        return t

    def self_test(self, samples=50):
        rng = np.random.default_rng(0)
        worst = 0.0
        for _ in range(samples):
            q = rng.uniform(-1.5, 1.5, 14)
            sdk = self.poses(q)
            for arm, _, sl in ARMS:
                worst = max(worst, np.abs(sdk[arm] - self.numpy_pose(q[sl], JOINTS[sl])).max())
        return worst


def axis_rotation(axis, angle):
    """Rodrigues formula: rotation by angle [rad] about a (normalized) axis."""
    k = np.asarray(axis, dtype=float)
    k /= np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def zyx_euler_deg(r):
    """ZY'X'' Euler angles (rx, ry, rz) [deg] with R = Rz(rz) Ry(ry) Rx(rx), as the RB5 uses."""
    ry = np.arcsin(np.clip(-r[2, 0], -1.0, 1.0))
    rz = np.arctan2(r[1, 0], r[0, 0])
    rx = np.arctan2(r[2, 1], r[2, 2])
    return np.degrees([rx, ry, rz])


def rotation_angle_deg(r_a, r_b):
    r = r_a.T @ r_b
    return np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))


def render(q_deg, poses, baseline, age, stats):
    lines = [f"read {stats['rate']:5.1f} Hz   sync read failures {stats['fails']}   "
             f"joints {'OK' if age < STALE_S else f'STALE {age:.1f} s'}   keys: b = baseline (right), q = quit",
             "joints [deg]  " + "  ".join(f"{'J' + str(i):>7}" for i in range(7))]
    for arm, _, sl in ARMS:
        lines.append(f"  {arm:<10}  " + "  ".join(f"{v:7.2f}" for v in q_deg[sl]))
    lines.append(f"wrist center [mm]  {'x':>8} {'y':>8} {'z':>8}   ZY'X'' [deg] {'rx':>7} {'ry':>7} {'rz':>7}"
                 f"   Link_6 z axis           tilt")
    for arm, _, _ in ARMS:
        t = poses[arm]
        p = t[:3, 3] * 1000
        e = zyx_euler_deg(t[:3, :3])
        z = t[:3, 2]
        tilt = np.degrees(np.arccos(np.clip(z[2], -1.0, 1.0)))
        lines.append(f"  {arm:<16} {p[0]:8.1f} {p[1]:8.1f} {p[2]:8.1f}                {e[0]:7.2f} {e[1]:7.2f} "
                     f"{e[2]:7.2f}   [{z[0]:6.3f} {z[1]:6.3f} {z[2]:6.3f}]  {tilt:6.1f}")
    d = (poses["right"][:3, 3] - poses["left"][:3, 3]) * 1000
    lines.append(f"right - left [mm]  {d[0]:8.1f} {d[1]:8.1f} {d[2]:8.1f}   distance {np.linalg.norm(d):7.1f}")
    if baseline is None:
        lines.append("right from baseline: (press b)")
    else:
        dp = (poses["right"][:3, 3] - baseline[:3, 3]) * 1000
        rot = rotation_angle_deg(baseline[:3, :3], poses["right"][:3, :3])
        lines.append(f"right from baseline [mm] dx {dp[0]:7.1f}  dy {dp[1]:7.1f}  dz {dp[2]:7.1f}  "
                     f"distance {np.linalg.norm(dp):7.1f}   rotation {rot:6.2f} deg")
    return lines


def draw(lines, first):
    out = [] if first else [f"\x1b[{len(lines)}F"]
    out += [CLEAR_LINE + line + "\n" for line in lines]
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def read_key():
    """Return one pressed key, or None (stdin is in cbreak mode)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USB-serial device of the leader arm")
    args = parser.parse_args()

    fk = LeaderFK(URDF_PATH)
    worst = fk.self_test()
    print(f"FK self-test (rby1_sdk vs numpy URDF chain, 50 random poses): max difference {worst:.2e}")
    if worst > SELF_TEST_TOL:
        raise SystemExit("rby1_sdk FK and the numpy URDF chain differ. Not starting.")

    latency_timer = read_latency_timer(args.port)
    print(f"Leader arm FK live (read-only)   {args.port}   FTDI latency_timer {latency_timer} ms")
    if latency_timer == 1:
        dxl_port_handler.LATENCY_TIMER = 1   # dynamixel_sdk reply timeout follows this constant (12)
    elif latency_timer is not None:
        print(f"Note: latency_timer is {latency_timer} ms; reads are slower but 50 Hz is not required here.")

    port = PortHandler(args.port)
    ph = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise SystemExit(f"Failed to open {args.port}")
    stdin_attrs = termios.tcgetattr(sys.stdin)
    try:
        if not port.setBaudRate(BAUDRATE):
            raise SystemExit(f"Failed to set baud rate {BAUDRATE}")
        missing = [i for i in JOINT_IDS if ph.ping(port, i)[1] != COMM_SUCCESS]
        if missing:
            raise SystemExit(f"Joint IDs {missing} did not answer the ping.")
        sync = GroupSyncRead(port, ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for dxl_id in JOINT_IDS:
            sync.addParam(dxl_id)

        tty.setcbreak(sys.stdin.fileno())
        sys.stdout.write(HIDE_CURSOR)
        q = None
        t_ok = None
        baseline = None
        stats = {"fails": 0, "rate": 0.0}
        cycles = window = 0
        first = True
        t_window = next_t = time.monotonic()
        while True:
            result = sync.txRxPacket()
            if result == COMM_SUCCESS and all(
                    sync.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) for i in JOINT_IDS):
                raw = np.array([to_signed32(sync.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
                                for i in JOINT_IDS], dtype=float)
                q = raw / TICKS_PER_REV * 2 * np.pi
                t_ok = time.monotonic()
            else:
                stats["fails"] += 1   # keep the last joint values

            key = read_key()
            if key == "q":
                break
            if q is not None:
                poses = fk.poses(q)
                if key == "b":
                    baseline = poses["right"].copy()
                cycles += 1
                window += 1
                if cycles % DISPLAY_EVERY == 0:
                    now = time.monotonic()
                    stats["rate"] = window / max(now - t_window, 1e-9)
                    window, t_window = 0, now
                    draw(render(np.degrees(q), poses, baseline, now - t_ok, stats), first)
                    first = False

            next_t += PERIOD_S
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, stdin_attrs)
        sys.stdout.write(SHOW_CURSOR)
        sys.stdout.flush()
        port.closePort()
    print("Stopped.")


if __name__ == "__main__":
    main()

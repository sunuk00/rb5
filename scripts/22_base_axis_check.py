"""Check where the RB5 base +x / +y axes point on the real robot. Read-only, no motion command.

Give the real arm's joint angles as shown on the tablet (in Simulation mode the controller's jnt_ref
can differ from the physical arm). The script computes, with the controller's calc_fk_tcp:
  - the TCP pose of those joints
  - the tool flange center (TFC) position: TCP - R * offset, where the TCP offset (in the flange
    frame) is derived from the current get_tcp_info / get_tfc_info as in 17
and prints the horizontal (x, y) position of the flange center, which is the visible point to
measure with a tape from the center of the base.

Expected from the manual (Appendix F): origin = center of the base surface, +Z up, +Y toward the
base cable connector, so +X is 90 deg clockwise from the connector direction seen from above.

Usage:
    python scripts/22_base_axis_check.py --joints J1 J2 J3 J4 J5 J6     # tablet values [deg]
    python scripts/22_base_axis_check.py                                # use the data channel jnt_ang
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

TIMEOUT_S = 1.0


def rot_zyx(rx, ry, rz):
    """Rotation matrix of a ZY'X'' Euler pose (deg): R = Rz(rz) Ry(ry) Rx(rx)."""
    a, b, c = np.radians([rx, ry, rz])
    rx_m = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    ry_m = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    rz_m = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def query(name, call):
    res, pnt = call()
    if not res.is_success():
        raise SystemExit(f"{name} failed ({res})")
    return np.array(pnt, dtype=float).reshape(6)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joints", type=float, nargs=6, metavar="J", help="real arm joint angles [deg] (tablet)")
    args = parser.parse_args()
    np.set_printoptions(precision=1, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)
    rc = rb.ResponseCollector()
    data = rb.CobotData(config.ROBOT_IP).request_data(TIMEOUT_S)
    if data is None:
        raise SystemExit("request_data() timed out")
    mode = "Simulation" if data.sdata.real_vs_simulation_mode == 1 else "Real"

    if args.joints is not None:
        q = np.array(args.joints)
        source = "--joints (tablet)"
    else:
        q = np.array(data.sdata.jnt_ang, dtype=float)
        source = f"data channel jnt_ang ({mode} mode)"
        if mode == "Simulation":
            # jnt_ang stayed at the start pose while Simulation moves changed jnt_ref (17 vs 18/19), so it is
            # probably the physical encoder reading. Not confirmed; compare with the tablet values.
            print("Note: Simulation mode - jnt_ang is probably the physical arm (it does not follow Simulation "
                  "moves). Compare with the tablet joint values.")

    # TCP offset in the flange frame from the current pose (17): R_tfc^T (tcp - tfc)
    tcp_now = query("get_tcp_info", lambda: robot.get_tcp_info(rc, timeout=TIMEOUT_S))
    tfc_now = query("get_tfc_info", lambda: robot.get_tfc_info(rc, timeout=TIMEOUT_S))
    offset = rot_zyx(*tfc_now[3:]).T @ (tcp_now[:3] - tfc_now[:3])
    rot_offset = np.abs(tcp_now[3:] - tfc_now[3:]).max()

    tcp = query("calc_fk_tcp", lambda: robot.calc_fk_tcp(rc, j0=q[0], j1=q[1], j2=q[2], j3=q[3], j4=q[4], j5=q[5],
                                                          timeout=TIMEOUT_S))
    flange = tcp[:3] - rot_zyx(*tcp[3:]) @ offset   # TCP and flange orientations are equal (offset rot 0)

    print(f"Controller mode: {mode}   joints from {source}")
    print(f"Joints [deg]                 {q}")
    print(f"TCP offset (flange frame)    {offset} mm" + ("" if rot_offset < 0.01 else
          f"   WARNING: orientation offset {rot_offset:.2f} deg, flange estimate not valid"))
    print(f"TCP (calc_fk_tcp)            {tcp}   (x, y, z [mm], rx, ry, rz [deg])")
    print(f"Flange center                {flange} mm")
    print()
    print("Measure from the center of the base to the point on the floor/table right below the flange center:")
    print(f"  along +Y (toward the base cable connector):          {flange[1]:8.1f} mm "
          f"({'toward' if flange[1] >= 0 else 'away from'} the connector)")
    print(f"  along +X (90 deg clockwise from +Y, seen from above): {flange[0]:8.1f} mm")
    print(f"  horizontal distance:                                  {np.hypot(flange[0], flange[1]):8.1f} mm")
    print(f"  height of the flange center above the base surface:   {flange[2]:8.1f} mm")
    print(f"  direction from base center, seen from above: {np.degrees(np.arctan2(flange[1], flange[0])):6.1f} deg "
          f"counterclockwise from +X")

    try:
        rc.error().throw_if_not_empty()
        print("No error messages from the controller.")
    except Exception as e:
        print(f"Controller error message(s): {e}")


if __name__ == "__main__":
    main()

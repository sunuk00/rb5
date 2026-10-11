"""Compare the ways to read the RB5 TCP pose. Read-only, no motion command.

Reads, in this order:
  1. data channel: jnt_ref, jnt_ang, tcp_ref, tcp_pos, operation mode
  2. get_tcp_info(), get_tfc_info()
  3. calc_fk_tcp(jnt_ref) and calc_fk_tcp(jnt_ang)
  4. data channel again (to check that nothing moved while reading)

The script commands in 2-3 are print(...) queries; no operation mode, speed or motion is sent.
Prints every pose and the differences used to answer:
  - is get_tcp_info the current TCP pose (same as tcp_ref) or the TCP offset setting?
  - does calc_fk_tcp(current joints) give the same frame and units as tcp_ref?
  - is tcp_pos different from tcp_ref (it is documented as "overwritten based on the current reference")?

Usage:
    python scripts/17_tcp_read_compare.py
"""
import sys
from pathlib import Path

import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

TIMEOUT_S = 1.0
LABELS = ("x", "y", "z", "rx", "ry", "rz")


def read_state(data_channel):
    data = data_channel.request_data(TIMEOUT_S)
    if data is None:
        raise SystemExit("request_data() timed out")
    s = data.sdata
    return {
        "mode": "Simulation" if s.real_vs_simulation_mode == 1 else "Real",
        "jnt_ref": np.array(s.jnt_ref, dtype=float),
        "jnt_ang": np.array(s.jnt_ang, dtype=float),
        "tcp_ref": np.array(s.tcp_ref, dtype=float),
        "tcp_pos": np.array(s.tcp_pos, dtype=float),
    }


def query(name, call):
    """Run one print(...) query. Returns the pose, or None on failure."""
    res, pnt = call()
    if not res.is_success():
        print(f"  {name}: failed ({res})")
        return None
    return np.array(pnt, dtype=float).reshape(6)


def fmt(v):
    return "  ".join(f"{x:10.3f}" for x in v)


def main():
    data_channel = rb.CobotData(config.ROBOT_IP)
    robot = rb.Cobot(config.ROBOT_IP)
    rc = rb.ResponseCollector()

    before = read_state(data_channel)

    poses = {}
    poses["get_tcp_info"] = query("get_tcp_info", lambda: robot.get_tcp_info(rc, timeout=TIMEOUT_S))
    poses["get_tfc_info"] = query("get_tfc_info", lambda: robot.get_tfc_info(rc, timeout=TIMEOUT_S))
    # Keyword joint arguments select the overload that returns (res, point)
    for src in ("jnt_ref", "jnt_ang"):
        q = before[src]
        poses[f"calc_fk_tcp({src})"] = query(
            f"calc_fk_tcp({src})",
            lambda q=q: robot.calc_fk_tcp(rc, j0=q[0], j1=q[1], j2=q[2], j3=q[3], j4=q[4], j5=q[5],
                                          timeout=TIMEOUT_S))

    after = read_state(data_channel)
    poses = {"tcp_ref": before["tcp_ref"], "tcp_pos": before["tcp_pos"], **poses}

    print(f"Operation mode (data channel): {before['mode']}")
    print(f"{'joints [deg]':<22}" + "  ".join(f"{'j' + str(i):>10}" for i in range(6)))
    print(f"{'jnt_ref':<22}{fmt(before['jnt_ref'])}")
    print(f"{'jnt_ang':<22}{fmt(before['jnt_ang'])}")
    print()
    print(f"{'pose [mm, deg]':<22}" + "  ".join(f"{s:>10}" for s in LABELS))
    for name, p in poses.items():
        print(f"{name:<22}{fmt(p) if p is not None else '  (not read)'}")

    print()
    print("Differences from tcp_ref (rotation columns: plain subtraction of the numbers)")
    for name, p in poses.items():
        if name != "tcp_ref" and p is not None:
            print(f"{name + ' - tcp_ref':<36}{fmt(p - before['tcp_ref'])}")
    p = poses["get_tcp_info"]
    q = poses["get_tfc_info"]
    if p is not None and q is not None:
        d = p[:3] - q[:3]
        print(f"{'get_tcp_info - get_tfc_info (xyz)':<36}{fmt(d)}   |d| = {np.linalg.norm(d):.3f} mm")

    print()
    moved = max(np.max(np.abs(after["jnt_ref"] - before["jnt_ref"])),
                np.max(np.abs(after["jnt_ang"] - before["jnt_ang"])))
    print(f"Joint change while reading: {moved:.4f} deg (should be about 0)")

    try:
        rc.error().throw_if_not_empty()
        print("No error messages from the controller.")
    except Exception as e:
        print(f"Controller error message(s): {e}")


if __name__ == "__main__":
    main()

"""Checks before the teleop starts, and the ready-pose move plan.

Each check returns a list of problems (empty = OK). The teleop does not start if any list is not empty.
The ready-pose plan follows 19: per joint the equivalent angle (target + k x 360) inside the joint
limit closest to the current angle; the move_j speed is scaled by the speed bar (19).
"""
import numpy as np

from .kinematics import CylinderRange, RB5Model


def nearest_equivalent(target, current, limit):
    """target + k x 360 inside +/-limit that is closest to current, or None if none is inside."""
    candidates = [target + 360.0 * k for k in range(-2, 3) if abs(target + 360.0 * k) <= limit]
    return min(candidates, key=lambda a: abs(a - current)) if candidates else None


def plan_ready_move(q_now, ready, joint_limits, rp, speed_bar):
    """Plan the move to the ready pose. Returns a dict:
    target (joints or None), change, skip (already there), wrist_warnings, expected_s, timeout_s,
    speed (move_j argument), acc, problems."""
    q_now = np.asarray(q_now, dtype=float)
    target, problems = np.zeros(6), []
    for i, (t, c) in enumerate(zip(ready, q_now)):
        a = nearest_equivalent(t, c, joint_limits[i])
        if a is None:
            problems.append(f"ready pose J{i + 1} {t} has no equivalent inside +/-{joint_limits[i]:.0f} deg")
            a = np.nan
        target[i] = a
    change = target - q_now
    speed = rp.move_speed_deg_s
    acc = 2.0 * speed                                    # 19: acceleration = 2 x speed
    eff_speed = speed * speed_bar                        # the controller scales move_j by the speed bar (19)
    largest = float(np.nanmax(np.abs(change))) if not problems else 0.0
    expected_s = largest / eff_speed + eff_speed / (acc * speed_bar)
    wrist = [f"J{i + 1} turns {change[i]:+.1f} deg" for i in (3, 4, 5)
             if not problems and abs(change[i]) > rp.wrist_warning_deg]
    return {"target": None if problems else target, "change": change,
            "skip": not problems and largest < rp.tolerance_deg, "wrist_warnings": wrist,
            "expected_s": expected_s, "timeout_s": 1.5 * expected_s + 5.0, "speed": speed, "acc": acc,
            "problems": problems}


def check_leader(latency_ms, required_ms, open_problems):
    problems = []
    if latency_ms != required_ms:
        problems.append(f"FTDI latency_timer is {latency_ms} ms (need {required_ms}); see docs/teleop_guide.md (udev rule)")
    problems += [f"leader arm: {p}" for p in open_problems]
    return problems


def check_robot_mode(real_requested, reports_simulation, moving, error_text):
    problems = []
    if real_requested and reports_simulation:
        problems.append("Real was requested but the controller reports Simulation mode")
    if not real_requested and not reports_simulation:
        problems.append("Simulation was requested but the controller reports Real mode")
    if moving:
        problems.append("the robot is moving")
    if error_text:
        problems.append(f"controller error messages: {error_text}")
    return problems


def check_tcp_offset(offset, rot_offset_deg, tcp_settings):
    """The TCP must have no orientation offset (all mapping assumes that); in Real the position offset
    must match the expected value from the settings."""
    problems = []
    if rot_offset_deg > 0.01:
        problems.append(f"the TCP has an orientation offset of {rot_offset_deg:.2f} deg (expected none)")
    exp = tcp_settings.expected_offset_mm
    if exp is not None and np.linalg.norm(np.asarray(offset) - np.asarray(exp)) > tcp_settings.offset_tolerance_mm:
        problems.append(f"TCP offset {np.round(offset, 2).tolist()} mm differs from the expected {list(exp)} mm "
                        f"(tolerance {tcp_settings.offset_tolerance_mm} mm)")
    return problems


def check_start_pose(tcp0, q0, model: RB5Model, watch, s):
    """At the ready pose: DH model vs tcp_ref, TCP inside the range, every watch at its slow margin."""
    problems = []
    fk_error = float(np.linalg.norm(model.tcp(q0)[0] - tcp0[:3]))
    if fk_error > s.safety.max_fk_error_mm:
        problems.append(f"DH model differs from tcp_ref by {fk_error:.2f} mm (> {s.safety.max_fk_error_mm})")
    r0 = float(np.hypot(tcp0[0], tcp0[1]))
    cyl = range_for(s)
    if not cyl.r_min <= r0 <= cyl.r_max_at(tcp0[2]):
        problems.append(f"TCP radius {r0:.1f} mm is outside {cyl.r_min:.0f}..{cyl.r_max_at(tcp0[2]):.0f} mm")
    if not cyl.z_lo <= tcp0[2] <= cyl.z_hi:
        problems.append(f"TCP z {tcp0[2]:.1f} mm is outside {cyl.z_lo:.0f}..{cyl.z_hi:.0f} mm")
    az = float(np.degrees(np.arctan2(tcp0[1], tcp0[0])))
    if abs((az - cyl.az0 + 180.0) % 360.0 - 180.0) > cyl.az_half:
        problems.append(f"TCP azimuth {az:.1f} deg is outside {s.range.azimuth_min_deg:.0f}.."
                        f"{s.range.azimuth_max_deg:.0f} deg")
    problems += watch.start_problems(q0, model)
    return problems, fk_error


def range_for(s):
    return CylinderRange.from_settings(s.range)

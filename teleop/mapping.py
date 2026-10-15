"""Leader -> RB5 mapping: clutch-relative targets, per-cycle limits (B / C), position and yaw.

Moved from scripts/21_leader_rb5_position.py without changes in behavior; the limits come from the
settings instead of module constants.

Position (mapping.mode):
  cartesian:   desired = RB5 start + scale * Rz(frame_yaw) * (leader wrist center now - at press) [mm].
  cylindrical: the leader motion is split about a vertical pivot axis (mapping.pivot_xy_mm, leader frame)
               into an angle, a radius and a height change, applied to the RB5 target about the base z
               axis: angle 1:1 (so a swing of the hand turns J1), radius and height x scale. The angle is
               the sideways motion / max(hand radius, min_pivot_radius_mm), so the gain stays bounded
               near the axis. Integrated per cycle; frame_yaw is not used.
Vertical mode (left handle button held, mapping.vertical_button): x and y of the desired position stay
at the target's x and y when the mode or the clutch was switched on; only z follows, x scale x
mapping.vertical_z_scale. Every switch re-anchors the references, so the RB5 does not jump.
Yaw:      desired = RB5 yaw at press + (leader J6 now - at press) [deg], about base z, TCP fixed.
Yaw mode follow_base (yaw.mode): the tool yaw follows the azimuth of the target about the base z axis,
so the gripper turns with the arm and the RB5 wrist joints (J6) stay still when J1 turns; the leader J6
is not used and the yaw speed / range limits do not apply (the position limits do).
Per cycle (plan_step): range first (the excess outside the range is kept), then the speed limits
scaled by the singularity / joint limit watches (sideways about the base z axis up to
speed_tangential_mm_s, radial and vertical up to speed_mm_s: (t / T)^2 + (o / O)^2 <= 1), then the
per-joint speed limit, then the lead limit.
B drops the cut part of the step; C keeps the lag up to max_lag and drops only the part beyond it.
"""
from types import SimpleNamespace

import numpy as np

from .kinematics import clip_norm, rot_z, wrap_deg


class Limits:
    """Per-cycle limits derived from the settings."""

    def __init__(self, s):
        period = s.servo.period_s
        self.max_step_mm = s.mapping.speed_mm_s * period
        self.max_tangential_step_mm = s.mapping.speed_tangential_mm_s * period
        self.max_lead_mm = s.mapping.lead_per_step * max(self.max_step_mm, self.max_tangential_step_mm)
        self.max_lag_mm = s.mapping.max_lag_mm
        self.max_yaw_step_deg = s.yaw.speed_deg_s * period
        self.yaw_range_deg = s.yaw.range_deg
        self.max_yaw_lead_deg = s.mapping.lead_per_step * self.max_yaw_step_deg
        # B for the position means B for the yaw too
        self.max_yaw_lag_deg = s.yaw.max_lag_deg if self.max_lag_mm > 0 else 0.0
        self.max_dq_deg = np.asarray(s.safety.joint_speed_deg_per_cycle, dtype=float)


def clip_step(step, at, max_other, max_tangential):
    """Scale the step down (same direction) so that (t / max_tangential)^2 + (o / max_other)^2 <= 1, with
    t = the horizontal part perpendicular to the radius at `at` (sideways about the base z axis) and o =
    the rest (radial and vertical)."""
    r = np.hypot(at[0], at[1])
    u = np.array([-at[1] / r, at[0] / r, 0.0]) if r > 1e-9 else np.zeros(3)
    t = float(step @ u)
    o = float(np.linalg.norm(step - t * u))
    if max_other <= 0.0 or max_tangential <= 0.0:
        return np.zeros(3)
    k = np.hypot(t / max_tangential, o / max_other)
    return step / k if k > 1.0 else step


def azimuth_from(p, az0_deg):
    """Azimuth of p about the base z axis relative to az0 [deg]."""
    return wrap_deg(np.degrees(np.arctan2(p[1], p[0])) - az0_deg)


def plan_step(target, desired, tcp_pos, q, model, cyl, yaw, lim, watch):
    """One cycle of the limits for the position and the yaw.

    yaw: dict with "target", "desired", "tcp" = yaw about base z relative to the start rz [deg], and
    "base_az0" = the start azimuth [deg] when the yaw follows the base (follow_base), else None.
    Returns a SimpleNamespace: target (new position), yaw (new yaw), dropped (position vector),
    dropped_yaw, flags, margins (J5, shoulder, elbow, joint limit), max_dq, lag, lag_yaw, stop.
    """
    flags = dict.fromkeys(("speed", "j5slow", "shslow", "elslow", "jlslow", "jspeed", "range", "lead",
                           "yspeed", "yrange", "ylead"), False)
    az0 = yaw.get("base_az0")
    follow = az0 is not None
    yaw_at = (lambda p: azimuth_from(p, az0)) if follow else None   # follow_base: yaw of a position
    desired_in = cyl.clamp(desired)
    flags["range"] = bool(np.any(desired_in != desired))
    if follow:
        ydes_in = yaw_at(desired_in)
    else:
        ydes_in = float(np.clip(yaw["desired"], -lim.yaw_range_deg, lim.yaw_range_deg))
        flags["yrange"] = ydes_in != yaw["desired"]
    step = desired_in - target
    ystep = wrap_deg(ydes_in - yaw["target"]) if follow else ydes_in - yaw["target"]
    jac = model.jacobian(q)   # all predictions below are from the same q: compute it once
    lead_p, lead_y = target - tcp_pos, yaw["target"] - yaw["tcp"]   # parts that are only the robot catching up

    # Speed limits, scaled by the predicted singularity / joint limit margins of a full-size step
    full = clip_step(step, target, lim.max_step_mm, lim.max_tangential_step_mm)
    yfull = (wrap_deg(yaw_at(target + full) - yaw["target"]) if follow
             else np.clip(ystep, -lim.max_yaw_step_deg, lim.max_yaw_step_deg))
    q_full = q + model.predict(q, lead_p + full, jac, lead_y + yfull)
    scale, slow = watch.speed_scale(q_full, model)
    flags.update(slow)
    limited = clip_step(step, target, lim.max_step_mm * scale, lim.max_tangential_step_mm * scale)
    flags["speed"] = np.linalg.norm(limited) < np.linalg.norm(step) - 1e-9
    if follow:
        ylimited = wrap_deg(yaw_at(target + limited) - yaw["target"])
    else:
        ylimited = float(np.clip(ystep, -lim.max_yaw_step_deg * scale, lim.max_yaw_step_deg * scale))
        flags["yspeed"] = abs(ylimited) < abs(ystep) - 1e-9

    # Per-joint speed limit on the combined motion; both parts are scaled by the same factor
    dq_step = np.abs(model.predict(q, lead_p + limited, jac, lead_y + ylimited))
    dq_lead = np.abs(model.predict(q, lead_p, jac, lead_y))
    over = (dq_step > lim.max_dq_deg) & (dq_step > dq_lead)
    if np.any(over):
        f = max(0.0, np.min((lim.max_dq_deg[over] - dq_lead[over]) / (dq_step[over] - dq_lead[over])))
        limited, ylimited = limited * f, ylimited * f
        flags["jspeed"] = True
    new = target + limited
    clamped = cyl.clamp(new)   # a straight step between two inside points can cut the inner radius
    flags["range"] = flags["range"] or bool(np.any(clamped != new))
    if np.linalg.norm(clamped - tcp_pos) > lim.max_lead_mm:
        clamped = target.copy()
        flags["lead"] = True
    if follow:
        ynew = yaw_at(clamped)   # exactly the azimuth of the new target: J1 turns, the wrist does not
    else:
        ynew = float(np.clip(yaw["target"] + ylimited, -lim.yaw_range_deg, lim.yaw_range_deg))
        if abs(ynew - yaw["tcp"]) > lim.max_yaw_lead_deg:
            ynew = yaw["target"]
            flags["ylead"] = True

    # Dropped part (the leader start is shifted by it)
    if follow:   # the yaw is not mapped from the leader: nothing to drop
        dropped = (step - limited if lim.max_lag_mm <= 0.0 else _lag_drop(desired_in - clamped, lim.max_lag_mm))
        ydropped = 0.0
    elif lim.max_lag_mm <= 0.0:
        dropped, ydropped = step - limited, ystep - ylimited
    else:
        lag = desired_in - clamped
        n = np.linalg.norm(lag)
        dropped = lag * (1.0 - lim.max_lag_mm / n) if n > lim.max_lag_mm else np.zeros(3)
        ylag = ydes_in - ynew
        ydropped = (float(np.sign(ylag) * (abs(ylag) - lim.max_yaw_lag_deg))
                    if abs(ylag) > lim.max_yaw_lag_deg else 0.0)
    lag_after = np.linalg.norm(desired_in - dropped - clamped)
    ylag_after = abs(wrap_deg(ydes_in - ydropped - ynew))

    dq = model.predict(q, clamped - tcp_pos, jac, ynew - yaw["tcp"])
    margins = watch.margins(q + dq, model)
    moving = not (np.array_equal(clamped, target) and ynew == yaw["target"])
    stop = watch.stop_reason(margins) if moving else None
    return SimpleNamespace(target=clamped, yaw=ynew, dropped=dropped, dropped_yaw=ydropped, flags=flags,
                           margins=margins[:4], max_dq=np.abs(dq).max(), lag=lag_after, lag_yaw=ylag_after,
                           stop=stop)


def to_cyl(p):
    """(angle [rad], radius, z) of a point about the z axis."""
    return np.arctan2(p[1], p[0]), np.hypot(p[0], p[1]), p[2]


def from_cyl(angle, radius, z):
    return np.array([radius * np.cos(angle), radius * np.sin(angle), z])


def _lag_drop(lag, max_lag):
    """C: the part of the lag beyond max_lag."""
    n = np.linalg.norm(lag)
    return lag * (1.0 - max_lag / n) if n > max_lag else np.zeros(3)


class Mapper:
    """Clutch-relative mapping state: the RB5 and leader references taken at each press, shifted by
    the dropped parts (B / C) so they are not caught up later."""

    def __init__(self, s, target0):
        self.scale = s.mapping.scale
        self.frame = rot_z(s.mapping.frame_yaw_deg)
        self.cylindrical = s.mapping.mode == "cylindrical"
        self.pivot = np.asarray(s.mapping.pivot_xy_mm, dtype=float)
        self.min_pivot_radius = s.mapping.min_pivot_radius_mm
        self.des = self.leader_prev = self.pending = None   # cylindrical: desired position and last leader used
        self.vertical, self.vertical_xy = False, None       # vertical mode: x, y held at vertical_xy
        self.vertical_z_scale = s.mapping.vertical_z_scale  # extra z gain in the vertical mode
        self.yaw_enabled = s.yaw.enabled
        self.target = np.array(target0, dtype=float)   # RB5 TCP position target [mm]
        self.yaw = 0.0                                  # RB5 yaw relative to the start rz [deg]
        self.rb5_start = self.leader_start = None
        self.yaw_rb5_start = self.yaw_leader_start = None

    def engage(self, leader_p_mm, leader_j6_deg):
        self.rb5_start, self.leader_start = self.target.copy(), np.array(leader_p_mm, dtype=float)
        self.yaw_rb5_start, self.yaw_leader_start = self.yaw, leader_j6_deg
        self.des, self.leader_prev, self.pending = self.target.copy(), self.leader_start.copy(), None
        self.vertical_xy = self.target[:2].copy()

    def _cylindrical_step(self, leader_p_mm):
        """Desired position after the leader moved from the last used reading to leader_p_mm."""
        p = np.asarray(leader_p_mm, dtype=float)
        (x0, y0), (x1, y1) = self.leader_prev[:2] - self.pivot, p[:2] - self.pivot
        a0, r0, a1, r1 = np.arctan2(y0, x0), np.hypot(x0, y0), np.arctan2(y1, x1), np.hypot(x1, y1)
        r_mid = 0.5 * (r0 + r1)
        d_angle = np.radians(wrap_deg(np.degrees(a1 - a0))) * r_mid / max(r_mid, self.min_pivot_radius)
        angle, radius, z = to_cyl(self.des)
        return from_cyl(angle + d_angle, max(radius + self.scale * (r1 - r0), 1.0),
                        z + self._z_scale() * (p[2] - self.leader_prev[2]))

    def _z_scale(self):
        return self.scale * (self.vertical_z_scale if self.vertical else 1.0)

    def desired(self, leader_p_mm, leader_j6_deg):
        """(desired position, desired yaw) for the current leader reading."""
        if self.cylindrical:
            pos = self._cylindrical_step(leader_p_mm)
        else:
            pos = self.rb5_start + self.scale * (self.frame @ (np.asarray(leader_p_mm) - self.leader_start))
        if self.vertical:
            pos[:2] = self.vertical_xy
            if not self.cylindrical:   # frame is a rotation about z, so z is not mixed with x, y
                pos[2] = self.rb5_start[2] + self._z_scale() * (leader_p_mm[2] - self.leader_start[2])
        if self.cylindrical:
            self.pending = (pos.copy(), np.array(leader_p_mm, dtype=float))   # taken by apply()
        yaw = (self.yaw_rb5_start + wrap_deg(leader_j6_deg - self.yaw_leader_start)) if self.yaw_enabled else 0.0
        return pos, yaw

    def apply(self, plan):
        """Take the planned target and shift the leader references by the dropped parts. A cycle that is
        not applied (late) leaves the cylindrical state as it was, so its motion is taken next cycle."""
        if self.cylindrical:
            pos, leader_p = self.pending
            self.des, self.leader_prev, self.pending = pos - plan.dropped, leader_p, None
        else:
            shift = self.frame.T @ plan.dropped / self.scale
            shift[2] *= self.scale / self._z_scale()
            self.leader_start = self.leader_start + shift
        self.target = np.array(plan.target, dtype=float)
        self.yaw_leader_start += plan.dropped_yaw
        self.yaw = plan.yaw

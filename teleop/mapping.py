"""Leader -> RB5 mapping: clutch-relative targets, per-cycle limits (B / C), position and yaw.

Started from scripts/21_leader_rb5_position.py (2026-10-12) and extended in real-1 (2026-10-16); every
change is in docs/decisions.md. A cylindrical mapping (a hand swing about a pivot turns J1) was tried and
removed (decisions 2026-10-16).

Position: desired = RB5 start + scale * Rz(frame_yaw) * (leader wrist center now - at press) [mm].
Left handle button held (mapping.left_button), every switch re-anchors the references (no jump):
  vertical: x and y of the desired position stay at the target's x and y when the mode or the clutch was
            switched on; only z follows, x scale x mapping.vertical_z_scale.
  rotate:   radius and z stay; only the azimuth about the base z axis changes, by
            mapping.rotate_deg_per_mm x the leader's sideways (leader y) motion: J1 turns, nothing else.
            The lag kept for catch-up is mapping.rotate_max_lag_deg (as an arc at the current radius), and
            the target is put back on the circle every cycle (the steps are straight).
A switch re-anchors at the point where the target would stop from its current step at the acceleration
limit (not at the target itself), so it slows down there instead of passing it and coming back.
Azimuth lock (mapping.lock_azimuth, without the left button): the azimuth stays; the leader's forward /
back motion (leader x) moves the target along the arm's current direction (radius) and up / down moves
z, both x scale; sideways motion is not used. So J1 turns only in the rotate mode.
Yaw:      desired = RB5 yaw at press + (leader J6 now - at press) [deg], about base z, TCP fixed.
Yaw mode follow_base (yaw.mode): the tool yaw follows the azimuth of the target about the base z axis,
so the gripper turns with the arm and the RB5 wrist joints (J6) stay still when J1 turns; the leader J6
is not used and the yaw speed / range limits do not apply (the position limits do).
Per cycle (plan_step): range first (the excess outside the range is kept), then the speed limits
scaled by the singularity / joint limit watches (sideways about the base z axis up to
speed_tangential_mm_s, radial and vertical up to speed_mm_s: (t / T)^2 + (o / O)^2 <= 1), then the
per-joint speed limit, then the acceleration limit as the last stage (mapping.accel_mm_s2: the step may
differ from the previous cycle's step by at most a x dt^2, and is not larger than the step from which it
can still stop at that acceleration), then the lead limit. The acceleration limit is last so that a
speed or joint limit that switches from one cycle to the next cannot make the step jump; while it slows
down, a cap can be exceeded for a few cycles (by less than the step change of those cycles).
B drops the cut part of the step; C keeps the lag up to max_lag and drops only the part beyond it.
"""
from types import SimpleNamespace

import numpy as np

from .kinematics import CylinderRange, clip_norm, rot_z, wrap_deg


class Limits:
    """Per-cycle limits derived from the settings."""

    def __init__(self, s):
        period = s.servo.period_s
        self.max_step_mm = s.mapping.speed_mm_s * period
        self.max_tangential_step_mm = s.mapping.speed_tangential_mm_s * period
        self.max_dstep_mm = s.mapping.accel_mm_s2 * period * period   # 0 = no acceleration limit
        self.max_lead_mm = s.mapping.lead_per_step * max(self.max_step_mm, self.max_tangential_step_mm)
        self.max_lag_mm = s.mapping.max_lag_mm
        self.max_yaw_step_deg = s.yaw.speed_deg_s * period
        self.yaw_range_deg = s.yaw.range_deg
        self.max_yaw_lead_deg = s.mapping.lead_per_step * self.max_yaw_step_deg
        # B for the position means B for the yaw too
        self.max_yaw_lag_deg = s.yaw.max_lag_deg if self.max_lag_mm > 0 else 0.0
        self.max_dq_deg = np.asarray(s.safety.joint_speed_deg_per_cycle, dtype=float)


CLAMP_FIT_ITERATIONS = 8   # bisection steps when the clamped step is over a limit (step kept to 1/256)


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


def plan_step(target, desired, tcp_pos, q, model, cyl, yaw, lim, watch, prev_step=None, max_lag_mm=None):
    """One cycle of the limits for the position and the yaw.

    yaw: dict with "target", "desired", "tcp" = yaw about base z relative to the start rz [deg], and
    "base_az0" = the start azimuth [deg] when the yaw follows the base (follow_base), else None.
    prev_step: the position step of the previous cycle [mm] (zero after a release or a frozen cycle).
    max_lag_mm: the C lag for this cycle instead of lim.max_lag_mm (rotate mode); B stays B.
    Returns a SimpleNamespace: target (new position), yaw (new yaw), dropped (position vector),
    dropped_yaw, flags, margins (J5, shoulder, elbow, joint limit), max_dq, lag, lag_yaw, stop.
    """
    flags = dict.fromkeys(("speed", "j5slow", "shslow", "elslow", "jlslow", "jspeed", "range", "lead",
                           "yspeed", "yrange", "ylead", "accel"), False)
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
    asked = step   # what the hand asked for this cycle (the cut parts below go to the lag / are dropped)

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

    # Per-joint speed limit on this cycle's step (position and yaw scaled by the same factor). The robot's
    # lag (lead_p) is not added: the reported state is about one cycle old, so at a steady speed the lag is
    # about one step that is already being executed, and adding it limited the joints at about half their
    # cap (050415: joint limit 34 % of the time, with J1 at most 56 of 75 deg/s). The pose checks above and
    # below still predict the pose after catching up.
    dq_step = np.abs(model.predict(q, limited, jac, ylimited))
    over = dq_step > lim.max_dq_deg
    if np.any(over):
        f = float(np.min(lim.max_dq_deg[over] / dq_step[over]))
        limited, ylimited = limited * f, ylimited * f
        flags["jspeed"] = True

    # Acceleration limit, last: change the step by at most max_dstep, and not faster than it can still stop
    if lim.max_dstep_mm > 0.0:
        prev = np.zeros(3) if prev_step is None else np.asarray(prev_step, dtype=float)
        a, d = lim.max_dstep_mm, np.linalg.norm(asked)
        want = clip_norm(limited, 0.5 * (-a + np.sqrt(a * a + 8.0 * a * d)))   # v + (v - a) + ... <= d: stops in time
        smooth = prev + clip_norm(want - prev, a)
        flags["accel"] = bool(np.linalg.norm(smooth - limited) > 1e-9)
        limited = smooth
        if follow:
            ylimited = wrap_deg(yaw_at(target + limited) - yaw["target"])
    new = target + limited
    clamped = cyl.clamp(new)   # a straight step between two inside points can cut the inner radius
    if np.any(clamped != new):
        flags["range"] = True
        # The clamp moves the target (outward on the inner radius) after the speed and joint speed limits, so
        # check the step it gives; if it is over, shorten this cycle's step until the clamped one fits.
        # (Sim 20261017_142123: along the sloped inner radius J2 reached 73 deg/s with the 60 deg/s cap.)
        def fits(f):
            c = cyl.clamp(target + f * limited)
            d = c - target
            yd = wrap_deg(yaw_at(c) - yaw["target"]) if follow else f * ylimited
            speed_ok = np.linalg.norm(clip_step(d, target, lim.max_step_mm * scale, lim.max_tangential_step_mm * scale)
                                      - d) < 1e-6
            return speed_ok and bool(np.all(np.abs(model.predict(q, d, jac, yd)) <= lim.max_dq_deg + 1e-9)), c
        ok, c = fits(1.0)
        if not ok:
            lo, hi, c = 0.0, 1.0, cyl.clamp(target)
            for _ in range(CLAMP_FIT_ITERATIONS):
                mid = 0.5 * (lo + hi)
                ok, c_mid = fits(mid)
                if ok:
                    lo, c = mid, c_mid
                else:
                    hi = mid
            limited, ylimited = limited * lo, ylimited * lo
            flags["jspeed"] = True
        clamped = c
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
    lag_limit = lim.max_lag_mm if max_lag_mm is None or lim.max_lag_mm <= 0.0 else max_lag_mm
    if follow:   # the yaw is not mapped from the leader: nothing to drop
        dropped = (asked - limited if lim.max_lag_mm <= 0.0 else _lag_drop(desired_in - clamped, lag_limit))
        ydropped = 0.0
    elif lim.max_lag_mm <= 0.0:
        dropped, ydropped = asked - limited, ystep - ylimited
    else:
        dropped = _lag_drop(desired_in - clamped, lag_limit)
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
        self.left_mode = s.mapping.left_button              # off / vertical / rotate
        self.left_active = False                            # left button held
        self.vertical_xy = self.rotate_anchor = self.last_desired = None
        self.vertical_z_scale = s.mapping.vertical_z_scale  # extra z gain in the vertical mode
        self.rotate_deg_per_mm = s.mapping.rotate_deg_per_mm
        self.rotate_max_lag_deg = s.mapping.rotate_max_lag_deg
        self.cyl = CylinderRange.from_settings(s.range)   # the rotate circle stays 1 mm outside r_min (no flag flicker)
        self.max_dstep = s.mapping.accel_mm_s2 * s.servo.period_s ** 2   # acceleration limit per cycle^2
        self.prev_step = np.zeros(3)   # position step applied in the last cycle (acceleration limit)
        self.lock_azimuth = s.mapping.lock_azimuth
        self.yaw_enabled = s.yaw.enabled
        self.target = np.array(target0, dtype=float)   # RB5 TCP position target [mm]
        self.yaw = 0.0                                  # RB5 yaw relative to the start rz [deg]
        self.rb5_start = self.leader_start = None
        self.yaw_rb5_start = self.yaw_leader_start = None

    def stop_point(self):
        """Where the target stops from its last step at the acceleration limit: v (v - a) / (2 a) further
        along the step (the target itself without a limit or when not moving)."""
        v = float(np.linalg.norm(self.prev_step))
        a = self.max_dstep
        if a <= 0.0 or v <= a:
            return self.target.copy()
        return self.target + self.prev_step / v * (v * (v - a) / (2.0 * a))

    def rotate_lag_mm(self):
        """The C lag in the rotate mode as an arc at the current radius [mm]."""
        return np.hypot(self.target[0], self.target[1]) * np.radians(self.rotate_max_lag_deg)

    def engage(self, leader_p_mm, leader_j6_deg):
        stop = self.stop_point()
        self.rb5_start, self.leader_start = stop.copy(), np.array(leader_p_mm, dtype=float)
        self.yaw_rb5_start, self.yaw_leader_start = self.yaw, leader_j6_deg
        self.vertical_xy = stop[:2].copy()
        a = np.arctan2(stop[1], stop[0])
        _, r, z = to_cyl(self.target)
        r = max(r, self.cyl.r_min_at(z) + 1.0)   # a circle on r_min itself: every straight step cut inside it (flag flicker)
        self.rotate_anchor = (np.degrees(a), r, z)   # the radius and z of the target, the azimuth where it stops

    @property
    def vertical(self):
        return self.left_active and self.left_mode == "vertical"

    @property
    def rotating(self):
        return self.left_active and self.left_mode == "rotate"

    @property
    def locked(self):
        """Azimuth locked: lock_azimuth and no left-button mode active."""
        return self.lock_azimuth and not self.left_active

    def _z_scale(self):
        return self.scale * (self.vertical_z_scale if self.vertical else 1.0)

    def desired(self, leader_p_mm, leader_j6_deg):
        """(desired position, desired yaw) for the current leader reading."""
        pos = self.rb5_start + self.scale * (self.frame @ (np.asarray(leader_p_mm) - self.leader_start))
        if self.vertical:   # frame is a rotation about z, so z is not mixed with x, y
            pos[:2] = self.vertical_xy
            pos[2] = self.rb5_start[2] + self._z_scale() * (leader_p_mm[2] - self.leader_start[2])
        elif self.rotating:
            az, r, z = self.rotate_anchor
            az = az + self.rotate_deg_per_mm * (leader_p_mm[1] - self.leader_start[1])
            pos = from_cyl(np.radians(az), r, z)
        elif self.locked:
            az, r, z = self.rotate_anchor
            d = np.asarray(leader_p_mm) - self.leader_start
            pos = from_cyl(np.radians(az), max(r + self.scale * d[0], 1.0), z + self.scale * d[2])
        self.last_desired = pos.copy()
        yaw = (self.yaw_rb5_start + wrap_deg(leader_j6_deg - self.yaw_leader_start)) if self.yaw_enabled else 0.0
        return pos, yaw

    def apply(self, plan):
        """Take the planned target and shift the leader references by the dropped parts."""
        if self.rotating:   # the dropped part is an arc piece: shift the leader y by its angle
            p = self.last_desired
            d_az = wrap_deg(np.degrees(np.arctan2(p[1], p[0]))
                            - np.degrees(np.arctan2(p[1] - plan.dropped[1], p[0] - plan.dropped[0])))
            self.leader_start = self.leader_start + np.array([0.0, d_az / self.rotate_deg_per_mm, 0.0])
        elif self.locked:   # radial and z parts of the dropped vector back to leader x and z
            a = np.arctan2(self.last_desired[1], self.last_desired[0])
            self.leader_start = self.leader_start + np.array(
                [(np.cos(a) * plan.dropped[0] + np.sin(a) * plan.dropped[1]) / self.scale, 0.0,
                 plan.dropped[2] / self.scale])
        else:
            shift = self.frame.T @ plan.dropped / self.scale
            shift[2] *= self.scale / self._z_scale()
            self.leader_start = self.leader_start + shift
        new = np.array(plan.target, dtype=float)
        if self.rotating:   # straight steps cut inside the circle: put the target back on it
            _, r, z = self.rotate_anchor
            new = from_cyl(np.arctan2(new[1], new[0]), r, z)
        self.prev_step = new - self.target
        self.target = new
        self.yaw_leader_start += plan.dropped_yaw
        self.yaw = plan.yaw

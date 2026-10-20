"""Absolute TCP position targets -> the RB5 through the teleop's per-cycle limits (rec-4b).

The teleop turns leader motion into a desired position and passes it through plan_step (range, speed
caps with the singularity / joint limit watches, per-joint speed, acceleration, lead) before
move_servo_l. A dataset action or, later, a policy output is already an absolute desired position
(x, y, z in the base frame, rec-1), so TargetFollower is the same pipeline without the leader and the
clutch. Yaw: follow_base only (the tool turns with the azimuth, as in the recordings).
"""
import numpy as np

from .kinematics import CylinderRange, wrap_deg
from .mapping import Limits, plan_step
from .safety import Watch


class TargetFollower:
    def __init__(self, s, model, tcp0):
        if s.yaw.mode != "follow_base":
            raise ValueError(f"TargetFollower needs yaw.mode follow_base (the settings have {s.yaw.mode})")
        self.model = model
        self.lim, self.watch = Limits(s), Watch(s.safety)
        self.cyl = CylinderRange.from_settings(s.range)
        tcp0 = np.asarray(tcp0, dtype=float)
        self.target = tcp0[:3].copy()          # what is being sent [mm]
        self.rpy0 = tcp0[3:].copy()
        self.rpy = self.rpy0.copy()
        self.az0 = float(np.degrees(np.arctan2(tcp0[1], tcp0[0])))
        self.yaw = 0.0                          # tool yaw relative to the start rz [deg]
        self.prev_step = np.zeros(3)

    def plan(self, desired, tcp, q):
        """One cycle toward the desired position, not applied yet: the plan (target, yaw, flags, margins,
        stop, ...). The caller applies it with apply() unless the cycle is late or plan.stop is set."""
        yaw_tcp = wrap_deg(tcp[5] - self.rpy0[2])
        return plan_step(self.target, np.asarray(desired, dtype=float), np.asarray(tcp[:3], dtype=float), q,
                         self.model, self.cyl,
                         {"target": self.yaw, "desired": 0.0, "tcp": yaw_tcp, "base_az0": self.az0},
                         self.lim, self.watch, self.prev_step)

    def apply(self, plan):
        new = np.array(plan.target, dtype=float)
        self.prev_step = new - self.target
        self.target = new
        self.yaw = plan.yaw
        self.rpy[2] = wrap_deg(self.rpy0[2] + self.yaw)

    def step(self, desired, tcp, q):
        """plan() and apply() in one (applied unless plan.stop is set; then the caller stops)."""
        plan = self.plan(desired, tcp, q)
        if not plan.stop:
            self.apply(plan)
        return plan

    def hold(self):
        """A cycle in which the target did not move (late or stale state)."""
        self.prev_step = np.zeros(3)

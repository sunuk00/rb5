"""Singularity and joint limit watches (on predicted joint angles) and the start conditions.

Moved from scripts/21_leader_rb5_position.py; thresholds come from the settings `safety` section.
The three RB5 singularities (decisions 2026-10-12): wrist (J5 near 0 / 180), shoulder (wrist point
P05 on the radius-d4 cylinder about J1), elbow (J3 near 0 / 180). Below "slow" the speed limit falls
linearly to min_speed_scale; below "stop" the step is not sent.
"""
import numpy as np


def singular_distance(angle_deg):
    """Distance of a joint angle from 0 and +/-180 deg (J5: wrist singularity, J3: elbow straight)."""
    return np.degrees(np.arcsin(min(abs(np.sin(np.radians(angle_deg))), 1.0)))


def ramp_scale(value, slow, stop, min_scale):
    """1.0 at or above slow, falling linearly to min_scale at stop (and below)."""
    if value >= slow:
        return 1.0
    if value <= stop:
        return min_scale
    return min_scale + (1.0 - min_scale) * (value - stop) / (slow - stop)


def joint_limit_margin(q, limits_deg):
    """Smallest margin to the joint limits [deg] and the joint number (1-6) that has it."""
    margin = np.asarray(limits_deg, dtype=float) - np.abs(q)
    i = int(np.argmin(margin))
    return margin[i], i + 1


class Watch:
    """All four watches with the thresholds of one settings `safety` section."""

    def __init__(self, safety):
        self.s = safety

    def margins(self, q, model):
        """(J5 distance from singular [deg], shoulder margin [mm], J3 distance from straight [deg],
        joint limit margin [deg], joint number of that margin)."""
        jl, joint = joint_limit_margin(q, self.s.joint_limit_deg)
        return singular_distance(q[4]), model.shoulder_margin(q), singular_distance(q[2]), jl, joint

    def speed_scale(self, q, model):
        """Speed scale for the predicted q and the slow flags {j5slow, shslow, elslow, jlslow}."""
        s = self.s
        j5, sh, el, jl, _ = self.margins(q, model)
        scales = {"j5slow": ramp_scale(j5, s.j5_slow_deg, s.j5_stop_deg, s.min_speed_scale),
                  "shslow": ramp_scale(sh, s.shoulder_slow_mm, s.shoulder_stop_mm, s.min_speed_scale),
                  "elslow": ramp_scale(el, s.elbow_slow_deg, s.elbow_stop_deg, s.min_speed_scale),
                  "jlslow": ramp_scale(jl, s.joint_limit_slow_deg, s.joint_limit_stop_deg, s.min_speed_scale)}
        return min(scales.values()), {k: v < 1.0 for k, v in scales.items()}

    def stop_reason(self, margins):
        """Text if any margin is below its stop threshold, else None. Same order as 21."""
        s = self.s
        j5, sh, el, jl, joint = margins
        if j5 < s.j5_stop_deg:
            return f"predicted J5 {j5:.1f} deg (from the singular angle) < {s.j5_stop_deg} deg"
        if sh < s.shoulder_stop_mm:
            return f"predicted shoulder margin {sh:.1f} mm < {s.shoulder_stop_mm} mm"
        if el < s.elbow_stop_deg:
            return f"predicted J3 {el:.1f} deg from straight < {s.elbow_stop_deg} deg"
        if jl < s.joint_limit_stop_deg:
            return f"predicted J{joint} {jl:.1f} deg from its limit < {s.joint_limit_stop_deg} deg"
        return None

    def start_problems(self, q0, model):
        """Problems with the start pose: every margin must be at its slow threshold or above."""
        s = self.s
        j5, sh, el, jl, joint = self.margins(q0, model)
        problems = []
        if jl < s.joint_limit_slow_deg:
            problems.append(f"J{joint} is {jl:.1f} deg from its limit (need >= {s.joint_limit_slow_deg:.0f})")
        if sh < s.shoulder_slow_mm:
            problems.append(f"shoulder margin is {sh:.1f} mm (need >= {s.shoulder_slow_mm:.0f})")
        if el < s.elbow_slow_deg:
            problems.append(f"J3 is {el:.1f} deg from straight (need >= {s.elbow_slow_deg:.0f})")
        if j5 < s.j5_slow_deg:
            problems.append(f"J5 is {j5:.1f} deg from the singular angle (need >= {s.j5_slow_deg:.0f})")
        return problems

"""The teleop cycle: leader read -> clutch / gripper -> RB5 state checks -> plan -> late check -> send.

Moved from scripts/21_leader_rb5_position.py (run_loop) without changes in behavior. Devices are
passed in, so the tests can use fakes:
  leader:  read_joints_rad() -> 7 angles [rad] or None, read_button_raw() -> int or None
  leader_model: wrist_center_mm(q_rad)
  rb5:     check_errors(), send_servo(xyz, rpy)
  reader:  latest() -> (tcp pose, joints [deg], age [s]); optional measured() -> encoder joints [deg]
           (logged with their DH-model TCP to see how the real arm follows the commands)
"""
import sys
import time

import numpy as np

from .kinematics import CylinderRange, wrap_deg
from .leader import Clutch, GripSignal
from .mapping import Limits, Mapper, plan_step
from .safety import Watch, joint_limit_margin, singular_distance

STATUS_EVERY = 25
CLEAR_LINE = "\r\x1b[2K"

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
              + ["trigger_raw", "grip_close", "grip_state_closed", "tool_read_failed"]
              + [f"jnt_ang_{i}" for i in range(6)] + [f"meas_tcp_{a}" for a in "xyz"] + ["target_minus_meas_mm"])


class StopRequest(Exception):
    """Controlled stop: end servo mode normally (hold target, move_speed_j(0), poll)."""


def print_event(msg):
    sys.stdout.write(CLEAR_LINE + msg + "\n")
    sys.stdout.flush()


class TeleopLoop:
    def __init__(self, s, leader, leader_model, rb5, reader, model, event=print_event, status=True, sleep=time.sleep):
        self.s, self.leader, self.leader_model = s, leader, leader_model
        self.rb5, self.reader, self.model = rb5, reader, model
        self.event, self.status, self.sleep = event, status, sleep
        self.lim, self.watch = Limits(s), Watch(s.safety)
        self.rows = []
        self.target = self.rpy = None          # what is being sent (for the normal stop)
        self.clutch = Clutch()
        self.grip = GripSignal(s.gripper)

    def _measured_columns(self, q_meas, target):
        """Encoder joints, their TCP from the DH model (absolute about 2 mm off the controller, 17) and the
        distance of the target from it; blanks when the reader has no measured joints."""
        if q_meas is None:
            return [""] * 10
        p = self.model.tcp(q_meas)[0]
        return ([f"{v:.3f}" for v in q_meas] + [f"{v:.3f}" for v in p]
                + [f"{np.linalg.norm(np.asarray(target) - p):.3f}"])

    def labels(self):
        s, lim = self.s, self.lim
        return {"speed": f"speed limit ({lim.max_step_mm} mm/cycle)",
                "j5slow": f"J5 below {s.safety.j5_slow_deg:.0f} deg: slower",
                "shslow": f"shoulder margin below {s.safety.shoulder_slow_mm:.0f} mm: slower",
                "elslow": f"J3 within {s.safety.elbow_slow_deg:.0f} deg of straight: slower",
                "jlslow": f"a joint within {s.safety.joint_limit_slow_deg:.0f} deg of its limit: slower",
                "jspeed": f"joint speed limit ({lim.max_dq_deg.tolist()} deg/cycle predicted)",
                "range": f"range limit (cylinder r {s.range.r_min_mm:.0f}..{s.range.r_max_mm:.0f} mm, "
                         f"z -{s.range.z_below_mm:.0f} / +{s.range.z_above_mm:.0f} mm"
                         f"{'' if s.range.r_max_by_z is None else ', r_max falls with height'}, "
                         f"azimuth +/-{s.range.azimuth_half_deg:.0f} deg)",
                "lead": f"target waits for robot (lead limit {lim.max_lead_mm} mm)",
                "yspeed": f"yaw speed limit ({lim.max_yaw_step_deg} deg/cycle)",
                "yrange": f"yaw range limit (+/-{lim.yaw_range_deg:.0f} deg)",
                "ylead": f"yaw waits for robot (lead limit {lim.max_yaw_lead_deg} deg)"}

    def run(self):
        s, sf = self.s, self.s.safety
        period = s.servo.period_s
        tcp0, _, _ = self.reader.latest()
        rpy0 = tcp0[3:].copy()
        self.rpy = rpy0.copy()
        mapper = Mapper(s, tcp0[:3])
        self.target = mapper.target
        cyl = CylinderRange.from_settings(tcp0[:3], s.range)
        labels = self.labels()
        clutch, grip = self.clutch, self.grip
        leader_q = leader_p = None
        leader_fails = leader_fail_total = leader_reads = 0
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

            # 1. Leader arm joints -> wrist center, then the tool (button + trigger)
            t_leader0 = time.monotonic()
            leader_reads += 1
            q_l = self.leader.read_joints_rad()
            if q_l is None:
                leader_fails += 1
                leader_fail_total += 1
                if leader_fails >= sf.max_leader_fails:
                    raise StopRequest(f"{sf.max_leader_fails} leader joint read failures in a row")
            else:
                leader_fails = 0
                leader_q, leader_p = q_l, self.leader_model.wrist_center_mm(q_l)
            button_raw = self.leader.read_button_raw()
            t_leader = time.monotonic() - t_leader0
            grip_event = grip.update(button_raw)   # independent of the clutch
            if grip_event:
                self.event(f"[{time.monotonic() - t_start:7.2f} s] {grip_event}")

            # 2. Clutch
            button, clutch_event = clutch.update(button_raw)
            if clutch_event == "engaged":
                if leader_p is None:
                    clutch.engaged = False
                    clutch_event = "press ignored (no leader reading yet)"
                else:
                    mapper.engage(leader_p, np.degrees(leader_q[6]))
                    clutch_event = (f"engaged: RB5 start {np.round(mapper.rb5_start, 1)} mm, "
                                    f"leader start {np.round(mapper.leader_start, 1)} mm")
            if clutch_event:
                self.event(f"[{time.monotonic() - t_start:7.2f} s] clutch {clutch_event}")

            # 3. RB5 state and the 08 safety checks
            self.rb5.check_errors()
            tcp, q, state_age = self.reader.latest()
            q_meas = self.reader.measured() if hasattr(self.reader, "measured") else None
            now = time.monotonic()
            late_early = last_send is not None and now - last_send > sf.max_gap_s   # already late before planning
            stale = state_age > sf.max_state_age_s
            stale_cycles = stale_cycles + 1 if stale else 0
            if stale_cycles >= sf.max_stale_cycles:
                raise RuntimeError(f"{sf.max_stale_cycles} stale-state cycles in a row; state reader stopped?")
            if stale:
                self.event(f"stale RB5 state: {state_age * 1000:.0f} ms old, target frozen")

            # 4. Plan: follow the leader only while engaged and every check passed. The new target and
            #    the B / C shift are applied only after the gap check right before sending.
            t_plan0 = time.monotonic()
            desired = plan = yaw_desired = None
            flags = {}
            j5_pred, sh_pred, el_pred = singular_distance(q[4]), self.model.shoulder_margin(q), singular_distance(q[2])
            jl_pred = joint_limit_margin(q, sf.joint_limit_deg)[0]
            max_dq = lag = yaw_lag = 0.0
            dropped = np.zeros(3)
            yaw_tcp = wrap_deg(tcp[5] - rpy0[2])
            if clutch.engaged and q_l is not None and not late_early and not stale:
                desired, yaw_desired = mapper.desired(leader_p, np.degrees(leader_q[6]))
                plan = plan_step(mapper.target, desired, tcp[:3], q, self.model, cyl,
                                 {"target": mapper.yaw, "desired": yaw_desired, "tcp": yaw_tcp}, self.lim, self.watch)
                dropped, flags = plan.dropped, plan.flags
                j5_pred, sh_pred, el_pred, jl_pred = plan.margins
                max_dq, lag, yaw_lag = plan.max_dq, plan.lag, plan.lag_yaw
                if plan.stop:
                    raise StopRequest(plan.stop)
            t_plan = time.monotonic() - t_plan0

            # 5. Late check on the real send gap, then apply the new target
            gap = None if last_send is None else time.monotonic() - last_send
            late = late_early or (gap is not None and gap > sf.max_gap_s)
            late_cycles = late_cycles + 1 if late else 0
            if late_cycles >= sf.max_late_cycles:
                raise RuntimeError(f"{sf.max_late_cycles} late cycles in a row; loop too slow for servo")
            if late:
                self.event(f"[{time.monotonic() - t_start:7.2f} s] late: {gap * 1000:.0f} ms since last send, target frozen")
            elif plan is not None:
                mapper.apply(plan)
                self.rpy[2] = wrap_deg(rpy0[2] + mapper.yaw)
            self.target = mapper.target
            for key, on in flags.items():
                if on and not flags_prev.get(key, False):
                    self.event(f"[{time.monotonic() - t_start:7.2f} s] {labels[key]}")
            flags_prev = flags
            if flags.get("j5slow") and not warned["j5slow"]:
                self.event(f"WARNING: J5 {j5_pred:.1f} deg from the singular angle (stop below {sf.j5_stop_deg:.0f})")
            if flags.get("shslow") and not warned["shslow"]:
                self.event(f"WARNING: shoulder margin {sh_pred:.1f} mm (stop below {sf.shoulder_stop_mm:.0f})")
            if flags.get("elslow") and not warned["elslow"]:
                self.event(f"WARNING: J3 {el_pred:.1f} deg from straight (stop below {sf.elbow_stop_deg:.0f})")
            if flags.get("jlslow") and not warned["jlslow"]:
                self.event(f"WARNING: joint limit margin {jl_pred:.1f} deg (stop below {sf.joint_limit_stop_deg:.0f})")
            warned = {k: flags.get(k, False) for k in warned}

            # 6. Send every cycle (a released clutch sends the frozen target)
            t_send0 = time.monotonic()
            self.rb5.send_servo(mapper.target, self.rpy)
            t_send_end = time.monotonic()
            send_interval = None if last_send is None else t_send0 - last_send
            last_send = t_send0

            f = lambda v, fmt="{:.3f}": [fmt.format(x) for x in v]   # noqa: E731
            empty3 = [""] * 3
            self.rows.append(
                [f"{t_cycle - t_start:.4f}", "" if interval is None else f"{interval * 1000:.3f}",
                 "" if button_raw is None else button_raw, int(clutch.engaged)]
                + (f(np.degrees(leader_q)) if leader_q is not None else [""] * 7)
                + (f(leader_p) if leader_p is not None else empty3)
                + (f(desired) if desired is not None else empty3) + f(mapper.target) + f(tcp[:3]) + f(q)
                + [f"{j5_pred:.2f}", f"{sh_pred:.1f}", f"{max_dq:.3f}"]
                + [int(flags.get(k, False)) for k in ("speed", "j5slow", "shslow", "jspeed", "range", "lead")]
                + [int(late), int(late_early), int(stale), f"{state_age * 1000:.2f}"]
                + [f"{t_leader * 1000:.2f}", f"{t_plan * 1000:.2f}", f"{(t_send_end - t_send0) * 1000:.2f}",
                   f"{(t_send_end - t_cycle) * 1000:.2f}", "" if send_interval is None else f"{send_interval * 1000:.2f}"]
                + [f"{el_pred:.2f}", int(flags.get("elslow", False)), f"{lag:.3f}",
                   f"{np.linalg.norm(dropped) if not late else 0.0:.3f}"]
                + [f"{jl_pred:.2f}", int(flags.get("jlslow", False))]
                + ["" if yaw_desired is None else f"{yaw_desired:.3f}", f"{mapper.yaw:.3f}", f"{yaw_tcp:.3f}"]
                + [int(flags.get(k, False)) for k in ("yspeed", "yrange", "ylead")]
                + [f"{yaw_lag:.3f}", f"{abs(plan.dropped_yaw) if plan is not None and not late else 0.0:.3f}"]
                + ["" if grip.trigger is None else grip.trigger, "" if grip.close is None else f"{grip.close:.3f}",
                   "" if grip.closed is None else int(grip.closed), int(button_raw is None)]
                + self._measured_columns(q_meas, mapper.target))

            # 7. Status line
            if self.status and cycle % STATUS_EVERY == 0:
                tool_rate = 100.0 * clutch.failures / max(clutch.reads, 1)
                leader_rate = 100.0 * leader_fail_total / max(leader_reads, 1)
                sys.stdout.write(
                    f"{CLEAR_LINE}clutch {'ENGAGED ' if clutch.engaged else 'released'}  "
                    f"target-start {np.round(mapper.target - tcp0[:3], 1)} mm  yaw {mapper.yaw:6.1f}  "
                    f"J5 {singular_distance(q[4]):5.1f}  shoulder {self.model.shoulder_margin(q):5.0f} mm  "
                    f"grip {'-' if grip.close is None else f'{grip.close:.2f}'}"
                    f"{'' if grip.closed is None else (' CLOSED' if grip.closed else ' OPEN')}  |  "
                    f"button fail {tool_rate:4.1f}%  leader fail {leader_rate:4.1f}%")
                sys.stdout.flush()

            # 8. Sleep until the next cycle; after an overrun wait a full period (08, as in 13)
            cycle += 1
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                self.sleep(delay)
            else:
                next_t = time.monotonic() + period
                self.sleep(period)

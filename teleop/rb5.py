"""RB5 interface for the teleop: connection, operation mode, state reader, servo commands, stops and
the ready-pose move.

rbpodo functions used (checked in refs/rbpodo and the earlier scripts): set_operation_mode,
set_speed_bar, get_robot_state, flush, enable/disable_waiting_ack, move_servo_l, move_speed_j,
move_j, wait_for_move_started, task_stop, get_tcp_info, get_tfc_info, calc_fk_tcp,
CobotData.request_data (sdata.tcp_ref, jnt_ref, real_vs_simulation_mode).

State: the reference values (tcp_ref, jnt_ref) are used in both modes. In Simulation jnt_ang does not
follow the motion; tcp_pos is documented as overwritten from the reference (17).
"""
import threading
import time

import numpy as np

from .kinematics import tcp_offset_from

TIMEOUT_S = 1.0


class StateReader(threading.Thread):
    """Reads the data channel continuously and keeps the latest tcp_ref / jnt_ref (08, 21)."""

    def __init__(self, data_channel_factory):
        super().__init__(daemon=True)
        self.data_channel = data_channel_factory()
        self.lock = threading.Lock()
        self.sample = None
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            data = self.data_channel.request_data(TIMEOUT_S)
            if data is None:
                continue
            s = data.sdata
            sample = (np.array(s.tcp_ref, dtype=float), np.array(s.jnt_ref, dtype=float), time.monotonic())
            with self.lock:
                self.sample = sample

    def wait_first(self, timeout_s=2.0):
        t0 = time.monotonic()
        while self.sample is None:
            if time.monotonic() - t0 > timeout_s:
                raise RuntimeError("state reader got no data from the robot")
            time.sleep(0.01)

    def latest(self):
        """(tcp pose [x, y, z, rx, ry, rz], joints [deg], age [s])."""
        with self.lock:
            tcp, q, t = self.sample
        return tcp.copy(), q.copy(), time.monotonic() - t

    def stop(self):
        self.stop_event.set()
        self.join(2.0)


class RB5:
    """Thin wrapper over rbpodo.Cobot / CobotData with the calls the teleop needs."""

    def __init__(self, ip, servo):
        import rbpodo as rb   # imported here so tests need no rbpodo
        self.rb = rb
        self.ip = ip
        self.servo_args = (servo.t1_s, servo.t2_s, servo.gain, servo.alpha)
        self.robot = rb.Cobot(ip)
        self.rc = rb.ResponseCollector()
        self.data = rb.CobotData(ip)

    # ---- setup and checks (waiting for ACKs)
    def set_mode(self, real, speed_bar):
        rb = self.rb
        self.robot.set_operation_mode(self.rc, rb.OperationMode.Real if real else rb.OperationMode.Simulation)
        self.robot.set_speed_bar(self.rc, speed_bar)
        self.rc.error().throw_if_not_empty()
        self.rc.clear()

    def sdata(self):
        d = self.data.request_data(TIMEOUT_S)
        if d is None:
            raise RuntimeError("request_data() timed out")
        return d.sdata

    def reports_simulation(self):
        return self.sdata().real_vs_simulation_mode == 1

    def wait_for_mode(self, real, timeout_s=3.0):
        """After set_mode: poll the data channel until it reports the requested mode (the flag may follow
        the command with a delay). Returns (reached, seconds waited)."""
        t0 = time.monotonic()
        while True:
            if self.reports_simulation() != real:
                return True, time.monotonic() - t0
            if time.monotonic() - t0 > timeout_s:
                return False, time.monotonic() - t0
            time.sleep(0.05)

    def is_moving(self):
        return self.robot.get_robot_state(self.rc)[1] == self.rb.RobotState.Moving

    def error_text(self):
        """Controller error messages collected so far ('' if none); clears them."""
        try:
            self.rc.error().throw_if_not_empty()
            return ""
        except Exception as e:
            return str(e)
        finally:
            self.rc.clear()

    def tcp_offset(self):
        """(TCP offset in the flange frame [mm], orientation offset [deg]) from get_tcp_info / get_tfc_info."""
        res_t, tcp = self.robot.get_tcp_info(self.rc, timeout=TIMEOUT_S)
        res_f, tfc = self.robot.get_tfc_info(self.rc, timeout=TIMEOUT_S)
        if not (res_t.is_success() and res_f.is_success()):
            raise RuntimeError("get_tcp_info / get_tfc_info failed")
        return tcp_offset_from(np.asarray(tcp).reshape(6), np.asarray(tfc).reshape(6))

    def fk_tcp(self, q):
        res, pnt = self.robot.calc_fk_tcp(self.rc, j0=q[0], j1=q[1], j2=q[2], j3=q[3], j4=q[4], j5=q[5],
                                          timeout=TIMEOUT_S)
        return np.asarray(pnt, dtype=float).reshape(6) if res.is_success() else None

    def wait_until_stopped(self, timeout_s):
        t0 = time.monotonic()
        while self.is_moving():
            if time.monotonic() - t0 > timeout_s:
                return False
            time.sleep(0.01)
        return True

    def move_j(self, q, speed, acc, timeout_s):
        """move_j and poll until it stops (19). Returns (finished, seconds). task_stop on timeout."""
        self.robot.flush(self.rc)
        self.robot.move_j(self.rc, np.asarray(q, dtype=float), speed, acc)
        t0 = time.monotonic()
        if self.robot.wait_for_move_started(self.rc, 0.5).type() != self.rb.ReturnType.Success:
            return True, 0.0
        if not self.wait_until_stopped(timeout_s):
            self.robot.task_stop(self.rc)
            self.wait_until_stopped(10.0)
            return False, time.monotonic() - t0
        return True, time.monotonic() - t0

    def state_reader(self):
        return StateReader(lambda: self.rb.CobotData(self.ip))

    # ---- streaming (no ACK waits)
    def begin_streaming(self):
        self.robot.disable_waiting_ack(self.rc)

    def check_errors(self):
        """Raise if the controller reported an error since the last cycle (13, 21)."""
        self.robot.flush(self.rc)
        self.rc.error().throw_if_not_empty()
        self.rc.clear()

    def send_servo(self, xyz, rpy):
        self.robot.move_servo_l(self.rc, np.r_[xyz, rpy], *self.servo_args)

    def end_streaming(self, xyz, rpy, hold_s, period_s):
        """Normal end of servo mode: hold the target, then move_speed_j(0) and poll (13, 21)."""
        next_t = time.monotonic()
        t_end = next_t + hold_s
        while time.monotonic() < t_end:
            self.send_servo(xyz, rpy)
            next_t += period_s
            time.sleep(max(0.0, next_t - time.monotonic()))
        self.robot.move_speed_j(self.rc, np.zeros(6), *self.servo_args)
        self.robot.enable_waiting_ack(self.rc)
        self.robot.flush(self.rc)
        self.wait_until_stopped(10.0)
        self.rc.clear()

    def emergency_stop(self):
        """End servo mode after an error: no more servo commands, task_stop and poll (13, 21)."""
        self.robot.enable_waiting_ack(self.rc)
        self.robot.task_stop(self.rc)
        self.robot.flush(self.rc)
        self.wait_until_stopped(10.0)
        self.rc.clear()

"""Fake-device tests of the teleop loop (no hardware). Run: python -m unittest discover tests

The fake RB5 follows the full pose target (position and orientation) with the DH model; the fake
leader plays a script of (button, wrist center [mm], J6 [deg], trigger or None for a failed read).
"""
import copy
import importlib.util
import sys
import time
import types
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from teleop.kinematics import RB5Model, rot_zyx, zyx_from_rot  # noqa: E402
from teleop.loop import CSV_HEADER, StopRequest, TeleopLoop  # noqa: E402
from teleop.settings import load_settings  # noqa: E402

from fixtures import test_raw  # noqa: E402

RAW = test_raw()   # pose D era values, same as scripts/21
OFFSET = [0.0, -187.0, -40.0]
D = np.array([182.0, -11.0, -110.1, 31.1, -90.0, -2.0])   # pose D (tests are written for it)
IX = {h: i for i, h in enumerate(CSV_HEADER)}


def settings(**changes):
    """sim.yaml with changes given as section__key=value."""
    d = copy.deepcopy(RAW)
    for k, v in changes.items():
        sec, key = k.split("__")
        d[sec][key] = v
    return _ns(d)


def _ns(d):
    if isinstance(d, dict):
        return types.SimpleNamespace(**{k: _ns(v) for k, v in d.items()})
    return tuple(d) if isinstance(d, list) else d


class FakeRB5:
    def __init__(self, q, model):
        self.q, self.model = np.array(q, dtype=float), model

    def pose(self):
        p, r = self.model.tcp(self.q)
        return np.r_[p, zyx_from_rot(r)]

    def check_errors(self):
        pass

    def send_servo(self, xyz, rpy):
        rt = rot_zyx(*rpy)
        for _ in range(3):
            p, r = self.model.tcp(self.q)
            dr = rt @ r.T
            w = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2
            self.q += np.degrees(np.linalg.solve(self.model.jacobian(self.q), np.r_[np.asarray(xyz) - p, w]))


class FakeReader:
    def __init__(self, robot):
        self.robot = robot

    def latest(self):
        return self.robot.pose(), self.robot.q.copy(), 0.001


class ScriptLeader:
    """Steps: (button, wrist [mm], J6 [deg], trigger or None = failed tool read, joints ok)."""

    def __init__(self, steps, slow_at=None):
        self.steps, self.i, self.slow_at = steps, -1, slow_at

    def step(self):
        b, p, j6, *rest = self.steps[self.i]
        trig = rest[0] if rest else 1000          # released trigger
        ok = rest[1] if len(rest) > 1 else True
        return b, p, j6, trig, ok

    def read_joints_rad(self):
        self.i += 1
        if self.i >= len(self.steps):
            raise StopRequest("script end")
        if self.slow_at is not None and self.i == self.slow_at:
            time.sleep(0.05)
        _, _, j6, _, ok = self.step()
        if not ok:
            return None
        q = np.zeros(7)
        q[6] = np.radians(j6)
        return q

    def read_button_raw(self):
        b, _, _, trig, _ = self.step()
        return None if trig is None else (int(trig) << 16) | (0x100 if b else 0)


class ScriptModel:
    def __init__(self, leader):
        self.leader = leader

    def wrist_center_mm(self, q):
        return np.array(self.leader.step()[1], dtype=float)


def run(steps, s=None, q0=D, slow_at=None):
    """Run the loop on a script. Returns (loop, start pose, end pose, column getter, stop text)."""
    s = s or settings()
    model = RB5Model(OFFSET)
    robot = FakeRB5(q0, model)
    leader = ScriptLeader(steps, slow_at)
    p0 = robot.pose()
    loop = TeleopLoop(s, leader, ScriptModel(leader), robot, FakeReader(robot), model,
                      event=lambda m: None, status=False, sleep=lambda t: None)
    stop = None
    try:
        loop.run()
    except StopRequest as e:
        stop = str(e)
    col = lambda k: np.array([float(r[IX[k]]) if r[IX[k]] != "" else np.nan for r in loop.rows])   # noqa: E731
    return loop, p0, robot.pose(), col, stop


ARM = [(0, [0, 0, 0], 0)] * 3 + [(1, [0, 0, 0], 0)] * 3   # release once, then press (engaged at row 4)


class TestLoop(unittest.TestCase):
    def test_slow_follow_scaled(self):
        steps = ARM + [(1, [1.0 * i, 0, 0], 0) for i in range(1, 101)] + [(1, [100, 0, 0], 0)] * 20
        _, p0, p, col, _ = run(steps)
        np.testing.assert_allclose(col("target_x")[-1] - p0[0], 50.0, atol=1e-3)   # scale 0.5 (CSV has 3 decimals)

    def test_fast_hand_b_and_c(self):
        # straight down (z range +/-150 around the start; the hand asks for -100 mm at scale 0.5)
        steps = ARM + [(1, [0, 0, -20.0 * i], 0) for i in range(1, 11)] + [(1, [0, 0, -200], 0)] * 60
        k = len(ARM) + 9
        _, p0, _, col, _ = run(steps, settings(mapping__max_lag_mm=0.0))
        z = col("target_z") - p0[2]
        self.assertAlmostEqual(z[k], z[-1], places=6)                          # B: stops with the hand
        self.assertLessEqual(np.abs(np.diff(z)).max(), 4.0 + 2e-3)             # 200 mm/s
        _, p0, _, col, _ = run(steps)
        zc = col("target_z") - p0[2]
        self.assertAlmostEqual(zc[-1] - zc[k], -30.0, delta=2e-3)              # C: 30 mm more after the hand stops

    def test_release_freezes(self):
        steps = ARM + [(1, [2.0 * i, 0, 0], 0) for i in range(1, 11)] + [(0, [2.0 * i, 0, 0], 0) for i in range(11, 40)]
        _, p0, _, col, _ = run(steps)
        x = col("target_x")
        self.assertAlmostEqual(x[len(ARM) + 10], x[-1], places=9)

    def test_range_excess_is_kept(self):
        p0 = RB5Model(OFFSET).tcp(D)[0]
        u = np.r_[p0[:2] / np.hypot(*p0[:2]), 0]
        steps = ARM + [(1, list(4.0 * i * u), 0) for i in range(1, 76)] + [(1, list((300 - 4.0 * i) * u), 0) for i in range(1, 76)]
        _, p0, _, col, _ = run(steps)
        r = np.hypot(col("target_x"), col("target_y"))
        self.assertAlmostEqual(r.max(), 550.0, delta=2e-3)
        self.assertAlmostEqual(r[-1], np.hypot(*p0[:2]), delta=2e-3)           # back to the start

    def test_twist_maps_to_yaw_and_descent_keeps_yaw(self):
        steps = ARM + [(1, [0, 0, 0], a) for a in np.r_[np.linspace(0, 45, 60), np.linspace(45, -45, 120), np.linspace(-45, 0, 60)]]
        _, p0, p, col, _ = run(steps)
        y = col("yaw_target_deg")
        self.assertAlmostEqual(y.max(), 45.0, places=4)
        self.assertAlmostEqual(y.min(), -45.0, places=4)
        self.assertLess(np.linalg.norm(p[:3] - p0[:3]), 0.01)
        steps = ARM + [(1, [0, 0, -2.0 * i], 0) for i in range(1, 51)]
        _, _, _, col, _ = run(steps)
        self.assertEqual(np.abs(col("yaw_target_deg")).max(), 0.0)

    def test_j6_flicker_does_not_move_yaw(self):
        tick = 360.0 / 4096
        steps = ARM + [(1, [0, 0, 0], tick * (i % 2)) for i in range(40)] + [(1, [0, 0, 0], 2 * tick)] * 5
        _, _, _, col, _ = run(steps, settings(mapping__leader_deadband_mm=0.3, yaw__leader_deadband_deg=0.15))
        y = col("yaw_target_deg")
        self.assertEqual(np.abs(y[:len(ARM) + 40]).max(), 0.0)              # flicker: no yaw
        self.assertAlmostEqual(y[-1], 2 * tick, places=3)                     # a real two-tick turn passes

    def test_gripper_independent_of_clutch(self):
        # trigger 1000 = released (open), 0 = pulled (closed)
        steps = ([(0, [0, 0, 0], 0, 1000)] * 3 + [(0, [0, 0, 0], 0, 0)] * 3        # pulled while the clutch is released
                 + [(1, [0, 0, 0], 0, 0)] * 3 + [(1, [0, 0, 0], 0, None)] * 2      # failed reads
                 + [(1, [0, 0, 0], 0, 1000)] * 3)
        _, _, _, col, _ = run(steps)
        closed = col("grip_state_closed")
        self.assertEqual(closed[0], 0)
        self.assertEqual(closed[4], 1)              # closed while the clutch is released
        self.assertEqual(closed[10], 1)             # kept on failed reads
        self.assertEqual(closed[-1], 0)

    def test_measured_columns(self):
        steps = ARM + [(1, [1.0 * i, 0, 0], 0) for i in range(1, 11)]
        loop, _, _, col, _ = run(steps)
        self.assertTrue(np.all(np.isnan(col("jnt_ang_0"))))          # fake reader has no measured joints

        class MeasuredReader(FakeReader):
            def measured(self):
                return self.robot.q.copy()

        model = RB5Model(OFFSET)
        robot = FakeRB5(D, model)
        leader = ScriptLeader(steps)
        loop = TeleopLoop(settings(), leader, ScriptModel(leader), robot, MeasuredReader(robot), model,
                          event=lambda m: None, status=False, sleep=lambda t: None)
        with self.assertRaises(StopRequest):
            loop.run()
        r = loop.rows[-1]
        self.assertAlmostEqual(float(r[IX["jnt_ang_0"]]), robot.q[0], places=2)
        self.assertLess(float(r[IX["target_minus_meas_mm"]]), 1.0)   # the fake follows exactly

    def test_leader_failures_stop(self):
        steps = ARM + [(1, [0, 0, 0], 0, 0, False)] * 3 + [(1, [0, 0, 0], 0)] * 5
        _, _, _, _, stop = run(steps)
        self.assertIn("leader joint read failures", stop)

    def test_late_cycle_freezes_target(self):
        steps = ARM + [(1, [2.0 * i, 0, 0], 0) for i in range(1, 30)]
        loop, _, _, col, _ = run(steps, slow_at=len(ARM) + 10)
        late = col("late")
        k = int(np.argmax(late))
        self.assertEqual(late.sum(), 1)
        self.assertEqual(col("target_x")[k], col("target_x")[k - 1])

    def test_singularity_stop(self):
        # Near pose D the tool orientation locks J5, so this uses the old 21 start pose with J5 = 11 deg
        # (as the earlier fake test 7); its J3 (-160.6) is beyond +/-150, so the J3 limit is relaxed here.
        q = np.array([86.7854, -3.0706, -160.5958, 138.4536, 11.0, 31.8475])
        model = RB5Model(OFFSET)
        jinv = np.linalg.inv(model.jacobian(q))
        d = -jinv[4, :3] / np.linalg.norm(jinv[4, :3])                  # position direction that lowers J5
        steps = ARM + [(1, list(1.0 * i * d), 0) for i in range(1, 400)]
        wide = settings(range__r_min_mm=150.0, range__r_max_mm=900.0, range__z_below_mm=300.0, range__z_above_mm=300.0,
                        range__azimuth_half_deg=180.0, safety__joint_limit_deg=[360.0] * 6,
                        safety__elbow_slow_deg=5.0, safety__elbow_stop_deg=1.0)
        _, _, _, col, stop = run(steps, wide, q0=q)
        self.assertIn("J5", stop)
        self.assertGreater(col("j5_slow").sum(), 0)


def load_21():
    try:
        spec = importlib.util.spec_from_file_location("s21", ROOT / "scripts" / "21_leader_rb5_position.py")
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
    except Exception:
        return None
    m.MAX_STEP_MM, m.MAX_LEAD_MM, m.MAX_LAG_MM, m.MAX_YAW_LAG_DEG = 4.0, 8.0, 30.0, 15.0
    m.CYL_R_MIN, m.CYL_R_MAX, m.CYL_Z_HALF, m.CYL_AZ_HALF_DEG = 450.0, 550.0, 150.0, 90.0
    m.PERIOD_S, m.MAX_GAP_S, m.MAX_STATE_AGE_S, m.STATUS_EVERY = 0.0005, 10.0, 10.0, 10 ** 9
    m.event = lambda msg: None
    return m


class TestLoopSameAs21(unittest.TestCase):
    """The same script through scripts/21 run_loop and TeleopLoop gives the same targets."""

    def test_combined_script(self):
        m = load_21()
        if m is None:
            self.skipTest("scripts/21 needs rbpodo and dynamixel_sdk (conda env rb5)")
        n = 80
        steps = (ARM + [(1, [3.0 * i, 1.0 * i, -4.0 * i], 0.8 * i, 1000 if i > 40 else 0) for i in range(1, n + 1)]
                 + [(1, [240, 80, -320], 64, 1000)] * 30 + [(0, [240, 80, -320], 64, 0)] * 5)
        loop, _, _, col, _ = run(steps, settings(mapping__max_lag_mm=30.0))

        class RC:
            def error(self):
                return types.SimpleNamespace(throw_if_not_empty=lambda: None)

            def clear(self):
                pass

        class Robot21(FakeRB5):
            def flush(self, rc):
                pass

            def move_servo_l(self, rc, pnt, *a):
                self.send_servo(pnt[:3], pnt[3:])

        model = RB5Model(OFFSET)
        robot = Robot21(D, model)
        leader = ScriptLeader(steps)
        p0 = robot.pose()
        ctx = {"target": None, "rows": [], "clutch": None, "rpy": p0[3:].copy()}
        args = types.SimpleNamespace(yaw_deg=0.0, scale=0.5, no_yaw=False)
        try:
            m.run_loop(leader, ScriptModel(leader), robot, RC(), FakeReader(robot), m.RB5Model(OFFSET), args, ctx)
        except (m.StopRequest, StopRequest):
            pass
        h21 = {h: i for i, h in enumerate(m.CSV_HEADER)}
        for k in ("target_x", "target_y", "target_z", "yaw_target_deg", "clutch_engaged", "speed_limited", "range_limited"):
            a = col(k)
            b = np.array([float(r[h21[k]]) for r in ctx["rows"]])
            np.testing.assert_allclose(a, b, atol=1e-3, err_msg=k)
        # gripper columns: 21 logged grip_open = trigger / 1000; teleop logs grip_close = 1 - trigger / 1000
        np.testing.assert_allclose(col("grip_close"), [1.0 - float(r[h21["grip_open"]]) for r in ctx["rows"]], atol=1e-9)


if __name__ == "__main__":
    unittest.main()

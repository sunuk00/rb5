"""Tests for teleop.kinematics, teleop.safety, teleop.mapping and teleop.leader (no hardware).
Run: python -m unittest discover tests"""
import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.kinematics import CylinderRange, RB5Model, rot_zyx, tcp_offset_from, wrap_deg, zyx_from_rot  # noqa: E402
from teleop.leader import Clutch, GripSignal, LeaderModel  # noqa: E402
from teleop.mapping import Limits, Mapper, plan_step  # noqa: E402
from teleop.safety import Watch, joint_limit_margin, ramp_scale, singular_distance  # noqa: E402
from teleop.settings import load_settings  # noqa: E402

SIM, _ = load_settings(ROOT / "configs" / "sim.yaml")
URDF = ROOT / "refs" / "rby1-sdk" / "models" / "leader_arm" / "model.urdf"
OFFSET = [0.0, -187.0, -40.0]
POSE_D = np.array(SIM.ready_pose.joints_deg)


class TestKinematics(unittest.TestCase):
    def test_euler_round_trip(self):
        for rx, ry, rz in ((10, -20, 30), (90, 0, 90), (-1.577, 6.678, 89.947)):
            np.testing.assert_allclose(zyx_from_rot(rot_zyx(rx, ry, rz)), [rx, ry, rz], atol=1e-9)

    def test_tcp_offset_from_17(self):
        # 17: get_tcp_info and get_tfc_info at the same pose
        off, rot = tcp_offset_from([383.130, -190.082, 284.543, -1.577, 6.678, 89.947],
                                   [195.104, -185.856, 319.147, -1.577, 6.678, 89.947])
        np.testing.assert_allclose(off, OFFSET, atol=0.05)
        self.assertLess(rot, 0.01)

    def test_dh_matches_controller(self):
        # 17: flange at jnt_ref (DH within about 2 mm); pose D: controller calc_fk_tcp (483.3, 129.0, 102.2)
        t = RB5Model([0, 0, 0]).flange([86.785, -3.461, -160.645, 138.894, 3.684, 31.847])
        self.assertLess(np.linalg.norm(t[:3, 3] - [195.104, -185.856, 319.147]), 2.5)
        p, r = RB5Model(OFFSET).tcp(POSE_D)
        self.assertLess(np.linalg.norm(p - [483.3, 129.0, 102.2]), 2.5)
        np.testing.assert_allclose(zyx_from_rot(r), [90, 0, 90], atol=0.1)

    def test_cylinder(self):
        p0 = RB5Model(OFFSET).tcp(POSE_D)[0]
        cyl = CylinderRange.from_settings(p0, SIM.range)
        az0 = np.degrees(np.arctan2(p0[1], p0[0]))

        def at(r, daz, dz):
            a = np.radians(az0 + daz)
            return np.array([r * np.cos(a), r * np.sin(a), p0[2] + dz])

        def polar(p):
            return np.hypot(p[0], p[1]), wrap_deg(np.degrees(np.arctan2(p[1], p[0])) - az0), p[2] - p0[2]

        for given, expect in (((700, 0, 0), (550, 0, 0)), ((300, 0, 0), (450, 0, 0)), ((500, 120, 0), (500, 90, 0)),
                              ((500, 0, -200), (500, 0, -150)), ((500, 30, 50), (500, 30, 50))):
            np.testing.assert_allclose(polar(cyl.clamp(at(*given))), expect, atol=1e-6)
        floor = CylinderRange(p0, 450, 550, 150, 90, z_min_abs=30.0)
        self.assertAlmostEqual(floor.clamp(at(500, 0, -150))[2], 30.0)


class TestSafety(unittest.TestCase):
    def test_helpers(self):
        self.assertAlmostEqual(singular_distance(170.0), 10.0)
        self.assertAlmostEqual(singular_distance(-90.0), 90.0)
        self.assertEqual(ramp_scale(25, 20, 10, 0.25), 1.0)
        self.assertAlmostEqual(ramp_scale(15, 20, 10, 0.25), 0.625)
        self.assertEqual(ramp_scale(5, 20, 10, 0.25), 0.25)
        self.assertEqual(joint_limit_margin([0, 0, -140, 0, 0, 0], SIM.safety.joint_limit_deg), (10.0, 3))

    def test_start_problems_and_stop(self):
        watch, model = Watch(SIM.safety), RB5Model(OFFSET)
        self.assertEqual(watch.start_problems(POSE_D, model), [])
        old = [86.787, -3.457, -160.642, 138.899, 31.257, 31.853]   # 21 start pose: J3 beyond 150
        self.assertTrue(any("J3" in p for p in watch.start_problems(old, model)))
        self.assertIn("J5", watch.stop_reason((9.0, 300.0, 60.0, 50.0, 1)))
        self.assertIn("shoulder", watch.stop_reason((30.0, 10.0, 60.0, 50.0, 1)))
        self.assertIsNone(watch.stop_reason((30.0, 300.0, 60.0, 50.0, 1)))


class TestLeader(unittest.TestCase):
    def test_wrist_center_zero_pose(self):
        np.testing.assert_allclose(LeaderModel(URDF).wrist_center_mm(np.zeros(7)), [0, -150, -372.5], atol=1e-9)

    def test_clutch(self):
        c = Clutch()
        self.assertEqual(c.update(0x100), (1, None))            # held at start: not armed
        self.assertEqual(c.update(0)[1], "armed (button released, press to engage)")
        c.update(0x100)
        self.assertEqual(c.update(0x100)[1], "engaged")
        c.update(None); c.update(None)
        self.assertTrue(c.engaged)
        self.assertIn("released", c.update(None)[1])

    def test_grip_close_direction_and_hysteresis(self):
        g = GripSignal(SIM.gripper)
        out = []
        for t in [0, 300, 650, 500, 450, 350, None, None, None, 900, 380]:
            g.update(None if t is None else t << 16)
            out.append((g.close, g.closed))
        self.assertEqual(out[0], (0.0, False))                   # untouched trigger = open
        self.assertEqual(out[2][1], True)                        # 0.65 > 0.6: closed
        self.assertEqual(out[4][1], True)                        # 0.45 kept
        self.assertEqual(out[5][1], False)                       # 0.35 < 0.4: open
        self.assertEqual(out[8], out[5])                         # failed reads keep the value
        self.assertEqual(g.update(None), None)
        self.assertEqual(out[10], (0.38, False))


def load_21():
    """The original script 21 with its constants set to configs/sim.yaml, or None if it cannot load."""
    try:
        spec = importlib.util.spec_from_file_location("s21", ROOT / "scripts" / "21_leader_rb5_position.py")
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
    except Exception:
        return None
    m.MAX_STEP_MM, m.MAX_LEAD_MM, m.MAX_LAG_MM, m.MAX_YAW_LAG_DEG = 4.0, 8.0, 30.0, 15.0
    m.CYL_R_MIN, m.CYL_R_MAX, m.CYL_Z_HALF, m.CYL_AZ_HALF_DEG = 450.0, 550.0, 150.0, 90.0
    return m


class TestSameAs21(unittest.TestCase):
    """plan_step must give the same result as scripts/21 for the same inputs (sim.yaml values)."""

    def test_random_steps(self):
        m = load_21()
        if m is None:
            self.skipTest("scripts/21 needs rbpodo and dynamixel_sdk (conda env rb5)")
        rng = np.random.default_rng(1)
        model, watch = RB5Model(OFFSET), Watch(SIM.safety)
        seen = set()
        for lag in (30.0, 0.0):
            m.MAX_LAG_MM, m.MAX_YAW_LAG_DEG = lag, (15.0 if lag else 0.0)
            s = load_settings(ROOT / "configs" / "sim.yaml")[1]
            s["mapping"]["max_lag_mm"] = lag
            lim = Limits(_ns(s))
            for i in range(300):
                q = POSE_D + rng.normal(0, 8, 6)
                # also poses near the wrist singularity, the J3 limit and the elbow singularity
                if i % 3 == 1:
                    q[4] = rng.normal(0, 14)
                elif i % 3 == 2:
                    q[2] = rng.choice([rng.uniform(-150, -128), rng.uniform(-25, -5)])
                tcp = model.tcp(q)[0]
                p0 = model.tcp(POSE_D)[0]
                target = tcp + rng.normal(0, 2, 3)
                desired = target + rng.normal(0, 40, 3)
                yaw = {"target": rng.normal(0, 20), "desired": rng.normal(0, 60), "tcp": rng.normal(0, 20)}
                yaw["tcp"] = yaw["target"] + rng.normal(0, 1)
                a = plan_step(target, desired, tcp, q, model, CylinderRange.from_settings(p0, SIM.range), yaw, lim, watch)
                b = m.plan_step(target, desired, tcp, q, m.RB5Model(OFFSET), m.CylinderRange(p0), dict(yaw))
                np.testing.assert_allclose(a.target, b.target, atol=1e-9)
                self.assertAlmostEqual(a.yaw, b.yaw, places=9)
                np.testing.assert_allclose(a.dropped, b.dropped, atol=1e-9)
                self.assertAlmostEqual(a.dropped_yaw, b.dropped_yaw, places=9)
                self.assertEqual(a.flags, b.flags)
                np.testing.assert_allclose(a.margins, b.margins, atol=1e-9)
                self.assertAlmostEqual(a.max_dq, b.max_dq, places=9)
                self.assertAlmostEqual(a.lag, b.lag, places=9)
                self.assertEqual(a.stop, b.stop)
                seen.update(k for k, v in a.flags.items() if v)
                seen.update(["stop"] if a.stop else [])
        # the cases must exercise the limits, the watches and the stop
        for key in ("speed", "yspeed", "range", "lead", "jspeed", "j5slow", "elslow", "jlslow", "stop"):
            self.assertIn(key, seen)


def _ns(d):
    if isinstance(d, dict):
        return SimpleNamespace(**{k: _ns(v) for k, v in d.items()})
    return tuple(d) if isinstance(d, list) else d


class TestMapper(unittest.TestCase):
    def test_relative_mapping_and_shift(self):
        mp = Mapper(SIM, [400.0, 100.0, 200.0])
        mp.engage([10.0, 0.0, 0.0], 30.0)
        pos, yaw = mp.desired([30.0, 0.0, -10.0], 40.0)
        np.testing.assert_allclose(pos, [410.0, 100.0, 195.0])   # scale 0.5
        self.assertAlmostEqual(yaw, 10.0)
        mp.engage([10.0, 0.0, 0.0], 170.0)
        _, yaw = mp.desired([10.0, 0.0, 0.0], -170.0)
        self.assertAlmostEqual(yaw, 20.0)                         # J6 crossing +/-180
        mp.engage([10.0, 0.0, 0.0], 30.0)
        mp.apply(SimpleNamespace(target=np.array([401.0, 100.0, 200.0]), yaw=2.0,
                                 dropped=np.array([4.0, 0.0, 0.0]), dropped_yaw=3.0))
        pos, yaw = mp.desired([30.0, 0.0, -10.0], 40.0)
        np.testing.assert_allclose(pos, [406.0, 100.0, 195.0])   # 4 mm dropped
        self.assertAlmostEqual(yaw, 7.0)


if __name__ == "__main__":
    unittest.main()

"""Tests for teleop.kinematics, teleop.safety, teleop.mapping and teleop.leader (no hardware; the gripper: test_gripper).
Run: python -m unittest discover tests"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from teleop.kinematics import CylinderRange, RB5Model, rot_zyx, tcp_offset_from, wrap_deg, zyx_from_rot  # noqa: E402
from teleop.leader import Clutch, LeaderFilter, LeaderModel  # noqa: E402
from teleop.mapping import Limits, Mapper, clip_step, plan_step  # noqa: E402
from teleop.safety import Watch, joint_limit_margin, ramp_scale, singular_distance  # noqa: E402

from fixtures import TEST as SIM, test_raw  # noqa: E402  (pose D era values, 2026-10-12)
URDF = ROOT / "assets" / "leader_arm" / "model.urdf"
OFFSET = [0.0, -187.0, -40.0]
POSE_D = np.array([182.0, -11.0, -110.1, 31.1, -90.0, -2.0])   # pose D (tests are written for it)


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
        cyl = CylinderRange.from_settings(SIM.range)   # the fixture: +/-150 mm, +/-90 deg around pose D
        az0 = np.degrees(np.arctan2(p0[1], p0[0]))

        def at(r, daz, dz):
            a = np.radians(az0 + daz)
            return np.array([r * np.cos(a), r * np.sin(a), p0[2] + dz])

        def polar(p):
            return np.hypot(p[0], p[1]), wrap_deg(np.degrees(np.arctan2(p[1], p[0])) - az0), p[2] - p0[2]

        for given, expect in (((700, 0, 0), (550, 0, 0)), ((300, 0, 0), (450, 0, 0)), ((500, 120, 0), (500, 90, 0)),
                              ((500, 0, -200), (500, 0, -150)), ((500, 30, 50), (500, 30, 50))):
            np.testing.assert_allclose(polar(cyl.clamp(at(*given))), expect, atol=1e-6)
        floor = CylinderRange(450, 550, 30.0, p0[2] + 150, az0 - 90, az0 + 90)
        self.assertAlmostEqual(floor.clamp(at(500, 0, -150))[2], 30.0)
        across = CylinderRange(100, 900, -100, 100, 150, 210)     # an azimuth range across +/-180
        self.assertTrue(across.contains(np.array([-500.0, 0.0, 0.0])))
        np.testing.assert_allclose(np.degrees(np.arctan2(*across.clamp(np.array([0.0, 500.0, 0.0]))[1::-1])), 150.0)

    def test_cylinder_inner_radius_rises_toward_the_floor(self):
        cyl = CylinderRange(250, 800, -50.0, 465.0, -90, 90, r_min_by_z=[[0, 350], [50, 300], [100, 250]])
        self.assertEqual(cyl.r_min_at(-50.0), 350.0)                # below the first point
        self.assertAlmostEqual(cyl.r_min_at(25.0), 325.0)           # halfway between 0 and 50
        self.assertEqual(cyl.r_min_at(300.0), 250.0)                # above the last point
        np.testing.assert_allclose(cyl.clamp(np.array([260.0, 0.0, 200.0])), [260.0, 0.0, 200.0])   # high: inside
        np.testing.assert_allclose(cyl.clamp(np.array([260.0, 0.0, 25.0])), [325.0, 0.0, 25.0])     # low: pushed out
        np.testing.assert_allclose(cyl.clamp(np.array([200.0, 0.0, -100.0])), [350.0, 0.0, -50.0])  # floor, then r
        self.assertEqual(CylinderRange(350, 800, 0.0, 500.0, -90, 90).r_min_at(0.0), 350.0)        # no table

    def test_cylinder_outer_radius_falls_with_height(self):
        cyl = CylinderRange(350, 800, 0.0, 500.0, -90, 90, r_max_by_z=[[300, 800], [400, 700], [500, 650]])
        self.assertEqual(cyl.r_max_at(100.0 + 100), 800.0)          # below the first point
        self.assertAlmostEqual(cyl.r_max_at(100.0 + 250), 750.0)    # halfway between 200 and 300
        self.assertEqual(cyl.r_max_at(100.0 + 400), 650.0)
        out = cyl.clamp(np.array([900.0, 0.0, 100.0 + 250]))       # far and high: r cut at that height
        np.testing.assert_allclose(out, [750.0, 0.0, 350.0])
        out = cyl.clamp(np.array([900.0, 0.0, 100.0 + 600]))       # above the top: z to +400, then r 650
        np.testing.assert_allclose(out, [650.0, 0.0, 500.0])
        np.testing.assert_allclose(cyl.clamp(np.array([780.0, 0.0, 150.0])), [780.0, 0.0, 150.0])   # low: inside


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

    def test_leader_filter(self):
        f = LeaderFilter(0.3, 0.15)
        tick = 360.0 / 4096
        outs = [f.update([0.0, 0.0, 0.1 * (i % 2)], 10.0 + tick * (i % 2))[1] for i in range(6)]
        self.assertEqual(outs, [10.0] * 6)                                   # one-tick flicker ignored
        p, j6 = f.update([0.0, 0.0, 0.0], 10.0 + 2 * tick)
        self.assertAlmostEqual(j6, 10.0 + 2 * tick)                          # two ticks pass
        p, _ = f.update([0.0, 0.0, 0.25], j6)
        np.testing.assert_allclose(p, [0.0, 0.0, 0.0])                       # 0.25 mm ignored
        p, _ = f.update([0.0, 0.0, 0.31], j6)
        np.testing.assert_allclose(p, [0.0, 0.0, 0.31])                      # 0.31 mm taken
        f = LeaderFilter(0.3, 0.15)
        f.update([0, 0, 0], 179.95)
        self.assertAlmostEqual(f.update([0, 0, 0], -179.97)[1], 179.95)      # 0.08 deg across +/-180: ignored
        self.assertAlmostEqual(f.update([0, 0, 0], -179.8)[1], -179.8)       # 0.25 deg across +/-180: taken

    def test_leader_low_pass(self):
        f = LeaderFilter(0.0, 0.0, tau_s=0.03, period_s=0.02)               # alpha = 0.02 / 0.05 = 0.4
        np.testing.assert_allclose(f.update([0, 0, 0], 0.0)[0], [0, 0, 0])  # first value passes as is
        np.testing.assert_allclose(f.update([10, 0, 0], 0.0)[0], [4, 0, 0])
        np.testing.assert_allclose(f.update([10, 0, 0], 0.0)[0], [6.4, 0, 0])
        for _ in range(30):
            p, j6 = f.update([10, 0, 0], 5.0)
        np.testing.assert_allclose(p, [10, 0, 0], atol=1e-5)                 # settles on the hand
        self.assertEqual(j6, 5.0)                                            # J6 is not low-passed
        f = LeaderFilter(0.0, 0.0, tau_s=0.0)
        np.testing.assert_allclose(f.update([1, 2, 3], 0.0)[0], [1, 2, 3])
        np.testing.assert_allclose(f.update([4, 5, 6], 0.0)[0], [4, 5, 6])  # tau 0 = off
        f = LeaderFilter(0.3, 0.15, tau_s=0.03)
        outs = [f.update([0.0, 0.0, 0.1 * (i % 2)], 0.0)[0] for i in range(6)]
        np.testing.assert_allclose(outs, np.zeros((6, 3)))                   # flicker still removed at rest

    def test_clutch(self):
        c = Clutch()
        self.assertEqual(c.update(0x100), (1, None))            # held at start: not armed
        self.assertEqual(c.update(0)[1], "armed (button released, press to engage)")
        c.update(0x100)
        self.assertEqual(c.update(0x100)[1], "engaged")
        c.update(None); c.update(None)
        self.assertTrue(c.engaged)
        self.assertIn("released", c.update(None)[1])


class TestPlanStep(unittest.TestCase):


    def test_joint_speed_limit_ignores_the_lag(self):
        model, watch = RB5Model(OFFSET), Watch(SIM.safety)
        s = test_raw()
        s["mapping"]["max_lag_mm"] = 0.0
        lim = Limits(_ns(s))
        q = np.array(POSE_D, dtype=float)
        tcp = model.tcp(q)[0]
        cyl = CylinderRange.from_settings(SIM.range)
        yaw = {"target": 0.0, "desired": 0.0, "tcp": 0.0}
        step = np.array([0.0, 0.0, -3.0])                    # within the 4 mm speed cap
        for lag in (0.0, 3.0):                                # the robot is 0 or 3 mm behind the target
            target = tcp + np.array([0.0, 0.0, -lag])
            a = plan_step(target, target + step, tcp, q, model, cyl, yaw, lim, watch)
            dq = np.abs(model.predict(q, a.target - target, None, 0.0))
            self.assertLessEqual(dq.max(), max(lim.max_dq_deg) + 1e-9)
            np.testing.assert_allclose(a.target - target, step if not a.flags["jspeed"] else a.target - target)
            if lag:
                self.assertEqual(a.flags["jspeed"], without_lag)   # the lag does not change the decision
            without_lag = a.flags["jspeed"]


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



class TestClipStep(unittest.TestCase):
    def test_sideways_faster_than_radial_and_vertical(self):
        at = np.array([500.0, 0.0, 100.0])                   # sideways = +/- y here
        np.testing.assert_allclose(clip_step(np.array([0, 6.0, 0]), at, 4.0, 6.0), [0, 6, 0])
        np.testing.assert_allclose(clip_step(np.array([0, 9.0, 0]), at, 4.0, 6.0), [0, 6, 0])
        np.testing.assert_allclose(clip_step(np.array([6.0, 0, 0]), at, 4.0, 6.0), [4, 0, 0])
        np.testing.assert_allclose(clip_step(np.array([0, 0, -6.0]), at, 4.0, 6.0), [0, 0, -4])
        d = clip_step(np.array([4.0, 6.0, 0]), at, 4.0, 6.0)   # on the ellipse edge after scaling
        self.assertAlmostEqual(np.hypot(d[1] / 6.0, d[0] / 4.0), 1.0)
        np.testing.assert_allclose(d / np.linalg.norm(d), np.array([4.0, 6.0, 0]) / np.hypot(4, 6))   # same direction
        np.testing.assert_allclose(clip_step(np.array([1.0, 1.0, 1.0]), at, 4.0, 6.0), [1, 1, 1])   # small: unchanged



if __name__ == "__main__":
    unittest.main()

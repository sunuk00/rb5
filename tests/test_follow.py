"""Tests for teleop.follow.TargetFollower (rec-4b) with the fake DH robot of test_loop: it reaches a
target, keeps the speed cap, stays in the range, turns the tool with the azimuth. Run: python -m unittest discover tests"""
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from teleop.follow import TargetFollower  # noqa: E402
from teleop.kinematics import RB5Model  # noqa: E402
from test_loop import D, OFFSET, FakeRB5, settings  # noqa: E402


def make(**changes):
    s = settings(yaw__mode="follow_base", **changes)
    model = RB5Model(OFFSET)
    robot = FakeRB5(D, model)
    return s, robot, TargetFollower(s, model, robot.pose())


def run(robot, f, desired, cycles):
    steps, plans = [], []
    for _ in range(cycles):
        before = f.target.copy()
        plan = f.step(desired, robot.pose(), robot.q.copy())
        plans.append(plan)
        if plan.stop:
            break
        robot.send_servo(f.target, f.rpy)
        steps.append(np.linalg.norm(f.target - before))
    return np.array(steps), plans


class TestTargetFollower(unittest.TestCase):
    def test_reaches_a_target_within_the_speed_cap(self):
        s, robot, f = make()
        goal = robot.pose()[:3] + np.array([0.0, 0.0, 40.0])   # straight up
        steps, plans = run(robot, f, goal, 200)
        np.testing.assert_allclose(f.target, goal, atol=1e-6)
        np.testing.assert_allclose(robot.pose()[:3], goal, atol=0.05)
        self.assertLessEqual(steps.max(), s.mapping.speed_mm_s * s.servo.period_s + 1e-9)
        self.assertTrue(any(p.flags.get("speed") for p in plans))

    def test_stays_inside_the_range(self):
        s, robot, f = make()
        p0 = robot.pose()[:3]
        goal = p0 + np.array([0.0, 0.0, 1000.0])               # far above z_max
        run(robot, f, goal, 400)
        self.assertLessEqual(f.target[2], s.range.z_max_mm + 1e-6)

    def test_tool_turns_with_the_azimuth(self):
        s, robot, f = make()
        p0 = robot.pose()
        a0 = np.arctan2(p0[1], p0[0])
        r = np.hypot(p0[0], p0[1])
        goal = np.array([r * np.cos(a0 + np.radians(5)), r * np.sin(a0 + np.radians(5)), p0[2]])
        run(robot, f, goal, 300)
        self.assertAlmostEqual(f.yaw, 5.0, delta=0.05)
        rz = np.degrees(np.arctan2(np.sin(np.radians(robot.pose()[5] - p0[5])), np.cos(np.radians(robot.pose()[5] - p0[5]))))
        self.assertAlmostEqual(rz, 5.0, delta=0.1)

    def test_needs_follow_base(self):
        s = settings()   # the test settings use leader_j6
        with self.assertRaises(ValueError):
            TargetFollower(s, RB5Model(OFFSET), np.zeros(6))


if __name__ == "__main__":
    unittest.main()

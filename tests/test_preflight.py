"""Tests for teleop.preflight (no hardware). Run: python -m unittest discover tests"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.kinematics import RB5Model  # noqa: E402
from teleop.preflight import (check_leader, check_robot_mode, check_start_pose, check_tcp_offset,  # noqa: E402
                              nearest_equivalent, plan_ready_move)
from teleop.safety import Watch  # noqa: E402
from teleop.settings import load_settings  # noqa: E402

SIM, _ = load_settings(ROOT / "configs" / "sim.yaml")
OFFSET = [0.0, -187.0, -40.0]
D = np.array([182.0, -11.0, -110.1, 31.1, -90.0, -2.0])   # pose D (tests are written for it)


class TestReadyMove(unittest.TestCase):
    def test_nearest_equivalent(self):
        # 19 cases
        self.assertEqual(nearest_equivalent(-165, 138.9, 360), 195.0)
        self.assertEqual(nearest_equivalent(-165, -100, 360), -165.0)
        self.assertEqual(nearest_equivalent(-90, 100, 150), -90.0)
        self.assertIsNone(nearest_equivalent(200, 0, 150))

    def test_plan_from_pose_c(self):
        p = plan_ready_move([180, 4, -111, 17, -90, 0], D, SIM.safety.joint_limit_deg, SIM.ready_pose, 0.1)
        np.testing.assert_allclose(p["target"], D)
        self.assertFalse(p["skip"])
        self.assertEqual(p["wrist_warnings"], [])
        self.assertAlmostEqual(p["expected_s"], 15.0 / 3.0 + 3.0 / 6.0)   # largest change J2 -15 deg at 3 deg/s

    def test_skip_and_wrist_warning(self):
        p = plan_ready_move(D + 0.01, D, SIM.safety.joint_limit_deg, SIM.ready_pose, 0.1)
        self.assertTrue(p["skip"])
        p = plan_ready_move([180, -15, -90, 195, 90, 0], D, SIM.safety.joint_limit_deg, SIM.ready_pose, 0.1)   # pose A
        self.assertTrue(any("J4" in w for w in p["wrist_warnings"]) and any("J5" in w for w in p["wrist_warnings"]))

    def test_unreachable_target(self):
        rp = SimpleNamespace(move_speed_deg_s=30.0, tolerance_deg=0.05, wrist_warning_deg=90.0)
        p = plan_ready_move(np.zeros(6), [0, 0, 200, 0, 0, 0], SIM.safety.joint_limit_deg, rp, 0.1)
        self.assertIsNone(p["target"])
        self.assertTrue(p["problems"])


class TestChecks(unittest.TestCase):
    def test_leader(self):
        self.assertEqual(check_leader(1, 1, []), [])
        self.assertEqual(len(check_leader(16, 1, ["IDs [128] did not answer the ping"])), 2)

    def test_robot_mode(self):
        self.assertEqual(check_robot_mode(False, True, False, ""), [])
        self.assertTrue(check_robot_mode(True, True, False, ""))       # Real requested, Simulation reported
        self.assertTrue(check_robot_mode(False, False, False, ""))
        self.assertTrue(check_robot_mode(False, True, True, ""))
        self.assertTrue(check_robot_mode(False, True, False, "error 12"))

    def test_tcp_offset(self):
        tcp_sim = SimpleNamespace(expected_offset_mm=None, offset_tolerance_mm=1.0)
        self.assertEqual(check_tcp_offset(OFFSET, 0.0, tcp_sim), [])
        tcp_real = SimpleNamespace(expected_offset_mm=(0.0, -187.0, -40.0), offset_tolerance_mm=1.0)
        self.assertEqual(check_tcp_offset([0.2, -187.1, -40.0], 0.0, tcp_real), [])
        self.assertTrue(check_tcp_offset([0.0, -150.0, -40.0], 0.0, tcp_real))
        self.assertTrue(check_tcp_offset(OFFSET, 5.0, tcp_sim))

    def test_start_pose(self):
        model, watch = RB5Model(OFFSET), Watch(SIM.safety)
        tcp0 = np.r_[model.tcp(D)[0], 90, 0, 90]
        problems, err = check_start_pose(tcp0, D, model, watch, SIM)
        self.assertEqual(problems, [])
        self.assertLess(err, 1e-9)
        a = np.array([180, -15, -90, 195, 90, 0.])                       # pose A: TCP r 432 < 450
        problems, _ = check_start_pose(np.r_[model.tcp(a)[0], 0, 0, 0], a, model, watch, SIM)
        self.assertTrue(any("radius" in p for p in problems))
        problems, _ = check_start_pose(tcp0 + [10, 0, 0, 0, 0, 0], D, model, watch, SIM)
        self.assertTrue(any("DH model" in p for p in problems))


if __name__ == "__main__":
    unittest.main()

"""Tests for teleop.settings. Run: python -m unittest discover tests"""
import copy
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.settings import SettingsError, load_settings  # noqa: E402

SIM = ROOT / "configs" / "sim.yaml"
REAL = ROOT / "configs" / "real.yaml"


def write_tmp(d):
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(d, f)
    f.close()
    return f.name


def raw(path):
    with open(path) as f:
        return yaml.safe_load(f)


class TestSettings(unittest.TestCase):
    def assert_problem(self, d, text, real=False):
        with self.assertRaises(SettingsError) as cm:
            load_settings(write_tmp(d), real=real)
        self.assertTrue(any(text in p for p in cm.exception.problems), cm.exception.problems)

    def test_sim_loads(self):
        s, _ = load_settings(SIM)
        self.assertEqual(s.mode, "sim")
        self.assertEqual(s.safety.joint_speed_deg_per_cycle, (1.5, 1.2, 1.2, 1.2, 1.2, 1.2))   # = real.yaml
        self.assertEqual(len(s.ready_pose.joints_deg), 6)
        self.assertEqual(s.runtime.cpu_cores, (0, 11))

    def test_sim_matches_real(self):
        """real-1 final: Simulation uses the Real values, so practice feels the same."""
        sim, real = raw(SIM), raw(REAL)
        allowed = {"mode", "tcp.expected_offset_mm"}
        diffs = []

        def walk(a, b, path=""):
            if isinstance(a, dict):
                for k in set(a) | set(b):
                    walk(a.get(k), b.get(k), f"{path}.{k}" if path else k)
            elif a != b and path not in allowed:
                diffs.append(f"{path}: sim {a!r} / real {b!r}")
        walk(sim, real)
        self.assertEqual(diffs, [], "configs/sim.yaml must match configs/real.yaml (edit real.yaml first)")

    def test_real_refused_while_tbd_values_are_null(self):
        d = raw(REAL)
        for sec, key in (("tcp", "expected_offset_mm"), ("ready_pose", "joints_deg")):
            d[sec][key] = None
        with self.assertRaises(SettingsError) as cm:
            load_settings(write_tmp(d), real=True)
        problems = "\n".join(cm.exception.problems)
        for key in ("tcp.expected_offset_mm", "ready_pose.joints_deg"):
            self.assertIn(key, problems)
        load_settings(REAL, real=True)   # the file itself has provisional values now
        # With the TBD values filled in, the conservative file passes the Real limits
        d = raw(REAL)
        d["tcp"]["expected_offset_mm"] = [0.0, -187.0, -40.0]
        d["ready_pose"]["joints_deg"] = [182.0, -11.0, -110.1, 31.1, -90.0, -2.0]
        s, _ = load_settings(write_tmp(d), real=True)
        self.assertEqual(s.mapping.max_lag_mm, 20.0)

    def test_mode_must_match(self):
        self.assert_problem(raw(SIM), "mode", real=True)
        d = raw(REAL)
        self.assert_problem(d, "mode", real=False)

    def test_unknown_and_missing_keys(self):
        d = raw(SIM)
        d["mapping"]["scael"] = 1.0
        self.assert_problem(d, "mapping.scael: unknown key")
        d = raw(SIM)
        del d["yaw"]["speed_deg_s"]
        self.assert_problem(d, "yaw.speed_deg_s: missing")

    def test_value_checks(self):
        cases = [
            (("mapping", "scale"), "0.5", "expected a number"),
            (("safety", "joint_limit_deg"), [360.0] * 5, "list of 6"),
            (("safety", "max_late_cycles"), 2.5, "integer"),
            (("yaw", "enabled"), 1, "true or false"),
            (("mapping", "speed_mm_s"), None, "must not be null"),
        ]
        for (sec, key), value, text in cases:
            d = raw(SIM)
            d[sec][key] = value
            self.assert_problem(d, text)

    def test_robot_file(self):
        s, d = load_settings(SIM)
        self.assertEqual(s.robot.ip, raw(ROOT / "configs" / "robot.yaml")["ip"])
        self.assertIn("robot", d)                       # the run log's settings copy includes it
        with self.assertRaises(SettingsError) as cm:   # speed bar above 0.1 in robot.yaml
            load_settings(SIM, robot_path=write_tmp({"ip": "10.0.2.6", "speed_bar": 0.2}))
        self.assertTrue(any("above 0.1" in p for p in cm.exception.problems), cm.exception.problems)
        d = raw(SIM)                                    # a robot section in a settings file is refused
        d["robot"] = {"ip": "10.0.2.6", "speed_bar": 0.1}
        self.assert_problem(d, "robot: set in robot.yaml")

    def test_relations(self):
        d = raw(SIM); d["safety"]["j5_stop_deg"] = 25.0
        self.assert_problem(d, "J5 stop")
        d = raw(SIM); d["range"]["r_min_mm"] = d["range"]["r_max_mm"] + 1.0
        self.assert_problem(d, "r_min_mm")
        d = raw(SIM); d["gripper"]["open_ticks"] = d["gripper"]["closed_ticks"] - 10
        self.assert_problem(d, "at least 50 ticks apart")
        d = raw(SIM); d["gripper"]["port"] = d["leader"]["port"]
        self.assert_problem(d, "gripper.port is the leader arm's port")
        d = raw(SIM); d["gripper"]["grasp_current"] = d["gripper"]["goal_current"] - 1
        self.assert_problem(d, "grasp_current")
        d = raw(SIM); d["gripper"]["goal_current"] = 400
        self.assert_problem(d, "goal_current")
        d = raw(SIM); d["servo"]["t1_s"] = 0.01
        self.assert_problem(d, "t1_s")
        d = raw(SIM); d["range"]["r_max_by_z"] = [[300.0, 700.0], [250.0, 750.0]]
        self.assert_problem(d, "heights must increase")
        d = raw(SIM); d["range"]["r_max_by_z"] = [[200.0, 900.0]]
        self.assert_problem(d, "at most r_max_mm")
        d = raw(SIM); d["range"]["r_min_by_z"] = [[50.0, 300.0], [0.0, 350.0]]
        self.assert_problem(d, "r_min_by_z: heights must increase")
        d = raw(SIM); d["range"]["r_min_by_z"] = [[0.0, 200.0]]          # below r_min_mm (250)
        self.assert_problem(d, "must be at least r_min_mm")
        d = raw(SIM); d["range"]["r_min_by_z"] = [[400.0, 690.0], [465.0, 700.0]]   # r_max is 650 at the top
        self.assert_problem(d, "is not below r_max")
        d = raw(SIM); d["range"]["r_max_by_z"] = [[200.0]]
        self.assert_problem(d, "pairs")
        d = raw(SIM); d["record"]["cameras"]["grip"]["device"] = d["record"]["cameras"]["top"]["device"]
        self.assert_problem(d, "two cameras have the same device")
        d = raw(SIM); d["record"]["cameras"]["front"]["format"] = "YUYV"
        self.assert_problem(d, "record.cameras.front.format: YUYV for a RealSense")
        d = raw(SIM); d["record"]["cameras"]["grip"]["format"] = "RGB"
        self.assert_problem(d, "record.cameras.grip.format: RGB for a webcam")
        d = raw(SIM); del d["record"]["cameras"]["grip"]
        self.assert_problem(d, "record.cameras.grip: missing")

    def test_real_limits(self):
        d = raw(REAL)
        d["tcp"]["expected_offset_mm"] = [0.0, -187.0, -40.0]
        d["ready_pose"]["joints_deg"] = [182.0, -11.0, -110.1, 31.1, -90.0, -2.0]
        for (sec, key), value in ((("mapping", "speed_mm_s"), 300.0), (("yaw", "speed_deg_s"), 60.0),
                                  (("safety", "joint_speed_deg_per_cycle"), [1.6] * 6)):
            dd = copy.deepcopy(d)
            dd[sec][key] = value
            self.assert_problem(dd, "Real limit", real=True)

    def test_settings_are_read_only_tuples(self):
        s, _ = load_settings(SIM)
        self.assertIsInstance(s.safety.joint_limit_deg, tuple)


if __name__ == "__main__":
    unittest.main()

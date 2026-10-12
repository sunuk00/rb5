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
        self.assertEqual(s.mapping.scale, 0.5)
        self.assertEqual(s.safety.joint_speed_deg_per_cycle, (1.0, 1.0, 1.0, 2.0, 2.0, 2.0))
        self.assertEqual(s.ready_pose.joints_deg, (182.0, -11.0, -110.1, 31.1, -90.0, -2.0))

    def test_real_file_is_valid_but_refused_until_tbd_values_are_set(self):
        with self.assertRaises(SettingsError) as cm:
            load_settings(REAL, real=True)
        problems = "\n".join(cm.exception.problems)
        for key in ("range.z_min_abs_mm", "tcp.expected_offset_mm", "ready_pose.joints_deg"):
            self.assertIn(key, problems)
        # With the TBD values filled in, the conservative file passes the Real limits
        d = raw(REAL)
        d["range"]["z_min_abs_mm"] = 30.0
        d["tcp"]["expected_offset_mm"] = [0.0, -187.0, -40.0]
        d["ready_pose"]["joints_deg"] = [182.0, -11.0, -110.1, 31.1, -90.0, -2.0]
        s, _ = load_settings(write_tmp(d), real=True)
        self.assertEqual(s.mapping.max_lag_mm, 0.0)

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
            (("robot", "speed_bar"), 0.2, "above 0.1"),
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

    def test_relations(self):
        d = raw(SIM); d["safety"]["j5_stop_deg"] = 25.0
        self.assert_problem(d, "J5 stop")
        d = raw(SIM); d["range"]["r_min_mm"] = 600.0
        self.assert_problem(d, "r_min_mm")
        d = raw(SIM); d["gripper"]["open_below"] = 0.7
        self.assert_problem(d, "open_below")
        d = raw(SIM); d["servo"]["t1_s"] = 0.01
        self.assert_problem(d, "t1_s")

    def test_real_limits(self):
        d = raw(REAL)
        d["range"]["z_min_abs_mm"] = 30.0
        d["tcp"]["expected_offset_mm"] = [0.0, -187.0, -40.0]
        d["ready_pose"]["joints_deg"] = [182.0, -11.0, -110.1, 31.1, -90.0, -2.0]
        for (sec, key), value in ((("mapping", "speed_mm_s"), 100.0), (("yaw", "speed_deg_s"), 60.0),
                                  (("safety", "joint_speed_deg_per_cycle"), [1.0] * 6)):
            dd = copy.deepcopy(d)
            dd[sec][key] = value
            self.assert_problem(dd, "Real limit", real=True)

    def test_settings_are_read_only_tuples(self):
        s, _ = load_settings(SIM)
        self.assertIsInstance(s.safety.joint_limit_deg, tuple)


if __name__ == "__main__":
    unittest.main()

"""Settings used by the tests: configs/sim.yaml with the values the tests were written for (pose D era,
2026-10-12, first taken from scripts/21), so the tests do not change when the settings are tuned."""
import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from teleop.kinematics import RB5Model
from teleop.settings import load_settings

ROOT = Path(__file__).resolve().parent.parent
RAW_SIM = load_settings(ROOT / "configs" / "sim.yaml")[1]
POSE_D = [182.0, -11.0, -110.1, 31.1, -90.0, -2.0]


def test_raw():
    d = copy.deepcopy(RAW_SIM)
    d["mapping"]["scale"] = 0.5
    d["mapping"]["max_lag_mm"] = 30.0
    d["mapping"]["speed_mm_s"] = 200.0
    d["mapping"]["leader_deadband_mm"] = 0.0      # 21 had no deadband
    d["mapping"]["leader_filter_tau_s"] = 0.0     # nor a low-pass
    d["mapping"]["left_button"] = "off"           # without the left button modes
    d["mapping"]["rotate_deg_per_mm"] = 0.3
    d["mapping"]["lock_azimuth"] = False
    d["mapping"]["accel_mm_s2"] = 0.0
    d["mapping"]["rotate_max_lag_deg"] = 0.0
    d["mapping"]["vertical_z_scale"] = 1.0
    d["mapping"]["speed_tangential_mm_s"] = d["mapping"]["speed_mm_s"]   # one speed limit in all directions
    d["yaw"]["mode"] = "leader_j6"
    d["yaw"]["leader_deadband_deg"] = 0.0
    p0 = RB5Model([0.0, -187.0, -40.0]).tcp(POSE_D)[0]   # 21: +/-150 mm and +/-90 deg around pose D
    az0 = float(np.degrees(np.arctan2(p0[1], p0[0])))
    d["range"].update(r_min_mm=450.0, r_max_mm=550.0, z_min_mm=float(p0[2]) - 150.0, z_max_mm=float(p0[2]) + 150.0,
                      azimuth_min_deg=az0 - 90.0, azimuth_max_deg=az0 + 90.0, r_max_by_z=None, r_min_by_z=None)
    d["safety"]["joint_limit_deg"] = [360.0, 360.0, 150.0, 360.0, 360.0, 360.0]
    d["safety"]["joint_limit_slow_deg"] = 20.0    # 21's value (the settings use 15 since 2026-10-16)
    # sim.yaml values of the 2026-10-12 tests (sim.yaml = real.yaml since 2026-10-16)
    d["safety"]["joint_speed_deg_per_cycle"] = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]
    d["yaw"].update(speed_deg_s=90.0, range_deg=90.0, max_lag_deg=15.0)
    d["ready_pose"]["move_speed_deg_s"] = 30.0
    d["ready_pose"]["joints_deg"] = list(POSE_D)
    return d


def ns(d):
    if isinstance(d, dict):
        return SimpleNamespace(**{k: ns(v) for k, v in d.items()})
    return tuple(d) if isinstance(d, list) else d


TEST = ns(test_raw())

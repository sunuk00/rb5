"""Settings used by the tests: configs/sim.yaml with the values the tests were written for (pose D era,
the same values as scripts/21), so the tests do not change when sim.yaml is tuned."""
import copy
from pathlib import Path
from types import SimpleNamespace

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
    d["yaw"]["leader_deadband_deg"] = 0.0
    d["range"].update(r_min_mm=450.0, r_max_mm=550.0, z_below_mm=150.0, z_above_mm=150.0, azimuth_half_deg=90.0,
                      z_min_abs_mm=None, r_max_by_z=None)
    d["safety"]["joint_limit_deg"] = [360.0, 360.0, 150.0, 360.0, 360.0, 360.0]
    d["ready_pose"]["joints_deg"] = list(POSE_D)
    return d


def ns(d):
    if isinstance(d, dict):
        return SimpleNamespace(**{k: ns(v) for k, v in d.items()})
    return tuple(d) if isinstance(d, list) else d


TEST = ns(test_raw())

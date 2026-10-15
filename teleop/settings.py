"""Teleop settings: load a YAML file, check every value, and return a read-only nested object.

Every key in the schema below must be present (unknown keys are errors, to catch typos). A value of
null means "not decided yet" (TBD). Some keys may be null in Simulation but are required for Real;
`load_settings(path, real=True)` refuses to return settings that still have one of them unset.

Usage:
    s = load_settings("configs/sim.yaml")
    s.mapping.scale, s.safety.joint_limit_deg, ...
"""
from pathlib import Path
from types import SimpleNamespace

import yaml


class SettingsError(Exception):
    """Raised with the full list of problems found in a settings file."""

    def __init__(self, path, problems):
        self.problems = list(problems)
        super().__init__(f"{path}:\n  " + "\n  ".join(self.problems))


def _num(lo=None, hi=None, nullable=False, integer=False):
    return {"kind": "num", "lo": lo, "hi": hi, "nullable": nullable, "integer": integer}


def _vec(n, lo=None, hi=None, nullable=False):
    return {"kind": "vec", "n": n, "lo": lo, "hi": hi, "nullable": nullable}


def _pairs(lo=None, hi=None, nullable=False):
    """A list of [a, b] number pairs (any length >= 1); lo / hi are (a, b) bounds."""
    return {"kind": "pairs", "lo": lo, "hi": hi, "nullable": nullable}


def _str(choices=None):
    return {"kind": "str", "choices": choices}


def _bool():
    return {"kind": "bool"}


# Ranges are hard sanity limits for any mode; the Real limits below are stricter.
SCHEMA = {
    "mode": _str(("sim", "real")),
    "robot": {
        "ip": _str(),
        "speed_bar": _num(0.01, 0.1),               # CLAUDE.md: speed bar <= 0.1
    },
    "servo": {
        "period_s": _num(0.005, 0.05),
        "t1_s": _num(0.002, 0.05),                  # UI Script 7.35: t1 >= 0.002
        "t2_s": _num(0.021, 0.199),                 # UI Script 7.35: 0.02 < t2 < 0.2
        "gain": _num(0.01, 10.0),
        "alpha": _num(0.01, 1.0),
    },
    "leader": {
        "port": _str(),
        "required_latency_timer_ms": _num(1, 16, integer=True),
    },
    "mapping": {
        "scale": _num(0.1, 2.0),
        "frame_yaw_deg": _num(-180.0, 180.0),
        "speed_mm_s": _num(1.0, 400.0),               # Real is capped lower (REAL_MAX)
        "lead_per_step": _num(1.0, 5.0),
        "max_lag_mm": _num(0.0, 50.0),
        "leader_deadband_mm": _num(0.0, 5.0),         # leader wrist center changes smaller than this are ignored
        "leader_filter_tau_s": _num(0.0, 0.2),        # low-pass time constant on the wrist center; 0 = off
        "mode": _str(("cartesian", "cylindrical")),    # position mapping (teleop/mapping.py)
        "pivot_xy_mm": _vec(2, -1000.0, 1000.0),       # cylindrical: vertical pivot axis in the leader frame
        "min_pivot_radius_mm": _num(50.0, 1000.0),    # cylindrical: angle gain bound near the pivot axis
    },
    "yaw": {
        "enabled": _bool(),
        "speed_deg_s": _num(1.0, 90.0),
        "range_deg": _num(0.0, 180.0),
        "max_lag_deg": _num(0.0, 30.0),
        "leader_deadband_deg": _num(0.0, 2.0),        # leader J6 changes smaller than this are ignored
    },
    "range": {
        "r_min_mm": _num(150.0, 900.0),
        "r_max_mm": _num(150.0, 900.0),
        "z_below_mm": _num(0.0, 300.0),               # start z - this
        "z_above_mm": _num(0.0, 500.0),               # start z + this
        # [start z + dz, r_max] points: r_max falls with height (linear between points, the last value above)
        "r_max_by_z": _pairs(lo=(0.0, 150.0), hi=(500.0, 900.0), nullable=True),
        "z_min_abs_mm": _num(-200.0, 800.0, nullable=True),
        "azimuth_half_deg": _num(0.0, 180.0),
    },
    "safety": {
        "joint_speed_deg_per_cycle": _vec(6, 0.01, 5.0),
        "joint_limit_deg": _vec(6, 10.0, 360.0),
        "joint_limit_slow_deg": _num(0.0, 90.0),
        "joint_limit_stop_deg": _num(0.0, 90.0),
        "j5_slow_deg": _num(0.0, 90.0),
        "j5_stop_deg": _num(0.0, 90.0),
        "shoulder_slow_mm": _num(0.0, 300.0),
        "shoulder_stop_mm": _num(0.0, 300.0),
        "elbow_slow_deg": _num(0.0, 90.0),
        "elbow_stop_deg": _num(0.0, 90.0),
        "min_speed_scale": _num(0.05, 1.0),
        "max_fk_error_mm": _num(0.1, 20.0),
        "max_gap_s": _num(0.01, 0.2),
        "max_late_cycles": _num(1, 20, integer=True),
        "max_state_age_s": _num(0.01, 1.0),
        "max_stale_cycles": _num(1, 20, integer=True),
        "max_leader_fails": _num(1, 20, integer=True),
        "hold_on_exit_s": _num(0.0, 2.0),
    },
    "tcp": {
        "expected_offset_mm": _vec(3, -500.0, 500.0, nullable=True),
        "offset_tolerance_mm": _num(0.01, 10.0),
    },
    "ready_pose": {
        "joints_deg": _vec(6, -360.0, 360.0, nullable=True),
        "move_speed_deg_s": _num(1.0, 60.0),          # move_j argument, x speed bar (0.1) = deg/s
        "tolerance_deg": _num(0.001, 1.0),
        "wrist_warning_deg": _num(0.0, 360.0),
    },
    "gripper": {
        "trigger_full": _num(1.0, 65535.0),
        "close_above": _num(0.0, 1.0),
        "open_below": _num(0.0, 1.0),
        "fail_warn": _num(1, 100, integer=True),
    },
    "logging": {
        "dir": _str(),
    },
    "runtime": {
        "cpu_cores": _vec(2, 0, 1023, nullable=True),   # [first, last] CPU to pin the process to; null = no pinning
    },
}

ROBOT_FILE = Path(__file__).resolve().parent.parent / "configs" / "robot.yaml"

# Null is allowed in Simulation (meaning "not used / not checked") but not in Real.
REQUIRED_FOR_REAL = ("range.z_min_abs_mm", "tcp.expected_offset_mm", "ready_pose.joints_deg")

# Upper limits for Real on top of SCHEMA (conservative until the RB5 values are confirmed).
REAL_MAX = {
    "mapping.speed_mm_s": 200.0,               # user, 2026-10-16 option B (was 100, 50); decisions 2026-10-16
    "mapping.scale": 1.0,
    "yaw.speed_deg_s": 30.0,
    "ready_pose.move_speed_deg_s": 50.0,      # x speed bar 0.1 = 5 deg/s (user, 2026-10-12; was 10, then 30)
}
REAL_MAX_JOINT_SPEED_DEG_PER_CYCLE = 1.2   # 60 deg/s (user, 2026-10-16 option B; was 0.8, 0.5)


def _check(spec, value, key, problems):
    if isinstance(spec, dict) and "kind" not in spec:
        if not isinstance(value, dict):
            problems.append(f"{key}: expected a section, got {value!r}")
            return
        for k in value:
            if k not in spec:
                problems.append(f"{key + '.' if key else ''}{k}: unknown key")
        for k, sub in spec.items():
            name = f"{key + '.' if key else ''}{k}"
            if k not in value:
                problems.append(f"{name}: missing")
            else:
                _check(sub, value[k], name, problems)
        return
    kind = spec["kind"]
    if kind == "str":
        if not isinstance(value, str):
            problems.append(f"{key}: expected text, got {value!r}")
        elif spec["choices"] and value not in spec["choices"]:
            problems.append(f"{key}: {value!r} is not one of {spec['choices']}")
    elif kind == "bool":
        if not isinstance(value, bool):
            problems.append(f"{key}: expected true or false, got {value!r}")
    elif kind == "num":
        if value is None:
            if not spec["nullable"]:
                problems.append(f"{key}: must not be null")
            return
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            problems.append(f"{key}: expected a number, got {value!r}")
            return
        if spec["integer"] and not float(value).is_integer():
            problems.append(f"{key}: expected an integer, got {value!r}")
        _range(spec, value, key, problems)
    elif kind == "vec":
        if value is None:
            if not spec["nullable"]:
                problems.append(f"{key}: must not be null")
            return
        if (not isinstance(value, list) or len(value) != spec["n"]
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)):
            problems.append(f"{key}: expected a list of {spec['n']} numbers, got {value!r}")
            return
        for i, v in enumerate(value):
            _range(spec, v, f"{key}[{i}]", problems)
    elif kind == "pairs":
        if value is None:
            if not spec["nullable"]:
                problems.append(f"{key}: must not be null")
            return
        if (not isinstance(value, list) or not value
                or any(not isinstance(p, list) or len(p) != 2
                       or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in p) for p in value)):
            problems.append(f"{key}: expected a list of [number, number] pairs, got {value!r}")
            return
        for i, (a, b) in enumerate(value):
            for j, v in enumerate((a, b)):
                _range({"lo": spec["lo"][j], "hi": spec["hi"][j]}, v, f"{key}[{i}][{j}]", problems)


def _range(spec, v, key, problems):
    if spec["lo"] is not None and v < spec["lo"]:
        problems.append(f"{key}: {v} is below {spec['lo']}")
    if spec["hi"] is not None and v > spec["hi"]:
        problems.append(f"{key}: {v} is above {spec['hi']}")


def _get(d, dotted):
    for part in dotted.split("."):
        d = d[part]
    return d


def _check_relations(d, problems):
    """Checks between values: stop thresholds below slow thresholds, ranges ordered, timing."""
    s = d["safety"]
    for name, slow, stop in (("joint limit", "joint_limit_slow_deg", "joint_limit_stop_deg"),
                             ("J5", "j5_slow_deg", "j5_stop_deg"),
                             ("shoulder", "shoulder_slow_mm", "shoulder_stop_mm"),
                             ("elbow", "elbow_slow_deg", "elbow_stop_deg")):
        if s[stop] >= s[slow]:
            problems.append(f"safety: {name} stop ({s[stop]}) must be below slow ({s[slow]})")
    r = d["range"]
    if r["r_min_mm"] >= r["r_max_mm"]:
        problems.append(f"range: r_min_mm ({r['r_min_mm']}) must be below r_max_mm ({r['r_max_mm']})")
    table = r.get("r_max_by_z")
    if isinstance(table, list) and table and all(isinstance(p, list) and len(p) == 2 for p in table):
        dz = [p[0] for p in table]
        if any(b <= a for a, b in zip(dz, dz[1:])):
            problems.append(f"range.r_max_by_z: heights must increase, got {dz}")
        if dz[-1] > r["z_above_mm"]:
            problems.append(f"range.r_max_by_z: height {dz[-1]} is above z_above_mm ({r['z_above_mm']})")
        for a, b in table:
            if not r["r_min_mm"] < b <= r["r_max_mm"]:
                problems.append(f"range.r_max_by_z: r {b} at height {a} must be above r_min_mm and at most r_max_mm")
    g = d["gripper"]
    if g["open_below"] >= g["close_above"]:
        problems.append(f"gripper: open_below ({g['open_below']}) must be below close_above ({g['close_above']})")
    sv = d["servo"]
    if abs(sv["t1_s"] - sv["period_s"]) > 1e-9:
        problems.append(f"servo: t1_s ({sv['t1_s']}) must equal period_s ({sv['period_s']}) (06 / 10)")
    if s["max_gap_s"] <= sv["period_s"]:
        problems.append(f"safety: max_gap_s ({s['max_gap_s']}) must be above period_s ({sv['period_s']})")


def _check_real(d, problems):
    for key in REQUIRED_FOR_REAL:
        if _get(d, key) is None:
            problems.append(f"{key}: TBD (null) - required for Real")
    for key, hi in REAL_MAX.items():
        v = _get(d, key)
        if v is not None and v > hi:
            problems.append(f"{key}: {v} is above the Real limit {hi}")
    js = d["safety"]["joint_speed_deg_per_cycle"]
    if isinstance(js, list) and any(isinstance(v, (int, float)) and v > REAL_MAX_JOINT_SPEED_DEG_PER_CYCLE for v in js):
        problems.append(f"safety.joint_speed_deg_per_cycle: {js} has values above the Real limit "
                        f"{REAL_MAX_JOINT_SPEED_DEG_PER_CYCLE}")


def _freeze(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k: _freeze(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(value)
    return value


def load_settings(path, real=False, robot_path=ROBOT_FILE):
    """Load and check a settings file. real=True also applies the Real requirements and limits.
    The robot section (ip, speed_bar) comes from robot_path (configs/robot.yaml, shared with the
    scripts) and must not be in the settings file. Returns (settings, raw dict with the robot section).
    Raises SettingsError listing every problem."""
    path = Path(path)
    with open(path) as f:
        d = yaml.safe_load(f)
    problems = []
    if "robot" in d:
        problems.append(f"robot: set in {Path(robot_path).name}, not in this file")
        raise SettingsError(path, problems)
    with open(robot_path) as f:
        d = {"robot": yaml.safe_load(f), **d}
    _check(SCHEMA, d, "", problems)
    if not problems:
        _check_relations(d, problems)
        expected_mode = "real" if real else "sim"
        if d["mode"] != expected_mode:
            problems.append(f"mode: the file says {d['mode']!r} but {expected_mode!r} was requested")
        if real:
            _check_real(d, problems)
    if problems:
        raise SettingsError(path, problems)
    return _freeze(d), d

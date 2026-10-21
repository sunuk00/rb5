"""configs/policy.yaml (p2-b): load and check the learning and policy-run settings.

Every key of the schema must be present and unknown keys are errors (as in teleop/settings.py, whose checker
this reuses), except under train.act: those are LeRobot's ACT options, passed through as --policy.<name>, so
any of them may be added there (lerobot-train rejects a name it does not know).

    cfg, raw = load_policy_config()            # cfg.dataset.image_width, cfg.train.act.chunk_size, ...
"""
from pathlib import Path

import yaml

from teleop.settings import SettingsError, _bool, _check, _freeze, _num, _str, _vec

ROOT = Path(__file__).resolve().parent.parent
DEFAULT = ROOT / "configs" / "policy.yaml"

SCHEMA = {
    "dataset": {
        "image_width": _num(32, 1280, integer=True),
        "image_height": _num(24, 960, integer=True),
        "min_clutch_s": _num(0.0, 60.0),
        "min_path_mm": _num(0.0, 1000.0),
        "keep_still": _bool(),
        "cut_pauses_s": _num(0.0, 60.0),
    },
    "train": {
        "batch_size": _num(1, 256, integer=True),
        "steps": _num(1, 10_000_000, integer=True),
        "save_freq": _num(1, 10_000_000, integer=True),
        "log_freq": _num(1, 100_000, integer=True),
        "num_workers": _num(0, 32, integer=True),
        "seed": _num(0, 2**31 - 1, integer=True),
        "device": _str(("cuda", "cpu")),
        # "act": free (see the module doc), checked below
    },
    "run": {
        "time_limit_s": _num(1.0, 3600.0),
        "max_jump_mm": _num(1.0, 300.0),
        "stale_s": _num(0.05, 5.0),
        "stop_stale_s": _num(0.1, 30.0),
        "ensemble_coeff": _num(0.0, 1.0),
        "servo_cores": _vec(2, 0, 1023),
        "policy_cores": _vec(2, 0, 1023),
        "torch_threads": _num(1, 64, integer=True),
    },
}


def check_policy_config(d):
    """List of problems of a policy config dict (empty = fine)."""
    problems = []
    if not isinstance(d, dict):
        return [f"expected sections dataset / train / run, got {d!r}"]
    train = d.get("train")
    act = train.pop("act", None) if isinstance(train, dict) else None
    try:
        _check(SCHEMA, d, "", problems)
    finally:
        if isinstance(train, dict) and act is not None:
            train["act"] = act
    if not isinstance(act, dict) or not act:
        problems.append("train.act: expected a section of ACT options (e.g. chunk_size: 100)")
    else:
        for k, v in act.items():
            if not isinstance(v, (bool, int, float, str)):
                problems.append(f"train.act.{k}: expected a number, text or true / false, got {v!r}")
        cs, na = act.get("chunk_size"), act.get("n_action_steps")
        if isinstance(cs, int) and isinstance(na, int) and na > cs:
            problems.append(f"train.act: n_action_steps ({na}) must not exceed chunk_size ({cs})")
    if problems:
        return problems
    r = d["run"]
    for k in ("servo_cores", "policy_cores"):
        if r[k][0] > r[k][1]:
            problems.append(f"run.{k}: first {r[k][0]} is after last {r[k][1]}")
    if r["stop_stale_s"] <= r["stale_s"]:
        problems.append(f"run.stop_stale_s ({r['stop_stale_s']}) must be above stale_s ({r['stale_s']})")
    s, p = r["servo_cores"], r["policy_cores"]
    if not (s[1] < p[0] or p[1] < s[0]):
        problems.append(f"run: servo_cores {s} and policy_cores {p} overlap (rec-6b: the servo loop overran)")
    return problems


def load_policy_config(path=DEFAULT):
    """(settings as read-only namespaces, the raw dict). Raises SettingsError listing every problem."""
    path = Path(path)
    with open(path) as f:
        d = yaml.safe_load(f)
    problems = check_policy_config(d)
    if problems:
        raise SettingsError(path, problems)
    return _freeze(d), d

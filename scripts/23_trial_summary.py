"""Summarize Real teleop pick-and-place trials (real-1). Read-only: reads log files, no robot or leader arm.

One trial = one teleop run = one log, data/teleop/teleop_<mode>_<time>.csv with its _settings.yaml.
Task: from the ready pose, go down at P ("pick"), then go down at Q ("place"), release the clutch, Ctrl+C.
The gripper is not connected and the trigger often reads 0 (run 20261014_035701), so pick and place are
found from the measured TCP (meas_tcp_*): the first cycle within --radius of P horizontally and below
--z-max, then the first later cycle like that at Q. Without --p / --q the task end is the last clutch
release.

Per trial (cycles with the clutch engaged unless noted):
  time_s        first clutch press -> place (or last release)
  presses       clutch presses (0 -> 1)
  engaged_s     time with the clutch engaged
  hand_mm       leader hand path x scale (leader_used_*, what the mapping used)
  target_mm     path of the target sent to the RB5
  follow_%      target_mm / hand_mm (100 % = the robot did everything the hand asked)
  dropped_mm    hand motion dropped by the limits (sum of dropped_mm; B/C mode)
  speed/joint/range/yaw %   share of engaged cycles limited by: TCP speed, joint speed, range, yaw speed
  slow %        share of engaged cycles slowed by a singularity / joint limit watch
  late          late cycles (whole run)
Manual results come from --trials, a CSV with the columns run, practice, success, failure_type, note
(run = log name without .csv; practice 1 = practice trial, shown but not in the summary).

Usage:
    python scripts/23_trial_summary.py data/teleop/teleop_real_20261016_*.csv
    python scripts/23_trial_summary.py data/teleop/teleop_real_20261016_*.csv --trials data/teleop/trials.csv \\
        --p 480 -120 --q 470 180 --z-max 120 --radius 30
"""
import argparse
import csv
import statistics
from pathlib import Path

import numpy as np
import yaml

LIMIT_COLUMNS = {"speed": "speed_limited", "joint": "joint_speed_limited", "range": "range_limited",
                 "yaw": "yaw_speed_limited"}
SLOW_COLUMNS = ["j5_slow", "shoulder_slow", "elbow_slow", "joint_limit_slow"]
FAILURE_TYPES = {"", "missed_P", "missed_Q", "timeout", "safety_stop", "contact", "other"}


def read_log(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: empty log")
    return rows


def column(rows, name, default=np.nan):
    """Float array of one column; empty cells become default."""
    return np.array([float(r[name]) if r.get(name, "") != "" else default for r in rows])


def xyz(rows, prefix, suffix=""):
    return np.column_stack([column(rows, f"{prefix}{a}{suffix}") for a in "xyz"])


def path_length(p, mask):
    """Sum of |p[i] - p[i-1]| over steps where both cycles are in mask and have values."""
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    ok = mask[1:] & mask[:-1] & np.isfinite(step)
    return float(step[ok].sum())


def first_at(meas, t, point, radius, z_max, after_t):
    """First time after after_t when the measured TCP is within radius of point (x, y) and below z_max."""
    d = np.hypot(meas[:, 0] - point[0], meas[:, 1] - point[1])
    hit = (t > after_t) & (d <= radius) & (meas[:, 2] <= z_max)
    return float(t[np.argmax(hit)]) if hit.any() else None


def summarize(path, args):
    rows = read_log(path)
    settings_path = path.with_name(path.stem + "_settings.yaml")
    s = yaml.safe_load(open(settings_path)) if settings_path.exists() else {}
    m = s.get("mapping", {})
    scale = float(m.get("scale", np.nan))
    t = column(rows, "t_s")
    eng = column(rows, "clutch_engaged", 0).astype(bool)
    dt = np.diff(t, prepend=t[0])
    presses = int(np.sum(eng[1:] & ~eng[:-1]) + eng[0])
    r = {"run": path.stem, "scale": scale, "set_speed": m.get("speed_mm_s"), "set_lag": m.get("max_lag_mm"),
         "presses": presses, "engaged_s": float(dt[eng].sum()), "pick_s": None, "place_s": None}
    if not eng.any():
        r.update(time_s=None, hand_mm=0.0, target_mm=0.0, follow=None, dropped_mm=0.0)
    else:
        t0 = float(t[np.argmax(eng)])
        t_last = float(t[len(eng) - 1 - np.argmax(eng[::-1])])
        meas = xyz(rows, "meas_tcp_")
        if args.p is not None and np.isfinite(meas).any():
            r["pick_s"] = first_at(meas, t, args.p, args.radius, args.z_max, t0)
            if r["pick_s"] is not None and args.q is not None:
                r["place_s"] = first_at(meas, t, args.q, args.radius, args.z_max, r["pick_s"])
        end = r["place_s"] if r["place_s"] is not None else t_last
        r["time_s"] = end - t0
        r["pick_s"] = None if r["pick_s"] is None else r["pick_s"] - t0
        r["place_s"] = None if r["place_s"] is None else r["place_s"] - t0
        hand = path_length(xyz(rows, "leader_used_", "_mm"), eng) * scale
        target = path_length(xyz(rows, "target_"), eng)
        r.update(hand_mm=hand, target_mm=target, follow=100.0 * target / hand if hand > 0 else None,
                 dropped_mm=float(column(rows, "dropped_mm", 0)[eng].sum()))
    n = max(int(eng.sum()), 1)
    for key, col in LIMIT_COLUMNS.items():
        r[key] = 100.0 * float(column(rows, col, 0)[eng].sum()) / n
    slow = np.zeros(len(rows), dtype=bool)
    for col in SLOW_COLUMNS:
        slow |= column(rows, col, 0).astype(bool)
    r["slow"] = 100.0 * float(slow[eng].sum()) / n
    r["late"] = int(column(rows, "late", 0).sum())
    trig = column(rows, "trigger_raw")
    r["trigger"] = "none" if not np.isfinite(trig).any() else ("all 0" if np.nanmax(trig) == 0 else "ok")
    return r


def read_trials(path):
    if path is None:
        return {}
    with open(path) as f:
        trials = {row["run"].strip().removesuffix(".csv"): row for row in csv.DictReader(f)}
    for run, row in trials.items():
        if row.get("failure_type", "").strip() not in FAILURE_TYPES:
            print(f"Warning: {run}: unknown failure_type {row['failure_type']!r} (use one of "
                  f"{sorted(FAILURE_TYPES - {''})})")
    return trials


def fmt(v, spec):
    """Format a number, or a right-aligned "-" of the same width when there is no value."""
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "-".rjust(int(spec.split(".")[0]))
    return format(v, spec)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", type=Path, help="teleop log CSV files (one per trial)")
    parser.add_argument("--trials", type=Path, help="manual results CSV: run, practice, success, failure_type, note")
    parser.add_argument("--p", type=float, nargs=2, metavar=("X", "Y"), help="pick point P in the RB5 base frame [mm]")
    parser.add_argument("--q", type=float, nargs=2, metavar=("X", "Y"), help="place point Q in the RB5 base frame [mm]")
    parser.add_argument("--z-max", type=float, default=100.0, help="TCP z at or below this counts as down [mm]")
    parser.add_argument("--radius", type=float, default=30.0, help="horizontal distance to P / Q that counts [mm]")
    args = parser.parse_args()

    trials = read_trials(args.trials)
    logs = sorted(p for p in args.logs if not p.stem.endswith("_settings"))
    results = []
    for path in logs:
        r = summarize(path, args)
        m = trials.get(r["run"], {})
        r["practice"] = m.get("practice", "").strip() == "1"
        r["success"] = m.get("success", "").strip()
        r["failure"] = m.get("failure_type", "").strip()
        results.append(r)

    head = (f"{'run':28} {'set':>14} {'P':>3} {'ok':>3} {'time_s':>6} {'pick':>5} {'place':>5} {'press':>5} "
            f"{'eng_s':>5} {'hand':>6} {'follow%':>7} {'drop':>6} {'speed%':>6} {'joint%':>6} {'range%':>6} "
            f"{'yaw%':>5} {'slow%':>5} {'late':>4} {'trig':>5}  failure")
    print(head)
    for r in results:
        setting = f"{r['scale']:g}/{r['set_speed']}/{r['set_lag']}"
        print(f"{r['run']:28} {setting:>14} {'p' if r['practice'] else '':>3} {r['success'] or '-':>3} "
              f"{fmt(r['time_s'], '6.1f')} {fmt(r['pick_s'], '5.1f')} {fmt(r['place_s'], '5.1f')} "
              f"{r['presses']:5d} {r['engaged_s']:5.1f} {r['hand_mm']:6.0f} {fmt(r['follow'], '7.0f')} "
              f"{r['dropped_mm']:6.0f} {r['speed']:6.0f} {r['joint']:6.0f} {r['range']:6.0f} {r['yaw']:5.0f} "
              f"{r['slow']:5.0f} {r['late']:4d} {r['trigger']:>5}  {r['failure']}")
    print("set = scale / TCP speed [mm/s] / C lag [mm]; P = practice; pick / place = seconds after the first press")

    groups = {}
    for r in results:
        if not r["practice"]:
            groups.setdefault((r["scale"], r["set_speed"], r["set_lag"]), []).append(r)
    for (scale, speed, lag), rs in groups.items():
        ok = [r for r in rs if r["success"] == "1"]
        print(f"\nSetting scale {scale:g}, {speed} mm/s, C {lag} mm: {len(rs)} trials, "
              + (f"{len(ok)} successful" if trials else "success unknown (no --trials)"))
        for key, label in (("time_s", "time [s]"), ("presses", "clutch presses"), ("follow", "follow [%]"),
                           ("dropped_mm", "dropped [mm]"), ("speed", "speed limited [%]"),
                           ("joint", "joint limited [%]")):
            only_ok = key == "time_s" and bool(ok)          # task time only from successful trials
            vals = [r[key] for r in (ok if only_ok else rs) if r[key] is not None]
            if vals:
                print(f"  {label:20} median {statistics.median(vals):7.1f}   mean {statistics.mean(vals):7.1f}   "
                      f"min {min(vals):7.1f}   max {max(vals):7.1f}   "
                      + ("(successful trials)" if only_ok else "(all trials)"))


if __name__ == "__main__":
    main()

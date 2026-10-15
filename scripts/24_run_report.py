"""Report on teleop runs (real-1 analyses in one script). Read-only: reads log files, no robot or leader arm.

Input: data/teleop/teleop_<mode>_<time>.csv and its _settings.yaml (caps, range, modes of that run).
Older logs without some settings keys (e.g. before the absolute range, 2026-10-16) skip those parts.

Per run:
  time       run / clutch time, normal vs left-button (rotate / vertical) time, clutch presses
  speeds     normal mode: radial / vertical speed of the target and of what the hand asked, share above
             the cap; rotate mode: J1 speed asked (hand sideways x rotate_deg_per_mm) and done
  limits     share of the engaged time per limit and mode, range limit split (r_min, r_max, floor,
             top, azimuth), slowdowns and the smallest margins, dropped hand motion per mode
  smooth     target acceleration jitter; joint speed and acceleration from the RB5 reference using the
             real sample times (t_s - state_age_ms: the state arrives 13-22 ms apart, so dividing by
             20 ms inflates the derivatives), compared with the joint caps
  lag        robot lag behind the target (before the step) and the lead limit
  timing     leader read and work time, late cycles with their details
  other      azimuth lock (J1 change in the normal mode), descent drift (radius change / descent),
             range used (absolute r, z, azimuth), RB5 J6 range

Usage:
    python scripts/24_run_report.py data/teleop/teleop_real_20261016_051339.csv
    python scripts/24_run_report.py data/teleop/teleop_real_20261016_*.csv --compare
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import yaml

DT = 0.02


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: empty log")
    cols = {}
    for k in rows[0]:
        cols[k] = np.array([float(r[k]) if r[k] != "" else np.nan for r in rows])
    settings_path = path.with_name(path.stem + "_settings.yaml")
    s = yaml.safe_load(open(settings_path)) if settings_path.exists() else {}
    return cols, s


def get(s, dotted, default=None):
    for k in dotted.split("."):
        if not isinstance(s, dict) or k not in s:
            return default
        s = s[k]
    return s


def pct(mask, within):
    n = int(within.sum())
    return 100.0 * float((mask & within).sum()) / n if n else float("nan")


def q(x, p):
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if len(x) else float("nan")


def analyze(path):
    c, s = load(path)
    n = len(c["t_s"])
    e = c["clutch_engaged"] == 1
    left_col = "left_mode_active" if "left_mode_active" in c else "vertical_mode"   # name before 2026-10-16 rotate
    left = c[left_col] == 1 if left_col in c else np.zeros(n, bool)
    left_mode = get(s, "mapping.left_button", "vertical" if get(s, "mapping.vertical_button") else "off")
    tg = np.c_[c["target_x"], c["target_y"], c["target_z"]]
    de = np.c_[c["desired_x"], c["desired_y"], c["desired_z"]]
    r = {"run": path.stem, "mode": get(s, "mode", "?")}

    # time and clutch
    on = np.diff(np.r_[0, e.astype(int), 0])
    dur = (np.where(on == -1)[0] - np.where(on == 1)[0]) * DT
    m1 = e[1:] & e[:-1]                                   # step i -> i + 1 engaged
    nrm1, lft1 = m1 & ~left[1:] & ~left[:-1], m1 & left[1:] & left[:-1]
    r.update(run_s=n * DT, engaged_s=e.sum() * DT, normal_s=nrm1.sum() * DT, left_s=lft1.sum() * DT,
             left_mode=left_mode, presses=len(dur), press_median_s=float(np.median(dur)) if len(dur) else 0.0,
             short_presses=100.0 * float(np.mean(dur < 0.5)) if len(dur) else 0.0)
    if e.sum() < 3:
        r["empty"] = True   # the clutch was never held (e.g. only the ready-pose move)
        return r

    # speeds (target vs asked) in the normal mode, split along the arm (radius) and vertical
    v = np.diff(tg, axis=0) / DT
    vd = np.diff(de, axis=0) / DT
    az = np.arctan2(tg[:-1, 1], tg[:-1, 0])
    ur = np.c_[np.cos(az), np.sin(az)]
    rad, vert = np.abs((v[:, :2] * ur).sum(1)), np.abs(v[:, 2])
    rad_d, vert_d = np.abs((vd[:, :2] * ur).sum(1)), np.abs(vd[:, 2])
    cap = get(s, "mapping.speed_mm_s", np.nan)
    asked = np.hypot(rad_d, vert_d)
    r.update(cap=cap, rad_p90=q(rad[nrm1], 90), vert_p90=q(vert[nrm1], 90), rad_ask_p90=q(rad_d[nrm1], 90),
             vert_ask_p90=q(vert_d[nrm1], 90),
             above_cap=pct(asked > cap, nrm1 & np.isfinite(asked)) if np.isfinite(cap) else float("nan"))

    # rotate mode: J1 asked (leader sideways speed x deg/mm) and done (azimuth of the target)
    k = get(s, "mapping.rotate_deg_per_mm")
    azu = np.degrees(np.unwrap(np.arctan2(tg[:, 1], tg[:, 0])))
    j1_done = np.abs(np.diff(azu)) / DT
    if left_mode == "rotate" and k and "leader_used_y_mm" in c:
        j1_ask = np.abs(np.diff(c["leader_used_y_mm"])) / DT * k
        turning = lft1 & np.isfinite(j1_ask) & (j1_ask > 5.0)
        r.update(j1_ask_p90=q(j1_ask[turning], 90), j1_done_p90=q(j1_done[turning], 90),
                 j1_done_max=float(np.nanmax(j1_done[turning])) if turning.any() else float("nan"))
    r["j1_normal_deg"] = float(np.sum(np.abs(np.diff(azu))[nrm1]))

    # limits per mode
    def flag(name):
        return (c[name][1:] == 1) if name in c else np.zeros(n - 1, bool)
    for name, col in (("speed", "speed_limited"), ("joint", "joint_speed_limited"), ("accel", "accel_limited"),
                      ("range", "range_limited"), ("lead", "lead_limited")):
        r[f"{name}_normal"], r[f"{name}_left"] = pct(flag(col), nrm1), pct(flag(col), lft1)
    slow = flag("j5_slow") | flag("shoulder_slow") | flag("elbow_slow") | flag("joint_limit_slow")
    r["slow"] = pct(slow, m1)
    r["min_margins"] = {name: float(np.nanmin(c[col][e])) for name, col in
                        (("J5 deg", "j5_eff_pred_deg"), ("shoulder mm", "shoulder_margin_pred_mm"),
                         ("elbow deg", "elbow_from_straight_pred_deg"), ("joint limit deg", "joint_limit_margin_pred_deg"))
                        if col in c}
    rl = e & (c["range_limited"] == 1)
    rng = get(s, "range", {})
    if "z_min_mm" in rng:
        rd = np.hypot(de[:, 0], de[:, 1])
        azd = np.degrees(np.arctan2(de[:, 1], de[:, 0]))
        center = 0.5 * (rng["azimuth_min_deg"] + rng["azimuth_max_deg"])
        half = 0.5 * (rng["azimuth_max_deg"] - rng["azimuth_min_deg"])
        table = rng.get("r_max_by_z")
        rmax = (np.minimum(rng["r_max_mm"], np.interp(de[:, 2], *np.asarray(table, float).T)) if table
                else np.full(n, rng["r_max_mm"]))
        r["range_split_s"] = {
            "r_min": float((rl & (rd < rng["r_min_mm"] - 0.5)).sum() * DT),
            "r_max": float((rl & (rd > rmax + 0.5)).sum() * DT),
            "floor": float((rl & (de[:, 2] < rng["z_min_mm"] - 0.5)).sum() * DT),
            "top": float((rl & (de[:, 2] > rng["z_max_mm"] + 0.5)).sum() * DT),
            "azimuth": float((rl & (np.abs((azd - center + 180) % 360 - 180) > half + 0.01)).sum() * DT)}
    dropped = c["dropped_mm"][1:]
    r["dropped_normal"], r["dropped_left"] = float(np.nansum(dropped[nrm1])), float(np.nansum(dropped[lft1]))

    # smoothness: target jitter, and joints from the RB5 reference with the real sample times
    a = np.linalg.norm(np.diff(v, axis=0), axis=1) / DT / 1000.0
    m2 = e[2:] & e[1:-1] & e[:-2]
    r.update(jitter_p50=q(a[m2], 50), jitter_p99=q(a[m2], 99))
    ts = c["t_s"] - c["state_age_ms"] / 1000.0
    new = np.r_[True, np.abs(np.diff(ts)) > 1e-4]
    jq = np.c_[[c[f"jnt_ref_{i}"] for i in range(6)]].T[new]
    t2, e2 = ts[new], e[new]
    jv = np.abs(np.diff(jq, axis=0)) / np.diff(t2)[:, None]
    tm = 0.5 * (t2[1:] + t2[:-1])
    ja = np.abs(np.diff(jv, axis=0)) / np.diff(tm)[:, None]
    mv, ma = e2[1:] & e2[:-1], e2[2:] & e2[1:-1] & e2[:-2]
    jcap = np.asarray(get(s, "safety.joint_speed_deg_per_cycle", [np.nan] * 6), float) / DT
    r.update(sample_ms=[q(np.diff(t2) * 1000, p) for p in (5, 50, 95)],
             joint_speed_max=[float(np.nanmax(jv[mv][:, i])) if mv.any() else float("nan") for i in range(6)],
             joint_cap=jcap.tolist(), above_joint_cap=pct((jv > jcap * 1.05).any(axis=1), mv),
             joint_accel_p99=q(ja[ma].max(axis=1), 99) if ma.any() else float("nan"))

    # lag behind the target and the lead limit
    prev = np.r_[[tg[0]], tg[:-1]]
    lag = np.linalg.norm(prev - np.c_[c["tcp_ref_x"], c["tcp_ref_y"], c["tcp_ref_z"]], axis=1)
    step = np.r_[0.0, np.linalg.norm(np.diff(tg, axis=0), axis=1)]
    fast = e & (step > 4.0)
    r.update(lag_p99=q(lag[e], 99), lag_steps_p99=q((lag / np.maximum(step, 1e-6))[fast], 99))

    # timing and late cycles
    r.update(leader_p99=q(c["leader_read_ms"], 99), work_p99=q(c["work_ms"], 99), late=int(np.nansum(c["late"])),
             late_details=[f"t {c['t_s'][i]:.2f} s: send gap {c['send_interval_ms'][i]:.1f} ms, plan "
                           f"{c['plan_ms'][i]:.1f} ms, work {c['work_ms'][i]:.1f} ms"
                           for i in np.where(c["late"] == 1)[0]])

    # descents in the normal mode (drift of the radius per mm of descent), range used, J6
    seg = np.cumsum(np.r_[0, np.diff(e.astype(int)) == 1])
    radius = np.hypot(tg[:, 0], tg[:, 1])
    drifts = []
    for kk in np.unique(seg[e]):
        idx = np.where(e & (seg == kk) & ~left)[0]
        if len(idx) < 10:
            continue
        z = tg[idx, 2]
        i0 = int(np.argmax(z))
        i1 = i0 + int(np.argmin(z[i0:]))
        if z[i0] - z[i1] >= 50.0:
            drifts.append((radius[idx[i1]] - radius[idx[i0]]) / (z[i1] - z[i0]))
    azabs = np.degrees(np.arctan2(tg[:, 1], tg[:, 0]))
    r.update(descent_drift=float(np.median(drifts)) if drifts else float("nan"), descents=len(drifts),
             used=f"r {np.nanmin(radius[e]):.0f}..{np.nanmax(radius[e]):.0f}, z {np.nanmin(tg[e, 2]):.0f}.."
                  f"{np.nanmax(tg[e, 2]):.0f}, azimuth {np.nanmin(azabs[e]):.0f}..{np.nanmax(azabs[e]):.0f}",
             j6_range=float(np.nanmax(c["jnt_ang_5"][e]) - np.nanmin(c["jnt_ang_5"][e])) if "jnt_ang_5" in c else float("nan"))
    return r


def f(x, spec=".0f"):
    return "-" if x is None or (isinstance(x, float) and not np.isfinite(x)) else format(x, spec)


def report(r):
    print(f"== {r['run']} ({r['mode']})")
    if r.get("empty"):
        print(f"no clutch time (run {f(r['run_s'])} s): nothing to analyze")
        return
    print(f"time      run {f(r['run_s'])} s, clutch {f(r['engaged_s'])} s (normal {f(r['normal_s'])}, "
          f"{r['left_mode']} {f(r['left_s'])}), {r['presses']} presses, median {f(r['press_median_s'], '.1f')} s, "
          f"< 0.5 s {f(r['short_presses'])} %")
    print(f"speeds    normal: radial p90 {f(r['rad_p90'])} (asked {f(r['rad_ask_p90'])}), vertical p90 "
          f"{f(r['vert_p90'])} (asked {f(r['vert_ask_p90'])}) mm/s; asked above the {f(r['cap'])} mm/s cap "
          f"{f(r['above_cap'])} % of the normal time")
    if "j1_ask_p90" in r:
        print(f"          rotate: J1 asked p90 {f(r['j1_ask_p90'])}, done p90 {f(r['j1_done_p90'])} / max "
              f"{f(r['j1_done_max'])} deg/s")
    print("limits    % of the time, normal / left: " + ", ".join(
        f"{k} {f(r[k + '_normal'])}/{f(r[k + '_left'])}" for k in ("speed", "joint", "accel", "range", "lead"))
        + f"; slowdowns {f(r['slow'])} %")
    if "range_split_s" in r:
        print("          range limit [s]: " + ", ".join(f"{k} {v:.1f}" for k, v in r["range_split_s"].items()))
    print("          smallest margins: " + ", ".join(f"{k} {v:.1f}" for k, v in r["min_margins"].items()))
    print(f"          dropped: normal {f(r['dropped_normal'])} mm, {r['left_mode']} {f(r['dropped_left'])} mm")
    print(f"smooth    target jitter p50 {f(r['jitter_p50'], '.2f')} / p99 {f(r['jitter_p99'], '.1f')} m/s^2; "
          f"joint accel p99 {f(r['joint_accel_p99'])} deg/s^2 (sample times, spacing p5/p50/p95 "
          f"{'/'.join(f(x, '.1f') for x in r['sample_ms'])} ms)")
    print("          joint speed max " + " / ".join(f(x) for x in r["joint_speed_max"]) + " deg/s, caps "
          + " / ".join(f(x) for x in r["joint_cap"]) + f"; samples > 1.05 x cap {f(r['above_joint_cap'], '.2f')} %")
    print(f"lag       robot behind the target p99 {f(r['lag_p99'], '.1f')} mm ({f(r['lag_steps_p99'], '.2f')} steps "
          f"when fast)")
    print(f"timing    leader read p99 {f(r['leader_p99'], '.1f')} ms, work p99 {f(r['work_p99'], '.1f')} ms, late "
          f"{r['late']}" + "".join(f"\n          late: {d}" for d in r["late_details"]))
    print(f"other     J1 in the normal mode {f(r['j1_normal_deg'], '.3f')} deg; descent drift "
          f"{f(r['descent_drift'], '.2f')} ({r['descents']} descents >= 50 mm); used {r['used']}; RB5 J6 range "
          f"{f(r['j6_range'])} deg")


def compare(results):
    cols = [("presses", "presses", "d"), ("normal %", "speed_normal", ".0f"), ("joint %", "joint_normal", ".0f"),
            ("drop n", "dropped_normal", ".0f"), ("drop l", "dropped_left", ".0f"), ("jit p99", "jitter_p99", ".1f"),
            ("jacc p99", "joint_accel_p99", ".0f"), ("lag p99", "lag_p99", ".1f"), ("late", "late", "d"),
            ("slow %", "slow", ".0f")]
    print(f"{'run':30}" + "".join(f"{h:>9}" for h, _, _ in cols))
    for r in results:
        print(f"{r['run']:30}" + "".join(f"{f(r.get(k), spec):>9}" for _, k, spec in cols))
    print("normal % = speed cap share of the normal-mode time; joint % = joint speed limit share; drop n / l = "
          "dropped mm in the normal / left-button mode; jacc = joint acceleration from the sample times")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", type=Path, help="teleop log CSV files")
    parser.add_argument("--compare", action="store_true", help="one line per run instead of the full report")
    args = parser.parse_args()
    logs = sorted(p for p in args.logs if not p.stem.endswith("_settings"))
    results = [analyze(p) for p in logs]
    if args.compare:
        compare(results)
    else:
        for r in results:
            report(r)
            print()


if __name__ == "__main__":
    main()

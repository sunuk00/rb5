"""rec-4a: demonstration sessions (python -m teleop --record) -> one LeRobotDataset. No robot, no cameras.

Run in the `lerobot` conda env:
    conda activate lerobot
    python -m rb5_lerobot.convert data/demos/real_20261020_100000 [more sessions ...]
    python -m rb5_lerobot.convert data/demos/real_* --repo-id local/rb5_pick_place --cut-pauses 2.0
    python -m rb5_lerobot.convert data/demos/sim_20261019_134705 --check-only
    python -m rb5_lerobot.convert data/demos/real_20261019_141929 --cameras front --repo-id local/pilot_front

The dataset is always built anew from the sessions (raw files stay as they are). Output:
data/lerobot/<name of the repo id>/ (--overwrite replaces it).

Format (docs/decisions.md rec-1), 30 fps on a time grid from each kept episode's start (1/30 s steps):
- action (4): x, y, z = the sent TCP target [mm, base frame], interpolated at the grid times (send time
  = cycle start + work_ms); gripper = the command: trigger / trigger_full while the gripper thread
  follows the trigger, else (before the catch) the position it holds, both 0 = closed .. 1 = open
- observation.state (7): joints [deg] at their read time (cycle start + leader_read_ms - state_age_ms):
  jnt_ang (encoders) in Real, jnt_ref in Simulation (jnt_ang there is the physical robot, not the
  simulated one); gripper position 0 = closed .. 1 = open at its read time (- grip_age_ms)
- observation.images.<camera> (the cameras the sessions recorded, episodes.json "cameras": since cam-5a front,
  wrist, grip): the frame whose capture time (t_capture_s) is nearest to the grid time (the cameras run at
  30 fps but are not synchronized with the grid: up to half a frame). All sessions must have the same cameras.

Episodes left out (reported): discarded ones; ones whose camera frames do not cover them (none, or a
gap / missing start or end of more than 0.2 s); still ones (clutch engaged < --min-clutch-s or target path <
--min-path-mm; --keep-still keeps them); --cut-pauses S removes stretches where the clutch was released
for more than S seconds and the gripper command did not change (the robot stands still there, so the
cut is seamless; the still tail before the operator's key press is such a stretch).

Checks printed per episode: duration, clutch time, target path, image time error (max), camera frames,
gripper following, start pose vs the ready pose; at the end the 30 -> 50 Hz resampling error of the
action (the 30 Hz action interpolated back at the original send times vs the target sent).
"""
import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
FPS = 30
H, W = 480, 640
CAMERAS = ["front", "wrist"]   # for sessions that do not list their cameras (rec-1 tests)
STATE_NAMES = ["j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg", "gripper_pos"]
ACTION_NAMES = ["x_mm", "y_mm", "z_mm", "gripper_open"]
IMAGE_DT_WARN_MS = 25.0   # nearest frame further than this from the grid time (half a frame is 16.7 ms)
CAMERA_GAP_S = 0.2        # a camera without frames this long inside an episode: the episode is left out


def features(cameras=CAMERAS):
    return {
        "observation.state": {"dtype": "float32", "shape": (7,), "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": (4,), "names": ACTION_NAMES},
        **{f"observation.images.{c}": {"dtype": "video", "shape": (H, W, 3), "names": ["height", "width", "channels"],
                                       "info": {"is_depth_map": False}} for c in cameras},
    }


def num(rows, key):
    return np.array([float(r[key]) if r[key] != "" else np.nan for r in rows])


def fill(x):
    """Blanks (failed reads) -> the previous value (the first one -> the next value)."""
    x = x.copy()
    ok = ~np.isnan(x)
    if not ok.any():
        return x
    idx = np.where(ok, np.arange(len(x)), 0)
    np.maximum.accumulate(idx, out=idx)
    x = x[idx]
    first = np.argmax(ok)
    x[:first] = x[first]
    return x


def interp(t_grid, t, x):
    """Linear interpolation; t must increase (sorted here, ties kept in order)."""
    order = np.argsort(t, kind="stable")
    return np.interp(t_grid, t[order], x[order])


class Session:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.meta = json.loads((self.folder / "episodes.json").read_text())
        self.settings = yaml.safe_load((self.folder / "settings.yaml").read_text())
        with open(self.folder / "teleop.csv") as f:
            self.rows = list(csv.DictReader(f))
        r = self.rows
        self.real = self.settings["mode"] == "real"
        g = self.settings["gripper"]
        self.closed, self.opened, self.trigger_full = g["closed_ticks"], g["open_ticks"], g["trigger_full"]
        self.ready = np.array(self.settings["ready_pose"]["joints_deg"] or [np.nan] * 6, dtype=float)
        t0 = self.meta["loop_t0"]
        t = t0 + num(r, "t_s")
        work, lread = num(r, "work_ms") / 1000.0, num(r, "leader_read_ms") / 1000.0
        self.t_send = t + work
        self.t_state = t + lread - num(r, "state_age_ms") / 1000.0
        self.t_trigger = t + lread
        self.target = np.stack([num(r, f"target_{a}") for a in "xyz"], axis=1)
        jkey = "jnt_ang" if self.real else "jnt_ref"
        self.joints = np.stack([fill(num(r, f"{jkey}_{i}")) for i in range(6)], axis=1)
        self.joint_source = jkey
        self.clutch = num(r, "clutch_engaged")
        self.episode = np.array([x["episode"] for x in r])
        span = self.opened - self.closed
        trig = fill(num(r, "trigger_raw")) / self.trigger_full
        pos = num(r, "grip_position")
        self.has_gripper = not np.isnan(pos).all()
        if self.has_gripper:
            self.grip_pos = np.clip((fill(pos) - self.closed) / span, 0.0, 1.0)
            self.t_grip = t + work - fill(num(r, "grip_age_ms")) / 1000.0
            following = num(r, "grip_following") == 1
            self.grip_cmd = np.where(following, np.clip(trig, 0.0, 1.0), self.grip_pos)
            self.following = following
        else:   # --no-gripper run: the command is the trigger, no position
            self.grip_pos = np.zeros(len(r))
            self.t_grip = self.t_send
            self.grip_cmd = np.clip(trig, 0.0, 1.0)
            self.following = np.zeros(len(r), dtype=bool)


def pauses_to_cut(t_grid, clutch_g, gcmd_g, min_s):
    """Boolean mask of grid steps to remove: clutch released and gripper command unchanged for > min_s."""
    cut = np.zeros(len(t_grid), dtype=bool)
    still = (clutch_g < 0.5) & (np.abs(np.diff(gcmd_g, prepend=gcmd_g[0])) < 1e-3)
    i = 0
    while i < len(still):
        if still[i]:
            j = i
            while j < len(still) and still[j]:
                j += 1
            if (j - i) / FPS > min_s:
                cut[i + 1:j] = True   # keep the first still step: the cut joins the same pose
            i = j
        else:
            i += 1
    return cut


def episode_frames(s, a, args, report):
    """Grid samples of one kept attempt -> list of frame dicts (or None when left out)."""
    rows = s.episode == str(a["attempt"])
    if not rows.any():
        report.append("no CSV rows")
        return None
    t_start, t_end = a["t_start"], a["t_end"]
    clutch_s = float(s.clutch[rows].sum()) * s.settings["servo"]["period_s"]
    path = float(np.linalg.norm(np.diff(s.target[rows], axis=0), axis=1).sum())
    report += [f"{t_end - t_start:5.1f} s", f"clutch {clutch_s:5.1f} s", f"path {path:5.0f} mm"]
    if (clutch_s < args.min_clutch_s or path < args.min_path_mm) and not args.keep_still:
        report.append("LEFT OUT: still (--keep-still keeps it)")
        return None

    cams = {}
    for c in args.cameras:
        d = s.folder / a["dir"]
        with open(d / f"cam_{c}.csv") as f:
            crow = list(csv.DictReader(f))
        cams[c] = (np.array([float(x["t_capture_s"]) for x in crow]),
                   [d / f"cam_{c}" / f"{int(x['index']):06d}.jpg" for x in crow])
        problem = camera_gap(cams[c][0], t_start, t_end)
        if problem:   # e.g. the D405 that dropped off USB in real-pilot 141929
            report.append(f"LEFT OUT: {c} camera {problem}")
            return None
    t_last = min(t_end, *(cams[c][0][-1] for c in args.cameras))
    t_grid = t_start + np.arange(int(np.floor((t_last - t_start) * FPS)) + 1) / FPS

    act = np.stack([interp(t_grid, s.t_send, s.target[:, k]) for k in range(3)]
                   + [interp(t_grid, s.t_trigger, s.grip_cmd)], axis=1)
    state = np.stack([interp(t_grid, s.t_state, s.joints[:, k]) for k in range(6)]
                     + [interp(t_grid, s.t_grip, s.grip_pos)], axis=1)
    clutch_g = interp(t_grid, s.t_send, s.clutch)
    keep = ~pauses_to_cut(t_grid, clutch_g, act[:, 3], args.cut_pauses) if args.cut_pauses else np.ones(len(t_grid), bool)
    if not keep.all():
        report.append(f"cut {np.sum(~keep) / FPS:4.1f} s of pauses")

    nearest, dt_max = {}, 0.0
    for c in args.cameras:
        tc, _ = cams[c]
        k = np.clip(np.searchsorted(tc, t_grid), 1, len(tc) - 1)
        k = np.where(np.abs(tc[k - 1] - t_grid) <= np.abs(tc[k] - t_grid), k - 1, k)
        nearest[c] = k
        dt_max = max(dt_max, float(np.abs(tc[k] - t_grid)[keep].max()) * 1000.0)
    report.append(f"image dt max {dt_max:4.1f} ms" + (" WARNING" if dt_max > IMAGE_DT_WARN_MS else ""))
    stats = a.get("cameras") or {}
    lost = sum(v.get("gaps", 0) + v.get("dropped", 0) for v in stats.values())
    if lost:
        report.append(f"WARNING camera frames lost {lost}")
    if s.has_gripper:
        f = float(np.mean(s.following[rows]))
        if f < 1.0:
            report.append(f"gripper following {100 * f:.0f}%" + (" WARNING" if f < 0.5 else ""))
    if not np.isnan(s.ready).any():
        report.append(f"start vs ready pose max {np.abs(state[0, :6] - s.ready).max():5.1f} deg")

    frames = []
    for i in np.where(keep)[0]:
        fr = {"observation.state": state[i].astype(np.float32), "action": act[i].astype(np.float32),
              "task": a["task"]}
        for c in args.cameras:
            fr[f"observation.images.{c}"] = cams[c][1][nearest[c][i]]   # loaded when written
        frames.append(fr)
    report.append(f"{len(frames)} frames")
    resample_err = action_resample_error(s, rows, t_grid, act)
    return frames, resample_err


def camera_gap(t, t_start, t_end):
    """Text when the frames do not cover the episode (none, late start, early end, a gap), else None."""
    if len(t) == 0:
        return "has no frames"
    if t[0] > t_start + CAMERA_GAP_S:
        return f"starts {t[0] - t_start:.1f} s late"
    if t[-1] < t_end - CAMERA_GAP_S:
        return f"frames end at {t[-1] - t_start:.1f} s of {t_end - t_start:.1f} s"
    gaps = np.diff(t)
    if len(gaps) and gaps.max() > CAMERA_GAP_S:
        k = int(np.argmax(gaps))
        return f"has a {gaps[k]:.1f} s gap at {t[k] - t_start:.1f} s"
    return None


def action_resample_error(s, rows, t_grid, act):
    """The 30 Hz action interpolated back at the original 50 Hz send times vs the target sent [mm]."""
    t = s.t_send[rows]
    inside = (t >= t_grid[0]) & (t <= t_grid[-1])
    back = np.stack([np.interp(t[inside], t_grid, act[:, k]) for k in range(3)], axis=1)
    return np.linalg.norm(back - s.target[rows][inside], axis=1)


def session_cameras(folder):
    """Camera names a session recorded (episodes.json "cameras", in their order); CAMERAS if it does not say."""
    meta = json.loads((Path(folder) / "episodes.json").read_text())
    return list(meta.get("cameras") or CAMERAS)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("sessions", nargs="+", help="session folders (data/demos/...)")
    ap.add_argument("--repo-id", default="local/rb5_demos")
    ap.add_argument("--root", help="output folder (default data/lerobot/<name of the repo id>)")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--cut-pauses", type=float, default=0.0, metavar="S", help="cut clutch-released pauses > S s (0 = off)")
    ap.add_argument("--min-clutch-s", type=float, default=1.0)
    ap.add_argument("--min-path-mm", type=float, default=30.0)
    ap.add_argument("--keep-still", action="store_true", help="keep episodes without motion")
    ap.add_argument("--check-only", action="store_true", help="report only, write nothing")
    ap.add_argument("--cameras", help="cameras in the dataset, e.g. front (tests only; default: all the sessions "
                                       "recorded)")
    args = ap.parse_args()
    recorded = [session_cameras(f) for f in args.sessions]
    if any(r != recorded[0] for r in recorded):
        raise SystemExit("sessions recorded different cameras: " + "; ".join(
            f"{Path(f).name} {r}" for f, r in zip(args.sessions, recorded)))
    if args.cameras is None:
        args.cameras = recorded[0]
        print(f"Cameras: {', '.join(args.cameras)}")
    else:
        args.cameras = [c.strip() for c in args.cameras.split(",") if c.strip()]
        unknown = set(args.cameras) - set(recorded[0])
        if unknown or not args.cameras:
            raise SystemExit(f"--cameras: use names from {recorded[0]}")
        print(f"Cameras: {', '.join(args.cameras)} only (a test dataset; the sessions have {', '.join(recorded[0])})")

    root = Path(args.root) if args.root else ROOT / "data" / "lerobot" / args.repo_id.split("/")[-1]
    episodes, errors, sources = [], [], set()
    for folder in args.sessions:
        s = Session(folder)
        sources.add(s.joint_source)
        print(f"\n{s.folder.name} ({'Real' if s.real else 'Simulation'}, joints from {s.joint_source}"
              f"{'' if s.has_gripper else ', no gripper'}):")
        for a in s.meta["attempts"]:
            report = []
            if a["status"] != "kept":
                print(f"  attempt {a['attempt']:3d}: {a['status']} ({a['reason']}), left out")
                continue
            out = episode_frames(s, a, args, report)
            print(f"  attempt {a['attempt']:3d}: " + ", ".join(report))
            if out is not None:
                episodes.append(out[0])
                errors.append(out[1])
    if not episodes:
        raise SystemExit("\nno episodes to write")
    if len(sources) > 1:
        print("\nWARNING: sessions mix Real and Simulation (joints from jnt_ang and jnt_ref)")
    err = np.concatenate(errors)
    n = sum(len(e) for e in episodes)
    print(f"\n{len(episodes)} episodes, {n} frames ({n / FPS / 60:.1f} min). Action 30 -> 50 Hz resampling error "
          f"p50 / p99 / max {np.percentile(err, 50):.2f} / {np.percentile(err, 99):.2f} / {err.max():.2f} mm")
    if args.check_only:
        return

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    if root.exists():
        if not args.overwrite:
            raise SystemExit(f"{root} exists; --overwrite replaces it")
        shutil.rmtree(root)
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=FPS, features=features(args.cameras), root=root,
                               robot_type="rb5_850", use_videos=True)
    for i, frames in enumerate(episodes):
        for fr in frames:
            for c in args.cameras:
                key = f"observation.images.{c}"
                fr[key] = np.asarray(Image.open(fr[key]).convert("RGB"))
            ds.add_frame(fr)
        ds.save_episode()
        print(f"  written episode {i} ({len(frames)} frames)")
    ds.finalize()
    size = sum(f.stat().st_size for f in root.rglob("*") if f.is_file()) / 1e6
    print(f"\ndataset {args.repo_id}: {root} ({size:.0f} MB)")
    print(f"view: lerobot-dataset-viz --repo-id {args.repo_id} --root {root} --episode-index 0")


if __name__ == "__main__":
    sys.exit(main())

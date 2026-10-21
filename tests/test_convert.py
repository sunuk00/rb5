"""Tests for rb5_lerobot/convert.py (rec-4a) on a synthetic session: grid times, action and state
values, the gripper command before / after the catch, nearest camera frames, still episodes, pause cuts.
Only the parts before the LeRobot writer (runs in env rb5 too). Run: python -m unittest discover tests"""
import csv
import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.loop import CSV_HEADER  # noqa: E402

spec = importlib.util.spec_from_file_location("convert", ROOT / "rb5_lerobot" / "convert.py")
cv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv)

T0 = 1000.0          # loop_t0 (monotonic)
CLOSED, OPEN = 3968, 2196


def make_session(folder, real=True, follow_from=0, clutch_off=(), cam_offset_ms=(0.0, 10.0), episode_rows=(50, 350)):
    """10 s at 50 Hz; episode 0 = rows episode_rows; target x = 100 * t_send (mm), joints j = 10 * t_state,
    trigger 0..1000 ramp, gripper position follows the trigger from row follow_from."""
    folder.mkdir(parents=True)
    n = 500
    rows = []
    for i in range(n):
        t = i * 0.02
        r = dict.fromkeys(CSV_HEADER, "")
        r.update(t_s=f"{t:.4f}", work_ms="4.0", leader_read_ms="2.0", state_age_ms="5.0", grip_age_ms="10.0",
                 clutch_engaged=0 if i in clutch_off else 1, trigger_raw=f"{i * 2}",
                 grip_following=1 if i >= follow_from else 0)
        ts, tst = T0 + t + 0.004, T0 + t + 0.002 - 0.005
        r.update(target_x=f"{100 * ts:.3f}", target_y="0", target_z="0")
        for k in range(6):
            r[f"jnt_ang_{k}"] = f"{10 * tst + k:.4f}"
            r[f"jnt_ref_{k}"] = f"{-1:.1f}"
        frac = (i * 2) / 1000.0 if i >= follow_from else 0.5
        r["grip_position"] = f"{CLOSED + frac * (OPEN - CLOSED):.1f}"
        r["episode"] = 0 if episode_rows[0] <= i < episode_rows[1] else ""
        rows.append(r)
    with open(folder / "teleop.csv", "w", newline="") as f:
        w = csv.DictWriter(f, CSV_HEADER)
        w.writeheader()
        w.writerows(rows)
    t_start, t_end = T0 + episode_rows[0] * 0.02 + 0.01, T0 + episode_rows[1] * 0.02
    meta = {"loop_t0": T0, "attempts": [
        {"attempt": 0, "dir": "episode_000", "status": "kept", "reason": "-> key", "task": "pick",
         "t_start": t_start, "t_end": t_end, "cameras": {c: {"frames": 0, "gaps": 0, "dropped": 0} for c in cv.CAMERAS}},
        {"attempt": 1, "dir": "episode_001", "status": "discarded", "reason": "<- key", "task": "pick",
         "t_start": t_end, "t_end": t_end + 1, "cameras": None}]}
    (folder / "episodes.json").write_text(json.dumps(meta))
    settings = {"mode": "real" if real else "sim", "servo": {"period_s": 0.02},
                "gripper": {"closed_ticks": CLOSED, "open_ticks": OPEN, "trigger_full": 1000.0},
                "ready_pose": {"joints_deg": [0, 0, 0, 0, 0, 0]}}
    (folder / "settings.yaml").write_text(yaml.safe_dump(settings))
    ep = folder / "episode_000"
    for c, off in zip(cv.CAMERAS, cam_offset_ms):
        (ep / f"cam_{c}").mkdir(parents=True)
        n_f = int((t_end - t_start) * 30) + 3
        with open(ep / f"cam_{c}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["index", "frame_number", "t_capture_s"])
            for k in range(n_f):
                w.writerow([k, k, f"{t_start - 0.005 + off / 1000 + k / 30:.6f}"])
    return meta


def args(**kw):
    d = dict(min_clutch_s=1.0, min_path_mm=30.0, keep_still=False, cut_pauses=0.0, cameras=list(cv.CAMERAS))
    d.update(kw)
    return types.SimpleNamespace(**d)


class TestConvert(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name) / "s"

    def convert(self, **kw):
        s = cv.Session(self.dir)
        report = []
        out = cv.episode_frames(s, s.meta["attempts"][0], args(**kw), report)
        return s, out, report

    def test_grid_action_state_values(self):
        meta = make_session(self.dir)
        s, (frames, err), report = self.convert()
        a = meta["attempts"][0]
        t_grid = a["t_start"] + np.arange(len(frames)) / 30
        act = np.array([f["action"] for f in frames])
        st = np.array([f["observation.state"] for f in frames])
        np.testing.assert_allclose(act[:, 0], 100 * t_grid, atol=0.05)      # target at its send time
        np.testing.assert_allclose(st[:, 0], 10 * t_grid, atol=0.01)        # joints at their read time
        np.testing.assert_allclose(st[:, 5], 10 * t_grid + 5, atol=0.01)
        self.assertEqual(frames[0]["task"], "pick")
        self.assertEqual(s.joint_source, "jnt_ang")
        self.assertLess(err.max(), 0.05)                                     # linear target: exact back
        self.assertAlmostEqual(len(frames), (a["t_end"] - a["t_start"]) * 30, delta=1.5)

    def test_simulation_uses_jnt_ref(self):
        make_session(self.dir, real=False)
        s, (frames, _), _ = self.convert()
        self.assertEqual(s.joint_source, "jnt_ref")
        self.assertAlmostEqual(float(frames[0]["observation.state"][0]), -1.0)

    def test_gripper_command_before_and_after_catch(self):
        make_session(self.dir, follow_from=200)
        s, (frames, _), report = self.convert()
        g = np.array([f["action"][3] for f in frames])
        pos = np.array([f["observation.state"][6] for f in frames])
        t = s.meta["attempts"][0]["t_start"] + np.arange(len(frames)) / 30 - T0
        before = t < 200 * 0.02 - 0.05
        np.testing.assert_allclose(g[before], 0.5, atol=1e-6)               # holds its position: 0.5 open
        after = t > 200 * 0.02 + 0.05
        np.testing.assert_allclose(g[after], np.clip((t[after] - 0.002) / 0.02 * 2 / 1000, 0, 1), atol=0.01)
        self.assertTrue(np.all((pos >= 0) & (pos <= 1)))
        self.assertTrue(any("gripper following" in x for x in report))

    def test_nearest_camera_frame(self):
        make_session(self.dir, cam_offset_ms=(0.0, 12.0))
        _, (frames, _), report = self.convert()
        self.assertEqual(frames[0]["observation.images.front"].name, "000000.jpg")
        self.assertEqual(frames[30]["observation.images.front"].name, "000030.jpg")
        self.assertEqual(frames[30]["observation.images.wrist"].name, "000030.jpg")   # 12 - 5 = 7 ms later
        dt = [x for x in report if x.startswith("image dt max")][0]
        self.assertLess(float(dt.split()[3]), 16.7)

    def test_still_episode_left_out(self):
        make_session(self.dir, clutch_off=set(range(500)))
        _, out, report = self.convert()
        self.assertIsNone(out)
        self.assertIn("LEFT OUT", report[-1])
        _, out, _ = self.convert(keep_still=True)
        self.assertIsNotNone(out)

    def test_cut_pauses(self):
        # clutch released 3 s inside the episode, trigger frozen there
        make_session(self.dir, clutch_off=set(range(150, 300)))
        lines = (self.dir / "teleop.csv").read_text().splitlines()
        hdr = lines[0].split(",")
        k = hdr.index("trigger_raw")
        out = [lines[0]]
        for i, line in enumerate(lines[1:]):
            f = line.split(",")
            if 150 <= i < 300:
                f[k] = "300"
            out.append(",".join(f))
        (self.dir / "teleop.csv").write_text("\n".join(out) + "\n")
        _, (all_frames, _), _ = self.convert()
        _, (cut_frames, _), report = self.convert(cut_pauses=2.0)
        removed = len(all_frames) - len(cut_frames)
        self.assertAlmostEqual(removed / 30, 3.0, delta=0.15)
        self.assertTrue(any(x.startswith("cut") for x in report))
        _, (short, _), _ = self.convert(cut_pauses=5.0)   # the pause is shorter: nothing cut
        self.assertEqual(len(short), len(all_frames))

    def test_camera_that_stopped_leaves_the_episode_out(self):
        meta = make_session(self.dir)
        a = meta["attempts"][0]
        path = self.dir / "episode_000" / "cam_wrist.csv"
        lines = path.read_text().splitlines()
        path.write_text("\n".join(lines[:100]) + "\n")   # frames end after about 3.3 s of 6 s
        _, out, report = self.convert()
        self.assertIsNone(out)
        self.assertIn("LEFT OUT: wrist camera frames end at", report[-1])
        _, out, _ = self.convert(cameras=["front"])        # front only: kept
        self.assertIsNotNone(out)
        self.assertNotIn("observation.images.wrist", out[0][0])
        path.write_text(lines[0] + "\n")                   # no frames at all
        _, out, report = self.convert()
        self.assertIn("has no frames", report[-1])
        self.assertGreater(a["t_end"], a["t_start"])

    def test_camera_gap(self):
        t = np.arange(0, 6, 1 / 30)
        self.assertIsNone(cv.camera_gap(t, 0.0, 5.9))
        self.assertIn("gap", cv.camera_gap(np.r_[t[:50], t[60:]], 0.0, 5.9))
        self.assertIn("late", cv.camera_gap(t[30:], 0.0, 5.9))

    def test_fill_blanks(self):
        x = cv.fill(np.array([np.nan, 1.0, np.nan, 3.0, np.nan]))
        np.testing.assert_array_equal(x, [1.0, 1.0, 1.0, 3.0, 3.0])


if __name__ == "__main__":
    unittest.main()

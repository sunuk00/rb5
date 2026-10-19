"""Tests for teleop.cameras (rec-3a): the camera process with fake cameras (no hardware): kept and
discarded episodes, files, CSV, errors. Run: python -m unittest discover tests"""
import csv
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.cameras import CAMERA_HEADER, CameraRecorder  # noqa: E402

CAMS = [("front", "1"), ("wrist", "2")]


class TestCameraRecorder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rec = CameraRecorder(CAMS, width=64, height=48, fps=30, fake=True)
        cls.problems = cls.rec.open()

    @classmethod
    def tearDownClass(cls):
        cls.rec.close()

    def setUp(self):
        self.assertEqual(self.problems, [])
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_kept_episode_has_frames_and_times(self):
        ep = Path(self.tmp.name) / "episode_000"
        self.rec.start_episode(ep)
        time.sleep(0.5)
        self.rec.stop_episode(keep=True)
        stats = self.rec.wait_stopped()
        self.assertEqual(set(stats), {"front", "wrist"})
        for name, _ in CAMS:
            with open(ep / f"cam_{name}.csv") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], CAMERA_HEADER)
            n = len(rows) - 1
            self.assertEqual(n, stats[name]["frames"])
            self.assertTrue(10 <= n <= 18, n)   # 0.5 s at 30 fps
            self.assertEqual(len(list((ep / f"cam_{name}").glob("*.jpg"))), n)
            t = [float(r[2]) for r in rows[1:]]
            self.assertEqual(t, sorted(t))
            self.assertEqual(stats[name]["dropped"], 0)

    def test_discarded_episode_leaves_no_camera_files(self):
        ep = Path(self.tmp.name) / "episode_001"
        self.rec.start_episode(ep)
        time.sleep(0.2)
        self.rec.stop_episode(keep=False)
        stats = self.rec.wait_stopped()
        self.assertIsNotNone(stats)
        self.assertFalse(ep.exists())   # the whole episode folder, not only its files

    def test_no_frames_between_episodes(self):
        ep = Path(self.tmp.name) / "episode_002"
        self.rec.start_episode(ep)
        time.sleep(0.2)
        self.rec.stop_episode(keep=True)
        self.rec.wait_stopped()
        before = len(list((ep / "cam_front").glob("*.jpg")))
        time.sleep(0.3)
        self.assertEqual(len(list((ep / "cam_front").glob("*.jpg"))), before)

    def test_delete_after_stop_and_poll_does_not_wait(self):
        ep = Path(self.tmp.name) / "episode_003"
        self.rec.start_episode(ep)
        time.sleep(0.2)
        t0 = time.monotonic()
        self.rec.stop_episode(keep=True)
        self.rec.delete_episode(ep)
        self.assertLess(time.monotonic() - t0, 0.01)   # the teleop loop must not wait
        msgs, end = [], time.monotonic() + 5.0
        while time.monotonic() < end and not any(m[0] == "deleted" for m in msgs):
            msgs += self.rec.poll()
            time.sleep(0.01)
        kinds = [m[0] for m in msgs]
        self.assertLess(kinds.index("stopped"), kinds.index("deleted"))
        self.assertFalse(ep.exists())


class TestOpenFailure(unittest.TestCase):
    def test_missing_camera_is_reported(self):
        rec = CameraRecorder([("front", "no-such-serial")], fake=False)
        problems = rec.open(timeout=20.0)
        rec.close()
        self.assertEqual(len(problems), 1)
        self.assertIn("no-such-serial", problems[0])


if __name__ == "__main__":
    unittest.main()

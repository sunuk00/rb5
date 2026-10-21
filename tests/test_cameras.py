"""Tests for teleop.cameras (rec-3a): the camera process with fake cameras (no hardware): kept and
discarded episodes, files, CSV, errors. Run: python -m unittest discover tests"""
import csv
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.cameras import (CAMERA_HEADER, CameraRecorder, FakeCamera, brightness_problems,  # noqa: E402
                           capture_delay_s, image_level, level_info, open_camera, saturation_problems,
                           uvc_controls, uvc_mismatch)

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


class TestStall(unittest.TestCase):
    def test_a_camera_that_stops_is_reported(self):
        rec = CameraRecorder([("front", "1"), ("wrist", "stall@10")], width=64, height=48, fake=True)
        self.addCleanup(rec.close)
        self.assertEqual(rec.open(), [])
        errors, end = [], time.monotonic() + 5.0
        while not errors and time.monotonic() < end:
            errors += rec.poll_errors()
            time.sleep(0.05)
        self.assertEqual(len(errors), 1)
        self.assertIn("wrist: no frames for 1 s", errors[0])
        time.sleep(1.5)
        self.assertEqual(rec.poll_errors(), [])   # reported once


class TestSettingsAndBrightness(unittest.TestCase):
    SET = {"exposure_us": 8333.0, "gain": 61, "white_balance_k": 4600}

    def test_settings_and_brightness_reported(self):
        rec = CameraRecorder([("front", "bright@114", self.SET), ("wrist", "bright@80")], width=64, height=48, fake=True)
        self.addCleanup(rec.close)
        self.assertEqual(rec.open(), [])
        self.assertEqual(rec.info["front"]["settings"], self.SET)
        self.assertIsNone(rec.info["wrist"]["settings"])          # no settings: auto
        self.assertAlmostEqual(rec.info["front"]["brightness"], 114.0)
        problems = brightness_problems(rec.info, 114.0, 0.15)
        self.assertEqual(len(problems), 1)
        self.assertTrue(problems[0].startswith("wrist: mean brightness 80.0 outside 97..131"))
        self.assertTrue(rec.info["front"]["brightness_ok"])
        self.assertFalse(rec.info["wrist"]["brightness_ok"])

    def test_settings_read_back_mismatch_refuses(self):
        rec = CameraRecorder([("front", "badsettings", self.SET)], width=64, height=48, fake=True)
        problems = rec.open()
        rec.close()
        self.assertEqual(len(problems), 1)
        self.assertIn("front (badsettings): settings read back", problems[0])

    def test_saturated_start_is_reported(self):
        rec = CameraRecorder([("front", "bright@114", self.SET), ("wrist", "bright@253", self.SET)],
                             width=64, height=48, fake=True)
        self.addCleanup(rec.close)
        self.assertEqual(rec.open(), [])
        self.assertEqual(rec.info["front"]["saturated_frac"], 0.0)
        self.assertEqual(rec.info["wrist"]["saturated_frac"], 1.0)
        problems = saturation_problems(rec.info)
        self.assertEqual(len(problems), 1)
        self.assertTrue(problems[0].startswith("wrist: 100 % of the pixels saturated"))

    def test_saturation_measure(self):
        img = np.full((10, 10, 3), 100, dtype=np.uint8)
        img[:3] = 255                                    # 30 % white
        img[3, :, 0] = 255                               # red only: not saturated (one channel)
        mean, frac = image_level(img)
        self.assertAlmostEqual(frac, 0.3)
        self.assertEqual(saturation_problems({"a": level_info([(mean, frac)])})[0][:3], "a: ")
        self.assertEqual(saturation_problems({"a": level_info([(mean, 0.2)])}), [])   # under 25 %
        self.assertEqual(saturation_problems({"a": level_info([])}), [])              # unknown: not refused here

    def test_brightness_limits(self):
        info = {"a": {"brightness": 97.0}, "b": {"brightness": 131.0}, "c": {"brightness": 96.8}, "d": {"brightness": None}}
        problems = brightness_problems(info, 114.0, 0.15)
        self.assertEqual([p.split(":")[0] for p in problems], ["c", "d"])


class TestOpenFailure(unittest.TestCase):
    def test_missing_camera_is_reported(self):
        rec = CameraRecorder([("front", "no-such-serial")], fake=False)
        problems = rec.open(timeout=20.0)
        rec.close()
        self.assertEqual(len(problems), 1)
        self.assertIn("no-such-serial", problems[0])

    def test_missing_webcam_is_reported(self):
        rec = CameraRecorder([("wrist", "/dev/v4l/by-id/no-such-webcam")], fake=False)
        problems = rec.open(timeout=20.0)
        rec.close()
        self.assertEqual(len(problems), 1)
        self.assertIn("wrist (/dev/v4l/by-id/no-such-webcam): webcam", problems[0])
        self.assertIn("not found", problems[0])


class TestWebcam(unittest.TestCase):
    """UVC webcam (cam-4a) without the device: class choice, control values, read-back check, times."""
    SET = {"exposure_us": 8333.0, "gain": 128, "white_balance_k": 4600}

    def test_fake_wins_over_the_path(self):
        self.assertIsInstance(open_camera("/dev/video99", 64, 48, 30, fake=True), FakeCamera)

    def test_controls(self):
        self.assertEqual(uvc_controls(self.SET), {"auto_exposure": 1, "exposure_dynamic_framerate": 0,
                                                  "white_balance_automatic": 0, "exposure_time_absolute": 83,
                                                  "gain": 128, "white_balance_temperature": 4600})

    def test_read_back(self):
        want = uvc_controls(self.SET)
        self.assertEqual(uvc_mismatch(want, dict(want, exposure_time_absolute=88)), [])   # C270 while streaming
        self.assertEqual(uvc_mismatch(want, dict(want, exposure_time_absolute=94)),
                         ["exposure_time_absolute 94 (set 83)"])
        self.assertEqual(uvc_mismatch(want, dict(want, gain=129)), ["gain 129 (set 128)"])
        self.assertEqual(uvc_mismatch(want, dict(want, auto_exposure=3)), ["auto_exposure 3 (set 1)"])
        got = dict(want)
        del got["exposure_dynamic_framerate"]
        self.assertEqual(uvc_mismatch(want, got), ["exposure_dynamic_framerate None (set 0)"])

    def test_capture_delay(self):
        self.assertAlmostEqual(capture_delay_s(1000.0, "v4l2", 1.033, 5000.0), 0.033)        # monotonic ms
        self.assertAlmostEqual(capture_delay_s(4990.5, "global_time", 1.0, 5000.0), 0.0095)  # wall clock ms
        self.assertEqual(capture_delay_s(123.0, "hardware_clock", 1.0, 5000.0), 0.0)


if __name__ == "__main__":
    unittest.main()

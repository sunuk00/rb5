"""Tests for teleop.episodes (rec-3b): key parsing, recording / reset phases, episodes.json, the loop's
`episode` column and the q key. Fake keys, clock and cameras (no hardware). Run: python -m unittest discover tests"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from teleop.episodes import EpisodeRecorder, KeyReader, Speaker, parse_keys  # noqa: E402
from test_loop import ARM, IX, run  # noqa: E402

Q = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
TCP = [500.0, 0.0, 200.0, 180.0, 0.0, 90.0]


class FakeKeys:
    def __init__(self):
        self.pending = []

    def press(self, *keys):
        self.pending += keys

    def keys(self):
        out, self.pending = self.pending, []
        return out


class FakeCameras:
    """Records the commands; poll() answers every stop at once."""

    def __init__(self):
        self.calls, self.replies = [], []

    def start_episode(self, d):
        self.calls.append(("start", Path(d).name))

    def stop_episode(self, keep):
        self.calls.append(("stop", keep))
        self.replies.append(("stopped", keep, {"front": {"frames": 3, "gaps": 0, "dropped": 0, "write_ms_max": 1.0}}))

    def delete_episode(self, d):
        self.calls.append(("delete", Path(d).name))

    def poll(self):
        out, self.replies = self.replies, []
        return out


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class TestParseKeys(unittest.TestCase):
    def test_letters_arrows_and_esc(self):
        self.assertEqual(parse_keys(b"n"), ["next"])
        self.assertEqual(parse_keys(b"R"), ["rerecord"])
        self.assertEqual(parse_keys(b"\x1b[C"), ["next"])
        self.assertEqual(parse_keys(b"\x1bOD"), ["rerecord"])   # SS3 form (tmux, ssh)
        self.assertEqual(parse_keys(b"\x1b"), ["quit"])
        self.assertEqual(parse_keys(b"q"), ["quit"])

    def test_other_keys_ignored(self):
        self.assertEqual(parse_keys(b"\x1b[A\x1b[Bx s\r"), [])
        self.assertEqual(parse_keys(b"n\x1b[Dq"), ["next", "rerecord", "quit"])


class TestKeyReader(unittest.TestCase):
    def test_pseudo_terminal_keys_and_restore(self):
        import os
        import pty
        import termios
        import time
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        before = termios.tcgetattr(slave)
        kr = KeyReader(fd=slave)
        self.assertTrue(kr.start())
        self.assertFalse(termios.tcgetattr(slave)[3] & termios.ICANON)   # no Enter needed
        os.write(master, b"n")
        os.write(master, b"\x1b[D")
        keys, end = [], time.monotonic() + 2.0
        while len(keys) < 2 and time.monotonic() < end:
            keys += kr.keys()
            time.sleep(0.01)
        kr.stop()
        self.assertEqual(keys, ["next", "rerecord"])
        self.assertEqual(termios.tcgetattr(slave), before)

    def test_not_a_terminal(self):
        import os
        r, w = os.pipe()
        self.addCleanup(os.close, r)
        self.addCleanup(os.close, w)
        self.assertFalse(KeyReader(fd=r).start())


class TestEpisodeRecorder(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.keys, self.cams, self.clock = FakeKeys(), FakeCameras(), Clock()
        self.said = []
        self.rec = EpisodeRecorder(Path(self.tmp.name) / "s", "pick", 120.0, 0.0, self.keys, self.cams,
                                   clock=self.clock, say=self.said.append)

    def step(self, *keys, dt=0.02):
        self.clock.t += dt
        self.keys.press(*keys)
        return self.rec.cycle(Q, TCP)

    def saved(self):
        return json.loads((Path(self.tmp.name) / "s" / "episodes.json").read_text())

    def test_starts_in_reset_and_next_starts_and_keeps(self):
        self.assertFalse(self.rec.recording)
        self.assertEqual(self.rec.column(), "")
        self.step("next")
        self.assertTrue(self.rec.recording)
        self.assertEqual(self.rec.column(), 0)
        self.step(dt=5.0)
        self.step("next")
        self.assertFalse(self.rec.recording)
        a = self.saved()["attempts"][0]
        self.assertEqual((a["status"], a["start_joints_deg"], a["task"]), ("kept", Q, "pick"))
        self.assertAlmostEqual(a["t_end"] - a["t_start"], 5.02)
        self.assertEqual(self.cams.calls, [("start", "episode_000"), ("stop", True)])
        self.step()   # the camera result arrives on a later cycle
        self.assertEqual(self.saved()["attempts"][0]["cameras"]["front"]["frames"], 3)

    def test_left_discards_while_recording_and_rerecords(self):
        self.step("next")
        self.step("rerecord")
        self.step("next")
        a = self.saved()["attempts"]
        self.assertEqual([x["status"] for x in a], ["discarded", "recording"])
        self.assertEqual(self.rec.kept(), 0)
        self.assertEqual(self.cams.calls[1], ("stop", False))
        self.assertEqual(self.rec.column(), 1)   # attempt numbers are unique

    def test_left_in_reset_discards_the_episode_just_kept(self):
        self.step("next")
        self.step("next")
        self.assertEqual(self.rec.kept(), 1)
        self.step("rerecord")
        self.assertEqual(self.rec.kept(), 0)
        self.assertEqual(self.saved()["attempts"][0]["status"], "discarded")
        self.assertIn(("delete", "episode_000"), self.cams.calls)
        events, _ = self.step("rerecord")   # only the last one
        self.assertIn("nothing to discard", events[0])

    def test_time_limit_ends_and_keeps(self):
        self.step("next")
        self.step(dt=119.9)
        self.assertTrue(self.rec.recording)
        self.step(dt=0.2)
        self.assertFalse(self.rec.recording)
        a = self.saved()["attempts"][0]
        self.assertEqual(a["status"], "kept")
        self.assertIn("time limit 120", a["reason"])

    def test_no_automatic_start_with_reset_time_0(self):
        self.step("next")
        self.step("next")
        self.step(dt=600.0)
        self.assertFalse(self.rec.recording)

    def test_reset_time_starts_the_next_episode(self):
        self.rec.reset_time_s = 10.0
        self.step(dt=20.0)
        self.assertFalse(self.rec.recording)   # the first one only with ->
        self.step("next")
        self.step("next")
        self.step(dt=9.9)
        self.assertFalse(self.rec.recording)
        self.step(dt=0.2)
        self.assertTrue(self.rec.recording)

    def test_quit_discards_the_episode_being_recorded(self):
        self.step("next")
        _, quit_ = self.step("quit")
        self.assertTrue(quit_)
        events = self.rec.finish("teleop stopped")
        self.assertIn("DISCARDED", events[0])
        self.assertEqual(self.saved()["attempts"][0]["status"], "discarded")
        self.assertEqual(self.rec.pending_stops, 0)

    def test_camera_error_discards_and_blocks_new_episodes(self):
        self.step("next")
        self.cams.replies.append(("error", "wrist: USB disconnected"))
        events, _ = self.step()
        self.assertFalse(self.rec.recording)
        self.assertEqual(self.saved()["attempts"][0]["status"], "discarded")
        events, _ = self.step("next")
        self.assertFalse(self.rec.recording)
        self.assertIn("camera problem", events[-1])

    def test_camera_info_in_episodes_json(self):
        info = {"front": {"serial": "1", "settings": {"exposure_us": 16667.0, "gain": 76, "white_balance_k": 4600},
                          "brightness": 113.2, "brightness_ref": 114.0, "brightness_ok": True}}
        rec = EpisodeRecorder(Path(self.tmp.name) / "s2", "pick", 120.0, 0.0, self.keys, self.cams,
                              clock=self.clock, camera_info=info)
        rec.cycle(Q, TCP)
        data = json.loads((Path(self.tmp.name) / "s2" / "episodes.json").read_text())
        self.assertEqual(data["cameras"], info)

    def test_spoken_messages(self):
        self.step("next")
        self.step("next")
        self.step("rerecord")
        self.step("next")
        self.step(dt=120.0)
        self.step("next")
        self.cams.replies.append(("error", "front: USB"))
        self.step()
        self.assertEqual(self.said, ["Recording episode 0", "Episode saved", "Episode discarded", "Recording episode 0",
                                     "Time limit, episode saved", "Recording episode 1", "Camera error",
                                     "Episode discarded"])


class TestSpeaker(unittest.TestCase):
    def test_runs_the_command_with_the_text(self):
        import os
        import stat
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "said.txt"
            fake = Path(tmp) / "fake-say"
            fake.write_text(f"#!/bin/sh\necho \"$@\" >> {out}\n")
            fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
            sp = Speaker(True, command=str(fake))
            self.assertTrue(sp.enabled)
            sp.say("Episode saved")
            sp.wait()
            self.assertEqual(out.read_text().splitlines(), ["-C", "Episode saved"])
            self.assertTrue(os.access(fake, os.X_OK))

    def test_missing_command_disables(self):
        sp = Speaker(True, command="no-such-command-xyz")
        self.assertFalse(sp.enabled)
        sp.say("x")
        sp.wait()


class TestLoopWithRecorder(unittest.TestCase):
    def test_episode_column_and_q_stops_the_loop(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        keys = FakeKeys()
        rec = EpisodeRecorder(Path(tmp.name), "pick", 120.0, 0.0, keys)

        class Keyed:
            """Presses keys at given cycles (counted by key reads)."""
            def __init__(self, plan):
                self.plan, self.n = plan, 0

            def keys(self):
                self.n += 1
                return self.plan.get(self.n, [])

        rec.keys = Keyed({8: ["next"], 12: ["next"], 15: ["quit"]})
        steps = ARM + [(1, [0, 0, 0], 0)] * 30
        loop, _, _, _, stop = run(steps, recorder=rec)
        self.assertEqual(stop, "session ended (q / Esc)")
        ep = [r[IX["episode"]] for r in loop.rows]
        self.assertEqual(ep[:7], [""] * 7)
        self.assertEqual(ep[7:11], [0] * 4)
        self.assertEqual(ep[11:], [""] * (len(ep) - 11))
        self.assertEqual(len(loop.rows), 14)   # the quit cycle raises before its row
        self.assertIsInstance(loop.rows[0], tuple)


if __name__ == "__main__":
    unittest.main()

"""Tests for teleop.policy (rec-6a): ChunkBuffer (temporal ensembling by time), keys, PolicyRunner with a
fake robot and a stub policy on a simulated clock (follows like a direct replay, holds, stops), and the
real policy process with the replay policy and fake cameras. Run: python -m unittest discover tests"""
import sys
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from teleop.follow import TargetFollower  # noqa: E402
from teleop.kinematics import RB5Model  # noqa: E402
from teleop.policy import RUN_HEADER, ChunkBuffer, PolicyClient, PolicyRunner, StopRequest, parse_policy_keys  # noqa: E402
from test_loop import D, OFFSET, FakeRB5, FakeReader, FakeWorker, settings  # noqa: E402

FPS = 30
IX = {h: i for i, h in enumerate(RUN_HEADER)}


class TestChunkBuffer(unittest.TestCase):
    def test_one_chunk_interpolates_by_time(self):
        b = ChunkBuffer(FPS)
        b.add(10.0, np.arange(100)[:, None] * np.ones((1, 4)))      # action k = k
        a, n = b.action_at(10.0 + 2.5 / FPS)
        self.assertEqual(n, 1)
        np.testing.assert_allclose(a, 2.5)
        a, _ = b.action_at(10.0 + 150 / FPS)                          # past the end: dropped
        self.assertIsNone(a)

    def test_ensembling_weights_the_oldest_most(self):
        b = ChunkBuffer(FPS, coeff=1.0)
        b.add(0.0, np.zeros((100, 1)))
        b.add(1.0 / FPS, np.ones((100, 1)))
        a, n = b.action_at(2.0 / FPS)
        self.assertEqual(n, 2)
        w = np.exp(-np.arange(2))
        self.assertAlmostEqual(float(a[0]), w[1] / w.sum())           # 0 x w0 + 1 x w1
        b2 = ChunkBuffer(FPS, coeff=0.0)
        b2.add(0.0, np.zeros((100, 1)))
        b2.add(1.0 / FPS, np.ones((100, 1)))
        self.assertAlmostEqual(float(b2.action_at(2.0 / FPS)[0][0]), 0.5)   # equal weights

    def test_chunk_only_for_later_times_and_order(self):
        b = ChunkBuffer(FPS)
        b.add(5.0, np.ones((100, 2)))
        self.assertIsNone(b.action_at(4.9)[0])
        b.add(4.0, np.zeros((100, 2)))                                # older than the newest: ignored
        self.assertEqual(len(b.chunks), 1)


class TestKeys(unittest.TestCase):
    def test_policy_keys(self):
        self.assertEqual(parse_policy_keys(b"g"), ["run"])
        self.assertEqual(parse_policy_keys(b" "), ["hold"])
        self.assertEqual(parse_policy_keys(b"\x1b"), ["quit"])
        self.assertEqual(parse_policy_keys(b"xq"), ["quit"])


class SimClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class StubClient:
    """Replay chunks every 1/FPS of simulated time, arriving `delay` after their observation time;
    stops producing after `until` seconds."""

    def __init__(self, clock, actions, delay=0.0, until=1e9, every=1):
        self.clock, self.actions, self.delay, self.until, self.every = clock, actions, delay, until, every
        self.t0 = clock()
        self.k, self.pending, self.published = 0, [], []

    def poll(self):
        now = self.clock()
        while self.t0 + self.k / FPS <= now and self.k / FPS < self.until:
            t_obs = self.t0 + self.k / FPS
            idx = np.clip(np.arange(self.k, self.k + 100), 0, len(self.actions) - 1)
            if self.k % self.every == 0:
                self.pending.append((t_obs + self.delay, (t_obs, self.actions[idx], 30.0, self.k)))
            self.k += 1
        out = [c for ta, c in self.pending if ta <= now]
        self.pending = [(ta, c) for ta, c in self.pending if ta > now]
        return out

    def publish(self, t_state, state):
        self.published.append((t_state, state))

    def alive(self):
        return True


class Keys:
    def __init__(self, plan):
        self.plan, self.n = plan, 0

    def keys(self):
        self.n += 1
        return self.plan.get(self.n, [])


def line_actions(p0, d, seconds):
    """30 Hz actions from p0 along d [mm/s], gripper 1."""
    t = np.arange(int(seconds * FPS)) / FPS
    xyz = p0[None, :] + t[:, None] * np.asarray(d)[None, :]
    return np.c_[xyz, np.ones(len(t))]


def make(clock, actions, keys, **kw):
    s = settings(yaw__mode="follow_base")
    model = RB5Model(OFFSET)
    robot = FakeRB5(D, model)
    follower = TargetFollower(s, model, robot.pose())
    client = kw.pop("client", None) or StubClient(clock, actions, **{k: kw.pop(k) for k in ("delay", "until", "every") if k in kw})
    events = []
    runner = PolicyRunner(s, robot, FakeReader(robot), follower, client, keys, event=events.append,
                          sleep=clock.sleep, clock=clock, **kw)
    return runner, robot, follower, client, events


def run_for(runner, cycles):
    stop = None
    try:
        for _ in range(cycles):   # the runner loops forever: a key plan ends it
            runner.run()
    except StopRequest as e:
        stop = str(e)
    return stop


class TestPolicyRunner(unittest.TestCase):
    def test_follows_like_a_direct_replay(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 60.0], 2.0)                # up 60 mm/s for 2 s
        runner, robot, follower, client, events = make(clock, acts, Keys({1: ["run"], 120: ["quit"]}))
        self.assertEqual(run_for(runner, 1), "quit key")
        # direct replay: the same follower path with the action interpolated at the same times
        s = settings(yaw__mode="follow_base")
        model = RB5Model(OFFSET)
        r2 = FakeRB5(D, model)
        f2 = TargetFollower(s, model, r2.pose())
        t = np.array([float(r[0]) for r in runner.rows])
        g = np.arange(len(acts)) / FPS
        for ti in t:
            want = np.array([np.interp(ti, g, acts[:, k]) for k in range(3)])
            f2.step(want, r2.pose(), r2.q.copy())
            r2.send_servo(f2.target, f2.rpy)
        np.testing.assert_allclose(follower.target, f2.target, atol=1e-6)
        self.assertAlmostEqual(follower.target[2] - p0[2], 60.0 * min(t[-1], 2.0 - 1 / FPS), delta=1.0)
        self.assertEqual(sum(int(r[2]) for r in runner.rows), 0)       # no late cycles
        self.assertTrue(len(client.published) > 100)
        self.assertEqual(len(client.published[-1][1]), 7)             # joints + gripper

    def test_inference_delay_does_not_shift_the_path(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 60.0], 3.0)
        paths = []
        for delay, every in ((0.0, 1), (0.2, 1), (0.033, 2)):          # 200 ms late; every 2nd inference missing
            clock = SimClock()
            runner, robot, follower, _, _ = make(clock, acts, Keys({15: ["run"], 140: ["quit"]}), delay=delay,
                                                 every=every)   # g after the first (late) chunks arrived
            run_for(runner, 1)
            paths.append([float(r[IX["target_z"]]) for r in runner.rows[30:120]])   # target z once moving
        np.testing.assert_allclose(paths[1], paths[0], atol=0.5)        # a late chunk uses its own times
        np.testing.assert_allclose(paths[2], paths[0], atol=0.5)

    def test_holds_without_new_chunks_then_stops(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 30.0], 10.0)
        runner, _, follower, _, events = make(clock, acts, Keys({1: ["run"]}), until=1.0)
        stop = run_for(runner, 1)
        self.assertEqual(stop, "no chunk from the policy for 3 s")
        holds = [e for e in events if "HOLD" in e]
        self.assertEqual(len(holds), 1)
        self.assertIn("no chunk newer than 0.5 s", holds[0])
        t_hold = float(holds[0].split("]")[0].strip("[ s"))
        self.assertAlmostEqual(t_hold, 1.5, delta=0.1)

    def test_space_holds_and_g_runs(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 30.0], 5.0)
        runner, _, follower, _, events = make(clock, acts, Keys({1: ["run"], 30: ["hold"], 80: ["run"], 120: ["quit"]}))
        run_for(runner, 1)
        z = np.array([float(r[IX["target_z"]]) for r in runner.rows])
        self.assertAlmostEqual(z[70] - z[35], 0.0, delta=0.05)          # held
        self.assertGreater(z[115] - z[80], 1.0)                         # running again
        self.assertTrue(any("HOLD: space key" in e for e in events))

    def test_slow_plan_is_late_and_not_applied(self):
        """A plan that takes 35 ms makes the send late: that cycle keeps the old target (as the teleop)."""
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 30.0], 5.0)
        runner, _, follower, _, events = make(clock, acts, Keys({20: ["run"], 80: ["quit"]}))
        plan, calls = follower.plan, {"n": 0}

        def slow_plan(*a):
            calls["n"] += 1
            if calls["n"] == 30:
                clock.t += 0.035
            return plan(*a)
        follower.plan = slow_plan
        run_for(runner, 1)
        late = [i for i, r in enumerate(runner.rows) if r[IX["late"]] == 1]
        self.assertEqual(len(late), 1)
        i = late[0]
        self.assertEqual(runner.rows[i][IX["target_z"]], runner.rows[i - 1][IX["target_z"]])   # frozen
        self.assertGreater(float(runner.rows[i][IX["plan_ms"]]), 34.0)
        self.assertTrue(any("late:" in e for e in events))

    def test_three_late_cycles_stop(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 30.0], 5.0)
        runner, _, follower, _, _ = make(clock, acts, Keys({20: ["run"], 200: ["quit"]}))
        plan, calls = follower.plan, {"n": 0}

        def slow_plan(*a):
            calls["n"] += 1
            if calls["n"] >= 10:
                clock.t += 0.035
            return plan(*a)
        follower.plan = slow_plan
        with self.assertRaises(RuntimeError):
            runner.run()

    def test_run_refused_before_the_first_chunk(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 30.0], 3.0)
        runner, _, _, _, events = make(clock, acts, Keys({1: ["run"], 20: ["quit"]}), delay=0.2)
        run_for(runner, 1)
        self.assertIn("not running: no recent chunk from the policy yet", events[0])
        self.assertFalse(any("RUN:" in e for e in events))

    def test_big_jump_holds(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0 + np.array([0.0, 0.0, 150.0]), [0.0, 0.0, 0.0], 3.0)   # 150 mm away at once
        runner, _, follower, _, events = make(clock, acts, Keys({1: ["run"], 30: ["quit"]}))
        run_for(runner, 1)
        self.assertTrue(any("from the target (> 100)" in e for e in events))
        np.testing.assert_allclose(follower.target, p0, atol=1e-9)

    def test_time_limit_holds(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 10.0], 10.0)
        runner, _, _, _, events = make(clock, acts, Keys({1: ["run"], 200: ["quit"]}), time_limit_s=2.0)
        run_for(runner, 1)
        self.assertTrue(any("time limit 2 s" in e for e in events))
        self.assertAlmostEqual(runner.run_time, 2.0, delta=0.05)


class TestGripper(unittest.TestCase):
    def test_gripper_follows_only_while_running(self):
        clock = SimClock()
        p0 = FakeRB5(D, RB5Model(OFFSET)).pose()[:3]
        acts = line_actions(p0, [0.0, 0.0, 20.0], 5.0)
        acts[:, 3] = np.linspace(1.0, 0.0, len(acts))                     # closes over 5 s
        runner, _, _, client, _ = make(clock, acts, Keys({20: ["run"], 60: ["hold"], 90: ["quit"]}))
        worker = FakeWorker()
        runner.gripper = worker
        run_for(runner, 1)
        self.assertEqual(len(worker.fracs), 40)                            # cycles 20..59: running
        t_run = np.array([float(r[IX["t_s"]]) for r in runner.rows[19:59]])
        want = np.interp(t_run, np.arange(len(acts)) / FPS, acts[:, 3])
        np.testing.assert_allclose(worker.fracs, want, atol=0.02)          # the policy's gripper action
        gs = runner.s.gripper
        frac = (worker.row["position"] - gs.closed_ticks) / (gs.open_ticks - gs.closed_ticks)
        self.assertAlmostEqual(client.published[-1][1][6], float(np.clip(frac, 0, 1)))   # state: gripper position

    def test_catch_one_follows_at_once(self):
        from teleop.gripper import GripCommand
        cmd = GripCommand(3968, 2196, held=2200, catch_frac=1.0)          # open, the policy asks to close
        goal, event = cmd.update(0.0)
        self.assertEqual(goal, 3968)
        self.assertIn("following", event)
        teleop_cmd = GripCommand(3968, 2196, held=2200, catch_frac=0.05)
        self.assertEqual(teleop_cmd.update(0.0)[0], 2200)                   # the teleop's catch holds


class TestStateLikeTheConverter(unittest.TestCase):
    def test_gripper_fraction_and_joint_source(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location("convert", ROOT / "scripts" / "31_convert_to_lerobot.py")
        cv = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cv)
        clock = SimClock()
        runner, robot, _, client, _ = make(clock, np.zeros((10, 4)), Keys({3: ["quit"]}))
        gs = runner.s.gripper

        class G:
            def __init__(self, pos):
                self.pos = pos

            def latest(self):
                return {"position": self.pos}
        for pos in (gs.closed_ticks, gs.open_ticks, 3100, 4098, 2000):
            runner.gripper = G(pos)
            want = float(np.clip((pos - gs.closed_ticks) / (gs.open_ticks - gs.closed_ticks), 0.0, 1.0))   # converter rule
            self.assertAlmostEqual(runner._grip_frac(), want)
        runner.gripper = None
        run_for(runner, 1)
        t_state, state = client.published[-1]
        np.testing.assert_allclose(state[:6], robot.q, atol=1e-9)        # Simulation: jnt_ref (the reader's q)
        self.assertEqual(len(state), len(cv.STATE_NAMES))


class TestPolicyProcess(unittest.TestCase):
    def test_replay_policy_with_fake_cameras(self):
        acts = np.c_[np.arange(300)[:, None] * np.ones((1, 3)), np.ones(300)]
        client = PolicyClient({"kind": "replay", "actions": acts.tolist(), "fps": FPS, "delay_s": 0.05},
                              [("front", "bright@100", None), ("wrist", "bright@100", None)], fake_cameras=True)
        self.addCleanup(client.close)
        self.assertEqual(client.open(), [])
        client.publish(time.monotonic(), [0.0] * 7)
        chunks, end = [], time.monotonic() + 1.5
        while time.monotonic() < end:
            chunks += client.poll()
            time.sleep(0.02)
        self.assertGreater(len(chunks), 15)                              # 50 ms inference: about 20 per second
        t = [c[0] for c in chunks]
        self.assertEqual(t, sorted(t))
        self.assertEqual(chunks[0][1].shape, (100, 4))
        self.assertTrue(all(c[2] >= 50.0 for c in chunks))               # inference time includes the delay
        ages = [time.monotonic() - c[0] for c in client.poll()]
        self.assertTrue(all(a < 0.5 for a in ages))

    def test_unknown_policy_is_reported(self):
        client = PolicyClient({"kind": "nope"}, [], fake_cameras=True)
        problems = client.open()
        client.close()
        self.assertIn("unknown policy kind", problems[0])


if __name__ == "__main__":
    unittest.main()

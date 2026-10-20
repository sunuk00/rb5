"""Policy runner (rec-6): a policy process predicts action chunks; the 50 Hz servo loop follows them.

Decisions (docs/decisions.md 2026-10-21, rec-6): two processes; inference every 30 Hz step with temporal
ensembling by time; speed caps 100 % of the teleop; keyboard toggle (g = run, space = hold, q = quit).

Policy process (PolicyClient starts it, spawn): opens the cameras (teleop/cameras.py, the same fixed
settings as the recording) and keeps the latest frame of each in a thread; in a loop, at most once per
1/fps: observation = the latest frames + the latest robot state written by the servo process (shared
memory) -> policy -> chunk of actions [x, y, z, gripper] for the times t_obs + k / fps, sent back with
t_obs (the front camera's capture time), the inference time and a counter. It never waits for the servo.

Servo process (PolicyRunner, the caller's process): every 20 ms takes the chunks that arrived (no wait)
into a ChunkBuffer, asks it for the action at the current time (temporal ensembling: every recent chunk
that covers the time gives its action, interpolated between its steps; weights exp(-coeff i), i = 0 the
oldest, as ACT), passes the position through teleop.follow.TargetFollower (the teleop's limits) to
move_servo_l and the gripper value to the gripper thread, and writes the state for the next observation.

Holds (the target stays where it is) when: not running (start with g, hold with space), the newest
chunk is older than stale_s, no chunk covers the time, the action is more than max_jump_mm from the
current target, or the run reached time_limit_s; stops when no chunk has arrived for stop_stale_s.

Policies inside the process (spec dicts, so they can be pickled):
  {"kind": "replay", "actions": (N, 4) list, "fps": 30, "delay_s": 0.0}  dataset actions as chunks from
      the run start (tests, structure checks); delay_s simulates the inference time
  {"kind": "act", "checkpoint": path, "device": "cuda", "task": text}     a trained ACT (rec-6b)

Observation tensors for ACT are made by observation_batch(), the same steps as LeRobot's
prepare_observation_for_inference: image HWC uint8 RGB (the cameras stream rgb8, the recording saved the
same arrays) -> float32 / 255 -> CHW -> batch dimension; state float32 [J1..J6 deg, gripper 0..1] in the
dataset's order; then the checkpoint's own preprocessor (normalization with the training statistics).
tests/test_policy_obs.py checks that a dataset frame gives the same tensors through both paths.
"""
import gc
import json
from pathlib import Path
import multiprocessing as mp
import queue
import threading
import time

import numpy as np

STATE_LEN = 7          # joints J1..J6 [deg] + gripper position 0..1 (rec-1 observation.state)
CHUNK_QUEUE = 8


# ---------- servo side: chunks -> action at a time ----------

class ChunkBuffer:
    """Recent action chunks with their observation times; action_at(t) = temporal ensembling by time."""

    def __init__(self, fps, coeff=0.01):
        self.fps, self.coeff = float(fps), coeff
        self.chunks = []    # (t_obs, actions (N, D)), oldest first

    def add(self, t_obs, actions):
        a = np.asarray(actions, dtype=float)
        if self.chunks and t_obs <= self.chunks[-1][0]:
            return      # out of order or repeated: ignore
        self.chunks.append((float(t_obs), a))

    def newest_t(self):
        return self.chunks[-1][0] if self.chunks else None

    def action_at(self, t):
        """(action (D,), chunks used) or (None, 0) when no chunk covers t. Drops chunks that ended before t."""
        self.chunks = [c for c in self.chunks if t <= c[0] + (len(c[1]) - 1) / self.fps]
        values = []
        for t_obs, a in self.chunks:
            f = (t - t_obs) * self.fps
            if f < 0:
                continue        # predicted for later times only
            i = int(np.floor(f))
            if i >= len(a) - 1:
                values.append(a[-1])
            else:
                w = f - i
                values.append((1.0 - w) * a[i] + w * a[i + 1])
        if not values:
            return None, 0
        weights = np.exp(-self.coeff * np.arange(len(values)))   # index 0 = the oldest chunk
        return (weights[:, None] * np.stack(values)).sum(axis=0) / weights.sum(), len(values)


# ---------- policy process ----------

class _Latest(threading.Thread):
    """Keeps the latest frame of a camera: (image, capture time [monotonic s]) like teleop/cameras.py."""

    def __init__(self, cam, stop):
        super().__init__(daemon=True)
        self.cam, self.stop_event = cam, stop
        self.lock = threading.Lock()
        self.frame = None

    def run(self):
        while not self.stop_event.is_set():
            got = self.cam.read()
            if got is None:
                continue
            img, _, cam_ts, domain, mono, wall_ms = got
            delay_s = (wall_ms - cam_ts) / 1000.0 if domain in ("global_time", "system_time", "fake") else 0.0
            with self.lock:
                self.frame = (img, mono - delay_s)

    def latest(self):
        with self.lock:
            return self.frame


class ReplayPolicy:
    """Dataset actions as chunks: the chunk for t_obs starts at the action of (t_obs - start) x fps."""

    def __init__(self, spec):
        self.actions = np.asarray(spec["actions"], dtype=float)
        self.fps, self.delay_s = spec.get("fps", 30), spec.get("delay_s", 0.0)
        self.chunk = spec.get("chunk", 100)
        self.t_start = None

    def predict(self, obs):
        if self.delay_s:
            time.sleep(self.delay_s)
        if self.t_start is None:
            self.t_start = obs["t_obs"]
        k = int(round((obs["t_obs"] - self.t_start) * self.fps))
        idx = np.clip(np.arange(k, k + self.chunk), 0, len(self.actions) - 1)
        return self.actions[idx]


STATE_NAMES = ["j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg", "gripper_pos"]   # rec-1, converter
ACTION_NAMES = ["x_mm", "y_mm", "z_mm", "gripper_open"]


def policy_inputs(checkpoint):
    """(camera names, state length) the checkpoint expects, from its config.json (no torch needed)."""
    cfg = json.loads((Path(checkpoint) / "config.json").read_text())
    feats = cfg["input_features"]
    cams = [k.split(".", 2)[2] for k in feats if k.startswith("observation.images.")]
    return cams, feats["observation.state"]["shape"][0]


def observation_batch(images, state, task, cameras, device="cpu"):
    """Model input before the preprocessor: images {name: HWC uint8 RGB}, state (7,) -> dict of tensors."""
    import torch
    batch = {"observation.state": torch.as_tensor(np.asarray(state, dtype=np.float32)).unsqueeze(0).to(device)}
    for name in cameras:
        img = torch.from_numpy(np.require(images[name], requirements=["C", "W"]))   # copies a read-only array
        if img.dtype == torch.uint8:
            img = img.type(torch.float32) / 255
        batch[f"observation.images.{name}"] = img.permute(2, 0, 1).contiguous().unsqueeze(0).to(device)
    batch["task"] = [task]
    return batch


class ACTChunkPolicy:
    """A trained ACT checkpoint: observation -> chunk (chunk_size, 4) in mm / 0..1 (unnormalized)."""

    def __init__(self, spec):
        import torch
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        self.torch = torch
        self.device = spec.get("device", "cuda")
        self.task = spec.get("task", "")
        self.policy = ACTPolicy.from_pretrained(spec["checkpoint"])
        self.policy.config.device = self.device
        self.policy.to(self.device).eval()
        self.pre, self.post = make_pre_post_processors(
            self.policy.config, pretrained_path=str(spec["checkpoint"]),
            preprocessor_overrides={"device_processor": {"device": self.device}})
        self.cameras = [k.split(".", 2)[2] for k in self.policy.config.image_features]
        names = (self.policy.config.input_features["observation.state"].shape[0],
                 self.policy.config.output_features["action"].shape[0])
        if names != (len(STATE_NAMES), len(ACTION_NAMES)):
            raise ValueError(f"policy state / action sizes {names}, expected {len(STATE_NAMES)} / {len(ACTION_NAMES)}")

    def predict(self, obs):
        missing = [c for c in self.cameras if c not in obs["images"]]
        if missing:
            raise ValueError(f"the policy needs cameras {missing} that are not open")
        with self.torch.inference_mode():
            self.policy.reset()
            batch = observation_batch(obs["images"], obs["state"], self.task, self.cameras, self.device)
            chunk = self.post(self.policy.predict_action_chunk(self.pre(batch)))
        return chunk[0].float().cpu().numpy()


def make_policy(spec):
    if spec["kind"] == "replay":
        return ReplayPolicy(spec)
    if spec["kind"] == "act":
        return ACTChunkPolicy(spec)
    raise ValueError(f"unknown policy kind {spec['kind']!r}")


def _policy_main(spec, cameras, fake_cameras, fps, state, out_q, cmd_q, reply_q):
    """cameras: list of (name, serial, settings or None). Sends ("ready", info) or ("error", text) first."""
    from .cameras import FakeCamera, RealSenseCamera
    cls = FakeCamera if fake_cameras else RealSenseCamera
    stop = threading.Event()
    opened, readers = [], {}
    try:
        policy = make_policy(spec)
        for name, serial, settings in cameras:
            cam = cls(serial, 640, 480, 30, settings)
            opened.append(cam)
            readers[name] = _Latest(cam, stop)
            readers[name].start()
        t_end = time.monotonic() + 5.0
        while any(r.latest() is None for r in readers.values()):
            if time.monotonic() > t_end:
                raise RuntimeError("no frames from " + ", ".join(n for n, r in readers.items() if r.latest() is None))
            time.sleep(0.01)
    except Exception as e:
        stop.set()
        for cam in opened:
            cam.close()
        reply_q.put(("error", f"policy process start failed: {e}"))
        return
    info = {}
    for (name, serial, _), cam in zip(cameras, opened):   # start brightness, as the recording checks it
        values, t_end = [], time.monotonic() + 1.0
        while time.monotonic() < t_end:
            f = readers[name].latest()
            values.append(float(f[0].mean()))
            time.sleep(0.1)
        info[name] = {"serial": str(serial), "settings": cam.applied, "brightness": round(float(np.mean(values)), 1)}
    # Warm-up: the first inferences on the GPU are slow (about 230 ms: kernel selection), before "ready"
    first = next(iter(readers)) if readers else None
    for _ in range(3):
        policy.predict({"t_obs": time.monotonic(), "t_state": 0.0, "state": np.zeros(STATE_LEN),
                        "images": {name: r.latest()[0] for name, r in readers.items()}})
    if hasattr(policy, "t_start"):
        policy.t_start = None          # the replay policy starts at the first real observation
    # The model makes hundreds of thousands of Python objects; a full GC pass over them took about 80 ms every
    # 42 s and delayed an inference by that much (rec-6b endurance). Freeze them: GC stays on for new objects.
    gc.collect()
    gc.freeze()
    reply_q.put(("ready", {"cameras": info}))
    period, n = 1.0 / fps, 0
    try:
        while True:
            try:
                if cmd_q.get_nowait()[0] == "quit":
                    break
            except queue.Empty:
                pass
            t0 = time.monotonic()
            with state.get_lock():
                s = np.array(state[:])
            frames = {name: r.latest() for name, r in readers.items()}
            t_obs = frames[first][1] if first else t0
            obs = {"t_obs": t_obs, "t_state": s[0], "state": s[1:1 + STATE_LEN],
                   "images": {name: f[0] for name, f in frames.items()}}
            actions = policy.predict(obs)
            infer_ms = (time.monotonic() - t0) * 1000.0
            n += 1
            try:
                out_q.put_nowait((t_obs, np.asarray(actions, dtype=np.float32), infer_ms, n))
            except queue.Full:
                pass        # the servo side is not reading: drop rather than wait
            rest = period - (time.monotonic() - t0)
            if rest > 0:
                time.sleep(rest)
    finally:
        stop.set()
        for cam in opened:
            cam.close()


class PolicyClient:
    """Servo side of the policy process: start(), publish(state), poll() -> new chunks, close()."""

    def __init__(self, spec, cameras, fps=30, fake_cameras=False):
        ctx = mp.get_context("spawn")
        self.state = ctx.Array("d", 1 + STATE_LEN)     # t_state [monotonic s], then the state
        self.out_q, self.cmd_q, self.reply_q = ctx.Queue(CHUNK_QUEUE), ctx.Queue(), ctx.Queue()
        self.proc = ctx.Process(target=_policy_main, name="policy", daemon=True,
                                args=(spec, cameras, fake_cameras, fps, self.state, self.out_q, self.cmd_q,
                                      self.reply_q))
        self.info = {}

    def open(self, timeout=60.0):
        """Start the process and wait until the policy and the cameras are ready. Returns a list of problems."""
        self.proc.start()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                kind, value = self.reply_q.get(timeout=0.1)
            except queue.Empty:
                if not self.proc.is_alive():
                    return [f"policy process ended (exit code {self.proc.exitcode})"]
                continue
            if kind == "error":
                return [value]
            self.info = value
            return []
        return [f"policy process not ready after {timeout:.0f} s"]

    def publish(self, t_state, state):
        with self.state.get_lock():
            self.state[0] = t_state
            self.state[1:1 + STATE_LEN] = list(state)

    def poll(self):
        """Chunks that arrived since the last call, without waiting: [(t_obs, actions, infer_ms, n), ...]."""
        out = []
        while True:
            try:
                out.append(self.out_q.get_nowait())
            except queue.Empty:
                return out

    def alive(self):
        return self.proc.is_alive()

    def close(self):
        if self.proc.is_alive():
            self.cmd_q.put(("quit",))
            self.proc.join(3.0)
        if self.proc.is_alive():
            self.proc.terminate()


# ---------- keys ----------

POLICY_KEYS = {"g": "run", "G": "run", " ": "hold", "q": "quit", "Q": "quit"}


def parse_policy_keys(data):
    """Key bytes -> "run" (g) / "hold" (space) / "quit" (q or a lone Esc)."""
    text = data.decode("latin-1")
    if text == "\x1b":
        return ["quit"]
    return [POLICY_KEYS[c] for c in text if c in POLICY_KEYS]


# ---------- servo loop ----------

RUN_HEADER = (["t_s", "cycle_interval_ms", "late", "stale_state", "state_age_ms", "running", "hold_reason",
               "chunks_used", "chunk_age_ms", "infer_ms", "chunk_n"]
              + [f"desired_{a}" for a in "xyz"] + ["grip_action"] + [f"target_{a}" for a in "xyz"]
              + [f"tcp_ref_{a}" for a in "xyz"] + ["yaw_deg"] + [f"q{i}" for i in range(6)] + ["flags"])


class StopRequest(Exception):
    """Controlled end of the run: normal stop of the servo stream."""


class PolicyRunner:
    """The 50 Hz loop. Devices are passed in (tests use fakes):
      rb5:      check_errors(), send_servo(xyz, rpy)
      reader:   latest() -> (tcp pose, jnt_ref [deg], age [s]); measured() -> jnt_ang [deg] (Real state)
      follower: teleop.follow.TargetFollower
      client:   PolicyClient (publish, poll, alive)
      keys:     object with keys() -> ["run" / "hold" / "quit"]
      gripper:  None or a started GripperWorker (set(frac), latest()); closed / open ticks for the state
    """

    def __init__(self, s, rb5, reader, follower, client, keys, fps=30, gripper=None, real=False,
                 stale_s=0.5, stop_stale_s=3.0, max_jump_mm=100.0, time_limit_s=60.0, coeff=0.01,
                 event=print, sleep=time.sleep, clock=time.monotonic):
        self.s, self.rb5, self.reader, self.follower = s, rb5, reader, follower
        self.client, self.keys, self.gripper, self.real = client, keys, gripper, real
        self.buffer = ChunkBuffer(fps, coeff)
        self.stale_s, self.stop_stale_s = stale_s, stop_stale_s
        self.max_jump_mm, self.time_limit_s = max_jump_mm, time_limit_s
        self.event, self.sleep, self.clock = event, sleep, clock
        self.running = False
        self.run_time = 0.0       # seconds spent running (time limit)
        self.rows = []
        self.last_infer = (None, None)

    def _grip_frac(self):
        if self.gripper is None:
            return 0.0
        g = self.gripper.latest()
        if g is None or g["position"] is None:
            return 0.0
        gs = self.s.gripper
        return float(np.clip((g["position"] - gs.closed_ticks) / (gs.open_ticks - gs.closed_ticks), 0.0, 1.0))

    def _hold(self, reason, t):
        if self.running:
            self.event(f"[{t:7.2f} s] HOLD: {reason} (g to run again)")
        self.running = False
        return reason

    def run(self):
        sf, period = self.s.safety, self.s.servo.period_s
        t_start = next_t = self.clock()
        last_send = prev = None
        late_run = stale_run = 0
        last_arrival = t_start
        while True:
            now = self.clock()
            t = now - t_start
            interval = None if prev is None else now - prev
            prev = now
            # 1. chunks and keys (no waiting)
            for t_obs, actions, infer_ms, n in self.client.poll():
                self.buffer.add(t_obs, actions)
                self.last_infer = (infer_ms, n)
                last_arrival = now
            if now - last_arrival > self.stop_stale_s:
                raise StopRequest(f"no chunk from the policy for {self.stop_stale_s:.0f} s")
            for key in self.keys.keys():
                if key == "quit":
                    raise StopRequest("quit key")
                if key == "hold":
                    self._hold("space key", t)
                elif key == "run" and not self.running:
                    newest = self.buffer.newest_t()
                    if newest is None or now - newest > self.stale_s:
                        self.event(f"[{t:7.2f} s] not running: no recent chunk from the policy yet")
                    else:
                        self.running = True
                        self.event(f"[{t:7.2f} s] RUN: following the policy (space holds)")
            if not self.client.alive():
                raise StopRequest("policy process ended")
            # 2. robot state (the teleop's checks)
            self.rb5.check_errors()
            tcp, q, age = self.reader.latest()
            stale = age > sf.max_state_age_s
            stale_run = stale_run + 1 if stale else 0
            if stale_run >= sf.max_stale_cycles:
                raise RuntimeError(f"{sf.max_stale_cycles} stale-state cycles in a row")
            # 3. the action at this time
            newest = self.buffer.newest_t()
            chunk_age = None if newest is None else now - newest
            action, used = self.buffer.action_at(now)
            hold_reason = "" if self.running else "not running"
            if self.running:
                if chunk_age is None or chunk_age > self.stale_s:
                    hold_reason = self._hold(f"no chunk newer than {self.stale_s:.1f} s", t)
                elif action is None:
                    hold_reason = self._hold("no chunk covers this time", t)
                elif np.linalg.norm(action[:3] - self.follower.target) > self.max_jump_mm:
                    hold_reason = self._hold(f"action {np.linalg.norm(action[:3] - self.follower.target):.0f} mm "
                                             f"from the target (> {self.max_jump_mm:.0f})", t)
                elif self.run_time >= self.time_limit_s:
                    hold_reason = self._hold(f"time limit {self.time_limit_s:.0f} s", t)
            gap = None if last_send is None else self.clock() - last_send
            late = gap is not None and gap > sf.max_gap_s
            late_run = late_run + 1 if late else 0
            if late_run >= sf.max_late_cycles:
                raise RuntimeError(f"{sf.max_late_cycles} late cycles in a row")
            # 4. follow (limits) or hold
            flags = {}
            if self.running and not late and not stale:
                plan = self.follower.step(action[:3], tcp, q)
                if plan.stop:
                    raise StopRequest(plan.stop)
                flags = {k: v for k, v in plan.flags.items() if v}
                if self.gripper is not None:
                    self.gripper.set(float(np.clip(action[3], 0.0, 1.0)))
                self.run_time += period
            else:
                self.follower.hold()
            send_t = self.clock()
            self.rb5.send_servo(self.follower.target, self.follower.rpy)
            last_send = send_t
            # 5. state for the next observation (Real: encoders, Simulation: jnt_ref, as the converter)
            joints = self.reader.measured() if self.real else q
            self.client.publish(now - age, list(joints) + [self._grip_frac()])
            infer_ms, n = self.last_infer
            self.rows.append(tuple(
                [f"{t:.4f}", "" if interval is None else f"{interval * 1000:.3f}", int(late), int(stale),
                 f"{age * 1000:.2f}", int(self.running), hold_reason, used,
                 "" if chunk_age is None else f"{chunk_age * 1000:.1f}", "" if infer_ms is None else f"{infer_ms:.1f}",
                 "" if n is None else n]
                + ([f"{v:.3f}" for v in action] if action is not None else [""] * 4)
                + [f"{v:.3f}" for v in self.follower.target] + [f"{v:.3f}" for v in tcp[:3]]
                + [f"{self.follower.yaw:.3f}"] + [f"{v:.3f}" for v in q] + [" ".join(flags)]))
            # 6. next cycle (as the teleop: after an overrun wait a full period)
            next_t += period
            delay = next_t - self.clock()
            if delay > 0:
                self.sleep(delay)
            else:
                next_t = self.clock() + period
                self.sleep(period)

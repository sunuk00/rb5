"""Camera recorder in its own process (rec-3a): RealSense RGB -> JPEG per frame + a time CSV per episode.

Why a process: a thread in the teleop process would share its GIL, and image work at 2 x 30 fps could
delay the 50 Hz loop (as the gripper thread's reads were delayed, grip-4). The process has its own GIL;
time.monotonic() is the same system clock in both processes, so the times can be compared directly.

Inside the process: one grab thread per camera (streams all the time, so auto exposure is settled and an
episode starts at once) and one writer thread per camera (JPEG encoding with PIL, file writes). Frames
are handed over through a bounded queue; when it is full the frame is dropped and counted.

Episode files (dir given by start_episode):
    cam_<name>/<index>.jpg       index from 0 in the episode
    cam_<name>.csv               CAMERA_HEADER, one row per saved frame
stop_episode(keep=False) deletes the episode's files (discarded episode); delete_episode removes a stopped
one later. The teleop side never waits: results come from poll().

Times per frame: t_arrival_s = time.monotonic() when the frame reached the grab thread; cam_ts_ms = the
camera timestamp (global_time domain: the PC wall clock at capture, in ms); t_capture_s = t_arrival_s
minus the receive delay (PC wall clock at arrival - cam_ts_ms). The receive delay differs per camera
(cam-1 / 2026-10-19: D405 about 9.5 ms, D435 about 34 ms), so t_capture_s is the time to match on.
"""
import csv
import multiprocessing as mp
import os
import queue
import shutil
import threading
import time
from pathlib import Path

import numpy as np

CAMERA_HEADER = ["index", "frame_number", "t_capture_s", "t_arrival_s", "cam_ts_ms", "domain", "write_ms"]
READY_TIMEOUT_S = 15.0       # pipelines of two cameras start in about 2-3 s
WRITE_QUEUE = 30             # frames per camera waiting for the writer (1 s at 30 fps)


class FakeCamera:
    """Synthetic frames at the given rate (tests, runs without cameras)."""

    def __init__(self, serial, width, height, fps):
        self.width, self.height, self.period = width, height, 1.0 / fps
        self.n, self.next_t = 0, time.monotonic()

    def read(self):
        self.next_t += self.period
        delay = self.next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self.n += 1
        img = np.full((self.height, self.width, 3), self.n % 256, dtype=np.uint8)
        wall_ms = time.time() * 1000.0
        return img, self.n, wall_ms, "fake", time.monotonic(), wall_ms

    def close(self):
        pass


class RealSenseCamera:
    """Color stream (RGB8) of one RealSense camera found by its serial number."""

    def __init__(self, serial, width, height, fps):
        import pyrealsense2 as rs
        self.rs = rs
        devices = {d.get_info(rs.camera_info.serial_number): d for d in rs.context().query_devices()}
        if serial not in devices:
            raise RuntimeError(f"RealSense {serial} not found (connected: {', '.join(devices) or 'none'})")
        usb = devices[serial].get_info(rs.camera_info.usb_type_descriptor)
        if not usb.startswith("3"):
            raise RuntimeError(f"RealSense {serial} is on USB {usb}: needs USB 3")
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.pipe = rs.pipeline()
        self.pipe.start(cfg)

    def read(self):
        """(image, frame number, camera ts ms, domain, arrival monotonic s, arrival wall ms) or None."""
        try:
            frames = self.pipe.wait_for_frames(1000)
        except RuntimeError:
            return None
        mono, wall_ms = time.monotonic(), time.time() * 1000.0
        f = frames.get_color_frame()
        if not f:
            return None
        img = np.asanyarray(f.get_data()).copy()
        return (img, f.get_frame_number(), f.get_timestamp(), str(f.get_frame_timestamp_domain()).split(".")[-1],
                mono, wall_ms)

    def close(self):
        self.pipe.stop()


class _Episode:
    """Files of one camera for one episode (used by the writer thread only)."""

    def __init__(self, ep_dir, name):
        self.img_dir = Path(ep_dir) / f"cam_{name}"
        self.img_dir.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(Path(ep_dir) / f"cam_{name}.csv", "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(CAMERA_HEADER)
        self.index = 0

    def close(self):
        self.csv_file.close()


def _grab(cam, name, frames_q, recording, stats, stop, errors):
    last_fn = None
    while not stop.is_set():
        try:
            got = cam.read()
        except Exception as e:   # USB disconnect and the like
            errors.put(f"{name}: {e}")
            return
        if got is None:
            continue
        img, fn, cam_ts, domain, mono, wall_ms = got
        if recording.is_set():
            if last_fn is not None and fn - last_fn > 1:
                stats["gaps"] += fn - last_fn - 1
            last_fn = fn
            delay_s = (wall_ms - cam_ts) / 1000.0 if domain in ("global_time", "system_time", "fake") else 0.0
            try:
                frames_q.put_nowait((img, fn, mono - delay_s, mono, cam_ts, domain))
            except queue.Full:
                stats["dropped"] += 1
        else:
            last_fn = None


def _write(name, frames_q, commands_q, done_q, quality, stats):
    from PIL import Image
    ep = None
    while True:
        try:
            cmd = commands_q.get_nowait()
        except queue.Empty:
            cmd = None
        if cmd is not None:
            if cmd[0] == "start":
                ep = _Episode(cmd[1], name)
            elif cmd[0] in ("stop", "quit"):
                while True:   # frames grabbed before the stop still belong to the episode
                    try:
                        item = frames_q.get_nowait()
                    except queue.Empty:
                        break
                    if ep is not None:
                        _save(ep, item, quality, stats, Image)
                if ep is not None:
                    ep.close()
                done_q.put((name, cmd[0], ep.index if ep is not None else 0))
                ep = None
                if cmd[0] == "quit":
                    return
            continue
        try:
            item = frames_q.get(timeout=0.005)
        except queue.Empty:
            continue
        if ep is not None:
            _save(ep, item, quality, stats, Image)


def _save(ep, item, quality, stats, Image):
    img, fn, t_capture, mono, cam_ts, domain = item
    t0 = time.monotonic()
    Image.fromarray(img).save(ep.img_dir / f"{ep.index:06d}.jpg", quality=quality)
    write_ms = (time.monotonic() - t0) * 1000.0
    ep.writer.writerow([ep.index, fn, f"{t_capture:.6f}", f"{mono:.6f}", f"{cam_ts:.3f}", domain, f"{write_ms:.2f}"])
    ep.index += 1
    stats["write_ms_max"] = max(stats["write_ms_max"], write_ms)


def _process_main(cameras, width, height, fps, quality, fake, cmd_q, reply_q):
    """cameras: list of (name, serial). Runs until ("quit",)."""
    cls = FakeCamera if fake else RealSenseCamera
    opened, errors, stop = [], queue.Queue(), threading.Event()
    try:
        for name, serial in cameras:
            opened.append((name, cls(serial, width, height, fps)))
    except Exception as e:
        for _, cam in opened:
            cam.close()
        reply_q.put(("error", f"camera start failed: {e}"))
        return
    recording = threading.Event()
    per_cam, threads = {}, []
    for name, cam in opened:
        st = {"gaps": 0, "dropped": 0, "write_ms_max": 0.0}
        fq, cq, done = queue.Queue(maxsize=WRITE_QUEUE), queue.Queue(), queue.Queue()
        per_cam[name] = (cq, st, done)
        threads.append(threading.Thread(target=_grab, args=(cam, name, fq, recording, st, stop, errors), daemon=True))
        threads.append(threading.Thread(target=_write, args=(name, fq, cq, done, quality, st), daemon=True))
    for t in threads:
        t.start()
    reply_q.put(("ready", [name for name, _ in opened]))
    ep_dir = None
    try:
        while True:
            try:
                err = errors.get_nowait()
                reply_q.put(("error", err))
            except queue.Empty:
                pass
            try:
                cmd = cmd_q.get(timeout=0.05)
            except queue.Empty:
                continue
            if cmd[0] == "start":
                ep_dir = cmd[1]
                for cq, st, _ in per_cam.values():
                    st.update(gaps=0, dropped=0, write_ms_max=0.0)
                    cq.put(("start", ep_dir))
                recording.set()
                reply_q.put(("started", ep_dir))
            elif cmd[0] == "delete":
                shutil.rmtree(cmd[1], ignore_errors=True)
                reply_q.put(("deleted", cmd[1]))
            elif cmd[0] in ("stop", "quit"):
                recording.clear()
                frames = {}
                for name, (cq, st, done) in per_cam.items():
                    cq.put((cmd[0],))
                    frames[name] = done.get(timeout=10.0)[2]
                result = {name: dict(frames=frames[name], **st) for name, (_, st, _) in per_cam.items()}
                if cmd[0] == "stop":
                    if not cmd[1] and ep_dir is not None:
                        for name in per_cam:
                            shutil.rmtree(Path(ep_dir) / f"cam_{name}", ignore_errors=True)
                            (Path(ep_dir) / f"cam_{name}.csv").unlink(missing_ok=True)
                    reply_q.put(("stopped", cmd[1], result))
                    ep_dir = None
                else:
                    reply_q.put(("quit", result))
                    return
    finally:
        stop.set()
        for _, cam in opened:
            try:
                cam.close()
            except Exception:
                pass


class CameraRecorder:
    """Teleop side: starts the camera process and sends it episode commands (none of them blocks long).

    cameras: list of (name, serial), e.g. [("front", "943222070221"), ("wrist", "130322273474")].
    """

    def __init__(self, cameras, width=640, height=480, fps=30, quality=95, fake=False):
        ctx = mp.get_context("spawn")   # not fork: the teleop process has threads (gripper)
        self.cmd_q, self.reply_q = ctx.Queue(), ctx.Queue()
        self.names = [name for name, _ in cameras]
        self.proc = ctx.Process(target=_process_main, name="cameras", daemon=True,
                                args=(cameras, width, height, fps, quality, fake, self.cmd_q, self.reply_q))
        self.errors = []
        self._dead_reported = False

    def open(self, timeout=READY_TIMEOUT_S):
        """Start the process and wait until every camera streams. Returns a list of problems."""
        self.proc.start()
        reply = self._wait(("ready", "error"), timeout)
        if reply is None:
            return self.errors[-1:] or [f"camera process not ready after {timeout:.0f} s"]
        if reply[0] == "error":
            return [reply[1]]
        return []

    def start_episode(self, ep_dir):
        """Start saving frames into ep_dir (does not wait)."""
        Path(ep_dir).mkdir(parents=True, exist_ok=True)
        self.cmd_q.put(("start", str(ep_dir)))

    def stop_episode(self, keep):
        """End the episode (does not wait; keep=False deletes its camera files). The result comes from
        poll() as ("stopped", keep, {name: stats}) once the writers have saved the last frames."""
        self.cmd_q.put(("stop", bool(keep)))

    def delete_episode(self, ep_dir):
        """Delete the camera files of an episode that was already stopped (does not wait)."""
        self.cmd_q.put(("delete", str(ep_dir)))

    def wait_stopped(self, timeout=15.0):
        """Wait for the result of stop_episode (scripts and tests; the teleop uses poll). {name: stats} or None."""
        reply = self._wait(("stopped",), timeout)
        return None if reply is None else reply[2]

    def poll(self):
        """Messages from the process since the last call, without waiting: ("started", dir),
        ("stopped", keep, stats), ("deleted", dir), ("error", text). Errors are also kept in self.errors."""
        out = []
        while True:
            try:
                msg = self.reply_q.get_nowait()
            except queue.Empty:
                break
            if msg[0] == "error":
                self.errors.append(msg[1])
            out.append(msg)
        if not out and not self.proc.is_alive() and not self._dead_reported:
            self._dead_reported = True
            text = f"camera process ended (exit code {self.proc.exitcode})"
            self.errors.append(text)
            out.append(("error", text))
        return out

    def poll_errors(self):
        """Only the error texts of poll()."""
        return [m[1] for m in self.poll() if m[0] == "error"]

    def close(self, timeout=15.0):
        """Ends the process (an episode still open is closed and kept: call stop_episode first)."""
        if self.proc.is_alive():
            self.cmd_q.put(("quit",))
            self._wait(("quit",), timeout)
            self.proc.join(2.0)
        if self.proc.is_alive():
            self.proc.terminate()

    def cpu_seconds(self):
        """User + system CPU time of the camera process so far (Linux /proc), or None."""
        try:
            fields = Path(f"/proc/{self.proc.pid}/stat").read_text().rsplit(")", 1)[1].split()
            return (int(fields[11]) + int(fields[12])) / float(os.sysconf("SC_CLK_TCK"))
        except (OSError, ValueError, IndexError, TypeError):
            return None

    def _wait(self, kinds, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                msg = self.reply_q.get(timeout=0.1)
            except queue.Empty:
                if not self.proc.is_alive():   # died (crash, failed import): nothing more will come
                    self.errors.append(f"camera process ended (exit code {self.proc.exitcode})")
                    return None
                continue
            if msg[0] in kinds:
                return msg
            if msg[0] == "error":
                self.errors.append(msg[1])
        return None

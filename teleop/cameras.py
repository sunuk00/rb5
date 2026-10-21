"""Camera recorder in its own process (rec-3a): RealSense / UVC webcam RGB -> JPEG per frame + a time CSV per
episode. open_camera() picks the class: a serial starting with /dev/ is a UVC webcam (cam-4a), else a RealSense.

Why a process: a thread in the teleop process would share its GIL, and image work at 2 x 30 fps could
delay the 50 Hz loop (as the gripper thread's reads were delayed, grip-4). The process has its own GIL;
time.monotonic() is the same system clock in both processes, so the times can be compared directly.

Inside the process: one grab thread per camera (streams all the time, so auto exposure is settled and an
episode starts at once) and one writer thread per camera (JPEG encoding with PIL, file writes). Frames
are handed over through a bounded queue; when it is full the frame is dropped and counted.

Episode files (dir given by start_episode):
    cam_<name>/<index>.jpg       index from 0 in the episode
    cam_<name>.csv               CAMERA_HEADER, one row per saved frame
stop_episode(keep=False) deletes the episode folder (discarded episode); delete_episode removes a stopped
one later. The teleop side never waits: results come from poll().

Fixed camera settings (cam-2b): a camera given settings {exposure_us, gain, white_balance_k} gets auto
exposure and auto white balance off and these values when it opens; they are read back and a mismatch
refuses the start. The options are not stored in the camera (a replug returns to auto), so this runs at
every open. Exposure units: the D435 RGB sensor takes 0.1 ms steps, the D405 stereo module microseconds.
At the start the process also measures each camera's mean brightness (0-255) over BRIGHTNESS_FRAMES
frames; CameraRecorder.info has the applied values and the brightness, and brightness_problems() compares
it with the reference (the lighting must be the same as when the values were chosen).

Times per frame: t_arrival_s = time.monotonic() when the frame reached the grab thread; cam_ts_ms = the
camera timestamp (global_time domain: the PC wall clock at capture, in ms); t_capture_s = t_arrival_s
minus the receive delay (PC wall clock at arrival - cam_ts_ms). The receive delay differs per camera
(cam-1 / 2026-10-19: D405 about 9.5 ms, D435 about 34 ms), so t_capture_s is the time to match on.
A UVC webcam has no camera clock: cam_ts_ms is the kernel's buffer time (domain v4l2, time.monotonic() in
ms), about 33 ms before the arrival; capture_delay_s() gives the delay for every domain.
"""
import csv
import io
import multiprocessing as mp
import os
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

CAMERA_HEADER = ["index", "frame_number", "t_capture_s", "t_arrival_s", "cam_ts_ms", "domain", "write_ms"]
READY_TIMEOUT_S = 15.0       # pipelines of two cameras start in about 2-3 s
WRITE_QUEUE = 30             # frames per camera waiting for the writer (1 s at 30 fps)
BRIGHTNESS_SETTLE_S = 1.0    # frames skipped after opening before the brightness is measured
BRIGHTNESS_FRAMES = 10
SATURATED_LEVEL = 250        # a pixel whose R, G and B are all at least this is saturated (white, no detail)
SATURATED_MAX_FRAC = 0.25    # more saturated pixels than this at the start refuses (no typed yes): the
                             # p1 session 105618 recorded the D405 at 99-100 % (mean 253) after a yes;
                             # normal images have 1-2 %
MISSING_ROWS_MAX_FRAC = 0.02  # more missing rows than this in a start image refuses (cam-5a: 61 % in the bad case)
RED, RESET = "\033[1;31m", "\033[0m"
STALL_S = 1.0                # no frame for this long = camera error (real-pilot 141929: the D405 dropped off
                             # USB and came back as USB 2; its pipeline only timed out, nothing was reported)
UVC_START_FRAMES = 5         # frames read to start the webcam stream before its settings are written
UVC_SETTLE_S = 0.5           # frames read after writing the webcam settings, before they are read back
UVC_EXPOSURE_TOL = 10        # 0.1 ms steps: the C270 reports 5 more than written while it streams (cam-4a)


class FakeCamera:
    """Synthetic frames at the given rate (tests, runs without cameras). A serial "stall@N" stops delivering
    frames after N frames (read() then times out like a disconnected camera); "bright@V" gives images of
    brightness V; "badsettings" fails the settings read back."""

    def __init__(self, serial, width, height, fps, settings=None):
        self.width, self.height, self.period = width, height, 1.0 / fps
        self.n, self.next_t = 0, time.monotonic()
        serial = str(serial)
        self.stall_after = int(serial.split("@")[1]) if serial.startswith("stall@") else None
        self.level = int(serial.split("@")[1]) if serial.startswith("bright@") else None
        if settings is not None and serial == "badsettings":
            raise RuntimeError(f"settings read back {dict(settings, gain=-1)} differ from {settings}")
        self.applied = None if settings is None else dict(settings)

    def read(self):
        if self.stall_after is not None and self.n >= self.stall_after:
            time.sleep(0.1)
            return None
        self.next_t += self.period
        delay = self.next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self.n += 1
        img = np.full((self.height, self.width, 3), self.n % 256 if self.level is None else self.level, dtype=np.uint8)
        wall_ms = time.time() * 1000.0
        return img, self.n, wall_ms, "fake", time.monotonic(), wall_ms

    def close(self):
        pass


class RealSenseCamera:
    """Color stream (RGB8) of one RealSense camera found by its serial number; settings (optional): fixed
    exposure_us, gain, white_balance_k, applied and read back (applied = what the camera reports)."""

    def __init__(self, serial, width, height, fps, settings=None):
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
        self.applied = None
        if settings is not None:
            try:
                self.applied = self._apply(devices[serial], settings)
            except Exception:
                self.pipe.stop()
                raise

    def _apply(self, device, settings):
        rs = self.rs
        sensor = next(s for s in device.query_sensors()
                      if any(p.stream_type() == rs.stream.color for p in s.get_stream_profiles()))
        per_us = 0.01 if sensor.get_info(rs.camera_info.name) == "RGB Camera" else 1.0   # D435 RGB: 0.1 ms steps
        want = {rs.option.enable_auto_exposure: 0.0, rs.option.enable_auto_white_balance: 0.0,
                rs.option.exposure: float(max(1, round(settings["exposure_us"] * per_us))),
                rs.option.gain: float(settings["gain"]), rs.option.white_balance: float(settings["white_balance_k"])}
        for option, value in want.items():
            sensor.set_option(option, value)
        got = {option: sensor.get_option(option) for option in want}
        bad = [f"{str(o).split('.')[-1]} {got[o]:g} (set {v:g})" for o, v in want.items() if abs(got[o] - v) > 1e-6]
        if bad:
            raise RuntimeError(f"settings read back differ: {', '.join(bad)}")
        return {"exposure_us": got[rs.option.exposure] / per_us, "gain": got[rs.option.gain],
                "white_balance_k": got[rs.option.white_balance], "auto_exposure": 0, "auto_white_balance": 0}

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


class UVCCamera:
    """Color stream of a UVC webcam (cam-4a: Logitech C270, cam-5a: Brio 101, both on the gripper) found by its
    device path, e.g. /dev/v4l/by-id/usb-046d_0825_<serial>-video-index0 (the same after a replug). fmt YUYV
    (uncompressed, converted to RGB) or MJPG (cam-5a: less USB bandwidth; the JPEG bytes are decoded with PIL,
    as OpenCV's decoder printed a corrupt-JPEG warning for almost every C270 frame). Settings (optional): auto
    exposure, the variable frame rate and auto white balance off, then exposure_us (0.1 ms steps), gain,
    white_balance_k, written once the stream runs and read back (applied = what the camera reports). The webcam
    keeps them until it is unplugged, so they are written at every open. Frame numbers count the frame periods between the buffer times, so a missed
    frame shows as a gap as on a RealSense."""

    def __init__(self, path, width, height, fps, settings=None, fmt="YUYV"):
        self.path, self.mjpg = str(path), fmt == "MJPG"
        if not os.path.exists(self.path):
            raise RuntimeError(f"webcam {self.path} not found (ls /dev/v4l/by-id)")
        import cv2
        cv2.setNumThreads(1)   # only a color conversion here; no thread pool next to PyTorch in the policy process
        self.cv2 = cv2
        self.cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"webcam {self.path} could not be opened (another program using it?)")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
        if self.mjpg:   # the JPEG bytes as sent: decoded with PIL in read() (OpenCV's decoder prints a warning
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # for almost every C270 frame, cam-5a)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        size = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if size != (width, height):
            self.cap.release()
            raise RuntimeError(f"webcam {self.path} gives {size[0]}x{size[1]}, not {width}x{height}")
        self.period_ms, self.n, self.last_ts = 1000.0 / fps, 0, None
        # The stream must run before the settings are written: written before the first frame, the C270
        # read back the same values but gave images about 1.1 x brighter (127 instead of 114, cam-4b)
        if not any([self.cap.read()[0] for _ in range(UVC_START_FRAMES)]):   # a list: all frames are read
            self.cap.release()
            raise RuntimeError(f"webcam {self.path} gives no frames")
        self.applied = None
        if settings is not None:
            try:
                self.applied = self._apply(settings)
            except Exception:
                self.cap.release()
                raise

    def ctl(self, *args):
        """Runs v4l2-ctl on this webcam (also used by script 34); its output, or RuntimeError."""
        done = subprocess.run(["v4l2-ctl", "-d", self.path, *args], capture_output=True, text=True, timeout=5)
        if done.returncode != 0:
            raise RuntimeError(f"v4l2-ctl {' '.join(args)}: {done.stderr.strip()}")
        return done.stdout

    def _apply(self, settings):
        want = uvc_controls(settings)
        # auto off first: the exposure and white balance values are inactive while the auto modes are on
        self.ctl("--set-ctrl=auto_exposure=1,exposure_dynamic_framerate=0,white_balance_automatic=0")
        self.ctl("--set-ctrl=" + ",".join(f"{k}={want[k]}" for k in
                                           ("exposure_time_absolute", "gain", "white_balance_temperature")))
        t_end = time.monotonic() + UVC_SETTLE_S
        while time.monotonic() < t_end:
            self.cap.read()
        got = {}
        for line in self.ctl("-C", ",".join(want)).splitlines():   # "auto_exposure: 1 (Manual Mode)"
            name, value = line.split(":", 1)
            got[name.strip()] = int(value.split()[0])
        bad = uvc_mismatch(want, got)
        if bad:
            raise RuntimeError(f"settings read back differ: {', '.join(bad)}")
        return {"exposure_us": got["exposure_time_absolute"] * 100.0, "gain": got["gain"],
                "white_balance_k": got["white_balance_temperature"], "auto_exposure": 0, "auto_white_balance": 0}

    def read(self):
        """Same as RealSenseCamera.read; camera ts = the kernel buffer time (monotonic ms), domain v4l2."""
        ok, frame = self.cap.read()
        mono, wall_ms = time.monotonic(), time.time() * 1000.0
        if not ok:
            time.sleep(0.1)   # an unplugged webcam fails at once: no busy loop
            return None
        ts = self.cap.get(self.cv2.CAP_PROP_POS_MSEC)
        self.n += 1 if self.last_ts is None else max(1, round((ts - self.last_ts) / self.period_ms))
        self.last_ts = ts
        if self.mjpg:
            from PIL import Image
            try:
                img = np.asarray(Image.open(io.BytesIO(frame.tobytes())).convert("RGB"))
            except OSError:   # a broken JPEG: counted as a missed frame (the next one is a gap)
                return None
        else:
            img = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        return img, self.n, ts, "v4l2", mono, wall_ms

    def close(self):
        self.cap.release()


def uvc_controls(settings):
    """v4l2 control values for the fixed settings {exposure_us, gain, white_balance_k}."""
    return {"auto_exposure": 1, "exposure_dynamic_framerate": 0, "white_balance_automatic": 0,   # 1 = manual
            "exposure_time_absolute": max(1, round(settings["exposure_us"] / 100.0)),
            "gain": int(settings["gain"]), "white_balance_temperature": int(settings["white_balance_k"])}


def uvc_mismatch(want, got):
    """Texts for the read-back controls that differ (exposure: within UVC_EXPOSURE_TOL)."""
    bad = []
    for name, value in want.items():
        tol = UVC_EXPOSURE_TOL if name == "exposure_time_absolute" else 0
        if name not in got or abs(got[name] - value) > tol:
            bad.append(f"{name} {got.get(name)} (set {value})")
    return bad


def open_camera(serial, width, height, fps, settings=None, fake=False, fmt=None):
    """FakeCamera (fake), UVCCamera (serial = a /dev/ path; fmt YUYV, the default, or MJPG) or RealSenseCamera
    (serial number)."""
    if fake:
        return FakeCamera(serial, width, height, fps, settings)
    if str(serial).startswith("/dev/"):
        return UVCCamera(serial, width, height, fps, settings, fmt or "YUYV")
    return RealSenseCamera(serial, width, height, fps, settings)


def capture_delay_s(cam_ts, domain, mono, wall_ms):
    """Seconds from the camera time to the arrival of a read() result (0 when the camera time is no PC clock)."""
    if domain in ("global_time", "system_time", "fake"):
        return (wall_ms - cam_ts) / 1000.0
    if domain == "v4l2":
        return mono - cam_ts / 1000.0
    return 0.0


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
    last_frame_t, stalled = time.monotonic(), False
    while not stop.is_set():
        try:
            got = cam.read()
        except Exception as e:   # USB disconnect and the like
            errors.put(f"{name}: {e}")
            return
        if got is None:
            if not stalled and time.monotonic() - last_frame_t > STALL_S:
                stalled = True   # reported once; a camera that comes back does not clear the error
                errors.put(f"{name}: no frames for {STALL_S:.0f} s (USB disconnected? check lsusb -t)")
            continue
        last_frame_t = time.monotonic()
        img, fn, cam_ts, domain, mono, wall_ms = got
        if recording.is_set():
            if last_fn is not None and fn - last_fn > 1:
                stats["gaps"] += fn - last_fn - 1
            last_fn = fn
            delay_s = capture_delay_s(cam_ts, domain, mono, wall_ms)
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


def image_level(img):
    """(mean pixel value 0-255, fraction of saturated pixels, fraction of missing rows) of an HxWx3 uint8 RGB
    image. A missing row is pure green (R and B under 20, G over 100 in almost every pixel): what YUYV gives
    where the frame data did not arrive (Y = U = V = 0); cam-5a: the C270 under the uvcvideo FIX_BANDWIDTH
    quirk sent only the top 39 % of every frame, and the brightness check passed it."""
    green = (img[..., 0] < 20) & (img[..., 1] > 100) & (img[..., 2] < 20)
    return (float(img.mean()), float((img.min(axis=2) >= SATURATED_LEVEL).mean()),
            float((green.mean(axis=1) > 0.98).mean()))


def level_info(levels):
    """{"brightness", "saturated_frac", "missing_rows_frac"} from a list of image_level() results (None if
    empty)."""
    if not levels:
        return {"brightness": None, "saturated_frac": None, "missing_rows_frac": None}
    v = np.asarray(levels)
    return {"brightness": round(float(v[:, 0].mean()), 1), "saturated_frac": round(float(v[:, 1].mean()), 3),
            "missing_rows_frac": round(float(v[:, 2].max()), 3)}


def _brightness(cam):
    """level_info over BRIGHTNESS_FRAMES frames after BRIGHTNESS_SETTLE_S."""
    t_end, levels = time.monotonic() + BRIGHTNESS_SETTLE_S, []
    while time.monotonic() < t_end:
        cam.read()
    for _ in range(3 * BRIGHTNESS_FRAMES):
        got = cam.read()
        if got is not None:
            levels.append(image_level(got[0]))
        if len(levels) >= BRIGHTNESS_FRAMES:
            break
    return level_info(levels)


def _process_main(cameras, width, height, fps, quality, fake, cmd_q, reply_q):
    """cameras: list of (name, serial), (name, serial, settings) or (name, serial, settings, format). Runs
    until ("quit",)."""
    opened, errors, stop = [], queue.Queue(), threading.Event()
    info = {}
    try:
        for cam_def in cameras:
            name, serial = cam_def[0], cam_def[1]
            settings = cam_def[2] if len(cam_def) > 2 else None
            fmt = cam_def[3] if len(cam_def) > 3 else None
            try:
                cam = open_camera(serial, width, height, fps, settings, fake, fmt)
            except Exception as e:
                raise RuntimeError(f"{name} ({serial}): {e}") from e
            opened.append((name, cam))
            info[name] = {"serial": str(serial), "format": fmt, "settings": cam.applied}
        for name, cam in opened:   # before the threads start: read() is used directly here
            info[name].update(_brightness(cam))
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
    reply_q.put(("ready", info))
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
                    if not cmd[1] and ep_dir is not None:   # the episode folder holds only camera files
                        shutil.rmtree(ep_dir, ignore_errors=True)
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

    cameras: list of (name, serial), (name, serial, settings) or (name, serial, settings, format), e.g.
    camera_defs(settings.record) or [("front", "943222070221", {"exposure_us": 16667, "gain": 76,
    "white_balance_k": 4600}), ...]. After open(), info = {name: {serial, format, settings (as read back, or
    None = auto), brightness, saturated_frac, missing_rows_frac}}.
    """

    def __init__(self, cameras, width=640, height=480, fps=30, quality=95, fake=False):
        ctx = mp.get_context("spawn")   # not fork: the teleop process has threads (gripper)
        self.cmd_q, self.reply_q = ctx.Queue(), ctx.Queue()
        self.names = [c[0] for c in cameras]
        self.info = {}
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
        self.info = reply[1]
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


def saturation_problems(info, max_frac=SATURATED_MAX_FRAC):
    """Cameras whose start image is saturated (more than max_frac of the pixels white): list of texts. A
    saturated image has no detail to recover, so the callers refuse to start (no typed yes)."""
    problems = []
    for name, cam in info.items():
        f = cam.get("saturated_frac")
        if f is not None and f > max_frac:
            problems.append(f"{name}: {100 * f:.0f} % of the pixels saturated (white; limit {100 * max_frac:.0f} %), "
                            f"mean brightness {cam.get('brightness')}: the images would carry no detail. Check the "
                            f"camera (unplug / replug it, scripts/34) and the light on it, then start again")
    return problems


def missing_rows_problems(info, max_frac=MISSING_ROWS_MAX_FRAC):
    """Cameras whose start images lack rows (frame data that did not arrive, see image_level): list of texts.
    The callers refuse to start, as for a saturated image."""
    problems = []
    for name, cam in info.items():
        f = cam.get("missing_rows_frac")
        if f is not None and f > max_frac:
            problems.append(f"{name}: {100 * f:.0f} % of the image rows missing (pure green; limit {100 * max_frac:.0f} %): "
                            f"the frames arrive incomplete. USB bandwidth? (uvcvideo quirks, another camera on the "
                            f"same bus: lsusb -t)")
    return problems


def refusal_problems(info):
    """saturation_problems + missing_rows_problems: start images that must not be recorded (no typed yes)."""
    return saturation_problems(info) + missing_rows_problems(info)


def camera_defs(record, names=None):
    """[(name, device, settings, format)] of the settings' record.cameras (cam-5a), in the config order or in
    the order of names (e.g. the cameras a policy expects); KeyError for a name that is not configured."""
    cams = vars(record.cameras)
    return [(n, cams[n].device, {"exposure_us": cams[n].exposure_us, "gain": cams[n].gain,
                                 "white_balance_k": cams[n].white_balance_k}, cams[n].format)
            for n in (names or list(cams))]


def print_red(lines):
    for line in lines:
        print(f"{RED}{line}{RESET}")


def brightness_problems(info, reference, tolerance):
    """Cameras whose start brightness is outside reference x (1 +/- tolerance): list of texts. Also fills
    info[name]["brightness_ref"] / ["brightness_ok"] for the record."""
    problems = []
    lo, hi = reference * (1.0 - tolerance), reference * (1.0 + tolerance)
    for name, cam in info.items():
        b = cam.get("brightness")
        cam["brightness_ref"], cam["brightness_tolerance"] = reference, tolerance
        cam["brightness_ok"] = b is not None and lo <= b <= hi
        if not cam["brightness_ok"]:
            problems.append(f"{name}: mean brightness {b} outside {lo:.0f}..{hi:.0f} (reference {reference:g} "
                            f"+/- {100 * tolerance:.0f} %): is the lighting the same (blinds, ceiling lights)?")
    return problems

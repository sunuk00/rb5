"""Demonstration episodes in the teleop (rec-3b): keys, recording / reset phases, episodes.json.

Keys (as LeRobot's lerobot-record, read from the terminal):
    -> or n   recording: end the episode, kept        reset: start the next episode
    <- or r   recording: discard the episode          reset: discard the episode just finished
    Esc or q  end the session (an episode being recorded is discarded; LeRobot keeps it)
The session starts in the reset phase: the first episode starts with ->. An episode longer than
record.episode_time_s ends and is kept; record.reset_time_s > 0 starts the next one after that time.
Between episodes (reset) the operator moves the robot with the teleop to the next start position and
places the objects; nothing moves by itself.

Files in the session folder (data/demos/<mode>_<time>/):
    episodes.json                every attempt: number, status (recording / kept / discarded), reason, times,
                                 start joints and TCP, camera frames; rewritten at every change
    episode_<attempt>/cam_*      camera files (teleop/cameras.py); deleted for a discarded attempt
    teleop.csv                   the loop CSV; column `episode` = attempt number while recording, else blank
Attempt numbers count every start (kept or not); the kept ones in order make the dataset.

Sounds (record.sounds, rec-3c): spoken messages as LeRobot's (spd-say on Linux), so the operator does not
have to look at the terminal: "Recording episode 3", "Episode saved", "Episode discarded", "Camera error".
"""
import json
import os
import queue
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty
from pathlib import Path

KEYS = {"n": "next", "N": "next", "r": "rerecord", "R": "rerecord", "q": "quit", "Q": "quit"}
ARROWS = {"C": "next", "D": "rerecord"}   # ESC [ C / ESC O C = right, ... D = left


def parse_keys(data):
    """Key bytes from the terminal -> list of "next" / "rerecord" / "quit"."""
    out, i = [], 0
    text = data.decode("latin-1")
    while i < len(text):
        c = text[i]
        if c == "\x1b":
            if i + 2 < len(text) and text[i + 1] in "[O":
                if text[i + 2] in ARROWS:
                    out.append(ARROWS[text[i + 2]])
                i += 3
                continue
            if i + 1 == len(text):   # a lone Esc (an arrow arrives as one read of 3 bytes)
                out.append("quit")
            i += 1
            continue
        if c in KEYS:
            out.append(KEYS[c])
        i += 1
    return out


class KeyReader:
    """Reads keys from the terminal in a thread (cbreak mode: no Enter needed, no echo; Ctrl+C still works).
    keys() returns the keys since the last call without waiting. stop() restores the terminal."""

    def __init__(self, fd=None):
        self.fd = sys.stdin.fileno() if fd is None else fd
        self.q = queue.Queue()
        self.old = None
        self._stop = threading.Event()
        self.thread = None

    def start(self):
        """Returns False when stdin is not a terminal (no keys)."""
        if not os.isatty(self.fd):
            return False
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.thread = threading.Thread(target=self._run, name="keys", daemon=True)
        self.thread.start()
        return True

    def _run(self):
        while not self._stop.is_set():
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if ready:
                for k in parse_keys(os.read(self.fd, 32)):
                    self.q.put(k)

    def keys(self):
        out = []
        while True:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                return out

    def stop(self):
        self._stop.set()
        if self.thread is not None:
            self.thread.join(0.5)
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.old = None


class Speaker:
    """Speaks short messages with spd-say in its own thread (the loop only puts the text in a queue).
    enabled is False when spd-say is not installed; say() then does nothing."""

    def __init__(self, enabled=True, command="spd-say"):
        self.cmd = shutil.which(command) if enabled else None
        self.enabled = self.cmd is not None
        self.q = queue.Queue()
        if self.enabled:
            threading.Thread(target=self._run, name="speaker", daemon=True).start()

    def say(self, text):
        if self.enabled:
            self.q.put(text)

    def wait(self, timeout=2.0):
        """Wait until the queued messages were handed to spd-say (before the program exits)."""
        end = time.monotonic() + timeout
        while self.enabled and self.q.unfinished_tasks and time.monotonic() < end:
            time.sleep(0.02)

    def _run(self):
        while True:
            text = self.q.get()
            try:   # spd-say returns at once; the speech daemon plays it. -C cancels a message still playing
                subprocess.run([self.cmd, "-C"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
                subprocess.run([self.cmd, text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            except (OSError, subprocess.SubprocessError):
                pass
            self.q.task_done()


class EpisodeRecorder:
    """Recording / reset phases. The loop calls cycle() once per cycle; nothing here waits.

    keys: object with keys() -> list of key names (KeyReader or a fake)
    cameras: CameraRecorder or None (robot data only)
    """

    def __init__(self, folder, task, episode_time_s, reset_time_s, keys, cameras=None, clock=time.monotonic,
                 say=lambda text: None):
        self.folder, self.task = Path(folder), task
        self.episode_time_s, self.reset_time_s = episode_time_s, reset_time_s
        self.keys, self.cameras, self.clock, self.say = keys, cameras, clock, say
        self.folder.mkdir(parents=True, exist_ok=True)
        self.attempts = []          # dicts, written to episodes.json
        self.current = None         # attempt being recorded
        self.last_kept = None       # attempt that ← in the reset phase discards
        self.reset_since = clock()
        self.loop_t0 = None
        self.camera_problem = None  # text: the cameras stopped working
        self.pending_stops = 0      # stop_episode sent, "stopped" not back yet
        self._write()

    @property
    def recording(self):
        return self.current is not None

    def kept(self):
        return sum(1 for a in self.attempts if a["status"] == "kept")

    def column(self):
        """Value of the CSV column `episode`: the attempt number while recording, else ""."""
        return "" if self.current is None else self.current["attempt"]

    def status_text(self, now=None):
        now = self.clock() if now is None else now
        if self.current is not None:
            return f"REC {self.kept()} {now - self.current['t_start']:5.1f} s"
        return f"RESET (kept {self.kept()})"

    def cycle(self, q, tcp, now=None):
        """Handle the keys, the time limits and the camera messages. Returns (events, quit)."""
        now = self.clock() if now is None else now
        events, quit_ = [], False
        if self.cameras is not None:
            for msg in self.cameras.poll():
                events += self._camera_message(msg)
        for key in self.keys.keys():
            if key == "next":
                if self.current is not None:
                    events.append(self._end(now, "kept", "-> key"))
                else:
                    events.append(self._start(now, q, tcp))
            elif key == "rerecord":
                if self.current is not None:
                    events.append(self._end(now, "discarded", "<- key"))
                elif self.last_kept is not None:
                    a = self.last_kept
                    a.update(status="discarded", reason=a["reason"] + "; discarded in the reset (<- key)")
                    self.last_kept = None
                    if self.cameras is not None:
                        self.cameras.delete_episode(self.folder / a["dir"])
                    self._write()
                    events.append(f"episode attempt {a['attempt']} discarded; kept {self.kept()}")
                    self.say("Episode discarded")
                else:
                    events.append("<-: nothing to discard")
            elif key == "quit":
                quit_ = True
                break
        if not quit_:
            if self.current is not None and self.camera_problem:
                events.append(self._end(now, "discarded", f"camera problem: {self.camera_problem}"))
            elif self.current is not None and now - self.current["t_start"] >= self.episode_time_s:
                events.append(self._end(now, "kept", f"time limit {self.episode_time_s:.0f} s"))
            elif (self.current is None and self.reset_time_s > 0 and self.attempts
                  and now - self.reset_since >= self.reset_time_s):
                events.append(self._start(now, q, tcp))
        return events, quit_

    def finish(self, reason, timeout=10.0):
        """Session end (after the loop): an episode being recorded is discarded, then waits until the camera
        process has closed the files of every stopped episode. Returns event texts."""
        events = [] if self.current is None else [self._end(self.clock(), "discarded", reason)]
        end = time.monotonic() + timeout
        while self.cameras is not None and self.pending_stops > 0 and time.monotonic() < end:
            for msg in self.cameras.poll():
                events += self._camera_message(msg)
            time.sleep(0.01)
        if self.pending_stops > 0:
            events.append(f"WARNING: the camera process did not confirm {self.pending_stops} episode end(s)")
        return events

    def _start(self, now, q, tcp):
        if self.camera_problem:
            return f"not started: camera problem ({self.camera_problem})"
        n = len(self.attempts)
        a = {"attempt": n, "dir": f"episode_{n:03d}", "status": "recording", "reason": "", "task": self.task,
             "t_start": now, "t_end": None, "start_joints_deg": [round(float(v), 3) for v in q],
             "start_tcp": [round(float(v), 3) for v in tcp], "cameras": None}
        self.attempts.append(a)
        self.current, self.last_kept = a, None
        if self.cameras is not None:
            self.cameras.start_episode(self.folder / a["dir"])
        self._write()
        self.say(f"Recording episode {self.kept()}")
        return f"episode {self.kept()} RECORDING (attempt {n}): -> ends and keeps, <- discards, q quits"

    def _end(self, now, status, reason):
        a, self.current = self.current, None
        a.update(status=status, reason=reason, t_end=now)
        self.last_kept = a if status == "kept" else None
        self.reset_since = now
        if self.cameras is not None:
            self.cameras.stop_episode(keep=status == "kept")
            self.pending_stops += 1
        self._write()
        took = now - a["t_start"]
        self.say(("Time limit, episode saved" if reason.startswith("time limit") else "Episode saved")
                 if status == "kept" else "Episode discarded")
        if status == "kept":
            return (f"episode {self.kept() - 1} KEPT ({took:.1f} s, {reason}); reset: place the objects, move to a "
                    f"start pose, then ->  (<- discards it)")
        return f"attempt {a['attempt']} DISCARDED ({took:.1f} s, {reason}); kept {self.kept()}"

    def _camera_message(self, msg):
        if msg[0] == "stopped":
            self.pending_stops -= 1
            for a in self.attempts:   # the oldest stopped attempt without camera results
                if a["status"] != "recording" and a["cameras"] is None:
                    a["cameras"] = msg[2]
                    self._write()
                    problems = [f"{n}: {st['gaps']} missed, {st['dropped']} dropped"
                                for n, st in msg[2].items() if st["gaps"] or st["dropped"]]
                    if problems and a["status"] == "kept":
                        return [f"WARNING attempt {a['attempt']}: camera frames lost ({'; '.join(problems)})"]
                    return []
        elif msg[0] == "error":
            self.camera_problem = msg[1]
            self.say("Camera error")
            return [f"CAMERA ERROR: {msg[1]} (recording is not possible until the cameras work again)"]
        return []

    def _write(self):
        data = {"task": self.task, "episode_time_s": self.episode_time_s, "reset_time_s": self.reset_time_s,
                "time": "t_start / t_end: time.monotonic() [s], the same clock as the camera t_capture_s; "
                        "loop_t0: time.monotonic() at the CSV t_s = 0",
                "loop_t0": self.loop_t0, "attempts": self.attempts}
        tmp = self.folder / "episodes.json.tmp"
        tmp.write_text(json.dumps(data, indent=1))
        tmp.replace(self.folder / "episodes.json")

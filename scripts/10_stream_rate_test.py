"""Command streaming rate test with move_servo_j (Simulation mode only).

Streams J6 = base_J6 + 5 * sin(2*pi*0.5*t) deg for 10 s at a chosen rate, other joints
fixed at the current pose (base = current jnt_ref, so the first command does not jump).
jnt_ref is read in a background thread (latest-value pattern from 08_servo_jog_fast.py).

move_servo_j(joint, t1, t2, gain, alpha) is not documented in refs/rbpodo. From 06:
t1 should match the send period, so t1 defaults to 1 / rate (override with --t1).

Reports achieved send rate, send call duration, tracking error (mean / max), error in the
first 2 s vs the last 2 s, and an estimated lag. Saves CSV + plot to docs/measurements/.

Usage:
    python scripts/10_stream_rate_test.py --hz 50
    python scripts/10_stream_rate_test.py --hz 200 --t1 0.01
"""
import argparse
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # write PNG files, no window
import matplotlib.pyplot as plt
import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

JOINT_INDEX = 5            # J6
AMPLITUDE_DEG = 5.0
FREQ_HZ = 0.5
DURATION_S = 10.0
RATE_CHOICES = (20, 50, 100, 200)
SERVO_T2 = 0.1             # observed values from 06, not confirmed by a manual
SERVO_GAIN = 1.0
SERVO_ALPHA = 1.0
HOLD_ON_EXIT_S = 0.3       # keep sending the last target this long before ending servo mode
EDGE_WINDOW_S = 2.0        # compare the error in the first and last this many seconds
MAX_LAG_S = 0.15           # search range for the lag estimate
OUT_DIR = ROOT / "docs" / "measurements"

# Chart colors (reference palette from the dataviz guide, light mode)
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
SERIES_TARGET = "#2a78d6"  # slot 1
SERIES_REF = "#eb6834"     # slot 2
SERIES_ERROR = "#1baf7a"   # slot 3


class StateReader(threading.Thread):
    """Reads the data channel continuously on its own socket and keeps only the latest sample."""

    def __init__(self):
        super().__init__(daemon=True)
        self.data_channel = rb.CobotData(config.ROBOT_IP)
        self.lock = threading.Lock()
        self.sample = None     # (jnt_ref, receive time)
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            data = self.data_channel.request_data(1.0)
            if data is None:
                continue
            # Simulation mode: jnt_ang (measured) does not change, so use jnt_ref (commanded).
            q = np.array(data.sdata.jnt_ref, dtype=float)
            with self.lock:
                self.sample = (q, time.monotonic())

    def wait_first(self, timeout_s=2.0):
        t0 = time.monotonic()
        while self.sample is None:
            if time.monotonic() - t0 > timeout_s:
                raise SystemExit("State reader got no data from the robot.")
            time.sleep(0.01)

    def latest(self):
        """Return (jnt_ref, receive time) of the newest sample, without waiting."""
        with self.lock:
            q, t = self.sample
        return q.copy(), t

    def stop(self):
        self.stop_event.set()
        self.join(2.0)


def offset(t):
    """Target offset of J6 (deg) at time t (s) after the start; 0 before the start."""
    t = np.maximum(t, 0.0)
    return AMPLITUDE_DEG * np.sin(2 * np.pi * FREQ_HZ * t)


def wait_for_simulation(data_channel):
    t0 = time.monotonic()
    while True:
        sdata = data_channel.request_data(1.0).sdata
        if sdata.real_vs_simulation_mode == 1:
            return sdata
        if time.monotonic() - t0 > 2.0:
            raise SystemExit("Controller is not in Simulation mode. Stopping.")
        time.sleep(0.05)


def wait_until_stopped(robot, rc, timeout_s=10.0):
    t0 = time.monotonic()
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        if time.monotonic() - t0 > timeout_s:
            raise SystemExit(f"Robot still moving after {timeout_s} s.")
        time.sleep(0.01)


def sleep_until(next_t, period):
    """Absolute-schedule sleep. After an overrun, wait a full period so sends are never back to back."""
    delay = next_t - time.monotonic()
    if delay > 0:
        time.sleep(delay)
        return next_t
    time.sleep(period)
    return time.monotonic()


def stream(robot, rc, reader, q0, hz, t1):
    """Stream the sine target. Returns per-cycle rows, the start time, and the last target."""
    period = 1.0 / hz
    target = q0.copy()
    rows = []   # (send time, send duration, target J6, jnt_ref J6, state receive time), times from start
    t_start = time.monotonic()
    next_t = t_start
    while True:
        t = time.monotonic() - t_start
        if t >= DURATION_S:
            break
        target[JOINT_INDEX] = q0[JOINT_INDEX] + offset(t)
        t_send = time.monotonic()
        robot.move_servo_j(rc, target, t1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
        send_duration = time.monotonic() - t_send
        q, t_state = reader.latest()
        rows.append((t_send - t_start, send_duration, target[JOINT_INDEX], q[JOINT_INDEX], t_state - t_start))

        # ACK waiting is off, so read queued controller messages here and stop on any error
        robot.flush(rc)
        rc.error().throw_if_not_empty()
        rc.clear()

        next_t = sleep_until(next_t + period, period)
    return rows, target


def finish_streaming(robot, rc, target, hz, t1):
    """Hold the last target, then end servo mode as in examples/move_servo_j.cpp, polling for the stop."""
    period = 1.0 / hz
    next_t = time.monotonic()
    t_end = next_t + HOLD_ON_EXIT_S
    while time.monotonic() < t_end:
        robot.move_servo_j(rc, target, t1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
        next_t = sleep_until(next_t + period, period)
    robot.move_speed_j(rc, np.zeros(6), t1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)
    robot.enable_waiting_ack(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def emergency_stop(robot, rc):
    robot.enable_waiting_ack(rc)
    robot.task_stop(rc)
    robot.flush(rc)
    wait_until_stopped(robot, rc)
    rc.clear()


def analyze(rows, q0_j6, hz):
    arr = np.array(rows)
    t_send, send_dur, ref, t_state = arr[:, 0], arr[:, 1], arr[:, 3], arr[:, 4]

    send_iv = np.diff(t_send) * 1000
    print(f"\nSend rate   : {len(rows) / DURATION_S:6.1f} Hz achieved (requested {hz} Hz, {len(rows)} sends)")
    print(f"Send interval: mean {send_iv.mean():6.2f} ms   min {send_iv.min():6.2f}   max {send_iv.max():6.2f}   "
          f"(over 1.5x period: {np.sum(send_iv > 1500 / hz)})")
    dur = send_dur * 1000
    print(f"Send call   : mean {dur.mean():6.3f} ms   max {dur.max():6.3f} ms")

    # Each state sample once; compare it with the target at the time the sample was received
    _, idx = np.unique(t_state, return_index=True)
    ts, rs = t_state[idx], ref[idx]
    keep = (ts >= 0) & (ts <= DURATION_S)
    ts, rs = ts[keep], rs[keep]
    err = (q0_j6 + offset(ts)) - rs
    abs_err = np.abs(err)
    first = abs_err[ts < EDGE_WINDOW_S]
    last = abs_err[ts > DURATION_S - EDGE_WINDOW_S]
    print(f"Tracking error (target at state time - jnt_ref, {len(ts)} state samples):")
    print(f"              mean |err| {abs_err.mean():.3f} deg   max |err| {abs_err.max():.3f} deg")
    print(f"              first {EDGE_WINDOW_S:.0f} s mean {first.mean():.3f} deg (max {first.max():.3f})   "
          f"last {EDGE_WINDOW_S:.0f} s mean {last.mean():.3f} deg (max {last.max():.3f})")

    # Lag estimate: shift the target in time and find the shift that best matches jnt_ref
    lags = np.arange(0.0, MAX_LAG_S, 0.001)
    rms = [np.sqrt(np.mean(((q0_j6 + offset(ts - lag)) - rs) ** 2)) for lag in lags]
    best = int(np.argmin(rms))
    print(f"Lag estimate: {lags[best] * 1000:.0f} ms (residual RMS {rms[best]:.3f} deg after shifting)")
    return ts, rs, err, lags[best]


def save_csv(rows, q0_j6, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["send_time_s", "send_duration_ms", "target_j6_deg", "jnt_ref_j6_deg",
                         "state_time_s", "target_at_state_time_deg"])
        for t_send, dur, target, ref, t_state in rows:
            writer.writerow([f"{t_send:.6f}", f"{dur * 1000:.4f}", f"{target:.4f}", f"{ref:.4f}",
                             f"{t_state:.6f}", f"{q0_j6 + offset(t_state):.4f}"])


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def save_plot(rows, q0_j6, ts, err, lag, hz, t1, path):
    arr = np.array(rows)
    t_send, target, ref = arr[:, 0], arr[:, 2], arr[:, 3]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), dpi=150, sharex=True,
                                   gridspec_kw={"height_ratios": [2, 1]})
    fig.patch.set_facecolor(SURFACE)

    ax1.plot(t_send, target, color=SERIES_TARGET, linewidth=1.5, label="target (sent)", zorder=3)
    ax1.plot(ts, q0_j6 + offset(ts) - err, color=SERIES_REF, linewidth=1.5, label="jnt_ref (read)", zorder=2)
    ax1.set_ylabel("J6 (deg)", color=TEXT_SECONDARY)
    ax1.set_title(f"move_servo_j streaming at {hz} Hz (t1 = {t1:g}), Simulation", color=TEXT_PRIMARY,
                  loc="left", fontsize=11)
    leg = ax1.legend(loc="upper right", frameon=False, fontsize=9)
    for text in leg.get_texts():
        text.set_color(TEXT_PRIMARY)
    style_axes(ax1)

    for x0 in (0.0, DURATION_S - EDGE_WINDOW_S):
        ax2.axvspan(x0, x0 + EDGE_WINDOW_S, color=GRID, alpha=0.5, zorder=0)
    ax2.plot(ts, err, color=SERIES_ERROR, linewidth=1.5, zorder=3)
    ax2.axhline(0, color=TEXT_SECONDARY, linewidth=0.8, zorder=2)
    ax2.set_ylabel("error (deg)", color=TEXT_SECONDARY)
    ax2.set_xlabel("time (s)", color=TEXT_SECONDARY)
    ax2.set_title(f"tracking error = target - jnt_ref   (lag about {lag * 1000:.0f} ms; shaded: first / last "
                  f"{EDGE_WINDOW_S:.0f} s)", color=TEXT_SECONDARY, loc="left", fontsize=9)
    style_axes(ax2)

    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hz", type=int, required=True, choices=RATE_CHOICES, help="send rate")
    parser.add_argument("--t1", type=float, default=None, help="move_servo_j t1 (default: 1 / hz)")
    args = parser.parse_args()
    t1 = args.t1 if args.t1 is not None else 1.0 / args.hz
    np.set_printoptions(precision=3, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001), used before the reader starts

    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()
    rc.clear()
    sdata = wait_for_simulation(data_channel)

    q0 = np.array(sdata.jnt_ref, dtype=float)   # base pose = current pose
    print(f"Mode: Simulation   rate {args.hz} Hz   t1={t1:g} t2={SERVO_T2} gain={SERVO_GAIN} alpha={SERVO_ALPHA}")
    print("Base (current jnt_ref):", q0)
    print(f"J6 target: {q0[JOINT_INDEX]:.2f} + {AMPLITUDE_DEG} * sin(2*pi*{FREQ_HZ}*t) deg for {DURATION_S} s "
          f"(peak speed {AMPLITUDE_DEG * 2 * np.pi * FREQ_HZ:.1f} deg/s)")

    reader = StateReader()
    reader.start()
    reader.wait_first()

    rows, target = [], q0.copy()
    error = None
    robot.disable_waiting_ack(rc)   # servo commands must not wait for an ACK each cycle
    try:
        rows, target = stream(robot, rc, reader, q0, args.hz, t1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:   # controller error: stop without more servo commands
        error = e
        print(f"\nError: {e}")
    finally:
        if error is None:
            finish_streaming(robot, rc, target, args.hz, t1)
        else:
            emergency_stop(robot, rc)
        reader.stop()

    if len(rows) < 2:
        raise SystemExit("Not enough data.")
    ts, _, err, lag = analyze(rows, q0[JOINT_INDEX], args.hz)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stem = f"stream_{args.hz}hz_t1_{t1:g}_{datetime.now():%Y%m%d_%H%M%S}"
    csv_path, png_path = OUT_DIR / f"{stem}.csv", OUT_DIR / f"{stem}.png"
    save_csv(rows, q0[JOINT_INDEX], csv_path)
    save_plot(rows, q0[JOINT_INDEX], ts, err, lag, args.hz, t1, png_path)
    print(f"\nSaved: {csv_path.relative_to(ROOT)}")
    print(f"Saved: {png_path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

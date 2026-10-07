"""Estimate the data channel update period (Simulation mode only).

Moves J6 slowly in Simulation mode, reads the data channel as fast as possible
during the move, logs every sample to a CSV file, then prints:
  - read interval:   time between our consecutive reads
  - change interval: time between reads where jnt_ref actually changed
If we read faster than the controller updates, the change interval approximates
the controller's internal update period.

Usage:
    python scripts/04_data_rate.py
"""
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rbpodo as rb

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import config  # noqa: E402

JOINT_INDEX = 5            # J6 (wrist 3)
DELTA_DEG = 60.0           # large move; Simulation only
JOINT_SPEED = 10.0         # move_j speed (deg/s), scaled by the speed bar
JOINT_ACC = 20.0           # move_j acceleration (deg/s^2)
TAIL_S = 0.5               # keep recording this long after the move ends
TIMEOUT_S = 120.0          # stop recording after this long no matter what
STATE_MOVING = 3           # sdata.robot_state: 3 = executing motion, 1 = idle
DATA_DIR = ROOT / "data"


def stats_ms(intervals):
    ms = np.asarray(intervals) * 1000.0
    return f"mean {ms.mean():8.3f} ms   min {ms.min():8.3f} ms   max {ms.max():8.3f} ms   (n={len(ms)})"


def main():
    np.set_printoptions(precision=3, suppress=True)

    robot = rb.Cobot(config.ROBOT_IP)              # command channel (5000)
    rc = rb.ResponseCollector()
    data_channel = rb.CobotData(config.ROBOT_IP)   # data channel (5001)

    # 1. Simulation mode, and verify the controller actually reports it
    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    rc.error().throw_if_not_empty()
    # The mode flag can take a moment to update after the switch; wait up to 2 s.
    t0 = time.monotonic()
    while True:
        sdata = data_channel.request_data(1.0).sdata
        if sdata.real_vs_simulation_mode == 1:
            break
        if time.monotonic() - t0 > 2.0:
            raise SystemExit("Controller is not in Simulation mode. Stopping.")
        time.sleep(0.05)

    robot.set_speed_bar(rc, config.SPEED)
    rc.error().throw_if_not_empty()

    # 2. Target relative to the current commanded pose (jnt_ref; jnt_ang is frozen in Simulation)
    q_now = np.array(sdata.jnt_ref, dtype=float)
    q_target = q_now.copy()
    q_target[JOINT_INDEX] += DELTA_DEG
    print("Mode: Simulation")
    print("Current (jnt_ref):", q_now)
    print("Target:           ", q_target)

    # 3. Send the move, then read as fast as possible until it ends
    robot.flush(rc)
    robot.move_j(rc, q_target, JOINT_SPEED, JOINT_ACC)
    rc.error().throw_if_not_empty()

    rows = []
    missed = 0
    seen_moving = False
    t_end = None
    t_start = time.perf_counter()
    while True:
        data = data_channel.request_data(1.0)
        t = time.perf_counter()
        if data is None:
            missed += 1
        else:
            s = data.sdata
            rows.append([t - t_start, s.time, s.robot_state, *s.jnt_ref, *s.jnt_ang])
            if s.robot_state == STATE_MOVING:
                seen_moving = True
            elif seen_moving and t_end is None:
                t_end = t + TAIL_S
        if t_end is not None and t >= t_end:
            break
        if t - t_start > TIMEOUT_S:
            print(f"warning: stopped recording after {TIMEOUT_S} s timeout")
            break
    if not seen_moving:
        print("warning: robot_state never reported moving (3)")

    # 4. Save CSV
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"04_data_rate_{datetime.now():%Y%m%d_%H%M%S}.csv"
    header = (["pc_time_s", "ctrl_time_s", "robot_state"]
              + [f"jnt_ref_{i + 1}" for i in range(6)]
              + [f"jnt_ang_{i + 1}" for i in range(6)])
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

    # 5. Statistics
    arr = np.array(rows, dtype=float)
    pc_t = arr[:, 0]
    ctrl_t = arr[:, 1]
    state = arr[:, 2]
    jnt_ref = arr[:, 3:9]

    read_iv = np.diff(pc_t)

    # Samples (during the move) where jnt_ref differs from the previous sample
    changed = np.any(jnt_ref[1:] != jnt_ref[:-1], axis=1) & (state[1:] == STATE_MOVING)
    change_t = pc_t[1:][changed]
    change_iv = np.diff(change_t)

    # Same idea using the controller's own clock
    moving = state == STATE_MOVING
    ctrl_moving = ctrl_t[moving]
    ctrl_iv = np.diff(np.unique(ctrl_moving))

    print(f"\nSamples: {len(rows)}  (missed reads: {missed})  duration: {pc_t[-1]:.2f} s")
    print(f"Move duration (robot_state == 3): "
          f"{(pc_t[moving][-1] - pc_t[moving][0]) if moving.any() else 0:.2f} s")
    print(f"Saved: {path.relative_to(ROOT)}\n")
    print("Read interval (PC clock, all samples):")
    print("  " + stats_ms(read_iv))
    if len(change_iv) > 0:
        print("jnt_ref change interval (PC clock, during move):")
        print("  " + stats_ms(change_iv))
    else:
        print("jnt_ref change interval: not enough changes to compute")
    if len(ctrl_iv) > 0:
        print("Controller time (sdata.time) step (during move):")
        print("  " + stats_ms(ctrl_iv))

    if len(change_iv) > 0 and np.mean(read_iv) > 0.5 * np.mean(change_iv):
        print("\nwarning: reads are not clearly faster than value changes; "
              "the change interval may just reflect our read rate.")

    print(f"\nFinal jnt_ref: {jnt_ref[-1]}")


if __name__ == "__main__":
    main()

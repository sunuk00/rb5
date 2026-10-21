"""rec-6b: run a trained ACT policy on the RB5 in SIMULATION (rb5_lerobot/policy.py). Real is refused for now.

MOVES THE RB5 (Simulation mode) AND THE GRIPPER (the real gripper moves in Simulation too, as in the
teleop; --no-gripper runs without it). The gripper follows the policy's gripper action at once while
running (no catch: the dataset's gripper action is what the gripper did).

Run in the `lerobot` conda env, from the repository root:
    conda activate lerobot
    python -m rb5_lerobot.run data/train/act_<name>_<time>                     # its last checkpoint
    python -m rb5_lerobot.run data/train/act_<name>_<time>/checkpoints/005000  # a given checkpoint
    python -m rb5_lerobot.run <training folder> --config configs/<copy>.yaml   # other run settings

Settings: the run section of configs/policy.yaml (time limit, holds, temporal ensembling weight, CPU cores);
the hardware (cameras, robot, gripper, the teleop's limits) from configs/sim.yaml (--robot-config). The
camera images are shrunk to the size the checkpoint was trained on (its config.json), with the converter's
function (data_format.resize_image). The run JSON keeps the run section used.

Steps:
  1. settings (configs/sim.yaml); the cameras the policy expects (its config.json), opened in the policy
     process with the recording's fixed exposure / gain / white balance (read back) and the start
     brightness check (typed yes when outside record.brightness_ref +/- tolerance)
  2. gripper read-only checks (before anything moves); Simulation mode and the teleop's checks; move_j to
     the ready pose after a typed yes; the gripper start writes after a typed yes (rec-6c)
  3. the first chunks: the first action's distance from the current TCP is shown; typed yes
  4. the 50 Hz loop (PolicyRunner): keys in this terminal: g = run, space = hold, q = quit. Holds on a chunk
     older than run.stale_s, an action further than run.max_jump_mm from the target, the time limit; stops
     when no chunk came for run.stop_stale_s. Every target goes through the teleop's limits (TargetFollower), speed caps 100 %.
  5. normal stop (task_stop on errors); data/policy/run_<time>.csv (a row per cycle) and the summary:
     late cycles, chunk age, inference time, holds
"""
import argparse
import csv
import gc
import json
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop import preflight  # noqa: E402
from teleop.cameras import brightness_problems, camera_defs, print_red, refusal_problems  # noqa: E402
from teleop.episodes import KeyReader  # noqa: E402
from teleop.follow import TargetFollower  # noqa: E402
from teleop.gripper import MODE_AT_END, PRESENT_POSITION, open_gripper, start_lines, start_worker  # noqa: E402
from teleop.kinematics import RB5Model  # noqa: E402
from rb5_lerobot.config import DEFAULT, load_policy_config  # noqa: E402
from rb5_lerobot.policy import (RUN_HEADER, PolicyClient, PolicyRunner, StopRequest, parse_policy_keys,  # noqa: E402
                                policy_image_sizes, policy_inputs, resolve_checkpoint)
from teleop.safety import Watch  # noqa: E402
from teleop.settings import SettingsError, load_settings  # noqa: E402


def to_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def ask(prompt):
    return input(prompt).strip() == "yes"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", help="training folder data/train/act_<name>_<time> (its last checkpoint) or a "
                                       "checkpoint folder")
    ap.add_argument("--config", default=str(DEFAULT), help="policy settings file (its run section)")
    ap.add_argument("--robot-config", default=str(ROOT / "configs" / "sim.yaml"), help="hardware settings")
    ap.add_argument("--task", help="task text (default record.task)")
    ap.add_argument("--no-gripper", action="store_true", help="without the gripper (its action is only logged)")
    ap.add_argument("--real", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.real:
        raise SystemExit("Real is not allowed yet (rec-6b is Simulation only)")
    np.set_printoptions(precision=1, suppress=True)
    try:
        s, _ = load_settings(args.robot_config, real=False)
        pc, pc_raw = load_policy_config(args.config)
    except SettingsError as e:
        raise SystemExit(f"settings: {e}")
    rc = pc.run
    task = args.task or s.record.task
    ckpt = resolve_checkpoint(args.checkpoint)
    sizes = policy_image_sizes(ckpt)
    print(f"Checkpoint {ckpt}; images to the policy: "
          + ", ".join(f"{n} {w}x{h}" for n, (w, h) in sizes.items()) + " (cameras record 640x480)")
    print(f"Run settings ({args.config}): time limit {rc.time_limit_s:.0f} s, hold over {rc.max_jump_mm:.0f} mm or a "
          f"chunk older than {rc.stale_s} s, stop after {rc.stop_stale_s} s without a chunk, ensembling coeff "
          f"{rc.ensemble_coeff}")
    # Servo and policy on separate performance cores (cpu0-11 on the i7-13700H): in the first Simulation runs
    # PyTorch used every core and the servo loop overran 15-22 times in 30 s (rec-6b)
    servo_cores, policy_cores = list(rc.servo_cores), list(rc.policy_cores)
    os.sched_setaffinity(0, set(range(servo_cores[0], servo_cores[-1] + 1)))
    print(f"CPU cores: servo {sorted(os.sched_getaffinity(0))}, policy {policy_cores[0]}-{policy_cores[-1]} "
          f"({rc.torch_threads} PyTorch threads)")

    # 1. Policy process with the cameras the policy expects
    names, state_len = policy_inputs(ckpt)
    r = s.record
    known = list(vars(r.cameras))
    unknown = [n for n in names if n not in known]
    if unknown or state_len != 7:
        raise SystemExit(f"NOT STARTING - the policy expects cameras {names} / state {state_len} (known: {known}, 7)")
    cams = camera_defs(r, names)   # the same devices, formats and fixed settings as the recording
    print(f"Policy {ckpt} (cameras {names}), task '{task}'. Loading (model on the GPU, cameras) ...")
    client = PolicyClient({"kind": "act", "checkpoint": str(ckpt), "device": "cuda", "task": task,
                           "cpu_cores": policy_cores, "torch_threads": rc.torch_threads}, cams)
    problems = client.open(timeout=120.0)
    if problems:
        client.close()
        raise SystemExit("NOT STARTING - policy process:\n  " + "\n  ".join(problems))
    cam_info = client.info["cameras"]
    for n, c in cam_info.items():
        st = c["settings"]
        print(f"  {n} {c['serial']} ({c['format']}): exposure {st['exposure_us'] / 1000:.1f} ms, gain {st['gain']:g}, "
              f"white balance {st['white_balance_k']:g} K, brightness {c['brightness']}")
    problems = refusal_problems(cam_info)
    if problems:
        print_red(["NOT STARTING - camera image saturated or incomplete (no typed yes for this):"] + [f"  {p}" for p in problems])
        client.close()
        raise SystemExit(1)
    problems = brightness_problems(cam_info, r.brightness_ref, r.brightness_tolerance)
    for p in problems:
        print(f"  WARNING: {p}")
    if problems and not ask("Brightness differs from the recordings. Type yes to run anyway: "):
        client.close()
        raise SystemExit("Cancelled.")

    # 2. Gripper (read only here), robot (Simulation) and the ready pose
    motor = worker = grip_info = None   # grip_info: the gripper part of the run's JSON (None: off)
    if s.gripper.enabled and not args.no_gripper:
        motor, problems = open_gripper(s.gripper, s.leader.required_latency_timer_ms)
        if problems:
            client.close()
            raise SystemExit("NOT STARTING - gripper (--no-gripper runs without it):\n  " + "\n  ".join(problems))
        grip_info = {"closed_ticks": s.gripper.closed_ticks, "open_ticks": s.gripper.open_ticks,
                     "position_at_check": motor.read(PRESENT_POSITION)}
        print(f"Gripper: position {grip_info['position_at_check']} (closed {s.gripper.closed_ticks}, "
              f"open {s.gripper.open_ticks})")
    else:
        print("Gripper: off" + (" (--no-gripper)" if args.no_gripper else " (gripper.enabled is false)"))

    def close_all():
        client.close()
        if motor is not None:
            motor.close()

    from teleop.rb5 import RB5
    rb5 = RB5(s.robot.ip, s.servo)
    rb5.set_mode(False, s.robot.speed_bar)
    reached, waited = rb5.wait_for_mode(False)
    problems = preflight.check_robot_mode(False, rb5.reports_simulation(), rb5.is_moving(), rb5.error_text())
    if not reached:
        problems.append("Simulation mode not reached within 3 s")
    if problems:
        close_all()
        raise SystemExit("NOT STARTING - robot:\n  " + "\n  ".join(problems))
    offset, _ = rb5.tcp_offset()
    model, watch = RB5Model(offset), Watch(s.safety)
    q_now = np.array(rb5.sdata().jnt_ref, dtype=float)
    plan = preflight.plan_ready_move(q_now, s.ready_pose.joints_deg, s.safety.joint_limit_deg, s.ready_pose,
                                     s.robot.speed_bar)
    if plan["problems"]:
        close_all()
        raise SystemExit("NOT STARTING - ready pose:\n  " + "\n  ".join(plan["problems"]))
    if not plan["skip"]:
        print(f"\nMove to the ready pose with move_j (Simulation, about {plan['expected_s']:.0f} s):")
        print(f"  current {q_now}\n  target  {plan['target']}\n  change  {plan['change']}")
        if not ask("Type yes to move: "):
            close_all()
            raise SystemExit("Cancelled.")
        finished, took = rb5.move_j(plan["target"], plan["speed"], plan["acc"], plan["timeout_s"])
        q_after = np.array(rb5.sdata().jnt_ref, dtype=float)
        if not finished or rb5.error_text() or np.abs(q_after - plan["target"]).max() > s.ready_pose.tolerance_deg:
            close_all()
            raise SystemExit(f"ready pose move failed: finished {finished}, errors {rb5.error_text() or 'none'}")
    reader = rb5.state_reader()
    reader.start()
    reader.wait_first()
    tcp0, q0, _ = reader.latest()
    problems, _ = preflight.check_start_pose(tcp0, q0, model, watch, s)
    if problems:
        reader.stop()
        close_all()
        raise SystemExit("NOT STARTING - start pose:\n  " + "\n  ".join(problems))

    if motor is not None:
        g = s.gripper
        pos = motor.read(PRESENT_POSITION)
        print("\nGripper writes (the real gripper moves, also in Simulation):")
        for line in start_lines(g, pos):
            print(line)
        print("  the gripper follows the policy at once while running (g), no catch")
        if not ask("Type yes to start the gripper: "):
            reader.stop()
            close_all()
            raise SystemExit("Cancelled.")
        worker, _, held, problems = start_worker(motor, g, pos, s.servo.period_s, catch_frac=1.0)
        if problems:
            reader.stop()
            close_all()
            raise SystemExit("NOT STARTING - gripper start:\n  " + "\n  ".join(problems))
        print(f"Gripper: holding at {held}")
        grip_info.update(position_at_start=pos, held_ticks=held)

    # 3. First chunks (the policy sees the robot at the ready pose)
    grip0 = 0.0
    if worker is not None:
        # the worker owns the bus now: the held position is the gripper's position (converter's 0..1 rule)
        grip0 = float(np.clip((held - s.gripper.closed_ticks) / (s.gripper.open_ticks - s.gripper.closed_ticks), 0, 1))
    if grip_info is not None:
        grip_info["start_frac"] = round(grip0, 3)   # the first observation's gripper state
    client.publish(time.monotonic(), list(q0) + [grip0])
    first, t_end = [], time.monotonic() + 5.0
    while not first and time.monotonic() < t_end:
        first = client.poll()
        time.sleep(0.05)
    if not first:
        reader.stop()
        close_all()
        raise SystemExit("no chunk from the policy within 5 s")
    a0 = first[-1][1][0]
    print(f"\nStart TCP {tcp0[:3]} mm. The policy's first action {a0[:3]} mm, gripper {a0[3]:.2f}: "
          f"{np.linalg.norm(a0[:3] - tcp0[:3]):.0f} mm away (over {rc.max_jump_mm:.0f} mm holds). Inference {first[-1][2]:.0f} ms.")
    print(f"Keys here: g = run, space = hold, q = quit. Speed caps 100 % of the teleop; time limit {rc.time_limit_s:.0f} s.")
    if not ask("Type yes to start the loop (Simulation; the policy moves only after g): "):
        reader.stop()
        close_all()
        raise SystemExit("Cancelled.")

    # 4. Loop
    keys = KeyReader(parse=parse_policy_keys)
    follower = TargetFollower(s, model, tcp0)
    runner = PolicyRunner(s, rb5, reader, follower, client, keys, gripper=worker, real=False,
                          stale_s=rc.stale_s, stop_stale_s=rc.stop_stale_s, max_jump_mm=rc.max_jump_mm,
                          time_limit_s=rc.time_limit_s, coeff=rc.ensemble_coeff, event=lambda m: print(m, flush=True))
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, to_keyboard_interrupt)
    error = stop_text = None
    gc.collect()
    gc.freeze()          # long-lived objects out of the GC scans (a full pass delayed inference by 80 ms, rec-6b)
    rb5.begin_streaming()
    keys.start()
    try:
        runner.run()
    except StopRequest as e:
        stop_text = str(e)
    except KeyboardInterrupt:
        stop_text = "interrupted"
    except Exception as e:
        error = e
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        keys.stop()
        if error is None:
            rb5.end_streaming(follower.target, follower.rpy, s.safety.hold_on_exit_s, s.servo.period_s)
        else:
            rb5.emergency_stop()
        reader.stop()
        if worker is not None:
            t_back, m_back = worker.finish()
            print(f"Gripper: {worker.stop_reason or 'running until the end'}; read back: torque {t_back}, "
                  f"operating mode {m_back}")
            grip_info.update(stop_reason=worker.stop_reason, read_back_torque=t_back, read_back_mode=m_back,
                             read_back_ok=t_back == 0 and m_back == MODE_AT_END)
            if t_back != 0 or m_back != MODE_AT_END:
                print(f"WARNING: the gripper is NOT in torque 0 / mode {MODE_AT_END}: run scripts/28 or DYNAMIXEL Wizard")
        close_all()
    print(f"Error: {error}" if error else f"Stopped: {stop_text}")

    # 5. Log and summary
    out = ROOT / "data" / "policy"
    out.mkdir(parents=True, exist_ok=True)
    stem = f"run_sim_{datetime.now():%Y%m%d_%H%M%S}"
    with open(out / f"{stem}.csv", "w", newline="") as f:
        csv.writer(f).writerows([RUN_HEADER] + runner.rows)
    (out / f"{stem}.json").write_text(json.dumps({"checkpoint": str(ckpt), "task": task,
                                                  "policy_config": str(args.config), "run": pc_raw["run"],
                                                  "image_sizes": sizes, "cameras": cam_info,
                                                  "gripper": grip_info, "stop": stop_text,
                                                  "error": None if error is None else str(error)},
                                                 indent=1, default=int))
    ix = {h: i for i, h in enumerate(RUN_HEADER)}
    col = lambda k: np.array([float(x[ix[k]]) for x in runner.rows if x[ix[k]] != ""])   # noqa: E731
    if runner.rows:
        run = col("running").astype(bool)
        age, inf = col("chunk_age_ms"), col("infer_ms")
        work, plan_ms = col("work_ms"), col("plan_ms")
        print(f"work p50 / p99 / max {np.percentile(work, 50):.1f} / {np.percentile(work, 99):.1f} / {work.max():.1f} ms; "
              f"plan while running p99 / max {np.percentile(plan_ms[run], 99) if run.any() else 0:.1f} / "
              f"{plan_ms.max():.1f} ms")
        print(f"{len(runner.rows)} cycles ({run.sum() * s.servo.period_s:.1f} s running), late {int(col('late').sum())}, "
              f"interval max {col('cycle_interval_ms').max():.1f} ms; chunk age p50 / max {np.percentile(age, 50):.0f} / "
              f"{age.max():.0f} ms; inference p50 / p95 / max {np.percentile(inf, 50):.0f} / {np.percentile(inf, 95):.0f} / "
              f"{inf.max():.0f} ms")
        if grip_info is not None:
            grip, act = col("grip_pos"), col("grip_action")
            print(f"gripper position {grip.min():.2f} .. {grip.max():.2f} (action {act.min():.2f} .. {act.max():.2f})")
    print(f"saved {(out / stem).relative_to(ROOT)}.csv / .json")


if __name__ == "__main__":
    main()

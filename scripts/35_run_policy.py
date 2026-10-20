"""rec-6b: run a trained ACT policy on the RB5 in SIMULATION (teleop/policy.py). Real is refused for now.

MOVES THE RB5 (Simulation mode). The gripper is not moved (its action is logged; rec-6c adds it).

Run in the `lerobot` conda env:
    conda activate lerobot
    python scripts/35_run_policy.py data/train/<run>/checkpoints/last/pretrained_model [--time-limit 60]

Steps:
  1. settings (configs/sim.yaml); the cameras the policy expects (its config.json), opened in the policy
     process with the recording's fixed exposure / gain / white balance (read back) and the start
     brightness check (typed yes when outside record.brightness_ref +/- tolerance)
  2. Simulation mode and the teleop's checks; move_j to the ready pose after a typed yes
  3. the first chunks: the first action's distance from the current TCP is shown; typed yes
  4. the 50 Hz loop (PolicyRunner): keys in this terminal: g = run, space = hold, q = quit. Holds on a chunk
     older than 0.5 s, an action over 100 mm from the target, the time limit; stops when no chunk came
     for 3 s. Every target goes through the teleop's limits (TargetFollower), speed caps 100 %.
  5. normal stop (task_stop on errors); data/policy/run_<time>.csv (a row per cycle) and the summary:
     late cycles, chunk age, inference time, holds
"""
import argparse
import csv
import gc
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop import preflight  # noqa: E402
from teleop.cameras import brightness_problems  # noqa: E402
from teleop.episodes import KeyReader  # noqa: E402
from teleop.follow import TargetFollower  # noqa: E402
from teleop.kinematics import RB5Model  # noqa: E402
from teleop.policy import RUN_HEADER, PolicyClient, PolicyRunner, StopRequest, parse_policy_keys, policy_inputs  # noqa: E402
from teleop.safety import Watch  # noqa: E402
from teleop.settings import SettingsError, load_settings  # noqa: E402


def to_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def ask(prompt):
    return input(prompt).strip() == "yes"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", help="…/checkpoints/<step>/pretrained_model")
    ap.add_argument("--config", default=str(ROOT / "configs" / "sim.yaml"))
    ap.add_argument("--task", help="task text (default record.task)")
    ap.add_argument("--time-limit", type=float, default=60.0, help="seconds of running, then hold")
    ap.add_argument("--real", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.real:
        raise SystemExit("Real is not allowed yet (rec-6b is Simulation only)")
    np.set_printoptions(precision=1, suppress=True)
    try:
        s, _ = load_settings(args.config, real=False)
    except SettingsError as e:
        raise SystemExit(f"settings: {e}")
    task = args.task or s.record.task
    ckpt = Path(args.checkpoint)

    # 1. Policy process with the cameras the policy expects
    names, state_len = policy_inputs(ckpt)
    r = s.record
    fixed = {"front": (r.front_serial, {"exposure_us": r.front_exposure_us, "gain": r.front_gain,
                                         "white_balance_k": r.front_white_balance_k}),
             "wrist": (r.wrist_serial, {"exposure_us": r.wrist_exposure_us, "gain": r.wrist_gain,
                                         "white_balance_k": r.wrist_white_balance_k})}
    unknown = [n for n in names if n not in fixed]
    if unknown or state_len != 7:
        raise SystemExit(f"NOT STARTING - the policy expects cameras {names} / state {state_len} (known: {list(fixed)}, 7)")
    cams = [(n, fixed[n][0], fixed[n][1]) for n in names]
    print(f"Policy {ckpt} (cameras {names}), task '{task}'. Loading (model on the GPU, cameras) ...")
    client = PolicyClient({"kind": "act", "checkpoint": str(ckpt), "device": "cuda", "task": task}, cams)
    problems = client.open(timeout=120.0)
    if problems:
        client.close()
        raise SystemExit("NOT STARTING - policy process:\n  " + "\n  ".join(problems))
    cam_info = client.info["cameras"]
    for n, c in cam_info.items():
        st = c["settings"]
        print(f"  {n} {c['serial']}: exposure {st['exposure_us'] / 1000:.1f} ms, gain {st['gain']:g}, "
              f"white balance {st['white_balance_k']:g} K, brightness {c['brightness']}")
    problems = brightness_problems(cam_info, r.brightness_ref, r.brightness_tolerance)
    for p in problems:
        print(f"  WARNING: {p}")
    if problems and not ask("Brightness differs from the recordings. Type yes to run anyway: "):
        client.close()
        raise SystemExit("Cancelled.")

    # 2. Robot (Simulation) and the ready pose
    from teleop.rb5 import RB5
    rb5 = RB5(s.robot.ip, s.servo)
    rb5.set_mode(False, s.robot.speed_bar)
    reached, waited = rb5.wait_for_mode(False)
    problems = preflight.check_robot_mode(False, rb5.reports_simulation(), rb5.is_moving(), rb5.error_text())
    if not reached:
        problems.append("Simulation mode not reached within 3 s")
    if problems:
        client.close()
        raise SystemExit("NOT STARTING - robot:\n  " + "\n  ".join(problems))
    offset, _ = rb5.tcp_offset()
    model, watch = RB5Model(offset), Watch(s.safety)
    q_now = np.array(rb5.sdata().jnt_ref, dtype=float)
    plan = preflight.plan_ready_move(q_now, s.ready_pose.joints_deg, s.safety.joint_limit_deg, s.ready_pose,
                                     s.robot.speed_bar)
    if plan["problems"]:
        client.close()
        raise SystemExit("NOT STARTING - ready pose:\n  " + "\n  ".join(plan["problems"]))
    if not plan["skip"]:
        print(f"\nMove to the ready pose with move_j (Simulation, about {plan['expected_s']:.0f} s):")
        print(f"  current {q_now}\n  target  {plan['target']}\n  change  {plan['change']}")
        if not ask("Type yes to move: "):
            client.close()
            raise SystemExit("Cancelled.")
        finished, took = rb5.move_j(plan["target"], plan["speed"], plan["acc"], plan["timeout_s"])
        q_after = np.array(rb5.sdata().jnt_ref, dtype=float)
        if not finished or rb5.error_text() or np.abs(q_after - plan["target"]).max() > s.ready_pose.tolerance_deg:
            client.close()
            raise SystemExit(f"ready pose move failed: finished {finished}, errors {rb5.error_text() or 'none'}")
    reader = rb5.state_reader()
    reader.start()
    reader.wait_first()
    tcp0, q0, _ = reader.latest()
    problems, _ = preflight.check_start_pose(tcp0, q0, model, watch, s)
    if problems:
        reader.stop()
        client.close()
        raise SystemExit("NOT STARTING - start pose:\n  " + "\n  ".join(problems))

    # 3. First chunks (the policy sees the robot at the ready pose)
    client.publish(time.monotonic(), list(q0) + [0.0])
    first, t_end = [], time.monotonic() + 5.0
    while not first and time.monotonic() < t_end:
        first = client.poll()
        time.sleep(0.05)
    if not first:
        reader.stop()
        client.close()
        raise SystemExit("no chunk from the policy within 5 s")
    a0 = first[-1][1][0]
    print(f"\nStart TCP {tcp0[:3]} mm. The policy's first action {a0[:3]} mm, gripper {a0[3]:.2f}: "
          f"{np.linalg.norm(a0[:3] - tcp0[:3]):.0f} mm away (over 100 mm holds). Inference {first[-1][2]:.0f} ms.")
    print(f"Keys here: g = run, space = hold, q = quit. Speed caps 100 % of the teleop; time limit {args.time_limit:.0f} s.")
    if not ask("Type yes to start the loop (Simulation; the policy moves only after g): "):
        reader.stop()
        client.close()
        raise SystemExit("Cancelled.")

    # 4. Loop
    keys = KeyReader(parse=parse_policy_keys)
    follower = TargetFollower(s, model, tcp0)
    runner = PolicyRunner(s, rb5, reader, follower, client, keys, real=False, time_limit_s=args.time_limit,
                          event=lambda m: print(m, flush=True))
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
        client.close()
    print(f"Error: {error}" if error else f"Stopped: {stop_text}")

    # 5. Log and summary
    out = ROOT / "data" / "policy"
    out.mkdir(parents=True, exist_ok=True)
    stem = f"run_sim_{datetime.now():%Y%m%d_%H%M%S}"
    with open(out / f"{stem}.csv", "w", newline="") as f:
        csv.writer(f).writerows([RUN_HEADER] + runner.rows)
    (out / f"{stem}.json").write_text(json.dumps({"checkpoint": str(ckpt), "task": task, "cameras": cam_info,
                                                  "stop": stop_text, "error": None if error is None else str(error)},
                                                 indent=1))
    ix = {h: i for i, h in enumerate(RUN_HEADER)}
    col = lambda k: np.array([float(x[ix[k]]) for x in runner.rows if x[ix[k]] != ""])   # noqa: E731
    if runner.rows:
        run = col("running").astype(bool)
        age, inf = col("chunk_age_ms"), col("infer_ms")
        print(f"{len(runner.rows)} cycles ({run.sum() * s.servo.period_s:.1f} s running), late {int(col('late').sum())}, "
              f"interval max {col('cycle_interval_ms').max():.1f} ms; chunk age p50 / max {np.percentile(age, 50):.0f} / "
              f"{age.max():.0f} ms; inference p50 / p95 / max {np.percentile(inf, 50):.0f} / {np.percentile(inf, 95):.0f} / "
              f"{inf.max():.0f} ms")
    print(f"saved {(out / stem).relative_to(ROOT)}.csv / .json")


if __name__ == "__main__":
    main()

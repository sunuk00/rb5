"""rec-4b: replay one dataset episode's actions on the RB5 in SIMULATION and compare with the recording.

MOVES THE RB5 (Simulation mode only; Real is refused). No leader arm, no gripper (the gripper action is
only compared, not sent).

Run in the `lerobot` conda env (needs rbpodo there, rec-4b):
    conda activate lerobot
    python scripts/32_replay_episode.py --repo-id local/sim_test --episode 0

Steps:
  1. load the episode's actions (30 Hz: x, y, z [mm], gripper) and states (joints [deg], gripper)
  2. Simulation mode; the same checks as the teleop (mode, not moving, no errors)
  3. move_j to the episode's first joints after a typed yes (current / target / change shown)
  4. stream at 50 Hz: the desired position = the action interpolated linearly at the replay time (the
     last one held for --hold-s at the end), through teleop.follow.TargetFollower = the teleop's
     per-cycle limits (range, speeds with the watches, joint speed, acceleration, lead), move_servo_l
  5. normal stop (or task_stop on errors); compare: replay joints (jnt_ref) vs the recorded joints at
     the same episode time, target vs desired (what the limits changed), limit flags; CSV in data/replay/

The comparison makes sense for a Simulation recording (its recorded joints are jnt_ref too). For a Real
recording the recorded joints are encoders (jnt_ang), which lag jnt_ref.
"""
import argparse
import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop import preflight  # noqa: E402
from teleop.follow import TargetFollower  # noqa: E402
from teleop.kinematics import RB5Model  # noqa: E402
from teleop.safety import Watch  # noqa: E402
from teleop.settings import SettingsError, load_settings  # noqa: E402

HEADER = (["t_s", "late", "state_age_ms"] + [f"desired_{a}" for a in "xyz"] + [f"target_{a}" for a in "xyz"]
          + [f"tcp_{a}" for a in "xyz"] + ["yaw_deg"] + [f"q{i}" for i in range(6)]
          + [f"rec_q{i}" for i in range(6)] + ["grip_action", "flags"])


def load_episode(repo_id, root, episode):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(repo_id, root=root)
    h = ds.hf_dataset.select_columns(["episode_index", "action", "observation.state"])
    e = np.array(h["episode_index"])
    if episode not in set(e.tolist()):
        raise SystemExit(f"episode {episode} not in {repo_id} (episodes {sorted(set(e.tolist()))})")
    sel = np.where(e == episode)[0]
    actions = np.stack([np.asarray(h[int(i)]["action"]) for i in sel]).astype(float)
    states = np.stack([np.asarray(h[int(i)]["observation.state"]) for i in sel]).astype(float)
    return actions, states, ds.fps


def at(t, fps, x):
    """x (N, k) sampled at k / fps, linearly interpolated at time t (held after the end)."""
    g = np.arange(len(x)) / fps
    return np.array([np.interp(t, g, x[:, k]) for k in range(x.shape[1])])


def to_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo-id", default="local/sim_test")
    ap.add_argument("--root", help="dataset folder (default data/lerobot/<name of the repo id>)")
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--config", default=str(ROOT / "configs" / "sim.yaml"))
    ap.add_argument("--hold-s", type=float, default=1.0, help="keep sending the last action this long")
    ap.add_argument("--real", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.real:
        raise SystemExit("Real is not allowed for the replay (Simulation only, rec-4b)")
    np.set_printoptions(precision=2, suppress=True)
    root = Path(args.root) if args.root else ROOT / "data" / "lerobot" / args.repo_id.split("/")[-1]

    try:
        s, _ = load_settings(args.config, real=False)
    except SettingsError as e:
        raise SystemExit(f"settings: {e}")
    actions, states, fps = load_episode(args.repo_id, root, args.episode)
    duration = (len(actions) - 1) / fps
    print(f"Episode {args.episode} of {args.repo_id}: {len(actions)} steps at {fps} fps ({duration:.1f} s)")

    from teleop.rb5 import RB5
    rb5 = RB5(s.robot.ip, s.servo)
    rb5.set_mode(False, s.robot.speed_bar)
    reached, waited = rb5.wait_for_mode(False)
    problems = preflight.check_robot_mode(False, rb5.reports_simulation(), rb5.is_moving(), rb5.error_text())
    if not reached:
        problems.append("Simulation mode not reached within 3 s")
    if problems:
        raise SystemExit("NOT STARTING - robot:\n  " + "\n  ".join(problems))
    print(f"Controller: Simulation (reached after {waited:.1f} s)")
    offset, _ = rb5.tcp_offset()
    model, watch = RB5Model(offset), Watch(s.safety)

    # Move to the episode's first joints
    q_now = np.array(rb5.sdata().jnt_ref, dtype=float)
    plan = preflight.plan_ready_move(q_now, states[0, :6], s.safety.joint_limit_deg, s.ready_pose, s.robot.speed_bar)
    if plan["problems"]:
        raise SystemExit("NOT STARTING - start pose:\n  " + "\n  ".join(plan["problems"]))
    if not plan["skip"]:
        print(f"\nMove to the episode's first joints with move_j (Simulation, about {plan['expected_s']:.0f} s):")
        print(f"  current {q_now}")
        print(f"  target  {plan['target']}")
        print(f"  change  {plan['change']}")
        if input("Type yes to move: ").strip() != "yes":
            raise SystemExit("Cancelled.")
        finished, took = rb5.move_j(plan["target"], plan["speed"], plan["acc"], plan["timeout_s"])
        q_after = np.array(rb5.sdata().jnt_ref, dtype=float)
        if not finished or rb5.error_text() or np.abs(q_after - plan["target"]).max() > s.ready_pose.tolerance_deg:
            raise SystemExit(f"start move failed: finished {finished}, errors {rb5.error_text() or 'none'}, joints {q_after}")
        print(f"Start pose reached in {took:.1f} s.")

    reader = rb5.state_reader()
    reader.start()
    reader.wait_first()
    tcp0, q0, _ = reader.latest()
    problems, fk_error = preflight.check_start_pose(tcp0, q0, model, watch, s)
    if problems:
        reader.stop()
        raise SystemExit("NOT STARTING - start pose checks:\n  " + "\n  ".join(problems))
    print(f"Start TCP {tcp0[:3]} mm; first action {actions[0, :3]} mm (differs by "
          f"{np.linalg.norm(actions[0, :3] - tcp0[:3]):.1f} mm: the action leads the state)")
    if input(f"Type yes to replay {duration:.1f} s (Simulation): ").strip() != "yes":
        reader.stop()
        raise SystemExit("Cancelled.")

    follower = TargetFollower(s, model, tcp0)
    sf, period = s.safety, s.servo.period_s
    rows, flag_counts = [], {}
    error = stop_text = None
    late_run = stale_run = 0
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, to_keyboard_interrupt)
    rb5.begin_streaming()
    t_start = next_t = time.monotonic()
    last_send = None
    try:
        while True:
            t = time.monotonic() - t_start
            if t > duration + args.hold_s:
                break
            rb5.check_errors()
            tcp, q, age = reader.latest()
            stale = age > sf.max_state_age_s
            stale_run = stale_run + 1 if stale else 0
            if stale_run >= sf.max_stale_cycles:
                raise RuntimeError(f"{sf.max_stale_cycles} stale-state cycles in a row")
            a = at(t, fps, actions)
            gap = None if last_send is None else time.monotonic() - last_send
            late = gap is not None and gap > sf.max_gap_s
            late_run = late_run + 1 if late else 0
            if late_run >= sf.max_late_cycles:
                raise RuntimeError(f"{sf.max_late_cycles} late cycles in a row")
            flags = {}
            if late or stale:
                follower.hold()
            else:
                p = follower.step(a[:3], tcp, q)
                if p.stop:
                    stop_text = p.stop
                    break
                flags = {k: v for k, v in p.flags.items() if v}
                for k in flags:
                    flag_counts[k] = flag_counts.get(k, 0) + 1
            send_t = time.monotonic()
            rb5.send_servo(follower.target, follower.rpy)
            last_send = send_t
            rec_q = at(t - age, fps, states[:, :6])   # recorded joints at the time this state was read
            rows.append([f"{t:.4f}", int(late), f"{age * 1000:.2f}"] + [f"{v:.3f}" for v in a[:3]]
                        + [f"{v:.3f}" for v in follower.target] + [f"{v:.3f}" for v in tcp[:3]] + [f"{follower.yaw:.3f}"]
                        + [f"{v:.3f}" for v in q] + [f"{v:.3f}" for v in rec_q] + [f"{a[3]:.3f}", " ".join(flags)])
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic() + period
                time.sleep(period)
    except KeyboardInterrupt:
        stop_text = "interrupted"
    except Exception as e:
        error = e
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if error is None:
            rb5.end_streaming(follower.target, follower.rpy, sf.hold_on_exit_s, period)
        else:
            rb5.emergency_stop()
        reader.stop()
    if error is not None:
        print(f"Error: {error}")
    if stop_text:
        print(f"Stopped: {stop_text}")

    out = ROOT / "data" / "replay"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"replay_{datetime.now():%Y%m%d_%H%M%S}_{args.repo_id.split('/')[-1]}_ep{args.episode}.csv"
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows([HEADER] + rows)
    if not rows:
        return
    d = np.array([[float(x) for x in r[3:6]] for r in rows])
    tg = np.array([[float(x) for x in r[6:9]] for r in rows])
    tcp = np.array([[float(x) for x in r[9:12]] for r in rows])
    q = np.array([[float(x) for x in r[13:19]] for r in rows])
    rq = np.array([[float(x) for x in r[19:25]] for r in rows])
    t = np.array([float(r[0]) for r in rows])
    inside = t <= duration
    jerr = np.abs(q - rq)[inside]
    print(f"\n{len(rows)} cycles, late {sum(int(r[1]) for r in rows)}")
    print("replay joints vs recorded joints (same episode time) [deg]:")
    print(f"  p50 {np.percentile(jerr, 50, axis=0)}")
    print(f"  p95 {np.percentile(jerr, 95, axis=0)}")
    print(f"  max {jerr.max(axis=0)}")
    td = np.linalg.norm(tg - d, axis=1)[inside]
    print(f"target vs desired (what the limits changed) p50 / p95 / max {np.percentile(td, 50):.2f} / "
          f"{np.percentile(td, 95):.2f} / {td.max():.2f} mm")
    print(f"tcp_ref vs target p95 {np.percentile(np.linalg.norm(tcp - tg, axis=1), 95):.1f} mm (the controller's lag)")
    print(f"end: tcp {tcp[-1]} vs last action {actions[-1, :3]} -> {np.linalg.norm(tcp[-1] - actions[-1, :3]):.2f} mm")
    print(f"limit flags (cycles): {flag_counts or 'none'}")
    print(f"saved {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

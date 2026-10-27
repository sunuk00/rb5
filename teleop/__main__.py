"""Leader arm -> RB5 teleoperation (position, yaw, gripper). Simulation by default.

Steps:
  1. load configs/sim.yaml (or configs/real.yaml with --real; Real asks to type REAL)
  2. checks before moving: leader latency_timer and ping, gripper (read only: latency_timer, model,
     hardware error, voltage, position inside the open / closed range), RB5 mode matches, not moving, no
     controller errors, TCP offset (Real: equal to the expected value)
  3. move to the ready pose with move_j: first checks that the ready pose is a valid start (step 4
     checks, from the controller FK), then shows the change per joint and asks for "yes"; skipped when
     already there. Real: slower, "clear the area" note, warning for large wrist turns
  4. checks at the ready pose: DH model vs tcp_ref, TCP inside the range, singularity / joint limit margins
  5. gripper writes after a typed yes (operating mode 5, profile, goal current, goal = present position,
     torque on), the gripper thread started; it moves in Simulation too (--no-gripper: without it)
  6. teleop loop until Ctrl+C, SIGTERM / SIGHUP (or a stop condition); normal stop or task_stop on errors;
     the gripper: torque off and operating mode 3 (a held object is dropped: put it down first)
  7. CSV, the gripper thread CSV and a copy of the settings saved in the log folder

With --record (demonstrations, rec-3b): the cameras (record.cameras: front, top, grip) start before
anything moves with their fixed exposure / gain / white balance read back (refused if one is missing or a
value differs, or if a start image is saturated or has missing rows: cameras.refusal_problems, no typed
yes); a mean
brightness outside record.brightness_ref +/- brightness_tolerance asks for a typed yes (cam-2b), and the run
is a session in record.dir/<mode>_<time>/.
Keys in this terminal during the loop, as LeRobot's: -> / n end and keep the episode, or start the next
one; <- / r discard the episode (or, in the reset, the one just finished); Esc / q end the session (an
episode being recorded is discarded). The session starts in the reset phase. Episodes end and are kept
after record.episode_time_s. Ctrl+C stops as always (an episode being recorded is discarded).

Usage:
    python -m teleop                       # Simulation, configs/sim.yaml
    python -m teleop --config my_sim.yaml  # another Simulation settings file
    python -m teleop --real                # REAL robot, configs/real.yaml (asks to type REAL)
    python -m teleop --no-gripper          # without the gripper
    python -m teleop --record              # demonstrations (Simulation); --real --record on the robot
    python -m teleop --record --task "put the cup on the plate"
    python -m teleop --record --no-cameras # robot data only (trying the keys)
Run from the repository root.
The process pins itself to the CPU cores in settings runtime.cpu_cores (no taskset needed).
"""
import argparse
import os
import signal
import sys
from pathlib import Path

import numpy as np

from . import preflight
from .gripper import (MODE_AT_END, OPERATING_MODE, PRESENT_POSITION, TORQUE_ENABLE, GripperWorker, open_gripper,
                      start_lines, start_worker)
from .kinematics import RB5Model
from .leader import LeaderArm, LeaderModel, read_latency_timer
from .log import run_paths, save_csv, save_run
from .loop import CSV_HEADER, StopRequest, TeleopLoop, print_event
from .log import session_dir
from .safety import Watch
from .settings import SettingsError, load_settings

ROOT = Path(__file__).resolve().parent.parent   # repository root

URDF_PATH = ROOT / "assets" / "leader_arm" / "model.urdf"


def refuse(title, problems):
    print(f"\nNOT STARTING - {title}:")
    for p in problems:
        print(f"  - {p}")
    sys.exit(1)


def ask(prompt, answer):
    return input(prompt).strip() == answer


def to_keyboard_interrupt(signum, frame):
    """SIGTERM / SIGHUP (terminal closed, kill) end the run like Ctrl+C, so the cleanup runs."""
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(prog="python -m teleop", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--real", action="store_true", help="REAL robot (uses configs/real.yaml unless --config)")
    parser.add_argument("--config", help="settings file")
    parser.add_argument("--no-gripper", action="store_true", help="run without the gripper (gripper.enabled in the settings)")
    parser.add_argument("--record", action="store_true", help="record demonstration episodes (keys in this terminal)")
    parser.add_argument("--task", help="task text of the episodes (default: record.task in the settings)")
    parser.add_argument("--no-cameras", action="store_true", help="with --record: robot data only, no cameras")
    args = parser.parse_args()
    if (args.task or args.no_cameras) and not args.record:
        parser.error("--task and --no-cameras need --record")
    np.set_printoptions(precision=2, suppress=True)
    real = args.real
    path = Path(args.config) if args.config else ROOT / "configs" / ("real.yaml" if real else "sim.yaml")

    # 1. Settings
    try:
        s, raw = load_settings(path, real=real)
    except SettingsError as e:
        refuse(f"settings {path}", e.problems)
    mode = "REAL" if real else "Simulation"
    print(f"Mode: {mode}   settings: {path}")
    if s.runtime.cpu_cores is not None:
        first, last = int(s.runtime.cpu_cores[0]), int(s.runtime.cpu_cores[1])
        cores = set(range(first, last + 1)) & os.sched_getaffinity(0)
        if cores:
            os.sched_setaffinity(0, cores)   # threads started later (state reader) inherit it
        print(f"CPU cores: {sorted(os.sched_getaffinity(0))}")
    task = args.task if args.task is not None else s.record.task
    if args.record and not task.strip():
        refuse("record", ["--task is empty"])
    if args.record and not os.isatty(sys.stdin.fileno()):
        refuse("record", ["stdin is not a terminal: the episode keys could not be read"])
    if real:
        print("REAL mode: the RB5 will move. Keep the emergency stop within reach.")
        if not ask("Type REAL to continue: ", "REAL"):
            sys.exit("Cancelled.")

    # 2. Checks before moving
    latency = read_latency_timer(s.leader.port)
    leader = LeaderArm(s.leader.port)
    open_problems = leader.open(left_tool=s.mapping.left_button != "off" or s.gripper.trigger == "left")
    problems = preflight.check_leader(latency, s.leader.required_latency_timer_ms, open_problems)
    if problems:
        if not open_problems:
            leader.close()
        refuse("leader arm", problems)
    import dynamixel_sdk.port_handler as dxl_port_handler
    dxl_port_handler.LATENCY_TIMER = s.leader.required_latency_timer_ms   # reply timeout follows this (12)
    leader_model = LeaderModel(URDF_PATH)

    # Gripper: read only here; the writes come right before the loop, after a typed yes
    motor = None

    cameras = None

    def close_ports():
        leader.close()
        if motor is not None:
            motor.close()
        if cameras is not None:
            cameras.close()

    if s.gripper.enabled and not args.no_gripper:
        motor, problems = open_gripper(s.gripper, s.leader.required_latency_timer_ms)
        if problems:
            leader.close()
            refuse("gripper (python -m teleop --no-gripper runs without it)", problems)
        st = motor.state()
        torque, op_mode = motor.read(TORQUE_ENABLE), motor.read(OPERATING_MODE)
        print(f"Gripper: MX-106(2.0) ID {motor.id}, position {st[2]} (closed {s.gripper.closed_ticks}, open "
              f"{s.gripper.open_ticks}), {st[3]:.1f} V, {st[4]} C, mode {op_mode}, torque {torque}")
        if torque != 0 or op_mode != MODE_AT_END:
            print(f"  left by an earlier run that did not finish (expected torque 0, mode {MODE_AT_END}); the start writes "
                  "turn the torque off first")
    else:
        print("Gripper: off" + (" (--no-gripper)" if args.no_gripper else " (gripper.enabled is false)"))

    # Cameras (--record): start before anything moves; refuse when one is missing
    session = None
    if args.record:
        session = session_dir(s.record.dir, "real" if real else "sim", ROOT)
        raw["record"]["task"] = task
        if args.no_cameras:
            print(f"Record: session {session.relative_to(ROOT)}, NO cameras (--no-cameras), task '{task}'")
        else:
            from .cameras import CameraRecorder, brightness_problems, camera_defs, print_red, refusal_problems
            r = s.record
            cameras = CameraRecorder(camera_defs(r), quality=r.jpeg_quality)
            problems = cameras.open()
            if problems:
                close_ports()
                refuse("cameras (python -m teleop --record --no-cameras records without them)", problems)
            print(f"Record: session {session.relative_to(ROOT)}, task '{task}'")
            for name, c in cameras.info.items():
                st = c["settings"]
                print(f"  {name} {c['serial']} ({c['format']}): exposure {st['exposure_us'] / 1000:.1f} ms, gain "
                      f"{st['gain']:g}, white balance {st['white_balance_k']:g} K (fixed), mean brightness {c['brightness']}")
            problems = refusal_problems(cameras.info)
            if problems:
                print_red(["NOT STARTING - camera image saturated or incomplete (no typed yes for this):"]
                          + [f"  {p}" for p in problems])
                close_ports()
                sys.exit(1)
            problems = brightness_problems(cameras.info, r.brightness_ref, r.brightness_tolerance)
            for p in problems:
                print(f"  WARNING: {p}")
            if problems:
                ok = ask("Brightness differs from the reference: the images will look different from the other "
                         "sessions. Type yes to record anyway: ", "yes")
                for name in cameras.info:
                    cameras.info[name]["brightness_confirmed"] = ok if not cameras.info[name]["brightness_ok"] else None
                if not ok:
                    close_ports()
                    sys.exit("Cancelled: fix the lighting (docs/data_rules.md) or run scripts/34 again.")

    from .rb5 import RB5
    rb5 = RB5(s.robot.ip, s.servo)
    rb5.set_mode(real, s.robot.speed_bar)
    reached, waited = rb5.wait_for_mode(real)
    print(f"Controller mode after the switch: {'Real' if not rb5.reports_simulation() else 'Simulation'} "
          f"({'reached' if reached else 'NOT reached'} after {waited:.1f} s)")
    problems = preflight.check_robot_mode(real, rb5.reports_simulation(), rb5.is_moving(), rb5.error_text())
    if not reached:
        problems.append("the mode did not change within 3 s after set_operation_mode; check the tablet "
                        "(is Real mode allowed / is another program setting the mode?)")
    offset, rot_offset = rb5.tcp_offset()
    problems += preflight.check_tcp_offset(offset, rot_offset, s.tcp)
    if problems:
        close_ports()
        refuse("robot", problems)
    print(f"TCP offset (flange frame) {offset} mm")
    model = RB5Model(offset)
    watch = Watch(s.safety)

    # 3. Ready pose
    q_now = np.array(rb5.sdata().jnt_ref, dtype=float)
    plan = preflight.plan_ready_move(q_now, s.ready_pose.joints_deg, s.safety.joint_limit_deg, s.ready_pose,
                                     s.robot.speed_bar)
    if plan["problems"]:
        close_ports()
        refuse("ready pose", plan["problems"])
    if not plan["skip"]:
        # Check the ready pose before moving there, so a pose that would be refused at start is not reached
        tcp_ready = rb5.fk_tcp(plan["target"])
        if tcp_ready is None:
            close_ports()
            refuse("ready pose", ["calc_fk_tcp of the ready pose failed"])
        ready_problems, _ = preflight.check_start_pose(tcp_ready, plan["target"], model, watch, s)
        if ready_problems:
            close_ports()
            refuse("the ready pose would not be a valid start (nothing moved)", ready_problems)
    if plan["skip"]:
        print("Already at the ready pose.")
    else:
        print(f"\nMove to the ready pose with move_j ({plan['speed']} deg/s x speed bar {s.robot.speed_bar}, "
              f"about {plan['expected_s']:.0f} s):")
        print(f"  current {q_now}")
        print(f"  target  {plan['target']}")
        print(f"  change  {plan['change']}")
        print(f"  target TCP (calc_fk_tcp) {tcp_ready}")
        if real:
            print("REAL: clear the area around the robot.")
            for w in plan["wrist_warnings"]:
                print(f"  WARNING: large wrist turn: {w}")
            if plan["wrist_warnings"] and not ask("Wrist turns are large. Type yes to accept: ", "yes"):
                close_ports()
                sys.exit("Cancelled.")
        if not ask("Type yes to move: ", "yes"):
            close_ports()
            sys.exit("Cancelled.")
        finished, took = rb5.move_j(plan["target"], plan["speed"], plan["acc"], plan["timeout_s"])
        err = rb5.error_text()
        q_after = np.array(rb5.sdata().jnt_ref, dtype=float)
        if not finished or err or np.abs(q_after - plan["target"]).max() > s.ready_pose.tolerance_deg:
            close_ports()
            refuse("ready pose move", [f"finished {finished} after {took:.1f} s", f"errors: {err or 'none'}",
                                       f"joints {q_after} (target {plan['target']})"])
        print(f"Ready pose reached in {took:.1f} s.")

    # 4. Checks at the ready pose
    reader = rb5.state_reader()
    reader.start()
    reader.wait_first()
    tcp0, q0, _ = reader.latest()
    problems, fk_error = preflight.check_start_pose(tcp0, q0, model, watch, s)
    if problems:
        reader.stop()
        close_ports()
        refuse("start pose", problems)
    print(f"Start TCP {tcp0}   DH model vs tcp_ref {fk_error:.2f} mm")

    # 5. Gripper writes and thread
    worker = None
    if motor is not None:
        g = s.gripper
        pos = motor.read(PRESENT_POSITION)
        print("\nGripper writes (moves in Simulation too):")
        for line in start_lines(g, pos):
            print(line)
        if not ask("Type yes to start the gripper: ", "yes"):
            reader.stop()
            close_ports()
            sys.exit("Cancelled.")
        worker, command, held, problems = start_worker(motor, g, pos, s.servo.period_s, g.catch_frac)
        if problems:
            reader.stop()
            close_ports()
            refuse("gripper start", problems)
        print(f"Gripper: holding at {held}; move the {g.trigger} trigger to about "
              f"{command.frac_of(held) * g.trigger_full:.0f} to start following.")
    grip_text = (f"{s.gripper.trigger} trigger = gripper (released = closed, pulled = open)" if worker is not None
                 else "gripper off")
    print(f"Scale {s.mapping.scale}, {s.mapping.speed_mm_s} mm/s, "
          f"{'B' if s.mapping.max_lag_mm <= 0 else f'C {s.mapping.max_lag_mm:.0f} mm'}, "
          f"yaw {'on' if s.yaw.enabled else 'off'}. Right handle: button = clutch, twist = yaw; "
          f"{grip_text}. Ctrl+C to stop.\n")

    # 6. Teleop loop
    recorder = keys = None
    if args.record:
        from .episodes import EpisodeRecorder, KeyReader, Speaker
        keys = KeyReader()
        speaker = Speaker(s.record.sounds)
        if s.record.sounds and not speaker.enabled:
            print("Sounds: spd-say not found (sudo apt install speech-dispatcher); no spoken messages")
        recorder = EpisodeRecorder(session, task, s.record.episode_time_s, s.record.reset_time_s, keys, cameras,
                                   say=speaker.say, camera_info=None if cameras is None else cameras.info)
        print(f"RECORD: starts in the RESET phase. Keys here: -> or n = start / end and keep the episode, "
              f"<- or r = discard, q or Esc = end the session. Episodes end and are kept after "
              f"{s.record.episode_time_s:.0f} s.\n")
    loop = TeleopLoop(s, leader, leader_model, rb5, reader, model, gripper=worker, recorder=recorder)
    error = None
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, to_keyboard_interrupt)
    rb5.begin_streaming()
    if keys is not None:
        keys.start()   # after the last input(): the terminal is in cbreak mode until keys.stop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print_event("Interrupted.")
    except StopRequest as e:
        print_event(f"Stopping: {e}")
    except Exception as e:   # controller error, late loop, stale state: stop without more servo commands
        error = e
        print_event(f"Error: {e}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl+C must not skip the cleanup
        if keys is not None:
            keys.stop()   # terminal back to normal first
        if error is None and loop.target is not None:
            rb5.end_streaming(loop.target, loop.rpy, s.safety.hold_on_exit_s, s.servo.period_s)
        else:
            rb5.emergency_stop()
        reader.stop()
        if worker is not None:
            t_back, m_back = worker.finish()
            print_event(f"Gripper: {worker.stop_reason or 'running until the end'}; read back: torque {t_back}, "
                        f"operating mode {m_back}")
            if t_back != 0 or m_back != MODE_AT_END:
                print_event(f"WARNING: the gripper is NOT in torque 0 / mode {MODE_AT_END}: run scripts/28 (it turns "
                            "them off with the confirmed writes) or use DYNAMIXEL Wizard")
        if recorder is not None:
            for e in recorder.finish("teleop stopped while recording"):
                print_event(e)
            speaker.say("Stop recording")
        close_ports()

    # 7. Log
    if session is not None:
        csv_path, settings_path = session / "teleop.csv", session / "settings.yaml"
    else:
        csv_path, settings_path = run_paths(s.logging.dir, "real" if real else "sim", ROOT)
    save_run(loop.rows, CSV_HEADER, raw, csv_path, settings_path)
    if worker is not None and worker.rows:
        grip_path = csv_path.with_name("gripper.csv" if session is not None else f"{csv_path.stem}_gripper.csv")
        save_csv(grip_path, GripperWorker.ROW_HEADER, worker.csv_rows())
        print(f"Gripper thread log: {grip_path.relative_to(ROOT)} ({len(worker.rows)} cycles)")
    c = loop.clutch
    if c.reads:
        print(f"Tool reads {c.reads}, failed or unexpected {c.failures} ({100.0 * c.failures / c.reads:.1f}%)")
    print(f"Saved: {csv_path.relative_to(ROOT)} ({len(loop.rows)} cycles) and {settings_path.name}")
    if recorder is not None:
        kept = [a for a in recorder.attempts if a["status"] == "kept"]
        print(f"Episodes: {len(kept)} kept, {len(recorder.attempts) - len(kept)} discarded "
              f"(session {session.relative_to(ROOT)}, episodes.json)")
        speaker.wait()


if __name__ == "__main__":
    main()

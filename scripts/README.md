# scripts

Numbered experiment scripts, kept as they were run (the logs and `docs/decisions.md` refer to them
by number). The teleop is not here: `python -m teleop` (see `docs/teleop_guide.md`).

## Before you run anything
- Run from the repository root: `conda activate rb5`, then `python scripts/<name>.py ...`.
  Scripts 14, 15 and 20 need `conda activate rby1` instead (they use `rby1_sdk`).
- Robot IP and speed bar come from `configs/robot.yaml` through `scripts/config.py` (`import config`).
  01 and 02 have the IP written in the file.
- Every script prints its options with `--help` (all except 01, 02, 04 and 17, which have none).
- Leader arm scripts use `config.LEADER_PORT` (the adapter by serial number, `/dev/serial/by-id/...FT62AJW7...`)
  with FTDI `latency_timer` = 1 (`docs/teleop_guide.md`, udev rule).
- The leader arm URDF is `assets/leader_arm/model.urdf` (teleop, 15, 20, 21).
- Outputs go to `data/` (not committed), measurement plots to `docs/measurements/`.
- Values that are not options (poses, speeds, limits) are UPPER_CASE constants at the top of each file.

## Overview
**Moves** = what it moves: RB5 (Sim) = RB5 in Simulation mode only; RB5 (Sim/Real) = Simulation by
default, Real with `--real` and typed confirmation; **leader** = writes torque to the leader arm motors
(the leader arm moves or holds by itself); - = read only.
**Status**: tool = still useful; reference = result recorded; replaced = superseded by the teleop.

| # | Script | What it does | Moves | Status |
|---|---|---|---|---|
| 01 | `01_read_state.py` | Read the joint angles once | - | reference |
| 02 | `02_move_joint.py` | Move J6 by +2 deg with move_j | RB5 (Sim) | reference |
| 03 | `03_pose_sequence.py` | home -> pose_1 -> home with move_j | RB5 (Sim/Real) | reference |
| 04 | `04_data_rate.py` | Data channel update period | RB5 (Sim) | reference |
| 05 | `05_key_input.py` | Keyboard jog with move_j steps, 20 Hz | RB5 (Sim/Real) | replaced |
| 06 | `06_servo_j_probe.py` | How move_servo_j behaves | RB5 (Sim) | reference |
| 07 | `07_servo_jog.py` | Keyboard jog with move_servo_j, 20 Hz | RB5 (Sim/Real) | replaced |
| 08 | `08_servo_jog_fast.py` | 07 with a background state reader, selectable period | RB5 (Sim/Real) | replaced |
| 09 | `09_measure_read_rate.py` | Data channel read performance | - | reference |
| 10 | `10_stream_rate_test.py` | move_servo_j streaming rate | RB5 (Sim) | reference |
| 11 | `11_leader_arm_read_rate.py` | Leader arm read speed | - | reference |
| 12 | `12_leader_arm_live.py` | Live leader joints, button, trigger | - | tool |
| 13 | `13_leader_j6_teleop.py` | Leader J6 -> RB5 J6 with a clutch | RB5 (Sim/Real) | replaced |
| 14 | `14_leader_rby1_teleop.py` | Leader arm -> RB-Y1 simulator arms (env `rby1`) | leader, RB-Y1 sim | reference |
| 15 | `15_example35_nogripper.py` | rby1-sdk example 35 without the gripper (env `rby1`) | leader, RB-Y1 | reference |
| 16 | `16_leader_hold_diag.py` | Leader arm vibration while holding a pose | leader | tool |
| 17 | `17_tcp_read_compare.py` | Ways to read the RB5 TCP pose | - | reference |
| 18 | `18_servo_l_probe.py` | move_servo_l frame, axes, rotation convention | RB5 (Sim) | reference |
| 19 | `19_move_joint_sim.py` | move_j one joint or all six to a pose | RB5 (Sim) | tool |
| 20 | `20_leader_fk_live.py` | Live leader arm FK (env `rby1`) | - | tool |
| 21 | `21_leader_rb5_position.py` | Leader wrist center -> RB5 TCP with move_servo_l | RB5 (Sim) | replaced |
| 22 | `22_base_axis_check.py` | Where the RB5 base +x / +y point on the real robot | - | tool (Real checklist) |
| 23 | `23_trial_summary.py` | Summary of Real teleop trials from the logs (real-1) | - (log files only) | tool |
| 24 | `24_run_report.py` | Full report on teleop runs: speeds, limits per mode, drops, smoothness with the real sample times, lag, timing | - (log files only) | tool |
| 25 | `25_camera_rgb_test.py` | RGB from all RealSense cameras at once: fps, frame intervals, missed frames, camera vs PC time (cam-1) | - (cameras only) | tool |
| 26 | `26_jnt_ref_check.py` | What jnt_ref is: IK of the sent target vs jnt_ref vs jnt_ang, delays, model-free Cartesian check | - (log files only) | tool |
| 27 | `27_gripper_scan.py` | Read-only scan of the gripper bus (MX-106T on its U2D2): protocol, baud, model, firmware, settings, read time (grip-1) | - (gripper motor only) | tool |
| 28 | `28_trigger_gripper.py` | Leader trigger (left by default) -> gripper alone (no RB5): released 0 closed, pulled 1000 open, catch at start, current-based position mode with a low force cap (grip-2; code in `teleop/gripper.py`, gripper thread, since grip-3a) | gripper (writes), leader arm (read only) | test |
| 29 | `29_lerobot_dummy_dataset.py` | Dummy LeRobotDataset with the rec-1 format: write, discard, read back, ACT action chunk (env `lerobot`, rec-2) | - | test |
| 30 | `30_camera_recorder_test.py` | Camera recorder process alone (teleop/cameras.py): episodes kept / discarded, fps, gaps, JPEG write time, MB/s, CPU, and a 50 Hz loop beside it (rec-3a) | - (cameras only) | tool |
| 31 | `31_convert_to_lerobot.py` | Demonstration sessions (`--record`) -> one LeRobotDataset at 30 fps: kept episodes, still ones left out, optional pause cut, checks (env `lerobot`, rec-4a) | - (files only) | tool |
| 32 | `32_replay_episode.py` | Replay one dataset episode's actions through the teleop limits (teleop/follow.py) and compare with the recorded joints (env `lerobot`, rec-4b) | RB5 (Sim only) | test |
| 33 | `33_policy_check.py` | A trained policy offline: predicted chunks vs recorded actions, inference time GPU / CPU (env `lerobot`, rec-5) | - | tool |

## Usage per script

### 01 read state
```
python scripts/01_read_state.py
```
No options. Prints `jnt_ang` once.

### 02 move joint
```
python scripts/02_move_joint.py
```
No options; asks `yes`. Sets Simulation mode, moves J6 by `DELTA_DEG` (2.0) at `SPEED` 10 deg/s.
Edit `DELTA_DEG`, `SPEED`, `ACC` in the file.

### 03 pose sequence
```
python scripts/03_pose_sequence.py          # Simulation
python scripts/03_pose_sequence.py --real   # Real (type yes)
```
| Option | Meaning |
|---|---|
| `--real` | real robot |

In the file: `POSES` (home, pose_1 in deg), `SEQUENCE`, `HOME_TOLERANCE_DEG` (must start within 2 deg of
home), `JOINT_SPEED` 20 deg/s (x speed bar).

### 04 data rate
```
python scripts/04_data_rate.py
```
No options. Moves J6 +60 deg and back in Simulation, logs `data/04_data_rate_<time>.csv`.

### 05 / 07 / 08 keyboard jog
```
python scripts/05_key_input.py                      # keyboard only, no robot
python scripts/07_servo_jog.py --sim
python scripts/08_servo_jog_fast.py --sim --period 0.01
python scripts/08_servo_jog_fast.py --real          # type to confirm
```
| Option | Meaning |
|---|---|
| `--sim` / `--real` | connect in Simulation / Real (none = keyboard only) |
| `--device /dev/input/eventN` | keyboard device (default: the laptop keyboard by name) |
| `--period 0.05\|0.02\|0.01` | 08 only: loop period [s], default 0.02 |

Keys: Up/Down select joint, hold Left/Right to jog, type `done` to go home and exit, Ctrl+C stop.
Needs the user in the `input` group. In the file: `HOME`, `JOG_SPEED_SIM` / `JOG_SPEED_REAL` (07, 08).

### 06 servo_j probe
```
python scripts/06_servo_j_probe.py                 # t1 0.01, t2 0.1, gain 1, alpha 1, period 0.01
python scripts/06_servo_j_probe.py --t1 0.05
```
| Option | Default | Meaning |
|---|---|---|
| `--t1` | 0.01 | move_servo_j t1 [s] |
| `--t2` | 0.1 | move_servo_j t2 [s] |
| `--gain` | 1.0 | gain |
| `--alpha` | 1.0 | alpha |
| `--period` | 0.01 | send period [s] |

Simulation only. Log: `data/06_servo_<t1>_<t2>_<gain>_<alpha>_...csv`.

### 09 data channel read rate
```
python scripts/09_measure_read_rate.py
python scripts/09_measure_read_rate.py --count 5000
```
| Option | Default | Meaning |
|---|---|---|
| `--count` | 1000 | measured `request_data()` calls |

Output: `docs/measurements/read_rate_<time>.csv` / `.png`.

### 10 streaming rate
```
python scripts/10_stream_rate_test.py --hz 50
python scripts/10_stream_rate_test.py --hz 200 --t1 0.01
```
| Option | Default | Meaning |
|---|---|---|
| `--hz 20\|50\|100\|200` | required | send rate |
| `--t1` | 1 / hz | move_servo_j t1 [s] |

Simulation only; J6 sine of 5 deg for 10 s. Output: `docs/measurements/stream_<hz>hz_...csv` / `.png`.

### 11 leader arm read rate
```
python scripts/11_leader_arm_read_rate.py           # joints 0..6 + tool 128
python scripts/11_leader_arm_read_rate.py --all     # all 14 joints + both tools
```
| Option | Default | Meaning |
|---|---|---|
| `--port` | `config.LEADER_PORT` | leader arm device |
| `--all` | off | read all 16 devices |
| `--count` | 1000 | sync-read calls |
| `--seq-cycles` | 200 | one-by-one read cycles |

Output: `docs/measurements/leader_read_lt<latency>_<ids>_<time>.csv` / `.png`.

### 12 leader arm live
```
python scripts/12_leader_arm_live.py
python scripts/12_leader_arm_live.py --port /dev/ttyUSB0
```
| Option | Default | Meaning |
|---|---|---|
| `--port` | `config.LEADER_PORT` | leader arm device |

Shows joints [deg], button and trigger at about 10 Hz. Ctrl+C to stop.

### 13 leader J6 -> RB5 J6
```
python scripts/13_leader_j6_teleop.py           # Simulation
python scripts/13_leader_j6_teleop.py --real    # Real (type to confirm)
```
| Option | Default | Meaning |
|---|---|---|
| `--sim` / `--real` | Simulation | mode |
| `--port` | `config.LEADER_PORT` | leader arm device |

Hold the handle button (clutch) to move. Log: `data/13_leader_j6_<mode>_<time>.csv`.

### 14 leader arm -> RB-Y1 simulator (env `rby1`)
```
conda activate rby1
python scripts/14_leader_rby1_teleop.py                 # 50 Hz
python scripts/14_leader_rby1_teleop.py --period 0.01   # 100 Hz
```
| Option | Default | Meaning |
|---|---|---|
| `--address` | localhost:50051 | RB-Y1 simulator |
| `--port` | `config.LEADER_PORT` | leader arm device |
| `--period 0.02\|0.01` | 0.02 | loop period [s] |

The leader arm moves by itself at start (aligns to the READY pose with weak torque). Log:
`data/14_leader_rby1_<time>.csv`.

### 15 rby1-sdk example 35 without the gripper (env `rby1`)
```
conda activate rby1
python scripts/15_example35_nogripper.py --address localhost:50051 --model m
```
| Option | Default | Meaning |
|---|---|---|
| `--address` | required | robot address |
| `--model` | a | robot model name |
| `--power` | `.*` | power device regex |
| `--servo` | `torso_.*\|right_arm_.*\|left_arm_.*` | servo regex |
| `--mode position\|impedance` | position | control mode |
| `--align-duration` | 4.0 | leader alignment time [s], 0 = skip |

The leader arm moves by itself at start. Diff from the original: `docs/patches/15_example35_nogripper.diff`.

### 16 leader hold diagnosis
```
python scripts/16_leader_hold_diag.py --label adapter065       # measure
python scripts/16_leader_hold_diag.py --compare A.csv B.csv    # compare two runs, no hardware
```
| Option | Default | Meaning |
|---|---|---|
| `--label` | - | name of the power setup (in the file name) |
| `--port` | `config.LEADER_PORT` | leader arm device |
| `--compare A B` | - | compare two CSVs |

Writes torque: the leader arm holds poses (support the arm). Log: `data/16_hold_diag_<label>_<time>.csv`.

### 17 TCP read compare
```
python scripts/17_tcp_read_compare.py
```
No options. Prints tcp_ref, tcp_pos, get_tcp_info, calc_fk_tcp and their differences.

### 18 servo_l probe
```
python scripts/18_servo_l_probe.py --exp hold    # same target, expect no motion
python scripts/18_servo_l_probe.py --exp z       # z + 1 mm
python scripts/18_servo_l_probe.py --exp rz      # rz + 1 deg
```
| Option | Meaning |
|---|---|
| `--exp hold\|z\|rz` | experiment (required) |

Simulation only. Log: `data/18_servo_l_<exp>_<time>.csv`.

### 19 move joint (Simulation)
```
python scripts/19_move_joint_sim.py --joint 5 --to 45
python scripts/19_move_joint_sim.py --pose 180 -15 -90 -165 90 0 --speed 30
```
| Option | Default | Meaning |
|---|---|---|
| `--joint N --to DEG` | - | move only joint N (1-6) to DEG; change <= 60 deg |
| `--pose J1 .. J6` | - | all joints [deg]; change <= 180 deg; shortest equivalent angle |
| `--speed` | 10 | move_j speed [deg/s], at most 30, x speed bar (10 -> 1 deg/s) |

Asks before moving. J3 limit in this script is +/-150 (`JOINT_LIMIT_DEG`).

### 20 leader FK live (env `rby1`)
```
conda activate rby1
python scripts/20_leader_fk_live.py
```
| Option | Default | Meaning |
|---|---|---|
| `--port` | `config.LEADER_PORT` | leader arm device |

Keys: `b` save baseline (shows the right arm change from it), `q` quit.

### 21 leader -> RB5 position (Simulation)
```
python scripts/21_leader_rb5_position.py
python scripts/21_leader_rb5_position.py --speed-mm-s 200 --max-lag-mm 0 --scale 0.5
```
| Option | Default | Meaning |
|---|---|---|
| `--yaw-deg` | 0 | leader frame rotation about z [deg] |
| `--scale` | 1.0 | RB5 motion / leader motion |
| `--speed-mm-s` | 50 | TCP speed limit, at most 200 |
| `--max-lag-mm` | 30 | catch-up lag (C), 0 = B, at most 50 |
| `--no-yaw` | off | keep the start orientation |
| `--port` | `config.LEADER_PORT` | leader arm device |

Replaced by `python -m teleop` (settings in `configs/sim.yaml`). Log: `data/21_leader_pos_sim_<time>.csv`.

### 22 base axis check
```
python scripts/22_base_axis_check.py --joints J1 J2 J3 J4 J5 J6   # tablet values [deg]
python scripts/22_base_axis_check.py                              # use jnt_ang
```
| Option | Meaning |
|---|---|
| `--joints J1 .. J6` | real arm joint angles from the tablet [deg] |

Prints the flange center (x, y) to measure with a tape from the base center.

### 23 trial summary (real-1)
```
python scripts/23_trial_summary.py data/teleop/teleop_real_20261016_*.csv
python scripts/23_trial_summary.py data/teleop/teleop_real_20261016_*.csv --trials data/teleop/trials.csv \
    --p 480 -120 --q 470 180 --z-max 120 --radius 30
```
| Option | Default | Meaning |
|---|---|---|
| `logs` | required | teleop logs, one per trial (`_settings.yaml` files are skipped) |
| `--trials` | - | manual results: `run,practice,success,failure_type,note` |
| `--p X Y` / `--q X Y` | - | pick / place point in the RB5 base frame [mm] |
| `--z-max` | 100 | measured TCP z at or below this counts as "down" [mm] |
| `--radius` | 30 | horizontal distance to P / Q that counts [mm] |

Prints one line per trial (time, clutch presses, follow %, dropped mm, which limits were active) and a
summary per setting without the practice trials. Protocol: `docs/real_trial_protocol.md`.

### 24 run report
```
python scripts/24_run_report.py data/teleop/teleop_real_20261016_051339.csv          # full report
python scripts/24_run_report.py data/teleop/teleop_real_20261016_*.csv --compare     # one line per run
```
| Option | Meaning |
|---|---|
| `logs` | teleop logs (`_settings.yaml` files are skipped; the settings next to each log give its caps and range) |
| `--compare` | one line per run: presses, speed / joint limit shares, drops, jitter, joint acceleration, lag, late, slowdowns |

Joint speeds and accelerations are computed from the RB5 reference with the real sample times
(`t_s - state_age_ms`; the state arrives 13-22 ms apart). Runs without clutch time are reported as such.

### 25 camera RGB test (cam-1)
```
python scripts/25_camera_rgb_test.py                                        # 640x480 @ 30 for 60 s
python scripts/25_camera_rgb_test.py --res 640x480 --fps 60 --duration 60
python scripts/25_camera_rgb_test.py --ae-priority on                       # D435 color sensor option
```
| Option | Default | Meaning |
|---|---|---|
| `--res WxH` | 640x480 | color resolution (both cameras) |
| `--fps` | 30 | frame rate |
| `--duration` | 60 | seconds |
| `--warmup` | 2 | seconds left out of the statistics (auto exposure settling) |
| `--ae-priority on/off/keep` | keep | color sensor auto_exposure_priority while running (D405: not available) |
| `--allow-usb2` | off | run even if a camera is on USB 2 |

Needs `pyrealsense2` (environment.yml). Output: `data/cameras/cam_<time>_<res>_<fps>.csv` (one row per frame:
frame number, camera timestamp and domain, metadata timestamps, PC receive time) and a sample PNG per camera.

### 26 jnt_ref check
```
python scripts/26_jnt_ref_check.py data/teleop/teleop_real_20261016_051339.csv
```
One log at a time. Prints the DH model error (FK(jnt_ref) vs tcp_ref), the best delays of jnt_ref / jnt_ang
behind the IK of the sent target and of tcp_ref behind the sent target, and the differences at rest.

### 27 gripper scan
```
python scripts/27_gripper_scan.py --port /dev/serial/by-id/usb-FTDI_..._<U2D2 serial>-if00-port0
```
| option | default | meaning |
|---|---|---|
| `--port` | `config.GRIPPER_PORT` (not set yet) | U2D2 device; refused if it is the leader arm's adapter |
| `--all` | off | scan every baud rate (default: stop at the first one where a motor answers) |

Read-only (ping and read). Tries Protocol 2.0 and 1.0 at 57,600 (factory default), 1M, 2M, 3M, 4M, 115,200
and 9,600 bps. Prints the model number (320 = MX-106 Protocol 1.0 firmware, 321 = MX-106(2.0)), the
firmware version, the settings and the read time of the block the gripper thread would read.

### 28 trigger -> gripper
```
python scripts/28_trigger_gripper.py                                   # measure open / closed by hand
python scripts/28_trigger_gripper.py --closed 3968 --open 2196 --current 130
```
| option | default | meaning |
|---|---|---|
| `--closed`, `--open` | measured by hand (torque off, Enter at each) | the only position limits: the motor's own limits are not used in mode 5 |
| `--current` | 150 (0.50 A) | Goal Current = force cap, x 3.36 mA, max 300 (1.0 A) |
| `--trigger` | left | which leader trigger (released 0 = closed, pulled 1000 = open) |

**Moves the gripper.** Writes after typed `yes`: torque off, operating mode 5, profile velocity 210
(48.1 rpm), profile acceleration 200, goal current, goal position = present, torque on. Then it holds
until the trigger points within 5 % of the gripper position (catch, the needed value is shown). At the
end (Ctrl+C or an automatic stop: hardware error, < 10 V, > 70 C, 3 failed gripper or trigger reads,
any error such as a USB disconnect, SIGTERM / SIGHUP): torque off, operating mode 3, the log saved.
Torque on or mode 5 left by a run that did not finish is shown and turned off with the confirmed writes. The leader arm is only read. Log: `data/gripper/`.

### 29 dummy LeRobot dataset
Env `lerobot` (not `rb5`): `conda activate lerobot`.
```bash
python scripts/29_lerobot_dummy_dataset.py               # 2 episodes x 3 s
python scripts/29_lerobot_dummy_dataset.py --overwrite --episodes 3 --seconds 10
lerobot-dataset-viz --repo-id local/dummy_rec1 --root data/lerobot/dummy_rec1 --episode-index 0
```
Writes a LeRobotDataset with the rec-1 format (30 fps, state 7 = J1..J6 + gripper, action 4 = x, y, z +
gripper, `front` / `wrist` 480x640 video) from synthetic values to `data/lerobot/dummy_rec1`, discards
one more episode (clear_episode_buffer), reads it back and compares state, action and images with what
was written, and loads an ACT action chunk (100 steps, padded at the episode end). Prints OK or the
problems. No robot, no cameras.

### 30 camera recorder test
```bash
python scripts/30_camera_recorder_test.py                              # front + wrist, 2 x 20 s kept, 1 x 3 s discarded
python scripts/30_camera_recorder_test.py --seconds 60 --episodes 1
python scripts/30_camera_recorder_test.py --camera front=943222070221   # one camera
python scripts/30_camera_recorder_test.py --fake                        # synthetic frames, no cameras
```
Starts the camera process of `teleop/cameras.py` and records episodes the way the teleop will (the
last one discarded: its files must be gone). Meanwhile this process runs a 50 Hz loop and counts late
cycles (> 5 ms). Prints per episode and camera: frames, fps, frame interval, JPEG write time, size and
MB/s; the camera process CPU. Files: `data/cameras/rec_<time>/episode_<n>/`.

### 31 convert to LeRobot
Env `lerobot`: `conda activate lerobot`.
```bash
python scripts/31_convert_to_lerobot.py data/demos/sim_20261019_134705 --check-only     # report only
python scripts/31_convert_to_lerobot.py data/demos/real_* --repo-id local/rb5_pick_place --overwrite
python scripts/31_convert_to_lerobot.py data/demos/real_* --cut-pauses 2.0               # cut pauses > 2 s
```
| option | default | meaning |
|---|---|---|
| `--repo-id` | `local/rb5_demos` | dataset name; output `data/lerobot/<name>` (`--root` elsewhere) |
| `--cut-pauses S` | 0 (off) | remove clutch-released stretches longer than S s with an unchanged gripper command |
| `--min-clutch-s`, `--min-path-mm` | 1.0, 30 | episodes below either are still: left out (`--keep-still` keeps them) |
| `--check-only` | | print the report, write nothing |
| `--cameras` | `front,wrist` | e.g. `front` for a test dataset without the wrist camera |

Always builds the dataset anew from the sessions. Episodes whose camera frames do not cover them
(none, a gap or missing start / end over 0.2 s) are left out with the reason. About 1.1 x real time per
camera (JPEG decoding + AV1 encoding). Action / state / image rules: the docstring and
docs/decisions.md (rec-4a). Prints per episode: duration, clutch time, path, image time error, lost
camera frames, gripper following, start vs ready pose; at the end the 30 -> 50 Hz resampling error.

### 32 replay an episode (Simulation)
Env `lerobot`: `conda activate lerobot`. **Moves the RB5 in Simulation mode** (Real is refused).
```bash
python scripts/32_replay_episode.py --repo-id local/sim_test --episode 0
```
Loads the episode's 30 Hz actions and states, switches to Simulation, moves to the episode's first
joints with move_j (typed yes), then streams at 50 Hz: the action interpolated at the replay time
through `teleop.follow.TargetFollower` (the teleop's per-cycle limits) -> move_servo_l; the last action
is held for `--hold-s` (1 s). Prints: replay joints vs recorded joints at the same episode time,
target vs desired (what the limits changed), tcp_ref vs target, the end position, limit flags. CSV in
`data/replay/`. The gripper action is not sent.

### 33 policy check (offline)
Env `lerobot`. No robot.
```bash
python scripts/33_policy_check.py data/train/<run>/checkpoints/last/pretrained_model --repo-id local/pilot_front --two-cameras
```
Loads the policy and its normalization from the checkpoint; on frames spread over the dataset compares
the predicted chunk with the recorded actions (+1, +10, +30, +99 steps; training data, so it shows the
pipeline and the fit, not robot performance); times one inference (batch 1) on the GPU and the CPU;
`--two-cameras` also times an untrained ACT with two 480x640 cameras.

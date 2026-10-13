# scripts

Numbered experiment scripts, kept as they were run (the logs and `docs/decisions.md` refer to them
by number). The teleop is not here: `python -m teleop` (see `docs/teleop_guide.md`).

## Before you run anything
- Run from the repository root: `conda activate rb5`, then `python scripts/<name>.py ...`.
  Scripts 14, 15 and 20 need `conda activate rby1` instead (they use `rby1_sdk`).
- Robot IP and speed bar come from `configs/robot.yaml` through `scripts/config.py` (`import config`).
  01 and 02 have the IP written in the file.
- Every script prints its options with `--help` (all except 01, 02, 04 and 17, which have none).
- Leader arm scripts need `/dev/ttyUSB0` with FTDI `latency_timer` = 1 (`docs/teleop_guide.md`, udev rule).
- The leader arm URDF is `assets/leader_arm/model.urdf` (teleop, 20, 21); 15 still uses `refs/rby1-sdk/`.
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
| `--port` | /dev/ttyUSB0 | leader arm device |
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
| `--port` | /dev/ttyUSB0 | leader arm device |

Shows joints [deg], button and trigger at about 10 Hz. Ctrl+C to stop.

### 13 leader J6 -> RB5 J6
```
python scripts/13_leader_j6_teleop.py           # Simulation
python scripts/13_leader_j6_teleop.py --real    # Real (type to confirm)
```
| Option | Default | Meaning |
|---|---|---|
| `--sim` / `--real` | Simulation | mode |
| `--port` | /dev/ttyUSB0 | leader arm device |

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
| `--port` | /dev/ttyUSB0 | leader arm device |
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
| `--port` | /dev/ttyUSB0 | leader arm device |
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
| `--port` | /dev/ttyUSB0 | leader arm device |

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
| `--port` | /dev/ttyUSB0 | leader arm device |

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

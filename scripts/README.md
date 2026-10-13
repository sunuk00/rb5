# scripts

Numbered experiment scripts, kept as they were run (the logs and `docs/decisions.md` refer to them
by number). Run from the repository root, e.g. `python scripts/22_base_axis_check.py`. Most read the
robot IP from `config.py`; the teleop itself is `python -m teleop`.

Status: **tool** = still useful as is; **reference** = result recorded, kept for the record;
**replaced** = superseded by the teleop (`teleop/`). Env is `rb5` unless noted.

| # | Script | What it does | Status |
|---|---|---|---|
| 01 | `01_read_state.py` | Read the joint angles once (data channel) | reference |
| 02 | `02_move_joint.py` | Move J6 by 2 deg with move_j | reference |
| 03 | `03_pose_sequence.py` | home -> pose_1 -> home with move_j | reference |
| 04 | `04_data_rate.py` | Data channel update period (Simulation) | reference |
| 05 | `05_key_input.py` | Keyboard input with evdev in a 20 Hz loop | replaced |
| 06 | `06_servo_j_probe.py` | How move_servo_j behaves (Simulation) | reference |
| 07 | `07_servo_jog.py` | Keyboard jog with move_servo_j, 20 Hz | replaced |
| 08 | `08_servo_jog_fast.py` | 07 with a background state reader, shorter period | replaced |
| 09 | `09_measure_read_rate.py` | Data channel read performance (read-only) | reference |
| 10 | `10_stream_rate_test.py` | move_servo_j streaming rate (Simulation) | reference |
| 11 | `11_leader_arm_read_rate.py` | Leader arm read speed (latency_timer 16 vs 1 ms) | reference |
| 12 | `12_leader_arm_live.py` | Live leader arm joints, button and trigger (read-only) | tool |
| 13 | `13_leader_j6_teleop.py` | Leader J6 -> RB5 J6 with a clutch | replaced |
| 14 | `14_leader_rby1_teleop.py` | Leader arm -> RB-Y1 simulator arms (env `rby1`) | reference |
| 15 | `15_example35_nogripper.py` | rby1-sdk example 35 without the gripper (env `rby1`); diff in `docs/patches/` | reference |
| 16 | `16_leader_hold_diag.py` | Leader arm vibration while holding a pose | tool |
| 17 | `17_tcp_read_compare.py` | Ways to read the RB5 TCP pose (read-only) | reference |
| 18 | `18_servo_l_probe.py` | move_servo_l frame, axes and rotation convention (Simulation) | reference |
| 19 | `19_move_joint_sim.py` | move_j one joint or all six to a pose (Simulation) | tool |
| 20 | `20_leader_fk_live.py` | Live leader arm FK (env `rby1`, read-only) | tool |
| 21 | `21_leader_rb5_position.py` | Leader wrist center -> RB5 TCP with move_servo_l | replaced |
| 22 | `22_base_axis_check.py` | Where the RB5 base +x / +y point on the real robot (read-only) | tool (Real checklist) |

# Decisions

## 2026-10-10 — 14: align the leader arm to the RB-Y1 READY pose at startup

**Decision:** At startup, script 14 moves the real leader arm to the RB-Y1 READY arm pose
(example 35 `READY_POSE["M"]`) in 6 s with a 0.5 Nm current limit, then holds it with that weak
torque. Each arm's torque is switched off at that arm's first clutch press. During teleoperation the
only write to the leader arm is torque off (added to CLAUDE.md safety rules).

**Reason:** Both robots then start from the same joint pose, so the first clutch press maps a
matching pose.

**Rejected alternatives:**
- Move the RB-Y1 simulator to the leader arm's current pose (keeps the leader read-only). Rejected
  by choice; some leader poses are outside the RB-Y1 joint limits (e.g. left J6 180.2 deg vs +/-155).
- Full example 35 behavior (gravity-compensation torque while the button is held, position hold when
  released). Needs the dynamics model in the loop; out of scope for now.
- Switch the torque off right after alignment: the arm drops and the aligned pose is lost at once.

## 2026-10-10 — 14: one dynamixel_sdk PortHandler for the leader arm

**Decision:** Alignment writes, teleop reads and torque off all use one `dynamixel_sdk` PortHandler,
opened once and closed at the end.

**Reason:** `rby1_sdk.DynamixelBus` (used by example 35) has no Python method to close the port;
it is closed only by the C++ destructor. Mixing it with dynamixel_sdk would open the port twice or
depend on garbage collection.

**Rejected alternatives:** `rby1_sdk.DynamixelBus` for alignment then dynamixel_sdk for teleop;
`rby1_sdk.DynamixelBus` for everything (its fast sync read path was not measured).

## 2026-10-10 — 14: leader arm torque constant chosen per ID from the actual model (differs from Rainbow)

**Decision:** The goal current for a torque is `torque / Kt * 1000 / 2.69` (rby1-sdk
`dynamixel_bus.cpp:501`), with Kt chosen from the model number read by ping:
1130 (XM540-W150) -> 1.6591 Nm/A, 1030 (XM430-W210) -> 1.3043 Nm/A. Unknown model -> no write.

**Difference from Rainbow:** rby1-sdk `src/upc/leader_arm.cpp:12-13` and example 35 (line 282) use
`{1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043}` per arm by ID. On this leader arm the
ping (script 11) returned 1130 for IDs 0-3 and 7-10 and 1030 for IDs 4-6 and 11-13, so Rainbow gives
IDs 3 and 10 (elbows, XM540-W150) the XM430-W210 value. With 0.5 Nm requested that would be raw 142
instead of 112, about 0.63 Nm on those motors (+27 %).

**Evidence:**
- Model number -> name: `dynamixel_easy_sdk/control_table/dynamixel.model` in the installed
  dynamixel-sdk 4.1.0 (`1030 xm430_w210.model`, `1130 xm540_w150.model`).
- Addresses 11 / 64 / 102 / 116 / 132 are the same in `xm430_w210.model` and `xm540_w150.model`.
- Kt values: not found in any local file. 1.6591 = 7.3 / 4.4 and 1.3043 = 3.0 / 2.3 match the
  stall torque / stall current of XM540-W150 and XM430-W210 at 12 V in the ROBOTIS e-Manual
  (from memory, not verified in this repo). Both models use 2.69 mA per current unit.

**Rejected alternatives:** keep Rainbow's per-ID array (wrong for IDs 3 and 10 on this hardware).

## 2026-10-10 — 14: leader arm holds its pose while the clutch is released (rule changed)

**Decision:** Per arm, on clutch press: torque off (move freely by hand). On clutch release
(including "released" after 3 failed button reads): goal position = the joint angles just read,
then torque on, so the arm holds that pose with the startup current limit (0.5 Nm). Operating mode
(5) and goal current are not written again during teleop. CLAUDE.md safety rule changed from
"the only allowed write is torque off" to these two transition writes.

**Reason:** With torque off after the first press, the leader arm dropped whenever the button was
not held, so the pose could not be kept between clutch presses (same idea as example 35 lines
1198-1203: position hold while the button is released).

**Order:** the goal position must be written before torque on; otherwise the motor moves to the
stored goal (the READY pose or an older hold pose).

**Assumption to verify:** torque off does not clear operating mode or goal current (general
Dynamixel behavior, not checked in a document). Check that the arm holds after the first release.

**Rejected alternatives:** example 35's current control with gravity-compensation torque while the
button is held (needs the dynamics model in the loop); writing every cycle (not needed for holding).

## 2026-10-10 — 14: leader wrist joints always torque off after the alignment

**Decision:** Wrist joints (IDs 4-6, 11-13) have torque only during the 6 s startup alignment and are
switched off right after it. On clutch release only the shoulder and elbow joints (IDs 0-3, 7-10)
hold their pose.

**Reason:** While holding, right wrist IDs 4 and 6 oscillated by +/-0.18-0.26 deg (2-3 encoder
ticks) and changed direction on 90-97 of 158 cycles (`data/14_leader_rby1_20261010_112745.csv`);
the user reported strong vibration. The wrists are light, so letting them hang is acceptable.

**Notes:** Position gains read before (11:30) and after (11:32) a leader arm power cycle are the
same: D 6500 (XM540-W150) / 4000 (XM430-W210), I 0, P 800. So these are the power-up values of these
motors, not written by any program (our code never writes address 80). The power cycle was real:
goal current changed from 112 / 142 (ours) to 1000. Operating mode 5 persisted (EEPROM). The cause of
the vibration is not confirmed (high D gain is still only a guess). The left shoulder (ID 7) drifting while held (7.4 -> 18.2 deg in
3.3 s) is a separate open issue.

**Rejected / deferred:** raise the shoulder current limit (B, on hold until the left/right
difference is understood); D gain experiment (C, not now); example 35 gravity compensation (D).

## 2026-10-11 — 14: leader thread + Fast Sync Read; --period 0.02 (default) / 0.01; C++ not needed

**Decision:** During teleop a LeaderThread owns the leader port: Fast Sync Read of the 14 joints +
both buttons, as fast as it can, and it runs the clutch writes requested by the main loop. The main
loop only computes targets and calls `send_command`. `--period 0.02` (50 Hz) stays the default;
`--period 0.01` (100 Hz) is an option to test.

**Measurements (latency_timer 1 ms, 500 reads each, 0 failures):**

| Read of 14 joints | mean | p99 |
|---|---|---|
| dynamixel_sdk GroupSyncRead (previous 14) | 4.65 ms | 7.31 ms |
| dynamixel_sdk Fast Sync Read | 2.05 ms | 4.42 ms |
| rby1_sdk (C++) group_fast_sync_read | 2.14 ms | 4.66 ms |
| rby1_sdk get_motor_states (pos+vel+current) | 5.32 ms | 9.09 ms |
| rby1_sdk read_button_status x2 | 2.22 ms | 5.86 ms |

Fast Sync Read and Sync Read values agree within 1 tick. 100 Hz on the simulator, 300 cycles, arms
holding their pose:

| | single loop (fast sync + buttons + send) | thread split |
|---|---|---|
| main loop work mean / p99 | 8.93 / 14.72 ms | 4.13 / 9.86 ms |
| send interval mean / max | 10.52 / 21.94 ms | 10.00 / 16.50 ms |
| intervals over 15 ms | 21 / 299 | 10 / 299 |
| leader update (thread) | - | 4.53 ms mean (about 220 Hz), age at send 2.4 ms mean |

**Why no C++ (rby1_sdk) reads:** C++ fast sync read (2.14 ms) is no faster than Python Fast Sync
Read (2.05 ms). The time is spent on the bus (USB-serial + 2 Mbps transaction), not in Python. The
leader thread's reads were as fast as alone (1.94 ms), so the GIL was not a problem.

**Late-send limit:** 50 Hz keeps the tested 30 ms (1.5 x period). 100 Hz uses 20 ms (2 x period)
because `send_command` alone sometimes takes about 10 ms (p99 9.8 ms in both layouts). A cycle is
"late" if the gap before sending OR the end-to-end interval between commands (which includes a slow
`send_command`) is over the limit; targets are frozen only when it is known before sending. Every
cycle logs `late`, `late_before_send`, `late_send`, `send_interval_ms`, `send_call_ms` and the RB-Y1
joint angles, so jumps at late cycles can be checked. 3 late cycles in a row stop the run.

**Overrun policy changed for 14:** after an overrun the next cycle starts at once (the 20 ms limit was
measured this way). The 07/08 rule "wait a full period after an overrun" would add up to one more
period after every slow `send_command` (about 20-25 ms gaps at 100 Hz). RB-Y1 commands carry
`minimum_time`, so two commands close together are not the jump risk that RB5 `move_servo_j` had.

**Clutch debounce is time based:** 40 ms of pressed reads to engage, 60 ms of failed reads to release
(2 / 3 cycles at 50 Hz as before, 4 / 6 at 100 Hz). Only new leader samples are counted.

**Rejected alternatives:** single loop with Fast Sync Read (7 % of intervals over 15 ms at 100 Hz);
a separate sender thread (does not remove the slow `send_command` calls); rby1_sdk C++ reads (no speed
gain, and its port cannot be closed from Python).

## 2026-10-12 — 17/18: TCP pose sources and the move_servo_l probe

**Facts from 17 (read-only, Simulation, `scripts/17_tcp_read_compare.py`):** `get_tcp_info`, `tcp_pos`
and `calc_fk_tcp(jnt_ref)` equal `tcp_ref` (difference 0.000). So `get_tcp_info` returns the current
TCP pose (not the TCP setting), and `calc_fk_tcp` / `tcp_ref` share the base frame and mm / deg units.

**TCP offset:** a TCP offset of (0, -187, -40) mm, orientation offset 0, is set on the controller.
Derived as R^T (tcp - tfc) with R = Rz(rz) Ry(ry) Rx(rx) (ZY'X''); an XYZ order gives non-round
numbers. Who set it, and when, is not known. This is separate from the tool mass (0 kg).

**Rotation convention source:** Rainbow rb_cobot_docs, UI Script 1.10 Units ("Z-Y'-X'' Euler angle
(Degree)") and 7.35 Servo L ("Desired Cartesian posture in mm,deg unit (ZY'X'' Euler)"). The t1 / t2 /
gain / alpha text of 7.35 is identical to 7.34 Servo J. The frame of `move_servo_l` is not stated.

**Decision (18):** probe `move_servo_l` in Simulation with step targets, one experiment per run, in
the order hold -> z +1 mm -> rz +1 deg (t1 = send period 0.02 s, t2 0.1, gain 1, alpha 1; 1.0 s of
the same target; end with move_speed_j(0) as in 13).

**Interpretation rule:** "no motion" in the hold experiment cannot be told apart from "command
ignored". The hold result is read together with the z +1 mm experiment as the positive control: only
if z moves by the expected 1 mm does hold's "no motion" show that the command is an absolute pose in
the base frame.

**Rejected alternatives:** ramping the step over 10 cycles (not needed in Simulation; a step shows
the response time directly).

## 2026-10-12 — 18 results: move_servo_l convention confirmed; wrist singularity near J5 = 0

**Results (Simulation, user runs, `data/18_servo_l_{hold,z,rz}_20261012_0742*.csv`):**
hold: no change. z +1 mm: change (0, 0, +1.000). rz +1 deg: rz +1.000, position change 0, but joints
moved up to 6.06 deg (J4 +6.06, J6 -4.98) and the 3 deg joint limit stopped the run. Start J5 = 3.68 deg.

**Conclusion:** `move_servo_l(pnt[x, y, z, rx, ry, rz], ...)` is an absolute TCP pose in the base frame,
mm and deg, ZY'X'' Euler (R = Rz Ry Rx), and an orientation change rotates about the TCP (the TCP
position stays fixed). The hold result counts because z (positive control) moved exactly 1 mm.

**Wrist singularity evidence:**
- Rainbow analytic IK (rb_cobot_docs, technical_docs/analytic_inverse_kinematics) computes
  `th6 = atan2(.../sin(th5), .../sin(th5))`: undefined at th5 = 0 (and 180). The same page also
  writes `R = Rz * Ry * Rx`, and the tool axis is the flange Y axis (`P05 = P06 + d6*Y06`), which fits
  the -187 mm Y component of the TCP offset.
- The manual only describes singularities in general terms (area above/below the base, "rapid joint
  speeds or cause the robot to stop" for Cartesian motion, manual 1.7 Robot Work Area); it does not
  name J5 = 0.
- Model check with the RB5-850 DH table (technical_docs/dh_parameters): FK matches 17's TFC within
  2 mm in position and 0.0002 in rotation matrix elements. Joint change needed for rz +1 deg at the 18
  start pose (J5 = 3.68): J4 +8.0, J6 -6.2, J2 -3.8 deg (observed before the stop: J4 +6.06, J6 -4.98).
  Max joint change by J5: 0 deg -> singular, 1 deg -> 32, 3.68 -> 8.0, 10 -> 2.4, 20 -> 1.0, 45 -> 1.0,
  90 -> 1.5 deg.

**Observations to remember:**
- The TCP reached rz +1 at 80 ms while J4 and J6 kept moving in opposite directions: near J5 = 0,
  J4 + / J6 - barely changes the orientation, so a large joint motion is a nearly invisible TCP motion.
- The safety check is done once per cycle before sending, so the joints went 1.8 deg past the 3 deg
  limit (4.24 at the check -> 6.06 final) after the stop. In 20 ms a joint moved up to 2 deg
  (about 100 deg/s) for a 1 deg TCP command. A real-robot Cartesian teleop needs a joint speed limit
  and a guard on J5 near 0.

## 2026-10-12 — 18 rz away from J5 = 0: wrist singularity hypothesis supported

**Result (Simulation, user run, `data/18_servo_l_rz_20261012_075518.csv`):** start J5 = 31.26 deg (not
45, see below). rz +0.9995 deg, position change < 0.001 mm, max joint change 0.838 deg, no safety stop,
within 0.01 mm / 0.01 deg at 60 ms, max joint step per cycle 0.26 deg (at J5 = 3.68: about 2 deg).

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| measured change [deg] | +0.716 | -0.731 | +0.838 | +0.091 | +0.257 | -0.231 |
| DH model prediction | +0.729 | -0.748 | +0.843 | +0.095 | +0.245 | -0.222 |

The large J4 / J6 motion at J5 = 3.68 deg (J4 +6.06, J6 -4.98 before the stop) is gone at
J5 = 31 deg, and the DH model predicts both runs. Conclusion: the 18 rz joint motion was caused by
the wrist singularity at J5 = 0.

## 2026-10-12 — 19: move_j speed is scaled by the speed bar

**Observation:** 19 moved J5 by 41.3 deg with move_j(10 deg/s, 20 deg/s^2) and speed bar 0.1. The
expected 4.1 s was wrong: after 30 s J5 was at 31.26 deg (27.6 deg, about 0.9 deg/s average) and the
script sent task_stop. This fits an effective speed of 10 x 0.1 = 1 deg/s (plus acceleration time).
02 moved only 2 deg, so it was not noticed there. The UI Script docs (5.41 set_speed_bar) only say
"Set the overall speed control bar" and do not state the scaling; it is inferred from this run.
Servo commands are not scaled (06).

**Consequence:** with the speed bar at 0.1 (CLAUDE.md), a move_j speed argument of v gives about
0.1 v. Timeouts and expected durations must use the scaled speed.

## PENDING

- **14: RB-Y1 self-collision check during teleop.** Example 35 blocks the arm command when the
  nearest link distance is below 2 cm (`detect_collisions_or_nearest_links`, lines 1226-1240).
  Skipped for now (simulator only); needed before any real RB-Y1 use. Measure its cost in the 50 Hz loop.

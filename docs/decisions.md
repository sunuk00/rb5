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

## 2026-10-12 — 20: leader arm FK with rby1_sdk.dynamics; tip = wrist center

**Decision:** Leader arm FK uses `rby1_sdk.dynamics` on `refs/rby1-sdk/models/leader_arm/model.urdf`
(as example 35), with all 14 joints passed to `make_state`. The "tip" is the Link_6R / Link_6L origin,
the intersection of the three wrist axes (center of the J5 motor axis from outside). A handle tip
offset is not in the URDF; it is to be measured and added later. Orientation is shown as ZY'X'' Euler
(same as the RB5). Left arm joints are read too (read-only) for the left-right distance check.

**Reason:** Rainbow's own URDF parser, same code path as example 35; a numpy evaluation of the URDF
chain is used only as a startup self-test (max difference 6e-8 over 50 random poses).

**Pitfall found:** with only the 7 right-arm joints in `make_state`, `set_q` silently ignored J3..J6
(J0..J2 applied, no error). The script stops if the self-test fails.

**Ruler check (user, 2026-10-12):**
- V2 left-right wrist center distance, arms hanging: ruler 302 mm, screen 305 mm (pass, +/-5 mm).
- V3 right arm, J2 near 0, elbow -20 -> -94 deg: screen dx +181.9, dy -18.5, dz +169.1 mm; measured
  height change about 169 mm (measured in inches, converted) (pass).
- With J2 about -100 deg, bending the elbow moves the wrist sideways (-y), on screen and on the arm.
- Leader Base frame confirmed: x forward, y left, z up.
- Not done: V1 (link lengths).
- V4 (handle tilt with a phone inclinometer) skipped by choice. The orientation output is to be
  checked instead by a functional test in the orientation mapping step: compare the wrist rotation
  directions of the leader arm and the RB5.

**Rejected alternatives:** direct numpy FK as the main path (kept as the self-test only).

## 2026-10-12 — 21: leader -> RB5 position mapping, first Simulation results

**Setup:** `scripts/21_leader_rb5_position.py`, leader wrist center -> RB5 TCP position, orientation
fixed, clutch + B (drop the excess) + speed limit, move_servo_l at 50 Hz, J5 watch with the DH model.
Leader FK is the numpy URDF chain (20's self-test reference) so that it runs in the `rb5` env.

**Results (user runs, Simulation):**

| run | speed limit | RB5 path / hand path | speed limited | joint speed limited | range limited |
|---|---|---|---|---|---|
| 092533, 093034 | 50 mm/s | 22 % / 26 % | 86 % / 83 % | 0 % / 0.2 % | 7 % / 8 % |
| 094517 | 200 mm/s (lead limit 8 mm) | 38 % (54 % if only the 200 mm/s cap) | 52 % | 52 % | 9 % |

- Hand speed while engaged: p50 143-206 mm/s, p90 343-570 mm/s. The slow segment (16 % speed
  limited) followed 0.90 of the hand. With B, fast-out / slow-back moves shift the hand-robot offset
  (segment 5: hand net 10 mm, RB5 49 mm).
- Direction: cos(hand step, target step) median 1.000, no reversed step.
- "J1 barely turns": at the 092533 start pose J1 moves 0.318 deg per mm of TCP x but only 0.018 per mm
  of y. J1 follows the wrist point P05, not the TCP; with the orientation fixed and the 187 mm TCP
  offset, P05 sat about 60 deg away from the TCP direction. With a fixed orientation the joint motion
  of a pure translation does not depend on the TCP offset (TCP and flange give the same numbers).
- 094517 ended in a shoulder singularity: P05 horizontal radius 135.7 -> 110.8 mm (d4 = 110.7), up to
  14.2 deg of joint motion per mm. Only the joint speed limit held it (actual max 1.57 deg/cycle).

**Decisions:**
- `--speed-mm-s` (default 50, at most 200 in Simulation); the lead limit is 2 x the step (a fixed
  2 mm froze every 4 mm step).
- Shoulder singularity watch: margin = P05 horizontal radius - d4 (Rainbow IK
  `th1 = atan2(...) - acos(d4 / r)`). Below 50 mm: warning and slower (0.25x at 20 mm); below 20 mm:
  stop; start needs >= 50 mm. Thresholds chosen from 094517: at a 25 mm margin 0.67 deg/mm, at 8 mm
  1.1 deg/mm, at 1.7 mm 2.4 deg/mm.
- Per-cycle timing columns (leader read, plan, send call, work, real send interval) and a late check
  right before sending; the new target is applied only if that check passes.

**Late check gap (also in 13 and 08):** the late check measured "last send -> start of the next
cycle's check". After an overrun the loop waits a full period (08 rule), so the real send gap was
40-50 ms while `late` stayed 0. 21 had 6 / 6 / 27 such cycles (all while engaged, cause not found;
planning is 1.8 ms mean, not related to button read failures). 13 and 08 use the same check and have
the same blind spot; their logs' `late = 0` does not prove there were no 40 ms gaps. Not fixed in
13 / 08.

## 2026-10-12 — RB5 ready pose proposal for pick and place (not applied)

**Proposal A:** q = (180, -15, -90, -165, 90, 0) deg. Tool (flange -Y) points straight down.
Controller `calc_fk_tcp`: TCP (417.9, 110.7, 196.5) mm, (rx, ry, rz) = (90, 0, -90); DH model z
differs by 2 mm. Flange (377.9, 110.7, 381.6), elbow above the wrist.
- Margins: J5 90 deg from singular; shoulder P05 radius 393.8 mm -> margin 283.1 mm; elbow J3 90 deg
  from 0 / 180; J3 limit (+/-150 or +/-165 by model, manual 1.6) 60 deg left.
- Max joint motion per 1 mm of TCP move: x 0.168, y 0.152, z 0.141 deg (TCP and flange the same,
  orientation fixed). For comparison, the 21 start pose: x 0.318, y 0.140, z 0.440.
- J1 = 180 puts the tool on the base +x side; J1 only rotates the setup and does not change the
  margins. The physical +x direction is not checked yet (22).
- Alternative B (180, -15, -90, 15, -90, 0): same height, TCP x 559 mm, arm more stretched, shoulder
  margin 498.8 mm.
- Note: the current Simulation start pose has J3 = -160.6 deg, beyond +/-150 (so this RB5 has the
  +/-165 limit, or the value needs checking).

## 2026-10-12 — 19: --pose mode (all joints), shortest equivalent angles, RB5 joint limits

**Decision:** 19 gets `--pose J1..J6` (move_j of all joints, Simulation only) and `--speed` (default 10,
at most 30 deg/s; the controller scales it by the speed bar 0.1, so 30 -> 3 deg/s). For each joint the
target is the equivalent angle (target + k x 360) inside the joint limit closest to the current angle.
Per-joint change limit: 60 deg (--joint), 180 deg (--pose).

**Joint limits (evidence):** rb_cobot_docs manual 1.6 Robot Operating Range: J1, J2, J4, J5, J6
+/-360 deg, J3 "+/-165 / +/-150" (the page does not say which model has which). Appendix System
Specifications: "Joint Range +/-360 (Elbow: +/-165)", written for the E series. This robot is an
RB5-850 (N), so J3 uses the smaller +/-150 in 19. The 21 start pose had J3 = -160.6 deg, which hints
that this arm allows +/-165, but that is not confirmed. The controller's setup joint limits are not
read by any script.

**Ready pose A from the current Simulation pose (180, 1.1, -156.9, 140.8, -2.5, 43.9):** target
(180, -15, -90, 195, 90, 0): J4 195 instead of -165 (54 deg instead of 306 deg). Largest change J5
92.5 deg, about 31 s at 3 deg/s.

## 2026-10-12 — 21: cylinder range; range excess was being dropped (also in 13)

**Decision:** the box range (+/-150 mm per axis) is replaced by a cylinder piece about the base z axis:
r 300..650 mm, z = start z +/-150 mm, azimuth = start azimuth +/-90 deg (Simulation defaults; before
Real, the z lower bound is set from the table height and the gripper length). A target outside is
clamped in r, z and azimuth separately. The script refuses to start if the TCP radius is outside.

**Bug found and fixed:** the range clamp was applied to the new target only. While the target sat at
the boundary, desired kept moving out, desired - target grew, and the speed limit cut it and B dropped
it, so the range excess was dropped after all (fake test: the hand came back 4 mm instead of 82 mm
before the robot moved inward, and the final position was off). Now desired is clamped into the
range first, then the speed / joint limits and B apply. Fake test after the fix: inward motion starts
after 84 mm of hand return (82 mm + one step) and the final position equals the start. **13 has the
same structure** (range clamp after the speed limit), so its "range excess is not dropped" note is
wrong in effect. Not fixed in 13.

**Singularity margins over the range** (pose A orientation, tool down, DH model, 120 grid points,
IK walked from pose A): J5 stays at 90 deg; shoulder margin >= 151 mm (at r 300); |J3| <= 124 deg.
New: the **elbow singularity** (arm stretched, J3 -> 0): at r >= 550 J3 comes within 17-20 deg of
straight and the joint motion rises to 0.65-0.72 deg/mm (0.19-0.31 deg/mm at r 300-500). At z +150
the model IK did not converge for r 600-650 (out of reach or nearly stretched). Decision on r_max or
an elbow watch is pending.

## 2026-10-12 — 21: three singularities watched, r_max 500 mm, C mode (catch-up up to a lag)

**The three RB5 singularities and how 21 watches them** (all on the joint angles predicted by the DH
model for the next step, q + J^-1 dx; slow = warning and the speed limit falls linearly to 0.25x;
stop = the step is not sent and servo mode ends normally; start needs the slow margin):

| singularity | condition | measure | slow / stop | evidence |
|---|---|---|---|---|
| wrist | J4 and J6 axes line up | J5 distance from 0 / 180 deg | 20 / 10 deg | Rainbow IK divides by sin(th5); 18 rz test |
| shoulder | wrist point P05 on the cylinder of radius d4 about the J1 axis | P05 horizontal radius - d4 | 50 / 20 mm | Rainbow IK acos(d4 / r); 094517 end (14 deg/mm) |
| elbow | arm stretched (J3 -> 0; also folded, J3 -> 180) | J3 distance from 0 / 180 deg | 20 / 10 deg | cylinder grid: 0.65-0.72 deg/mm at r >= 550 |

The joint speed limit (1 deg/cycle predicted) stays as the last guard for anything not covered.

**r_max 500 mm** (was 650) for the Simulation cylinder: over r 300-500 the grid has <= 0.31 deg/mm and
the elbow >= 42 deg from straight in the ready pose. Fake test from pose A with the range off: elbow
slow from 19.3 deg (r 681), stop at 9.7 deg predicted (r 690).

**Bug fix recap:** desired is clamped into the range before the speed limit (entry above); 13 still has
the old order.

**C mode:** `--max-lag-mm L` (0..50, default 0 = B). The part cut by the speed / joint / singularity
limits is kept while the lag (clamped desired - target) is at most L; only the part beyond L is
dropped (leader start shifted). Release drops the lag. Range excess is never dropped (it is outside
the lag because desired is clamped first). Fake tests at 1 mm/cycle:

| test | B | C (30 mm) |
|---|---|---|
| hand 100 mm at 5 mm/cycle, then still | robot 20 mm, stops at once | robot 50 mm, moves 30 mm in 30 cycles (0.6 s) after the hand stops |
| slow hand 50 mm | 50 mm | 50 mm |
| release right after the fast move | stays at 20 mm | stays at 20 mm (lag dropped) |
| fast down 100 / slow up 100 (hand net 0) | robot net +80 mm | robot net +50 mm |

Correction to the C plan: C does not bound the hand-robot offset to 30 mm. The offset grows by what
is dropped (hand travel beyond the speed-limited travel + L); C only reduces it by L per fast move.

**Side effect seen in the fake tests:** the old 21 start pose (J3 = -160.6) is 19.4 deg from folded,
so the elbow watch slows it and the start check would refuse it. Use the ready pose A.

## 2026-10-12 — 21: default is C with a 30 mm lag

**Decision:** `--max-lag-mm` defaults to 30 (C). `--max-lag-mm 0` gives B.

**Reason (Simulation, 200 mm/s, user runs; hand speeds differ between runs, so not a controlled test):**

| run | mode | hand speed p50 | RB5 path / hand path | dropped / hand path | hand-robot offset at segment end (median / max) |
|---|---|---|---|---|---|
| 102624 | B | 246 mm/s | 44 % | 38 % | 83 / 233 mm |
| 102731 | C | 287 mm/s | 45 % | 26 % | 73 / 246 mm |
| 103206 | B | 93 mm/s | 57 % | 15 % | 11 / 126 mm |
| 103353 | C | 143 mm/s | 66 % | 8 % | 34 / 157 mm |

C drops less than B (38 -> 26 % at similar hand speed; 8 % for C even with a faster hand than the
second B run). The path ratio gain is small when the hand is above the speed limit most of the time,
because the 30 mm lag fills quickly. Robot motion while the hand was slower than 20 mm/s: C 11 mm,
B 14 mm (a 0.3 s hand stop was too rare to measure the after-motion directly). A controlled B / C
comparison was skipped by choice.

**Before Real:** C keeps moving up to the lag after the hand stops (fake test: 30 mm in 0.6 s at
50 mm/s). The lag length must be chosen again for the real robot.

**Also seen:** the range limit (r 300-500, z +/-150) was active in 28-56 % of engaged cycles, the
largest limiter; late cycles 0.6-5.2 %, all while engaged, after planning spikes up to 27 ms (cause
under test).

## 2026-10-12 — 21: cause of the late cycles (offline test)

**Test (no robot, no leader arm):** `plan_step` from pose A with a 5 mm step, 4 mm/cycle, C 30 mm,
1000-1500 calls.

| condition | p50 | p99 | max | > 12 ms |
|---|---|---|---|---|
| 20 ms cycle (sleep), numpy default threads | 7.3 ms | 16.8-18.3 ms | 21-26 ms | 53-71 / 1500 |
| 20 ms cycle (sleep), OPENBLAS/OMP/MKL threads = 1 | 7.3 ms | 16.0-16.1 ms | 21-23 ms | 60-64 / 1500 |
| no sleep (busy loop) | 2.05 ms | 3.3 ms | 5.5 ms | 0 / 1000 |

**Conclusion:** numpy (OpenBLAS) threads are not the cause. The same work takes 3.6x longer with
spikes when the loop sleeps between cycles. The PC runs the `powersave` CPU governor on an
i7-13700H (6 performance cores up to 4.8-5.0 GHz, 8 efficiency cores up to 3.7 GHz), so after each
sleep the core is likely clocked down or the thread lands on an efficiency core (not separated yet).
Also, `plan_step` computes the DH Jacobian at the same q four times (0.47 ms each).

**Options (not decided):** compute the Jacobian once per cycle (code only); pin the process to the
performance cores (`taskset`); switch the governor to `performance` while testing (PC setting, needs
sudo).

## 2026-10-12 — 21: one Jacobian per cycle; late-cycle cause is CPU power saving + hybrid cores

**Cause (offline tests, recorded in the entry above, now with core pinning):** the PC uses the
`powersave` governor on an i7-13700H (performance cores cpu0-11 up to 4.8-5.0 GHz, efficiency cores
cpu12-19 up to 3.7 GHz). The same planning work is several times slower right after the 20 ms sleep
than in a busy loop (clock-down after idle), and slower again when it runs on an efficiency core.
numpy thread count has no effect.

**Change:** `plan_step` computes the DH Jacobian once per cycle (it was computed 4 times at the same q).
Fake-test results are unchanged (only differences come from the new C default in tests that do not
set the mode).

**Offline planning time after the change (20 ms cycle with sleep, 1000 calls):**

| condition | p50 | p99 | max | > 12 ms |
|---|---|---|---|---|
| before (4 Jacobians), default cores | 7.3 ms | 16-18 ms | 21-26 ms | 53-71 / 1500 |
| one Jacobian, default cores | 2.4-3.6 ms | 7.9-9.0 ms | 8.9-11.5 ms | 0 |
| one Jacobian, `taskset -c 0-11` (performance cores) | 3.3-3.5 ms | 5.1-6.3 ms | 5.8-8.0 ms | 0 |
| one Jacobian, `taskset -c 12-19` (efficiency cores) | 5.8-6.0 ms | 7.8-9.7 ms | 8.5-13.6 ms | 0-1 |
| one Jacobian, busy loop (no sleep) | 0.85 ms | 1.7-2.8 ms | 2.4-4.0 ms | 0 |

With the leader read (p99 about 7 ms) the expected work p99 is about 16 ms (default cores) or about
13 ms (performance cores) against the 20 ms period. Governor change (`performance`): user's decision
after a Simulation run with these changes.

## 2026-10-12 — 21: --scale 0.5 Simulation result (104723, C 30 mm, 200 mm/s)

| | C runs at scale 1 (102731 / 103353) | scale 0.5 (104723) |
|---|---|---|
| speed limited | 74 % / 37 % | 14 % |
| joint speed limited | 35 % / 20 % | 9 % |
| range limited | 28 % / 56 % | 38 % (desired r < 300: 33 %, z outside: 18 %, r > 500: 1 %) |
| RB5 path / hand path | 45 % / 66 % | 39 % = 78 % of the scaled hand path (0.5 x hand) |
| dropped / hand path | 26 % / 8 % | 0 % |
| offset at segment end (vs scaled hand) | - | median 13 mm, all of it the range residual |

At scale 0.5 almost nothing is dropped; what is lost is the range limit, mostly the inner radius
(the hand pulled toward the base) and z.

## 2026-10-12 — Ready pose: wrist folded inward in A (user); proposal C with the wrist outside

**User report:** in A (J4 = 195) the wrist folds in toward the base, which makes picking hard; the
end effector should face outward.

**Proposal C (not applied):** q = (180, 4, -111, 17, -90, 0), the J5 = -90 branch (J2 + J3 + J4 = -90
keeps the tool down). Controller `calc_fk_tcp`: TCP (415.9, 110.7, 196.8), (rx, ry, rz) = (90, 0, 90),
the same place as A. The wrist point P05 is outside the TCP (radius 469 vs 430; in A 394 vs 432).
Margins: J5 90 deg, shoulder 358 mm, elbow 69 deg; 0.156 deg/mm (A: 0.168).
Over the cylinder (75 points, r 300-500): shoulder >= 182 mm, elbow >= 31.7 deg, <= 0.26 deg/mm, but
at r 300, z -150 J3 reaches -148.3 deg, near the +/-150 limit used in 19.
The tool is rotated 180 deg about its axis compared with A (rz +90 vs -90); J6 = 180 would keep A's
hand rotation but moves the TCP by the 40 mm z offset.

## 2026-10-12 — Ready pose C applied (Simulation); runs pinned to performance cores

**Decision:** the RB5 ready pose for the leader position mapping is C = (180, 4, -111, 17, -90, 0) deg,
J6 = 0 (tool rotated 180 deg about its axis compared with A). Move with
`taskset -c 0-11 python scripts/19_move_joint_sim.py --pose 180 4 -111 17 -90 0 --speed 30`.
J1 = 180 is for Simulation; for Real it is set again from the table direction.

**Reason:** the grasp is top-down (approach from above). The tool must point down, and the wrist must
not fold in toward the base as in A (J4 = 195, wrist point inside the TCP radius), which got in the
way of picking. C keeps the tool down with the wrist outside (wrist point radius 469 mm vs TCP 430 mm),
at the same TCP place as A, with more shoulder margin (358 vs 283 mm) and slightly less joint motion
per mm (0.156 vs 0.168 deg/mm).

**Known limit of C:** low and inner targets bend J3 toward its limit (+/-150 used): at z = start - 150
and r = 300 the margin is 1.7 deg (grid over the cylinder, worst azimuth; the margin does depend on
the azimuth, see the next entry). Joint limit watch
and the inner radius are decided next.

**Performance cores:** from the next run, scripts are started with `taskset -c 0-11` (offline: planning
p99 5-6 ms instead of 8-9 ms). The CPU governor stays `powersave` unless the user decides otherwise.

## 2026-10-12 — 21: joint limit watch, r_min 375 mm; pick positions r 400-500 mm recommended

**Joint limit watch:** margin = limit - |predicted joint| with J3 +/-150 (until the user checks +/-150
vs +/-165 on the teach pendant), other joints +/-360. Below 20 deg: warning and slower (0.25x at
10 deg); below 10 deg: stop; start needs >= 20 deg. Same scheme as the singularity watches.
**r_min 375 mm** (was 300) for the cylinder.

**Joint limit margin over the cylinder, pose C orientation (tool down, wrist outside), worst of
azimuth -90..+90 deg, DH model IK walked from C** [deg, the smallest is always J3]:

| z - start z \ r [mm] | 300 | 325 | 350 | 375 | 400 | 450 | 500 |
|---|---|---|---|---|---|---|---|
| +150 | 42.9 | 44.6 | 46.5 | 48.5 | 50.8 | 56.0 | 62.1 |
| +100 | 34.9 | 36.7 | 38.6 | 40.8 | 43.2 | 48.5 | 54.6 |
| +50 | 27.4 | 29.2 | 31.3 | 33.6 | 36.1 | 41.6 | 47.9 |
| 0 | 20.3 | 22.3 | 24.5 | 27.0 | 29.6 | 35.5 | 42.0 |
| -50 | 13.6 | 15.8 | 18.3 | 20.9 | 23.8 | 30.0 | 36.7 |
| -100 | 7.3 | 9.9 | 12.6 | 15.5 | 18.6 | 25.2 | 32.3 |
| -150 | 1.7 | 4.6 | 7.7 | 10.9 | 14.2 | 21.2 | 28.6 |

The margin depends on the azimuth (the tool orientation is fixed in the base frame, so it turns
relative to the arm): at r 375, z -150 it is 13.7 / 17.3 / 10.9 deg at azimuth -90 / 0 / +90 deg
(an earlier note said "independent of azimuth"; that was wrong, the table values are the worst case).

Other measures over r 400-500 in pose C (75-point grid): shoulder margin >= 281 mm, elbow >= 44 deg
from straight, <= 0.21 deg/mm.

**Recommendation: pick positions at r 400-500 mm.** In that band nothing stops over the whole z range;
at r >= 450 there is no joint limit slowdown either; at r 400 the slowdown starts below z - 100 mm.
r 375-400 is reachable but slows down for z <= -50 and may stop at z -150 (10.9 deg at azimuth +90).
If the teach pendant shows +/-165 for J3, every value above gains 15 deg.

**Fake tests:** from pose C with the range off, straight down: slowdown from margin 20.0 deg
(dz -279 mm), no stop. Default range, inward then down to r 375 / z -150 at azimuth 0: margin 17.4 deg,
38 slow cycles, matching the IK value 17.3. Old tests (old start pose, J3 -160.6, TCP r 367) give the
same results with the joint limit watch and r_min relaxed; with the real settings that pose is refused.

## 2026-10-12 — Object on the RB5 table (z 0): ready pose D, cylinder r 450-550 mm

**Decision:** objects are on the same table as the RB5 (TCP z 0 = base bottom). Ready pose
D = (182, -11.0, -110.1, 31.1, -90, -2) deg: controller `calc_fk_tcp` TCP (483.3, 129.0, 102.2) mm,
r 500.2, (rx, ry, rz) = (90, 0, 90): tool down, wrist outside, same tool rotation as C, 100 mm above
the object height. Margins at D: J3 limit 39.9 deg, elbow 70 deg, shoulder 428 mm, J5 90 deg.
Move: `taskset -c 0-11 python scripts/19_move_joint_sim.py --pose 182 -11.0 -110.1 31.1 -90 -2 --speed 30`.

**Cylinder:** r 450-550 mm (was 375-500), z = start z +/-150 (-50..252 mm from D), azimuth +/-90 deg.
J3 limit margin at the object height z 0 by r (worst azimuth, DH IK): r 450 18.5, 475 22.2, 500 26.1,
525 30.0, 550 34.1 deg; at z -50 and r 450 it is 16.4 deg (slow, no stop). With r_max 550 the
worst spot stays at the inner low corner; the outer side kept elbow >= 35 deg in the grid.

**Before Real (must do):** set the z lower bound again from the table height and the gripper length.
With D the current lower bound is z -50 mm, below the table surface (z 0); in Real that would drive the
tool into the table. The TCP offset itself (0, -187, -40) is still being checked by the lab.

## 2026-10-12 — 21 from ready pose D: depth reached; vertical-hand habit halves the backward drift

**Run 113617 (Simulation, scale 0.5, C 30 mm, 200 mm/s, taskset 0-11, user moved the hand vertically):**
- Deepest TCP z -48.0 mm (start 102.2, dz -150.2): the z lower bound, below the object height z 0.
  The first descent already reached z -19. The depth goal is met.
- At the deepest point only the range limit was active: z lower bound and r_min 450 (desired r < 450
  in 100 % of that segment). Joint limit margin >= 23.6 deg, elbow >= 53.6 deg, no singularity or
  speed limits.
- Hand |dx| / |dz| over the 10 descents of more than 30 mm: 0.45-0.9 (mostly about 0.5). The previous
  run (112319, no habit) was mostly >= 1 (up to 2.3). The hand still drifts toward the body, so the
  robot is pushed to r_min 450 during 6 of 10 descents; z keeps going down because r and z are clamped
  separately.
- Late cycles 0; work p99 8.8 ms, max 11.0 ms.

## 2026-10-12 — Leader handle point instead of the wrist center: small effect, not applied

**Offline analysis (logged leader joints, no robot, no code change):** handle point = Link_6R origin +
R6 (0, 0, -135) mm, taken from the URDF Link_6R inertial origin (0.00089, -0.00028, -0.13533) m (the
handle's center of mass; the URDF has no handle frame). Wrist FK matches the logged leader position
within 0.005 mm.

| run | descents (> 30 mm) | median hand \|dx\|/\|dz\|, wrist center | handle point |
|---|---|---|---|
| 112319 (no habit) | 20 | 0.70 | 0.71 |
| 113617 (vertical habit) | 10 | 0.59 | 0.45 |

The handle tilted 30-130 deg from straight down and turned by 15-55 deg during the descents, so the
handle point also picks up wrist rotation (its dz is 1.1-1.6x the wrist dz). Effect: none without
the habit, about 25 % with it. Decision: keep the wrist center; next step is orientation (yaw) mapping.

## 2026-10-12 — Yaw mapping: RB5 yaw change = leader J6 change (method B)

**Decision:** for top-down grasping only the rotation about the base z axis is mapped. RB5 yaw =
start + (leader J6 now - leader J6 at the clutch press), applied as rz in move_servo_l (rx, ry fixed,
the tool keeps pointing down; servo_l rotates about the TCP, 18). Limits: 90 deg/s (1.8 deg/cycle),
+/-90 deg around the start rz, lead 3.6 deg, C lag 15 deg (B with `--max-lag-mm 0`). The joint
predictions (singularities, joint limits, joint speed) include the yaw. `--no-yaw` keeps the start
orientation.

**Reason (offline, logged leader joints of 112319 and 113617, no robot):**

"Fake yaw" at the lowest point of each descent (> 30 mm, no twist intended):

| run | (A) twist about base z of R_now R_start^T | (B) leader J6 change |
|---|---|---|
| 112319 (20 descents) | median 6.7, max 28.0 deg | median 3.0, max 26.5 deg |
| 113617 (10 descents) | median 4.1, max 64.1 deg | median 0.4, max 10.6 deg |

|A - B| by handle tilt from straight down (all engaged samples, both runs):

| tilt | samples | median | p90 | max |
|---|---|---|---|---|
| 0-30 deg | 390 | 2.5 | 5.8 | 9.1 |
| 30-60 deg | 1244 | 2.5 | 10.5 | 28.2 |
| 60-90 deg | 1515 | 3.4 | 23.1 | 67.3 |
| 90-120 deg | 893 | 4.8 | 28.2 | 57.5 |
| > 120 deg | 134 | 9.9 | 23.2 | 45.1 |

The handle was tilted 30-120 deg in 86 % of the samples (held nearly horizontal). B is quieter in the
descents; A follows the true rotation about vertical but picks up wrist bending (e.g. -64 deg when J4
and J5 moved -26 / +59 deg). With the handle near vertical the two agree. Accepted trade-off of B: a
J6 twist with a tilted handle is a rotation about a non-vertical axis but still drives the RB5 yaw;
"twist the handle = turn the gripper" is the operating rule.

**Fake tests (DH fake robot following position and orientation, scale 0.5, ready pose D):** twist
+/-45 deg -> target and TCP yaw +/-45.00 deg, position change 0.0005 mm; descend 100 mm without twist
-> yaw 0.0000; pick mimic (down + 30 deg) -> yaw 30.00 at the bottom and after returning; fast twist
90 deg in 10 cycles -> C 10.8 then 25.8 (+15 lag), B 10.8; range +120 -> 90.00, back to 60.00;
release freezes yaw; `--no-yaw` 0; J6 crossing +/-180 -> +20.00. Note: the yaw step was 1.08 deg/cycle,
not 1.8: at D a yaw step is mostly J6 (0.92 deg per deg), so the joint speed limit (1 deg/cycle
predicted) caps the yaw at about 54 deg/s.

## 2026-10-12 — Yaw mapping Simulation result (115603, ready pose D, scale 0.5, C)

Segments classified from the data (no time marks given): twist only (|dJ6| > 15 deg, hand < 40 mm),
descend (hand dz < -30 mm, |dJ6| < 10 deg), combined.

- **Twist only (3 segments, handle tilt 12-17 deg):** RB5 yaw followed the leader J6 within the limits:
  +119.1 deg -> +90.0 (range limit), -149.1 -> -108.7, +32.1 -> +3.7 (back-and-forth twists faster than
  the yaw limit; the part beyond the 15 deg lag was dropped). TCP moved 1.6-7.6 mm, which is the
  hand's own motion x 0.5. The true twist about vertical (A) had the same sign as dJ6 (106 / -134 /
  33 deg), so +J6 is counterclockwise seen from above, the same as +rz on the RB5.
- **Descend without twist (6 segments):** yaw change 0.0-2.5 deg, max 3.1 deg during the segment
  (A would have given 7-14 deg). Method B works as intended here.
- **Combined (2 segments):** dJ6 -20.7 / +30.5 deg -> yaw -20.65 / +30.50 deg while moving 41-53 mm.
- Yaw tracking: |target - tcp| yaw p99 1.09 deg. Late cycles 0, work p99 9.0 ms.
- Speed: leader J6 p50 44, p90 75, max 92 deg/s during twisting; RB5 yaw max 48 deg/s because the
  joint speed limit (1 deg/cycle predicted for any joint) caps J6. Yaw speed limited 32 %, joint speed
  limited 37 % of engaged cycles; 110 deg of yaw dropped in total.

## 2026-10-12 — 21: joint speed limit per joint (Simulation)

**Decision:** the predicted joint motion limit is per joint: J1-J3 1 deg/cycle (50 deg/s), wrist J4-J6
2 deg/cycle (100 deg/s), Simulation only. The step (position and yaw together) is scaled by the
smallest factor over the joints that exceed their limit. **Real values are to be set later from the
RB5 specification** (the manual gives "Velocity Joint: Max 180 deg/s, Tool: 1 m/s" for the E series;
per-joint values for this RB5-850 (N) still have to be found).

**Reason:** in 115603 the RB5 yaw reached only 48 deg/s because a yaw step at ready pose D is mostly J6
(0.92 deg per deg) and the common 1 deg/cycle limit capped it. Fake test after the change: fast twist
-> 1.80 deg/cycle (the 90 deg/s yaw limit), C 18.0 then 33.0 deg (+15 lag), B 18.0. Old fake tests
unchanged.

## 2026-10-12 — Gripper signal from the leader trigger: rules

**Signal:** grip_cmd = clip(trigger / 1000, 0, 1) (Rainbow example 35 lines 1173-1176 use trigger / 1000
as the gripper target). Open / closed state with hysteresis: above 0.6 one state, below 0.4 the other,
in between the previous state is kept. Trigger = bits 16-31 of the tool's raw value at address 132
(rby1-sdk `dynamixel_bus.cpp:57`), read together with the clutch button.

**Measured range (user, script 12):** the trigger is continuous: 0 = fully closed, the value rises as it
is opened, 1000 = fully open. Logged data so far: 115603 had only 0 and 1000; earlier runs only 0.
The signal is named `grip_open` (0 closed .. 1 open) to keep the direction explicit; state open above
0.6, closed below 0.4.

**Implemented in 21 (logging only):** columns `trigger_raw`, `grip_open`, `grip_state_open`,
`tool_read_failed`; events on state changes; status line. Fake tests: hysteresis (650 -> open,
450 stays open, 350 -> closed), 4 failed reads keep the value with a warning at the 3rd, state changes
while the clutch is released and while engaged.

**The gripper works independently of the clutch:** the gripper can be closed or opened while the robot
holds its position.

**Failure rule - the two signals fail differently on purpose:**
- Gripper: on a failed tool read, keep the previous value. Reason: an object in the gripper must not be
  dropped because of a lost packet.
- Clutch: after 3 failed reads in a row, treat it as released. Reason: motion must stop when the input
  is not known.

## 2026-10-12 — Teleop cleanup: teleop.py + teleop/ modules + configs + tests

**Decision:** the experiment scripts stay as they are. The teleop is `teleop.py` with modules in
`teleop/` (settings, kinematics, safety, mapping, leader, rb5, preflight, loop, log), settings in
`configs/sim.yaml` (values tested with 21) and `configs/real.yaml` (conservative, TBD values as null:
z lower bound, expected TCP offset, ready pose), tests with `unittest` in `tests/` (no new packages).
Simulation is the default; `--real` needs a valid real.yaml and typing REAL. At start the teleop
checks the leader (latency, ping), the RB5 (mode, not moving, errors, TCP offset), moves to the ready
pose after showing the change and a typed "yes" (skipped when already there; Real: clear-area note and
a warning for wrist turns over 90 deg), then checks the start pose.

**Behavior equal to 21:** plan_step gives the same results as 21 on 600 random cases (including near
the wrist singularity, the J3 limit and the elbow), and a combined script through both loops gives the
same targets, yaw and flags.

**Changes from 21:** gripper signal is `grip_close = trigger / 1000` (pulled = closed, untouched = open;
example 35 maps trigger / 1000 to a gripper target whose open / closed end depends on the gripper and
its GRIPPER_DIRECTION flag, so the source does not settle it). Optional absolute z floor
(`range.z_min_abs_mm`). Settings are checked (unknown keys, ranges, stop < slow, t1 = period).

**udev rule:** `udev/99-rby1-leader-arm.rules` sets the latency timer on the usb-serial device and
matches vendor / product / serial (FT62AJW7) on its parent USB device (checked with `udevadm info -a`;
`udevadm verify` passes). Not installed yet (needs sudo).

**Next feature candidate:** left handle button switches the scale (precise 0.5 / fast 1.5).

## 2026-10-12 — Teleop: ready pose from the real RB5, scale 1.5, J3 +/-165, range around the new pose

**User requests:** run with plain `python teleop.py` (no taskset), the RB5 should go farther (scale
about 1.5), and use the ready pose the user set on the real RB5. J3 limit confirmed +/-165 on the
teach pendant.

**Changes:**
- The process pins itself to `runtime.cpu_cores` ([0, 11], the performance cores) from the settings.
- `mapping.scale` 1.5 in sim.yaml.
- Ready pose (178.969, 8.514, -138.775, 46.535, -87.239, 56.265) deg, read from jnt_ang in Simulation
  mode (the encoder of the real arm). Controller FK: TCP (356.7, 84.6, 64.6) mm, r 366.6, rpy
  (84.2, 3.7, 145.2): the tool is 6.9 deg from straight down.
- J3 limit +/-165 in both settings files. With +/-150 the new pose had only 11.2 deg margin (refused).
- Range (sim.yaml) around the new pose: r 350-600 mm, z start -100 / +150 mm (about -37 .. 213),
  azimuth +/-90 deg. The z range is now asymmetric (`z_below_mm`, `z_above_mm`). DH grid with the
  pose's orientation over r 330-600, z start -150 .. +150, azimuth +/-90: elbow >= 25 deg, shoulder
  >= 198 mm, J5 >= 83 deg, all reachable; J3 (+/-165) margin is lowest at the inner low corner
  (10.2 deg at r 330, z -87). At the ready pose: J5 87.2 deg, shoulder 252.8 mm, elbow 41.2 deg,
  J3 26.2 deg.
- The teleop now checks the ready pose with the start checks (controller FK) before moving there.
- Note: with pose D the forward reach was only 50 mm (r 500 -> r_max 550); that, more than the scale,
  limited "how far forward" the RB5 went.

**Tests:** fixed to the pose D era values (same as 21) in `tests/fixtures.py`, plus a test that the
ready pose in sim.yaml passes the start checks with sim.yaml.

**Open:** the trigger read 1000 with the trigger apparently untouched (earlier logs: 0). With
`grip_close = trigger / 1000` that means "closed" at rest. To be checked by the user (released /
pulled values) before the direction is changed.

## 2026-10-12 — Gripper direction fixed: grip_close = 1 - trigger / 1000

**Fact (user):** the trigger reads 0 when it is closed (pulled) and 1000 when fully open (released).
This matches the 1000 read with the trigger untouched (dry run with the real leader arm). Earlier logs
had 0 at rest; the reason is not known.

**Decision:** `grip_close = 1 - trigger / 1000`: pulled = closed (1), released = open (0), as the user
wants. Hysteresis and the failure rule are unchanged (closed above 0.6, open below 0.4, keep the
value on a failed read). Replaces `grip_close = trigger / 1000` from the cleanup step 2.

## 2026-10-12 — Sim: speed 400 mm/s and lag 50 mm; Real: provisional values for a first test

**Why the RB5 stopped near x 500 (runs 130330 scale 1.5, 130757 scale 2.0):** r 600 (x about 590) was
reached only at the z upper bound (z 215); with z <= 100 the max x was 476 / 491 mm. The hand was
faster than 200 mm/s / scale in 50 % / 73 % of the engaged cycles (p50 134-172 mm/s), so C dropped
4.0 / 1.8 m of hand motion per run; one press moved the RB5 0.13-0.97 x the hand instead of 1.5-2.0.
Reaching forward also raised the leader wrist (desired z up to 370), clipped at the z upper bound.

**Sim change:** `mapping.speed_mm_s` 200 -> 400 (schema maximum raised to 400; Real stays capped at
50), `mapping.max_lag_mm` 30 -> 50 (schema maximum). The per-joint speed limit stays as the guard.
The user's own edit `mapping.scale: 2.0` is kept.

**Real (user wants to test now; table height, gripper length and TCP can come later):**
- `ready_pose.joints_deg` = the pose the user set on the real RB5, read again from jnt_ang:
  (178.969, 8.514, -138.775, 46.535, -87.239, 56.265); TCP (356.7, 84.6, 64.6), r 366.6.
- `range.z_min_abs_mm` 30.0 and `tcp.expected_offset_mm` (0, -187, -40) are PROVISIONAL placeholders:
  the floor keeps the TCP at z >= 30 mm, the TCP check refuses the start if the controller's TCP
  changes. Both are to be replaced (table height + gripper length + margin; the lab's TCP).
- Range r 340-420, z start -40 / +50 (floor 30 -> 30 .. 112.6 mm), azimuth +/-30; speeds unchanged
  (20 mm/s, yaw 15 deg/s, joints 0.2 deg/cycle = 10 deg/s, B). DH grid over the range (27 points):
  J5 >= 84.3 deg, shoulder >= 215.7 mm, elbow >= 34.0 deg, joint limit >= 19.0 deg (inner low corner,
  slow but no stop), <= 0.257 deg/mm (5.1 deg/s at 20 mm/s).
- Not done and not changed by the code: collision detection is Off and tool mass is 0 kg on the
  controller; move_servo_l has not been used on the real robot before (18 was Simulation).

## 2026-10-12 — Sim: r_max 600 -> 800 mm (reach x 750)

**Why x stayed below 600 at scale 2.0:** the cylinder r_max 600; the TCP azimuth is 13.4 deg, so
x = r cos(13.4) <= 584. x 750 needs r 771 at that azimuth.

**Check (DH model, the ready pose orientation kept, worst of azimuth -30 / 0 / +30, z -37 .. 213):**
every point up to r 850 is reachable. Elbow distance from straight: r 600 >= 70, r 700 >= 70, r 750
>= 58, r 800 >= 43, r 850 >= 19.5 deg (only at the top z 213: slowdown). Joint limit margins grow with
r (>= 55 deg). Decision: r_max 800 (no slowdown anywhere in the range).

## 2026-10-12 — Real: range as wide as Simulation (user request)

**Decision:** real.yaml range r 350-800 mm, z start -100 / +150 mm, azimuth +/-90 deg (same as
sim.yaml), keeping the PROVISIONAL floor z >= 30 mm. Was r 340-420, z -40 / +50, azimuth +/-30.

**Check (DH model, real ready pose orientation, 36 points over r 350-800, z 30 .. 214.6, azimuth
-90 / 0 / +90):** all reachable; J5 >= 83.4 deg, shoulder >= 218.1 mm, elbow >= 35.5 deg, joint limit
>= 20.5 deg (inner low corner r 350, z 30: just above the slow threshold); <= 0.393 deg/mm, i.e.
15.7 deg/s at 40 mm/s (joint limit 20 deg/s).

**Risk noted:** collision detection is still Off and the TCP / floor values are provisional; the wider
range lets the real arm sweep +/-90 deg of azimuth and reach r 800.

## 2026-10-12 — Real ready-pose move 3 deg/s

**Decision:** real.yaml `ready_pose.move_speed_deg_s` 10 -> 30 and the Real cap (REAL_MAX) 10 -> 30, so the
move_j to the ready pose runs at 30 x speed bar 0.1 = 3 deg/s (same as Simulation; was 1 deg/s, too
slow to wait for). The speed bar stays 0.1 (CLAUDE.md). The change display, the typed "yes" and the
wrist-turn warning before the move are unchanged.

## 2026-10-12 — Real runs 132310 / 133219; faster Real settings; measured joints logged

**Runs (real robot, scale 1.0, 40 mm/s, B):** 29 s and 122 s, no late cycles (send interval max 26 ms,
work p99 9.2 ms), no singularity or joint limit slowdown (J5 >= 84, shoulder >= 232 mm, elbow >= 41,
J3 >= 26 deg). The wide-range run reached x 330-661, r 350-663, z 65-215, yaw -27..+16 deg. The robot
target speed sat at 40 mm/s (speed limited 83-88 %, joint speed limited 27-34 %) while the hand moved
at p50 126-184 mm/s: the RB5 moved 16 % of the hand path and B dropped 1.9 / 8.7 m. Tool read failures
2-3 %. The trigger read 0 for the whole of both runs (gripper "closed" 100 %), unlike the 837-1000 read
at rest before: to be checked.

**Decision (user):** real.yaml `speed_mm_s` 50 (Real max), `joint_speed_deg_per_cycle` 0.5 (Real max,
25 deg/s; 50 mm/s x 0.393 deg/mm = 20 deg/s), `max_lag_mm` 20 (C: up to 20 mm, 0.4 s, after the hand
stops; yaw stays B since yaw.max_lag_deg is 0).

**Logging:** the state reader keeps the measured joints (jnt_ang); the CSV adds `jnt_ang_0..5`, their
DH-model TCP `meas_tcp_x/y/z` (absolute about 2 mm off the controller FK) and `target_minus_meas_mm`.
The logged tcp_ref / jnt_ref are the controller's reference (command) values; jnt_ang shows how the
real arm actually followed. In Simulation jnt_ang stays at the real arm's pose.

## 2026-10-12 — Real run 133743 (50 mm/s, C 20 mm) and z upper bound +200

**Run 133743 (66 s):** first run with the measured joints. jnt_ref - jnt_ang <= 0.145 deg (p50 0.03);
the measured TCP follows the target about 80 ms late (4 cycles; error 3.9 mm mean at 0 delay, 2.0 mm
when shifted, 2.0 mm while holding = the DH model offset). Measured joint speeds J1-J4 10-12 deg/s,
J6 24 deg/s. RB5 path 32 % of the hand path (16 % before), dropped 2.6 m, speed limited 91 %, joint
speed limited 10 %, range limited 52 % - all of it the z upper bound (start + 150 = 215 mm; reaching
forward raises the leader wrist). No late cycles, no singularity / joint limit slowdown. Tool read
failures 5.2 % (higher than before). The tool raw value was only 256 / 0: the trigger bits stayed 0.

**Decision:** real.yaml `range.z_above_mm` 150 -> 200 (z up to 264.6). DH grid with the real ready
pose orientation, r 350-800, azimuth +/-90: at +200 every point is reachable, elbow >= 25 deg (at r
800), J5 >= 84 deg, shoulder >= 218 mm, joint limit >= 48 deg; at +250 r 800 is out of reach.

## 2026-10-12 — Sim z upper bound +200 (same as Real)

sim.yaml `range.z_above_mm` 150 -> 200, same as real.yaml (user). Same ready pose and orientation as
Real, so the Real grid applies: reachable over r 350-800 at +200, elbow >= 25 deg at r 800.

## 2026-10-12 — Real run 134902 and TCP speed 100 mm/s

**Run 134902 (206 s, z top +200):** reached x 253-680, r 350-725, z 63-265, yaw -32..+22 deg. Range
limited 29 % (z top 19 %, r < 350 9 %; was 52 %). Speed limited 94 %, RB5 path 31 % of the hand path
(hand p50 120 mm/s). Measured tracking as before: jnt_ref - jnt_ang <= 0.139 deg, measured TCP about
80 ms behind the target. No late cycles, no singularity / joint limit slowdown. Tool read failures
0.8 %. Trigger bits 0 for the whole run again (third Real run).

**Decision (user):** Real TCP speed cap (REAL_MAX) 50 -> 100 mm/s and real.yaml `speed_mm_s` 100. The
joint speed limit stays 0.5 deg/cycle (25 deg/s): near the ready pose (about 0.22 deg/mm) 100 mm/s
fits, at the far / low corners (up to 0.393 deg/mm) the joint limit caps the TCP at about 64 mm/s.
With the 80 ms delay the real arm will trail the target by about 8 mm at 100 mm/s.

## 2026-10-12 — Real run 135230 (100 mm/s) and ready move 5 deg/s

**Run 135230 (68 s):** RB5 target at 100 mm/s in 63 % of the engaged cycles (60-95 mm/s in 21 %);
RB5 path 50 % of the hand path (31 % at 50 mm/s), dropped 1.9 m; speed limited 89 %, joint speed
limited 32 % (the 25 deg/s joint limit at far poses), range limited 24 % (all z top +200). Reached x
357-705, r 367-710, z 65-265. Measured: jnt_ref - jnt_ang <= 0.149 deg; measured TCP about 80 ms
behind (6.9 mm p50 while moving at 100 mm/s, 2.1 mm after shifting by 80 ms). Measured joint speeds up
to 23.4 deg/s. No late cycles, no singularity / joint limit slowdown; tool read failures 0.9 %. The
measured TCP speed shows peaks up to 171 mm/s (p99 125), probably from sampling (jnt_ang read every
20 ms from a 10 ms data stream), not checked. Trigger bits 0 again (4th Real run).

**Decision (user):** ready-pose move_j speed argument 30 -> 50 (5 deg/s with speed bar 0.1); schema
maximum 30 -> 60 and Real cap 30 -> 50. Speed bar unchanged.

## PENDING

- **14: RB-Y1 self-collision check during teleop.** Example 35 blocks the arm command when the
  nearest link distance is below 2 cm (`detect_collisions_or_nearest_links`, lines 1226-1240).
  Skipped for now (simulator only); needed before any real RB-Y1 use. Measure its cost in the 50 Hz loop.

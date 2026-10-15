# Decisions

## Contents

Entries below are in date order; this list groups them by topic. Open items: [PENDING](#pending).

**Leader arm (RB-Y1 scripts 14, 20, leader signal)**
- [2026-10-10 — 14: align the leader arm to the RB-Y1 READY pose at startup](#2026-10-10--14-align-the-leader-arm-to-the-rb-y1-ready-pose-at-startup)
- [2026-10-10 — 14: one dynamixel_sdk PortHandler for the leader arm](#2026-10-10--14-one-dynamixel_sdk-porthandler-for-the-leader-arm)
- [2026-10-10 — 14: leader arm torque constant chosen per ID from the actual model (differs from Rainbow)](#2026-10-10--14-leader-arm-torque-constant-chosen-per-id-from-the-actual-model-differs-from-rainbow)
- [2026-10-10 — 14: leader arm holds its pose while the clutch is released (rule changed)](#2026-10-10--14-leader-arm-holds-its-pose-while-the-clutch-is-released-rule-changed)
- [2026-10-10 — 14: leader wrist joints always torque off after the alignment](#2026-10-10--14-leader-wrist-joints-always-torque-off-after-the-alignment)
- [2026-10-11 — 14: leader thread + Fast Sync Read; --period 0.02 (default) / 0.01; C++ not needed](#2026-10-11--14-leader-thread--fast-sync-read---period-002-default--001-c-not-needed)
- [2026-10-12 — 20: leader arm FK with rby1_sdk.dynamics; tip = wrist center](#2026-10-12--20-leader-arm-fk-with-rby1_sdkdynamics-tip--wrist-center)
- [2026-10-12 — Leader handle point instead of the wrist center: small effect, not applied](#2026-10-12--leader-handle-point-instead-of-the-wrist-center-small-effect-not-applied)
- [2026-10-12 — Leader deadband (hysteresis) against the wrist shake](#2026-10-12--leader-deadband-hysteresis-against-the-wrist-shake)

**RB5 API probes (17-19)**
- [2026-10-12 — 17/18: TCP pose sources and the move_servo_l probe](#2026-10-12--1718-tcp-pose-sources-and-the-move_servo_l-probe)
- [2026-10-12 — 18 results: move_servo_l convention confirmed; wrist singularity near J5 = 0](#2026-10-12--18-results-move_servo_l-convention-confirmed-wrist-singularity-near-j5--0)
- [2026-10-12 — 18 rz away from J5 = 0: wrist singularity hypothesis supported](#2026-10-12--18-rz-away-from-j5--0-wrist-singularity-hypothesis-supported)
- [2026-10-12 — 19: move_j speed is scaled by the speed bar](#2026-10-12--19-move_j-speed-is-scaled-by-the-speed-bar)
- [2026-10-12 — 19: --pose mode (all joints), shortest equivalent angles, RB5 joint limits](#2026-10-12--19---pose-mode-all-joints-shortest-equivalent-angles-rb5-joint-limits)

**Ready pose**
- [2026-10-12 — RB5 ready pose proposal for pick and place (not applied)](#2026-10-12--rb5-ready-pose-proposal-for-pick-and-place-not-applied)
- [2026-10-12 — Ready pose: wrist folded inward in A (user); proposal C with the wrist outside](#2026-10-12--ready-pose-wrist-folded-inward-in-a-user-proposal-c-with-the-wrist-outside)
- [2026-10-12 — Ready pose C applied (Simulation); runs pinned to performance cores](#2026-10-12--ready-pose-c-applied-simulation-runs-pinned-to-performance-cores)
- [2026-10-12 — Object on the RB5 table (z 0): ready pose D, cylinder r 450-550 mm](#2026-10-12--object-on-the-rb5-table-z-0-ready-pose-d-cylinder-r-450-550-mm)
- [2026-10-12 — Teleop: ready pose from the real RB5, scale 1.5, J3 +/-165, range around the new pose](#2026-10-12--teleop-ready-pose-from-the-real-rb5-scale-15-j3--165-range-around-the-new-pose)
- [2026-10-12 — Real ready-pose move 3 deg/s](#2026-10-12--real-ready-pose-move-3-degs)

**Mapping, range and safety limits (21 -> teleop)**
- [2026-10-12 — 21: leader -> RB5 position mapping, first Simulation results](#2026-10-12--21-leader---rb5-position-mapping-first-simulation-results)
- [2026-10-12 — 21: cylinder range; range excess was being dropped (also in 13)](#2026-10-12--21-cylinder-range-range-excess-was-being-dropped-also-in-13)
- [2026-10-12 — 21: three singularities watched, r_max 500 mm, C mode (catch-up up to a lag)](#2026-10-12--21-three-singularities-watched-r_max-500-mm-c-mode-catch-up-up-to-a-lag)
- [2026-10-12 — 21: default is C with a 30 mm lag](#2026-10-12--21-default-is-c-with-a-30-mm-lag)
- [2026-10-12 — 21: --scale 0.5 Simulation result (104723, C 30 mm, 200 mm/s)](#2026-10-12--21---scale-05-simulation-result-104723-c-30-mm-200-mms)
- [2026-10-12 — 21: joint limit watch, r_min 375 mm; pick positions r 400-500 mm recommended](#2026-10-12--21-joint-limit-watch-r_min-375-mm-pick-positions-r-400-500-mm-recommended)
- [2026-10-12 — 21 from ready pose D: depth reached; vertical-hand habit halves the backward drift](#2026-10-12--21-from-ready-pose-d-depth-reached-vertical-hand-habit-halves-the-backward-drift)
- [2026-10-12 — 21: joint speed limit per joint (Simulation)](#2026-10-12--21-joint-speed-limit-per-joint-simulation)
- [2026-10-12 — Sim: r_max 600 -> 800 mm (reach x 750)](#2026-10-12--sim-r_max-600---800-mm-reach-x-750)
- [2026-10-12 — Real: range as wide as Simulation (user request)](#2026-10-12--real-range-as-wide-as-simulation-user-request)
- [2026-10-12 — Sim z upper bound +200 (same as Real)](#2026-10-12--sim-z-upper-bound-200-same-as-real)
- [2026-10-12 — z upper bound +400 with an outer radius that falls with height (Sim and Real)](#2026-10-12--z-upper-bound-400-with-an-outer-radius-that-falls-with-height-sim-and-real)

**Timing and CPU**
- [2026-10-12 — 21: cause of the late cycles (offline test)](#2026-10-12--21-cause-of-the-late-cycles-offline-test)
- [2026-10-12 — 21: one Jacobian per cycle; late-cycle cause is CPU power saving + hybrid cores](#2026-10-12--21-one-jacobian-per-cycle-late-cycle-cause-is-cpu-power-saving--hybrid-cores)

**Yaw and gripper**
- [2026-10-12 — Yaw mapping: RB5 yaw change = leader J6 change (method B)](#2026-10-12--yaw-mapping-rb5-yaw-change--leader-j6-change-method-b)
- [2026-10-12 — Yaw mapping Simulation result (115603, ready pose D, scale 0.5, C)](#2026-10-12--yaw-mapping-simulation-result-115603-ready-pose-d-scale-05-c)
- [2026-10-12 — Gripper signal from the leader trigger: rules](#2026-10-12--gripper-signal-from-the-leader-trigger-rules)
- [2026-10-12 — Gripper direction fixed: grip_close = 1 - trigger / 1000](#2026-10-12--gripper-direction-fixed-grip_close--1---trigger--1000)

**Teleop code and settings**
- [2026-10-12 — Teleop cleanup: teleop.py + teleop/ modules + configs + tests](#2026-10-12--teleop-cleanup-teleoppy--teleop-modules--configs--tests)
- [2026-10-12 — Sim: speed 400 mm/s and lag 50 mm; Real: provisional values for a first test](#2026-10-12--sim-speed-400-mms-and-lag-50-mm-real-provisional-values-for-a-first-test)

**Real runs (2026-10-12)**
- [2026-10-12 — Real runs 132310 / 133219; faster Real settings; measured joints logged](#2026-10-12--real-runs-132310--133219-faster-real-settings-measured-joints-logged)
- [2026-10-12 — Real run 133743 (50 mm/s, C 20 mm) and z upper bound +200](#2026-10-12--real-run-133743-50-mms-c-20-mm-and-z-upper-bound-200)
- [2026-10-12 — Real run 134902 and TCP speed 100 mm/s](#2026-10-12--real-run-134902-and-tcp-speed-100-mms)
- [2026-10-12 — Real run 135230 (100 mm/s) and ready move 5 deg/s](#2026-10-12--real-run-135230-100-mms-and-ready-move-5-degs)
- [2026-10-12 — Real joint speed limit 0.8 deg/cycle (40 deg/s)](#2026-10-12--real-joint-speed-limit-08-degcycle-40-degs)

**Real teleop speed (real-1)**
- [2026-10-16 — real-1: trial protocol for faster Real teleop](#2026-10-16--real-1-trial-protocol-for-faster-real-teleop)
- [2026-10-16 — Real speed limits to option B: TCP 200 mm/s, joints 1.2 deg/cycle (60 deg/s)](#2026-10-16--real-speed-limits-to-option-b-tcp-200-mms-joints-12-degcycle-60-degs)
- [2026-10-16 — Low-pass filter on the leader wrist center (tau 30 ms) against the shake at 200 mm/s](#2026-10-16--low-pass-filter-on-the-leader-wrist-center-tau-30-ms-against-the-shake-at-200-mms)
- [2026-10-16 — Ready pose moved to r 500, z 200 (Real and Sim); range kept in absolute terms](#2026-10-16--ready-pose-moved-to-r-500-z-200-real-and-sim-range-kept-in-absolute-terms)
- [2026-10-16 — Cylindrical mapping trial (Sim and Real): a hand swing about the leader base center turns J1](#2026-10-16--cylindrical-mapping-trial-sim-and-real-a-hand-swing-about-the-leader-base-center-turns-j1)
- [2026-10-16 — Cylindrical mapping reverted to cartesian after one Real run](#2026-10-16--cylindrical-mapping-reverted-to-cartesian-after-one-real-run)
- [2026-10-16 — Vertical mode on the left handle button (x, y fixed, only z)](#2026-10-16--vertical-mode-on-the-left-handle-button-x-y-fixed-only-z)
- [2026-10-16 — Vertical mode z x 2 (vertical_z_scale)](#2026-10-16--vertical-mode-z-x-2-vertical_z_scale)
- [2026-10-16 — TCP floor 30 -> -50 mm (nothing mounted on the flange)](#2026-10-16--tcp-floor-30----50-mm-nothing-mounted-on-the-flange)
- [2026-10-16 — Faster J1 (sideways 300 mm/s), RB5 J6 still (tool follows the base), vertical z x 1.5](#2026-10-16--faster-j1-sideways-300-mms-rb5-j6-still-tool-follows-the-base-vertical-z-x-15)
- [2026-10-16 — Left button = J1 rotate mode; azimuth range +/-120](#2026-10-16--left-button--j1-rotate-mode-azimuth-range--120)
- [2026-10-16 — Azimuth lock: J1 turns only in the rotate mode](#2026-10-16--azimuth-lock-j1-turns-only-in-the-rotate-mode)
- [2026-10-16 — Target acceleration limit 2.5 m/s^2 against the shake when the limits switch](#2026-10-16--target-acceleration-limit-25-ms2-against-the-shake-when-the-limits-switch)
- [2026-10-16 — Rotate mode: 15 deg catch-up, target kept on the circle, re-anchor at the stop point](#2026-10-16--rotate-mode-15-deg-catch-up-target-kept-on-the-circle-re-anchor-at-the-stop-point)
- [2026-10-16 — Range in absolute base-frame values; ready pose = the user's pose at r 450](#2026-10-16--range-in-absolute-base-frame-values-ready-pose--the-users-pose-at-r-450)
- [2026-10-16 — Joint-limit slow threshold 20 -> 15 deg](#2026-10-16--joint-limit-slow-threshold-20---15-deg)
- [2026-10-16 — Azimuth range widened on the minus side: -160 .. 133 deg](#2026-10-16--azimuth-range-widened-on-the-minus-side--160--133-deg)
- [2026-10-16 — J1 a bit faster (sideways 360 mm/s, J1 75 deg/s); acceleration limit moved to the last stage](#2026-10-16--j1-a-bit-faster-sideways-360-mms-j1-75-degs-acceleration-limit-moved-to-the-last-stage)

**Repository structure (2026-10-14)**
- [2026-10-14 — Leader arm URDF committed under assets/](#2026-10-14--leader-arm-urdf-committed-under-assets)
- [2026-10-14 — environment.yml for the rb5 env](#2026-10-14--environmentyml-for-the-rb5-env)
- [2026-10-14 — Teleop logs in data/teleop/](#2026-10-14--teleop-logs-in-datateleop)
- [2026-10-14 — Teleop entry point moved into the package: python -m teleop](#2026-10-14--teleop-entry-point-moved-into-the-package-python--m-teleop)
- [2026-10-14 — One robot file: configs/robot.yaml](#2026-10-14--one-robot-file-configsrobotyaml)
- [2026-10-14 — config.py moved to scripts/](#2026-10-14--configpy-moved-to-scripts)
- [2026-10-14 — Scripts 15, 20, 21 also use the assets/ URDF](#2026-10-14--scripts-15-20-21-also-use-the-assets-urdf)
- [2026-10-14 — udev rule moved to assets/udev/](#2026-10-14--udev-rule-moved-to-assetsudev)

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

## 2026-10-12 — Real joint speed limit 0.8 deg/cycle (40 deg/s)

**Decision (user):** real.yaml `joint_speed_deg_per_cycle` 0.5 -> 0.8 on all joints and the Real cap
0.5 -> 0.8. In 135230 the joint speed limit was active in 32 % of the engaged cycles (measured joints
up to 23.4 deg/s against 25). With 0.8, 100 mm/s fits everywhere in the range (worst 0.393 deg/mm x
100 = 39.3 deg/s). The RB5 specification for per-joint speeds is still to be found (manual: joint max
180 deg/s for the E series).

## 2026-10-12 — z upper bound +400 with an outer radius that falls with height (Sim and Real)

**Decision (user, option B):** both settings files: `range.z_above_mm` 200 -> 400 (about 463 mm) and a new
`range.r_max_by_z` table [[200, 800], [250, 775], [300, 725], [350, 700], [400, 650]] (start z + dz,
r_max). The cylinder is clamped in z first, then in r at that height; between table points r_max is
linear, below the first point it is the first value, above the last the last value. The low part of the
range is unchanged (r up to 800 below +200).

**Reason:** at +250 the arm could not reach r 800. Largest radius with every watch above its slow
threshold (DH grid with the ready pose orientation, azimuth -90 / 0 / +90): +200 800, +250 775, +300 725,
+350 700, +400 650, +450 600 mm. Option A (one smaller r_max for all heights) would have given up
forward reach at low height.

**Check of the sloped edge (9 heights from +200 to +400 incl. the interpolated ones, 3 azimuths):** all
reachable; J5 >= 83.4 deg, shoulder >= 518 mm, elbow >= 23.3 deg (at +250, r 775), joint limit >= 84 deg.
Tests: range clamp with the table, settings checks of the table (heights increasing, r within r_min..r_max).

**Real:** the real arm can now go up to about 463 mm; the space above has to be clear.

## 2026-10-12 — Leader deadband (hysteresis) against the wrist shake

**Observation (Real run 141108, user felt a slight motor shake):** the yaw target reversed direction in
18 % of its steps (position 3-6 %), and 79 of 81 reversals were exactly one leader encoder tick
(0.087-0.088 deg = 360 / 4096): the leader J6 flickered between two counts and the RB5 J6 followed.
Measured RB5 J6 had the largest fast wobble (p99 0.38 deg vs 0.11-0.19 for J1-J4). The log is at 50 Hz,
so vibration above 25 Hz cannot be seen in it.

**Decision (user):** a hysteresis on the leader values before the mapping (`LeaderFilter`): a new value is
taken only when it moved at least the deadband from the last taken value. `yaw.leader_deadband_deg`
0.15 (one tick ignored, two ticks pass), `mapping.leader_deadband_mm` 0.3. Both settings files. The CSV
adds `leader_used_x/y/z_mm` and `leader_j6_used_deg` (what the mapping used).

**Replay on 141108:** leader J6 reversals 81 -> 1; position reversals 71 / 197 / 127 -> 59 / 182 / 118
(x / y / z: mostly real hand tremor above 0.3 mm). Largest difference raw - used: 0.088 deg, 0.30 mm.
If the shake is still felt, next options: servo alpha < 1 (controller smoothing, adds delay) or a
low-pass on the position.

## 2026-10-14 — Leader arm URDF committed under assets/

**Problem:** `teleop.py` and `tests/test_core.py` read the leader URDF from `refs/rby1-sdk/`, which is
gitignored, so the teleop and the tests did not run from a fresh clone.

**Decision:** an unmodified copy at `assets/leader_arm/model.urdf` (rby1-sdk v0.10.0, Apache 2.0) with
`LICENSE`, `NOTICE` and a README (source, version, SHA-256). Only the URDF; the meshes are not used.
Scripts 15, 20 and 21 keep their `refs/` paths (kept as they were run).

**Rejected:** a git submodule of rby1-sdk (whole SDK for one 12 kB file); installing the rby1-sdk
Python package and reading its model (extra dependency, and the path inside the package is not
confirmed).

## 2026-10-14 — environment.yml for the rb5 env

**Decision:** `environment.yml` lists Python 3.10 and only the packages the code imports, pinned to
the versions in the lab PC's `rb5` env (rbpodo 0.16.14, dynamixel-sdk 4.1.0, numpy 2.2.6, PyYAML
6.0.3, matplotlib 3.10.9, evdev 1.9.3), all through pip. Checked: a new env built from the file
imports them all and runs the 43 tests (OK).

**Rejected:** `pip freeze` / `conda env export` of the whole env (it also lists the system ROS 2
packages and Jupyter, which the code does not use); unpinned versions (a later numpy or rbpodo could
change behavior without notice). rby1_sdk (scripts 14, 15, 20, env `rby1`) is only noted, not added.

## 2026-10-14 — Teleop logs in data/teleop/

**Decision:** `logging.dir: data/teleop` in both settings files; the 34 earlier teleop files
(`teleop_sim_*`, `teleop_real_*`, with their settings copies) were moved there. Script outputs stay in
`data/` (named by script number). `data/` is not committed. Demonstration data will get its own folder
once its format (e.g. LeRobot) is chosen.

## 2026-10-14 — Teleop entry point moved into the package: python -m teleop

**Problem:** `teleop.py` next to the `teleop/` package had the same name (confusing to read; Python
picks the package on import).

**Decision (user):** `teleop.py` -> `teleop/__main__.py` (git mv, history kept), relative imports,
`ROOT` is the repository root. Run from the repository root: `python -m teleop` (`--real`,
`--config` unchanged). README, teleop_guide and the package docstring updated; earlier entries in
this file still say `teleop.py`.

**Rejected:** renaming to `run_teleop.py` (still a second top-level name for the same thing).

## 2026-10-14 — One robot file: configs/robot.yaml

**Problem:** the robot IP was in `config.py`, `configs/sim.yaml` and `configs/real.yaml`.

**Decision (user):** `configs/robot.yaml` (ip, speed_bar) is the only place. `load_settings` adds it as
the `robot` section and checks it with the same schema (speed bar <= 0.1); a settings file that still
has a `robot` section is refused. `config.py` reads the same file and keeps `ROBOT_IP` and `SPEED`, so
the 14 scripts that import it are unchanged. Sim and Real now share one speed bar (both were 0.1).
Scripts 01 and 02 still have the IP written in them (kept as run).

**Rejected:** deleting `config.py` and editing the 14 scripts (they are kept as run); `config.py`
reading `sim.yaml` (the IP would still be in two settings files).

## 2026-10-14 — config.py moved to scripts/

Only the scripts use it (`import config`). In `scripts/` Python finds it without any change to the
scripts, because the folder of the script being run is on the import path. The repository root
keeps only folders, README and environment.yml. `scripts/README.md` now lists, per script, how to
run it, its options and what it moves.

## 2026-10-14 — Scripts 15, 20, 21 also use the assets/ URDF

Changes the earlier "scripts keep their refs/ paths" (user request): all code now reads
`assets/leader_arm/model.urdf`; `refs/` is reference only. The example 35 diff was regenerated.

## 2026-10-14 — udev rule moved to assets/udev/

`udev/99-rby1-leader-arm.rules` -> `assets/udev/` (user request: no single-file folder at the root).
Still not installed; the install commands in `docs/teleop_guide.md` use the new path.

## 2026-10-16 — real-1: trial protocol for faster Real teleop

**Problem (user):** Real teleop is too slow for demonstration collection; the operator drags the robot
with many clutch presses.

**Decision (user):** measure first, then change one setting at a time (`docs/real_trial_protocol.md`):
a fixed reach task P -> Q (gripper not connected), **3 practice trials before every setting (baseline
included) then 5 measured trials**, manual results in `data/teleop/trials.csv`, measures from the logs
with `scripts/23_trial_summary.py`. Baseline = the current real.yaml (scale 1.0, 100 mm/s, C 20 mm),
which already contains the scale 0.5 -> 1.0 change of the original request. Pick and place are found
from the measured TCP near P / Q, not from the trigger, because the trigger read 0 for whole runs
(20261014_035701).

**Evidence for the plan (5 Real runs 2026-10-14, free motion):** TCP speed limit active in 72-91 % of
engaged cycles, follow 30-79 %, joint speed limit 0-10 %, yaw limit 0 %. Plan: TCP 150, then joints 1.2
deg/cycle if needed, then 200, then C 40 mm; every raise of a Real limit is approved separately.

**Rejected:** comparing against scale 0.5 / B again (extra Real runs; the user chose the current
settings as the baseline); trigger as the pick / place marker (unreliable).

## 2026-10-16 — Real speed limits to option B: TCP 200 mm/s, joints 1.2 deg/cycle (60 deg/s)

**Request (user):** the robot should keep up with the hand instead of being dragged with the clutch,
without the coordinate-based trial protocol first.

**Hand speed (5 Real runs 2026-10-14, about 160 s with the clutch engaged, scale 1.0;
`leader_used_*` x scale per cycle):**

| | median | p90 | p95 | p99 | max |
|---|---|---|---|---|---|
| hand speed x scale [mm/s] | 121 | 278 | 360 | 515 | 689 |
| joint speed needed to follow [deg/s] (estimate) | 30 | 78 | 103 | 185 | 511 |
| leader J6 [deg/s] | 0 | 18 | 26 | 44 | 89 |

Share of the clutch time with the hand faster than 100 / 150 / 200 / 300 mm/s: 63 / 36 / 22 / 8 %.
Joint estimate = predicted joint motion per mm of each step (`max_dq_pred_deg` / target step, steps
> 0.3 mm) x hand speed; share above 40 / 60 / 90 deg/s: 35 / 18 / 7 %.

**Options shown:** A 300 mm/s, 80 deg/s, yaw 45 deg/s (about 92 % of the hand time); B 200 mm/s,
60 deg/s, yaw 30 (about 78 %); C 500 mm/s, 180 deg/s, yaw 90 (about 99 %, at the manual's joint
maximum).

**Decision (user): B.** `REAL_MAX` mapping.speed_mm_s 100 -> 200, `REAL_MAX_JOINT_SPEED_DEG_PER_CYCLE`
0.8 -> 1.2, and the same values in real.yaml. Yaw 30 deg/s, scale 1.0, C 20 mm, range and watches
unchanged. A only after B has been checked on the real robot.

**Reasons for B over A:** collision detection is Off and the tool mass is 0 kg on the controller, so
nothing stops the arm on contact; the kinetic energy of the moving arm grows with the square of the
speed: 200 mm/s is 4x the energy at 100 mm/s, 300 mm/s would be 9x. B removes most of the dragging
(hand faster than the cap 22 % of the time instead of 63 %) with the smaller step.

**Condition:** collision detection ON before every Real run (user turns it on with the lab; required
item at the top of `docs/real_first_run_checklist.md`). First run at the new speeds: slow, small
motions near the ready pose first.

## 2026-10-16 — Low-pass filter on the leader wrist center (tau 30 ms) against the shake at 200 mm/s

**Observation (Real run 20261016_021514, option B, user felt a shake):** compared with the 100 mm/s
runs, the target acceleration jitter rose from a median of 0.13-0.22 to 1.23 m/s^2 and the measured
joint acceleration p99 from 482-638 to 823 deg/s^2 (J2-J4 fast wobble p99 0.11-0.15 deg). The hand
itself was not jerkier (acceleration p99 3.7-4.4 m/s^2 in all runs). The jitter was larger when the
speed limit was **not** active (1.66 vs 0.41 m/s^2): the target then follows the hand step by step,
and the hand step per cycle jumps (e.g. 3.3, 4.0, 3.6 mm) because one leader encoder tick moves the
wrist center by about 0.5 mm (J0, J1) / 0.25 mm (J2, J3). At 100 mm/s the cap (2 mm per cycle) hid it.
The log is at 50 Hz, so vibration above 25 Hz cannot be seen.

**Decision (user, item 1 of 4):** a first-order low-pass on the wrist center after the deadband,
`mapping.leader_filter_tau_s` 0.03 s in sim.yaml and real.yaml (0 = off). J6 / yaw is not filtered.
Replay of 021514 through `LeaderFilter`: hand acceleration jitter median 1.74 -> 0.65 m/s^2, p99 4.4 ->
2.1; delay about 30 ms (largest filtered - raw distance 19 mm at the fastest hand motion). Offline
alternatives: 20 ms -> 0.77, 60 ms -> 0.47, 100 ms -> 0.38 m/s^2.

**Rejected / later:** servo alpha < 1 (controller filter; 06: alpha 0.5 added 19 ms and the settling
took 213 ms instead of 81 ms); an acceleration limit on the target (does not remove jitter below the
limit). Next items, one at a time after a Real check of this one: option A speeds (300 mm/s, 80 deg/s,
yaw 45 deg/s), C lag 20 -> 50 mm (275 mm of a forward reach was dropped in 021514), yaw range +/-45 ->
+/-90 (yaw at the range end 43 % of the clutch time).

## 2026-10-16 — Ready pose moved to r 500, z 200 (Real and Sim); range kept in absolute terms

**Problem (user, Real run 20261016_022416):** going straight down to place was hard. The hand asked for
z down to -130 mm, but the descents ran into r_min (350 mm): the leader wrist center moves back by
0.5-0.9 x the descent (it arcs about the shoulder), and the ready pose was at r 367, only 17 mm from
r_min (e.g. press 22: hand dz -394, dx -264, robot r 692 -> 350). The z floor (30 mm) was not the
cause for placing: the place height is 150 mm above the table (base on the table, so z 150).

**Why the ready pose was at r 367:** it is the pose the user set by hand on the real RB5 on
2026-10-12, read from jnt_ang and used as is; r_min was lowered from 450 to 350 so that it fitted
the range. The 17 mm gap to r_min was not pointed out then.

**Decision (user):** ready pose (182.920, -0.156, -107.813, 24.418, -87.677, 52.334) deg in sim.yaml
and real.yaml: DH TCP (486.5, 115.5, 200.0), r 500, azimuth 13.4 deg, same tool orientation as the
2026-10-12 pose (rpy 84.2, 3.7, 145.2). Found by damped least-squares IK marched from the old pose
(same arm configuration). The range is relative to the start z, so it was re-expressed to stay the
same in absolute terms (start z about 202 with the controller's +2 mm): z_above 400 -> 263 (top about
465), r_max_by_z heights -137 mm, z_below 100 -> 172 in Real (floor 30 = z_min_abs) and -> 237 in Sim
(floor about -35 as before).

**Margins (DH, Real thresholds slow 20 deg / 50 mm):** at the pose J5 87.7 deg, shoulder 386 mm, elbow
72.2 deg, joint limit 57.2 deg (J3; was 26.2); vertical line at r 500 from z 30 to 400: at least 87.7 /
386 / 58.0 / 43.0; place height z 150 over r 350-800, azimuth +/-60: at least 83.2 / 219 / 47.7 / 32.7.
Room to r_min 150 mm (was 17). Largest joint change from the old pose 31 deg (J3), about 6 s at 5 deg/s.

**Candidates computed:** r 450 / 475 / 500 x z 150 / 200 / 250, all with margins >= 1.6x the slow
thresholds; r 500 has the most J3 margin and room to r_min, z 200 is 50 mm above the place height.

**Next if descending is still awkward:** a "vertical mode" switched by the left handle button
(plan to be proposed separately). Cylindrical mapping for J1: on hold.

## 2026-10-16 — Cylindrical mapping trial (Sim and Real): a hand swing about the leader base center turns J1

**Problem (user):** turning the RB5 base (J1) is hard. Run 022416: up to 45 deg of J1 per clutch press
for about 240 mm of sideways hand motion, 99 deg in 30 presses; with the cartesian mapping 1 deg of J1
needs 7 mm of sideways motion at r 400 (10.5 mm at r 600).

**Decision (user, trial, to be reverted if it feels worse):** `mapping.mode: cylindrical` in sim.yaml
and real.yaml. Each cycle the leader motion is split about a vertical pivot axis (leader frame) into an
angle, a radius and a height change; the RB5 target turns about the base z axis by the same angle
(1:1), radius and height change x scale. Angle = sideways motion / max(hand radius,
`min_pivot_radius_mm` 150), so the gain stays bounded near the axis. The state is integrated per cycle;
B / C dropped parts are taken off the desired position (same meaning as the cartesian reference shift);
a late cycle that is not applied keeps its motion for the next cycle. Limits, range, watches, yaw and
servo are unchanged. Revert: `mode: cartesian` (identical to the mapping before; the old tests run in
cartesian).

**Pivot:** `pivot_xy_mm` [0, 0] = leader base center. From the 10-14 / 10-16 Real logs the hand was
205 mm or more from it in 95 % of the clutch time (sideways gain r_RB5 / r_hand median 1.6x, p95 2.4x);
about the right shoulder (0, -150) it was 87 mm (gain 2.3x median, 5.1x p95), too jumpy.

**Replay of 022416 (mapping only, no limits):** J1 swing per press median 7.3 -> 25.1 deg, max 36 -> 79
deg, total 411 -> 815 deg (2.0x).

**Expected side effects:** sideways motions are larger, so the speed cap (200 mm/s) is hit more often;
a straight sideways hand motion becomes an arc. Not addressed: the descent drift (the hand moves back
0.5-0.9 x the descent); about the base center that is a radius decrease and is mapped as one. Ready
pose r 500 removed the r_min stops (025735: 0 cycles at r_min), but the user still finds descending
awkward: next candidate is the "vertical mode" on the left handle button.

## 2026-10-16 — Cylindrical mapping reverted to cartesian after one Real run

**Run 031043 (cylindrical, Real):** J1 swing per clutch press median 9.1 -> 21.5 deg (vs 025735), but
the sideways speed the hand asked for rose to 127 mm/s median / 237 mm/s p90, the speed cap was hit in
65 % of the clutch time (29 %), joint speed limit 11 % (3 %), 2627 mm of hand motion dropped (115),
17 clutch presses (5). User: "J1 is easier, but it does not go where I want".

**Decision (user):** `mapping.mode: cartesian` in both files (the code stays, `cylindrical` turns it on
again). Next: make descents for pick and place easier, and try leader J2 (shoulder yaw) -> RB5 J1 with
the RB5 J6 no longer following the leader J6.

## 2026-10-16 — Vertical mode on the left handle button (x, y fixed, only z)

**Problem (user):** going straight down to pick / place is hard, also after the ready pose moved to
r 500. Analysis of 21 descents >= 60 mm (021514, 022416, 025735): hand -111 mm in z and -103 mm in x
(median); the leader elbow (J3) changed 51 deg, shoulder pitch / roll / yaw only 3 / 1 / 5 deg; the
elbow dominated in 90 % of the descents. At the typical pose +10 deg of elbow moves the wrist center
25 mm down and 18 mm back, so the drift is the arm's geometry, not a habit or a setting.

**Decision (user):** `mapping.vertical_button: true` (Sim and Real). While the **left** handle button is
held, the desired x and y stay at the target's x and y and only z follows the hand (x, y fully fixed,
user's choice); yaw still follows. Every switch (button or clutch) re-anchors the references, so the
RB5 does not jump. The left button uses the same debounce as the clutch (2 reads to press, 3 failed
reads = released, a button held at start must be released once). The left tool (ID 0x81) is read once
per cycle (read only; pinged at start only when the mode is on); the leader read was 4.9 ms p50 / 5.5
ms p99 before. CSV: `left_button_raw`, `vertical_mode`.

**Rejected:** "elbow motion -> z only" (reaching forward also extends the elbow, so forward motion
would break); slow x, y instead of fixed (user chose fixed: simpler to predict).

## 2026-10-16 — Vertical mode z x 2 (vertical_z_scale)

**Run 032657 (vertical mode, Real):** in the vertical mode x and y stayed fixed (0.000 mm change) and
the 17 switches caused no jump (0.0 mm); left button read failures 1 of 11869; leader read p50 / p99
4.9 / 5.5 -> 5.9 / 8.5 ms, work p99 10.4 ms, no late cycle; target jitter median 0.29 m/s^2. The user
found descending easier. 57 clutch presses (half shorter than 1 s), 31 of them with the vertical mode;
some vertical segments show the hand going up overall while the robot went down (re-gripping to go
further). The user said the many presses were mostly for turning J1, which was fine when moving slowly.

**Decision (user):** `mapping.vertical_z_scale` 2.0 (Sim and Real): in the vertical mode z follows the
hand x scale x 2, so a 170 mm descent needs 85 mm of hand motion. The B / C reference shift uses the
same gain. Normal mode unchanged.

## 2026-10-16 — TCP floor 30 -> -50 mm (nothing mounted on the flange)

**Request (user):** go lower for pick and place (run 032657: 8 of 9 s of range limit were the 30 mm
floor, all in the vertical mode).

**Finding:** the TCP offset (0, -187, -40) points 187 mm *down* from the flange center in the ready
orientation (the flange faces sideways, flange y is up), so at TCP z 30 the flange center is at about
220 mm. The 30 mm floor protected a tool tip at the TCP, but nothing is mounted (user, 2026-10-16).

**Decision (user):** `range.z_min_abs_mm` -50 in Real (flange center then about 140 mm above the table)
and `z_below_mm` 252 in Real and Sim (start z about 202 -> floor -50 in both). DH check at TCP z -50 over
r 350-800, azimuth +/-90 (ready orientation): all reachable; J5 >= 83 deg, shoulder >= 218 mm, elbow >=
29.5 deg, joint limit >= 14.5 deg at the inner low corner (r 350, azimuth -75): below the 20 deg slow
threshold there, so the teleop slows down in that corner (stop at 10).

**Must be raised again when a tool is mounted** (tool tip height + margin); also in
`docs/real_first_run_checklist.md`.

## 2026-10-16 — Faster J1 (sideways 300 mm/s), RB5 J6 still (tool follows the base), vertical z x 1.5

**Run 033917 (vertical z x 2, floor -50, Real):** while J1 turned (> 5 deg/s) its speed was 16 deg/s
median, 28 p90, 42 max, and the TCP speed cap was active 46 % of that time (032657: 59 %); sideways
target speed p90 197 mm/s against the 200 cap (about 25 deg/s of J1 at r 450); joint speed limit only
5 %. Vertical mode z speed median 122 mm/s, 40 % at the cap. Measured RB5 J6 range 140 deg: leader-J6
yaw (to the +/-45 end) plus J6 turning back whenever J1 turned (the tool kept its world orientation).
TCP reached z -52. User: J1 should turn faster, RB5 J6 should stay still, vertical a bit slower.

**Decisions (user):**
- `mapping.speed_tangential_mm_s` 300 in Real (400 in Sim, same as its speed): the per-cycle step is
  scaled so that (t / 300)^2 + (o / 200)^2 <= 1, t = sideways about the base z axis, o = radial and
  vertical; direction kept. New Real max 300 (`REAL_MAX`). Lead limit uses the larger step.
- `yaw.mode: follow_base` (Sim and Real): the tool yaw equals the azimuth change of the target, so the
  gripper turns with the arm; the leader J6 is not used and the yaw speed / range limits do not apply.
  Fake robot (straight sideways hand motion, 40 deg twist): J1 +4.6 deg, J6 -0.28 deg; with the old
  world-held tool J6 turned back by as much as J1 (-4.29 for 4.29). `leader_j6` keeps the old mapping.
- `mapping.vertical_z_scale` 2.0 -> 1.5.

## 2026-10-16 — Left button = J1 rotate mode; azimuth range +/-120

**Run 034808 (sideways 300 mm/s, tool follows the base, Real):** RB5 J6 range 6 deg (was 140); J1
speed-limited 21 % of its turning time (46 %), up to 50 deg/s; dropped 36 mm (1433). J1 turned from 90
to 183 deg: 9 s at the azimuth limit -90 (the hand asked 20 deg more), 7 s at r_min (a sideways hand
motion arcs toward the body), J3 joint-limit slowdown 13 % (margin >= 16.6 deg) in that inner corner.
Vertical mode used 6 s. User: the left button would be more useful for turning J1 than for going down.

**Decision (user):** `mapping.left_button` off / vertical / rotate (replaces `vertical_button`), now
`rotate` in Sim and Real: while held, the radius and z of the target stay and only the azimuth changes,
by `rotate_deg_per_mm` 0.3 x the leader's sideways (leader y) motion (100 mm = 30 deg; the cartesian
mapping needed about 236 mm at r 450). The tool still follows the base, so only J1 turns. Same speed
limits; a dropped arc piece shifts the leader y reference by its angle (B / C as elsewhere). The
vertical mode stays available (`left_button: vertical`). CSV column `vertical_mode` -> `left_mode_active`.
`range.azimuth_half_deg` 90 -> 120 (Sim and Real): with the tool turning with J1 the other joints are the
same as at +/-90 (DH grid r 350-800, z -50..400: J5 87.2, shoulder 236 mm, elbow 31.7, joint limit 16.7 deg,
all at the inner low corner); J1 then spans about 63-303 deg (limit +/-360). The space at the sides and
behind the RB5 must be clear.

## 2026-10-16 — Azimuth lock: J1 turns only in the rotate mode

**Run 035549 (rotate mode, Real):** 7 rotate segments turned J1 by 225 deg in 9 s (up to 60.6 deg in one),
with z unchanged (0.000 mm) and the radius within 0.9 mm (a speed-limited step is a chord, slightly
inside the circle); J1 speed 24 / 35 / 48 deg/s (p50 / p90 / max), speed cap active 41 % of that time.
Range limit 1 s (was 15), J3 slowdown 1 % (13). But J1 also turned 150 deg in the normal mode, by
accident. User: without the left button J1 should stay.

**Decision (user):** `mapping.lock_azimuth: true` (Sim and Real). Without a left-button mode the
azimuth stays at its value at the last re-anchor; leader forward / back (leader x) x scale moves the
target along the arm's current direction (radius), up / down (z) x scale moves z, sideways is not used
(user's choice "along the arm": after turning J1, forward still extends the arm). B / C: the radial and
z parts of a dropped vector shift the leader x / z reference. With the tool following the base, the
normal mode now moves J2-J5 in the arm's vertical plane only.

## 2026-10-16 — Target acceleration limit 2.5 m/s^2 against the shake when the limits switch

**Run 040257 (current operation, Real):** the user finds this operation the best for pick and place so
far. J1 locked in the normal mode (0.087 deg in total), descents nearly vertical (radius change 0.13 x
the descent, was 0.5-0.9), RB5 J6 6 deg, work p99 11.1 ms, no late cycle. But the joint acceleration
p99 was the highest so far (1295 deg/s^2, target 5.7 m/s^2 p99), and of the top 1 % of joint
accelerations 97 % were next to the TCP speed limit switching and 82 % next to the joint speed limit
(0 % next to a mode switch): the target velocity jumped from the hand's to the cap in one cycle.

**Decision (user, item 1 of the refinement list):** `mapping.accel_mm_s2` 2500 (Sim and Real; 0 = off).
In plan_step, before the speed limits, the step may differ from the previous cycle's step by at most
a x dt^2 (1.0 mm at 20 ms), and is at most (-a + sqrt(a^2 + 8 a d)) / 2 for a remaining distance d, so
it can still stop in time without overshoot (the continuous sqrt(2 a d) overshot 1.9 mm in the test).
The cut part goes to the C lag / is dropped like the other limits. A release, a late or a stale cycle
still freezes the target at once (the previous step is reset to zero). CSV column `accel_limited`; no
terminal message for it.

**Replay of 040257 through TeleopLoop (Real settings, fake robot, same hand data):**

| accel limit | target accel p99 / max | joint accel p99 / max | desired - target p50 / p95 | dropped |
|---|---|---|---|---|
| off | 4.0 / 20.6 m/s^2 | 971 / 2530 deg/s^2 | 0.3 / 51.9 mm | 2161 mm |
| 2.5 m/s^2 | 2.5 / 2.8 | 412 / 605 | 5.7 / 29.8 mm | 2526 mm |
| 4.0 m/s^2 | 4.0 / 4.5 | 579 / 977 | 3.5 / 36.8 mm | 2354 mm |

Cost of 2.5: about 5 mm more lag behind the hand (median) and 17 % more dropped motion. The test found
a missing event label for the new flag that would have stopped the teleop at the first limited cycle.

**Remaining list:** rotate-mode catch-up in degrees (1457 mm dropped while rotating), radial / vertical
cap 250 mm/s, asymmetric azimuth range (-120 reached, +120 unused), radius kept exactly on the circle in
the rotate mode (1.2 mm chord drift).

## 2026-10-16 — Rotate mode: 15 deg catch-up, target kept on the circle, re-anchor at the stop point

**Run 041233 (acceleration limit 2.5 m/s^2, Real).** Correction first: the "measured joint acceleration
p99" in the earlier entries (823-1295 deg/s^2) assumed RB5 samples 20 ms apart; they are 13-22 ms apart
(the state arrives every ~10 ms and the loop takes the latest), which inflated J1 in fast turns. With
the logged sample times: 040257 -> 041233 joint acceleration p99 533 -> 409 deg/s^2 (J1 493 -> 318), the
commanded azimuth acceleration p99 757 -> 310, target jitter p99 5.7 -> 2.5 m/s^2. Also: clutch presses
longer (median 0.9 -> 1.7 s, none < 0.5 s), normal-mode speed cap 40 -> 14 %, dropped 845 -> 386 mm; but
1477 mm dropped while rotating (25 % at the cap), and J1 moved 1.23 deg in the normal mode: after the
left button was released the target still had its turning speed, passed the anchor and came back.

**Decisions (user, items 1 + 2, and 4 with them):**
- `mapping.rotate_max_lag_deg` 15 (Sim and Real; the fixture uses 0): in the rotate mode the C lag is
  r x 15 deg (about 118 mm at r 450) instead of max_lag_mm, so a fast swing is finished after the hand
  stops; only the part beyond 15 deg is dropped. B stays B.
- In the rotate mode the applied target is put back on the circle (anchor radius and z) every cycle:
  the steps are straight chords (radius drift was up to 1.2 mm; now 0.001 mm in the replay).
- Every re-anchor (mode switch, clutch press) uses the point where the target stops from its last step
  at the acceleration limit, v (v - a) / (2 a) further along it, matching the discrete stopping rule.
  A clutch release still freezes at once (its step is reset to zero).

**Replay of 041233 through TeleopLoop (Real settings, fake robot):** rotate lag 0 / 15 deg: dropped while
rotating 3720 / 507 mm (-86 %), J1 turned in the rotate mode 460 / 798 deg (more of what the hand
asked); in the normal mode 1.24 deg of J1 motion within 0.3 s after the 9 releases (slowing down to the
stop point, no return) and 0.12 deg later (same in both); one release turned back by 0.05 deg (about
0.4 mm). Not changed: the 250 mm/s radial / vertical cap (the hand exceeded 200 mm/s only 8 % of the
normal time); the asymmetric azimuth range is still open.

## 2026-10-16 — Range in absolute base-frame values; ready pose = the user's pose at r 450

**Run 042300 (rotate catch-up, Real):** dropped while rotating 1477 -> 212 mm, radius drift in the rotate
mode 0.00 mm, J1 in the normal mode 0.076 deg, joint acceleration p99 (sample times) 409 -> 331 deg/s^2,
no range limit, no late cycle (slow hand motion this run: normal-mode radial speed p90 57 mm/s).

**Question (user):** use the pose the user set on the real RB5 as the ready pose (it looks better)?
Read from the data channel (read only): (171.53, 14.59, -127.82, 29.54, -87.31, 55.63), DH TCP r 383,
z 152, azimuth 5.3, same tool tilt. It passes the start checks, but (1) it is only 33 mm from r_min
(like the r 367 pose before) and (2) the range was relative to the start pose, so its z top would drop
465 -> 417, the r_max_by_z heights by 48 mm and the azimuth range by 8 deg.

**Decisions (user):**
- The range is absolute in the base frame: `z_min_mm`, `z_max_mm`, `r_max_by_z` [z, r_max] and
  `azimuth_min_deg` / `azimuth_max_deg` replace z_below / z_above / z_min_abs / dz pairs / azimuth_half.
  Values = the space in effect: z -50 .. 465, r_max 800 / 775 / 725 / 700 / 650 at z 265 / 315 / 365 /
  415 / 465, azimuth -107 .. 133 (was 13.4 +/- 120). The start check now also requires the ready pose
  inside the z and azimuth range. Changing the ready pose no longer moves the range.
- Ready pose (173.650, 4.300, -119.566, 31.666, -87.544, 53.521) in Real and Sim: the user's pose with
  the same azimuth, height and tool orientation (0.001 deg), moved out to r 450 (IK marched from it):
  DH TCP (448.1, 41.4, 152.3), 100 mm to r_min; margins J5 87.5, shoulder 336 mm, elbow 60.4, J3 45.4
  deg; vertical line z -50 .. 465 at r 450: J5 87.5, shoulder 336, elbow 47.2, J3 32.2. Largest joint
  change from the user's pose 10.3 deg (J2), from the previous ready pose 11.8 deg (J3).

## 2026-10-16 — Joint-limit slow threshold 20 -> 15 deg

**Run 043303 (new ready pose, absolute range, Real):** start TCP (447.8, 41.3, 154.3), r 449.8, as set.
J1 lock 0.099 deg, rotate radius drift 0.00 mm, no late cycle. Faster and deeper than before (vertical
speed asked p90 191 mm/s, floor -50 reached): normal-mode speed cap 24 %, 754 mm dropped. New: the
joint-limit slowdown was active 19.9 % of the clutch time (13 s), all J3, all at r 350-372 and z -50 ..
-5 (the inner low corner; going down from r 450 pulled the radius in by 0.26 x the descent). The J3
margin there was 16.7 deg, the smallest anywhere in the range (DH grid with the tool following the base:
16.7 at r 350, z -50).

**Decision (user, option A):** `safety.joint_limit_slow_deg` 15 in Real and Sim (stop stays 10). Inside
the range the J3 margin cannot go below 16.7, so the stop is unreachable there and the 20 deg slowdown
only slowed the corner. Replay of 043303 (TeleopLoop, Real settings, fake robot): slowdown 13.0 s -> 0 s,
smallest margin 17.4 deg either way. The test fixture keeps 20 (script 21's value).

**Rejected:** a higher r_min near the floor (would block reaching that corner); leaving it.

## 2026-10-16 — Azimuth range widened on the minus side: -160 .. 133 deg

**Run 044024 (J3 slow threshold 15, Real):** joint-limit slowdown 0 % (was 19.9), J3 margin >= 16.6 deg;
dropped 293 mm in total (the lowest of the active runs; normal 199, rotate 93 while turning J1 716 deg);
J1 lock 0.070 deg, rotate radius drift 0.00, joint acceleration p99 (sample times) 417 deg/s^2, work
p99 10.6 ms, no late cycle. The azimuth used was -107.0 .. 14.4 deg: 1.7 s at the -107 limit with the
hand asking up to 49 deg more; the + side beyond 14 was not used.

**Decision (user):** widen only the minus side (moving the hand to the right in the rotate mode turns
clockwise seen from above): `range.azimuth_min_deg` -107 -> -160 in Real and Sim, max stays 133 (width
293 deg). From the ready pose (azimuth 5.3) that is 165 deg clockwise (nearly behind the robot) and
128 deg counterclockwise. The tool turns with J1, so the other joints keep their margins; J1 spans about
8 .. 301 deg (limit +/-360). That side and the back of the RB5 must be clear (checklist).

**Rejected:** shifting the whole range to -160 .. 80 (user keeps the + side).

## 2026-10-16 — J1 a bit faster (sideways 360 mm/s, J1 75 deg/s); acceleration limit moved to the last stage

**Run 044916 (azimuth -160 .. 133, Real):** the whole new range was used (-160 .. 120). Faster operation
than before (normal-mode speed cap 52 %, hand above 200 mm/s 30 %). User: J1 turns a little slowly. In
the rotate mode the hand asked J1 p50 / p90 / p99 36 / 89 / 121 deg/s, the robot did 35 / 42 / max 49:
capped by the sideways speed (300 mm/s = 49 deg/s at r 350, 37 at r 459) and the J1 joint cap (60 deg/s;
the joint speed limit was active 50 % of the rotate time). Target jitter p99 6.9 m/s^2 in the rotate mode
(2.5 in the normal mode), 92 % of the top 1 % in the rotate mode: the joint limit factor came after the
acceleration limit and changed from cycle to cycle, because the joint prediction includes the robot's
lag, which jitters with the 13-22 ms state timing. 21 s of "range limited" in the rotate mode were the
rotate circle lying exactly on r_min (350): every straight step cut inside it and was pushed back.

**Decisions (user, all three):**
1. `mapping.speed_tangential_mm_s` 300 -> 360 and J1 joint cap 1.2 -> 1.5 deg/cycle (75 deg/s; J2-J6 stay
   1.2) in Real; `REAL_MAX` tangential 360, joint 1.5. Sim unchanged.
2. The acceleration limit is the last stage of plan_step (after the speed and joint limits), so a switching
   limit cannot make the step jump. While it slows down a cap can be exceeded for a few cycles.
3. The rotate circle is at least r_min + 1 mm.

**Replay of 044916 (TeleopLoop, Real settings, fake robot).** With the state delayed by a random 1-2
cycles (like the real 13-22 ms timing; harsher than reality): target accel p99 rotate / normal 10.0 / 9.7
-> 2.5 / 2.5 m/s^2; J1 in the rotate mode p90 41 -> 58 deg/s, max 61 -> 77; dropped while rotating 2261
-> 1535 mm; a joint above its cap by at most 0.012 -> 0.043 deg/cycle (about 2 deg/s, 36 cycles). Without
the delay: J1 p90 49 -> 58 deg/s, dropped while rotating 765 -> 506 mm, no cap exceeded, range flag in
the rotate mode 20.6 -> 1.5 s. The 6.9 m/s^2 seen on the real robot did not appear in the replay without
the delay (the fake state is exact), which supports the diagnosis.

## PENDING

- **14: RB-Y1 self-collision check during teleop.** Example 35 blocks the arm command when the
  nearest link distance is below 2 cm (`detect_collisions_or_nearest_links`, lines 1226-1240).
  Skipped for now (simulator only); needed before any real RB-Y1 use. Measure its cost in the 50 Hz loop.

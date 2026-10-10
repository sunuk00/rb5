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

## PENDING

- **14: RB-Y1 self-collision check during teleop.** Example 35 blocks the arm command when the
  nearest link distance is below 2 cm (`detect_collisions_or_nearest_links`, lines 1226-1240).
  Skipped for now (simulator only); needed before any real RB-Y1 use. Measure its cost in the 50 Hz loop.

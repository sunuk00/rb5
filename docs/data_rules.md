# Data rules: first ACT dataset (decided 2026-10-21)

One page for every collection session. Change a rule only with a docs/decisions.md entry; a rule that
changes the data (task, object, cameras, settings) starts a new dataset.

## Task
- **Pick the green block and place it in the box.** Task text, exactly:
  `python -m teleop --real --record --task "pick the green block and place it in the box"`
- One object (the green block), the paper box fixed in place (mark its corners on the table).

## Placement and start
| | rule |
|---|---|
| Block position | random inside a marked **20 x 20 cm** square; mark the square and its 4 corners + center |
| Block orientation | **always the same** (fingers close across the block; the gripper does not turn) |
| Robot start pose | in the reset phase, move with the teleop to a pose **within about 10 cm of the ready pose**, a different one each time; also start some at the ready pose itself |
| Gripper at start | **open**, trigger caught (status line not "catch") |

## How to demonstrate (one way, every time)
1. Approach above the block -> 2. go straight down -> 3. close (trigger fully released) -> 4. lift ->
5. move above the box -> 6. lower a little -> 7. open -> 8. **lift about 5 cm and stop** -> 9. `n`.
- Steady, no hesitation, no exploring. Same order every time.
- Discard (`r` while recording, or `r` in the reset right after `n`) when: the block slips or is missed,
  anything is hit, a pause over about 2 s inside the task, the robot jumps (clutch slip), a spoken
  "Camera error" or a frame-loss warning.

## Amount and sessions
- **50 kept episodes**, in sessions of **10-15 episodes** (the robot CSV is written at the session end).
- Same lighting for all sessions (blinds / lamps as on the first session; note the time of day).
- After each session: `python -m rb5_lerobot.convert <session> --check-only` and look at the
  report (left-out episodes, image time error, gripper following, start pose spread) before the next one.

## Cameras
- **D435 (`front`)**: fixed above the workspace, the 20 x 20 cm square and the box in view, not covered
  by the arm at the ready pose. Mark its mount; it must not move between sessions.
- **C270 webcam (`wrist`)**: on the gripper, sees the block (cam-4a; replaced the D405, broken 2026-10-21).
- **Brio 101 webcam (`grip`)**: on the gripper, sees the fingers and what they hold (cam-5a).
- The two webcams share one USB 2 bus: the C270 in MJPG and the Brio in YUYV (both in YUYV do not fit).
  Cables strain-relieved.
- **Exposure, gain and white balance fixed** (cam-2b, cam-5b, `record.cameras` in configs/*.yaml):
  front 16.7 ms / gain 76, wrist 8.3 ms / gain 86, grip 8.3 ms / gain 152, all 4600 K. Set and read back every
  time the teleop opens the cameras.
- **Brightness check at start**: each camera's mean brightness must be within 114 +/- 15 % (97..131);
  otherwise a typed yes is needed. Reference, measured value and settings go into episodes.json.
- 640x480 @ 30 for all three (the dataset resizes them to 320x240, `configs/policy.yaml`).

## Evaluation (after training, the same for every policy)
- Block at **5 positions** (the 4 corners and the center of the square) x **4 trials** = 20 trials.
- Start at the ready pose, gripper open.
- **Success**: the block is inside the box and the gripper is open within 60 s.
- Record every trial (`--record`), success / failure noted per trial.

## Before every session (in addition to docs/real_first_run_checklist.md)
- [ ] Collision detection ON, emergency stop within reach.
- [ ] D435 on USB 3 (5000M), mount at its mark; both webcams connected (docs/teleop_guide.md: USB autosuspend).
- [ ] **Same lighting as the reference** (2026-10-21, when the camera values were chosen): blinds in the
      same position, the same ceiling lights on, no extra lamps, no direct sun on the table. The start
      brightness check (114 +/- 15 %) must pass; if not, fix the lighting rather than typing yes.
- [ ] Gripper powered (12 V when available), fingers clear.
- [x] `range.z_min_mm` 19 mm for the mounted gripper (2026-10-21). If the gripper, the table or its height
      changes, measure again (lowest allowed TCP z from `tcp_ref`).
- [ ] Box and square at their marks.

## Rules nobody sees in the data (keep them fixed)
- Gripper orientation: rx, ry from the ready pose, rz follows the azimuth (`yaw.mode: follow_base`). The
  action is only x, y, z + gripper; changing the ready pose orientation or the yaw mode breaks the data.
- Teleop settings frozen (`configs/real.yaml`); LeRobot 0.6.1 (env `lerobot`).
- Dataset format: docs/decisions.md rec-1 (30 Hz; action [x, y, z, gripper]; state 6 joints + gripper).

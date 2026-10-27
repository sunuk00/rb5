# Data rules: meal box dataset (t2, decided 2026-10-28)

One page for every collection session. Change a rule only with a docs/decisions.md entry; a rule that
changes the data (task, object, cameras, settings) starts a new dataset. The first dataset (green block ->
paper box, p1 / p2, 2026-10-21 .. 22) used the rules of commit 03e4246 and the C270 on the gripper (`wrist`);
its sessions still convert as they were recorded (camera names come from each session's episodes.json).

## Task
- **Put the carrot in the large hole, then the snack in the small hole of the meal box.** Task text, exactly:
  `python -m teleop --real --record --task "put the carrot in the large hole, then the snack in the small hole of the meal box"` (the default `record.task`).
- Two objects in every episode, **always the carrot first** (user, 2026-10-28: a fixed order, one right
  action at every moment; a free order makes the policy average the two choices).
- The meal box fixed in place (mark its corners on the table).

## Placement and start
| | rule |
|---|---|
| Carrot and snack positions | random, **anywhere the `top` camera sees** (user, 2026-10-28), not on the meal box, inside the teleop range (a place the robot cannot reach is not used), the two not touching |
| Object orientation | so that the fingers close across it **without turning the gripper** (the tool turns only with J1; no wrist rotation since j6-2) |
| Robot start pose | in the reset phase, move with the teleop to a pose **within about 10 cm of the ready pose**, a different one each time; also start some at the ready pose itself |
| Gripper at start | **open**, trigger caught (status line not "catch") |

## How to demonstrate (one way, every time)
1. Approach above the carrot -> 2. go straight down -> 3. close (trigger fully released) -> 4. lift ->
5. move above the **large** hole -> 6. lower a little -> 7. open -> 8. lift ->
9. approach above the snack -> 10. down -> 11. close -> 12. lift -> 13. above the **small** hole -> 14. lower a
little -> 15. open -> 16. **lift about 5 cm and stop** -> 17. `n`.
- Steady, no hesitation, no exploring. Same order every time.
- Discard (`r` while recording, or `r` in the reset right after `n`) when: an object slips, is missed or
  lands outside its hole, anything is hit, a pause over about 2 s inside the task, the robot jumps (clutch
  slip), a spoken "Camera error" or a frame-loss warning. The 120 s episode limit ends and keeps an episode:
  discard it if the task was not finished.

## Amount and sessions
- **50 kept episodes**, in sessions of **10-15 episodes** (the robot CSV is written at the session end).
- Same lighting for all sessions (blinds / lamps as on the first session; note the time of day).
- After each session: `python -m rb5_lerobot.convert <session> --check-only` and look at the
  report (left-out episodes, image time error, gripper following, start pose spread) before the next one.

## Cameras
- **D435 (`front`)**: fixed above the workspace. Mark its mount; it must not move between sessions.
- **C270 webcam (`top`)**: **fixed above the workspace** (t2-a, 2026-10-28; was on the gripper as `wrist`),
  the objects' area and the meal box in view. Mark its mount; it must not move between sessions.
- **Brio 101 webcam (`grip`)**: on the gripper, sees the fingers and what they hold (cam-5a).
- The two webcams on **different USB ports** (2026-10-28: Brio on its own port, C270 on the hub): each has
  its own 480 Mbit/s. Formats unchanged (C270 MJPG, Brio YUYV). Cables strain-relieved.
- **Exposure, gain and white balance fixed** (cam-2b, cam-5b, `record.cameras` in configs/*.yaml):
  front 16.7 ms / gain 76, top TBD (script 34 at the new place), grip 8.3 ms / gain 152, all 4600 K. Set and
  read back every time the teleop opens the cameras.
- **Brightness check at start**: each camera's mean brightness must be within 114 +/- 15 % (97..131);
  otherwise a typed yes is needed. Reference, measured value and settings go into episodes.json.
- 640x480 @ 30 for all three (the dataset resizes them to 320x240, `configs/policy.yaml`).

## Evaluation (after training, the same for every policy)
- **5 placements** of the carrot and the snack, chosen and marked before the first training (spread over the
  `top` view, the same for every policy) x **4 trials** = 20 trials.
- Start at the ready pose, gripper open.
- **Success**: the carrot in the large hole and the snack in the small hole, gripper open, within **120 s**
  (user, 2026-10-28; `run.time_limit_s` 120). Note also the carrot alone (first stage).
- Record every trial (`--record`), success / failure noted per trial.

## Before every session (in addition to docs/real_first_run_checklist.md)
- [ ] Collision detection ON, emergency stop within reach.
- [ ] D435 on USB 3 (5000M); D435 and C270 mounts at their marks; the webcams on different USB ports
      (docs/teleop_guide.md: USB autosuspend).
- [ ] **Same lighting as the reference** (2026-10-21, when the camera values were chosen): blinds in the
      same position, the same ceiling lights on, no extra lamps, no direct sun on the table. The start
      brightness check (114 +/- 15 %) must pass; if not, fix the lighting rather than typing yes.
- [ ] Gripper powered (12 V when available), fingers clear.
- [x] `range.z_min_mm` 19 mm for the mounted gripper (2026-10-21). If the gripper, the table or its height
      changes, measure again (lowest allowed TCP z from `tcp_ref`).
- [ ] Meal box at its marks.

## Rules nobody sees in the data (keep them fixed)
- Gripper orientation: rx, ry from the ready pose, rz follows the azimuth (`yaw.mode: follow_base`). The
  action is only x, y, z + gripper; changing the ready pose orientation or the yaw mode breaks the data.
- Teleop settings frozen (`configs/real.yaml`); LeRobot 0.6.1 (env `lerobot`).
- Dataset format: docs/decisions.md rec-1 (30 Hz; action [x, y, z, gripper]; state 6 joints + gripper).

# Data rules: meal box dataset (t3, decided 2026-10-29; three objects since t3-d)

One page for every collection session. Change a rule only with a docs/decisions.md entry; a rule that
changes the data (task, object, cameras, settings) starts a new dataset. The first dataset (green block ->
paper box, p1 / p2, 2026-10-21 .. 22) used the rules of commit 03e4246 and the C270 on the gripper (`wrist`);
its sessions still convert as they were recorded (camera names come from each session's episodes.json).
The t2 sessions (Nutter Butter, dataset t1, checkpoint act_t1, 2026-10-28) are the pilot of this task and are
not part of the t3 dataset (t3-a). The t3 sessions up to t3-c (one snack per episode: u1, u3) stay in the
dataset: they are the cases of the t3-d order with one snack missing (t3-d).

## Task
- **Put the carrot in the black box, then the Oreo in the large hole and the RITZ in the small hole of the
  meal box** (t3-d, 2026-10-29: all three objects in every new episode). Task text, exactly:
  `python -m teleop --real --record --task "put the carrot in the black box, then the Oreo in the large hole and the RITZ in the small hole of the meal box"` (the default `record.task`).
  Until t3-c: "put the carrot in the black box, then the snack in its hole of the meal box (Oreo large, RITZ
  small)", one snack per episode (t2-e: the carrot did not fit the meal box's large hole).

  | Object | goes into |
  |---|---|
  | Carrot | the big black box |
  | Oreo | the meal box, **large** hole |
  | RITZ | the meal box, **small** hole |
- **Snacks: Oreo and RITZ** (t3-a, 2026-10-29: Nutter Butter was too big). The hole depends on the kind,
  which the cameras see: one right action, learned from the snack's look (a sorting task).
- **Three objects in every new episode, always in the order carrot -> Oreo -> RITZ** (t3-d, user: a fixed
  order, one right action at every moment; a free order makes the policy average the choices). An object
  not on the table is skipped: the u1 / u3 episodes (carrot and one snack) follow the same rule, and after
  the Oreo the RITZ on the table tells the policy to go on.
  (Until t3-c: one snack per episode, Oreo and RITZ alternating; the u3 policy did not know what to do
  after the carrot with both snacks on the table, run real_20261029_120825.)
- **The black box and the meal box fixed in place** (mark their corners on the table), inside the teleop
  range and seen by `front`.

## Placement and start
| | rule |
|---|---|
| Objects | carrot, Oreo and RITZ, all three on the table (t3-d) |
| Positions | random, **anywhere the `top` camera sees** (user, 2026-10-28), not on either box, inside the teleop range (a place the robot cannot reach is not used), none touching; vary which one is nearest the robot (the order must come from the kinds, not from the positions) |
| Object orientation | so that the fingers close across it **without turning the gripper** (the tool turns only with J1; no wrist rotation since j6-2) |
| Robot start pose | in the reset phase, move with the teleop to a pose **within about 10 cm of the ready pose**, a different one each time; also start some at the ready pose itself |
| Gripper at start | **open**, trigger caught (status line not "catch") |

## How to demonstrate (one way, every time)
1. Approach above the carrot -> 2. go straight down -> 3. close (trigger fully released) -> 4. lift ->
5. move above the **black box** -> 6. lower a little -> 7. open -> 8. lift ->
9. approach above the **Oreo** -> 10. down -> 11. close -> 12. lift -> 13. above the **large hole** ->
14. lower a little -> 15. open -> 16. lift -> 17. the same for the **RITZ** into the **small hole** ->
18. **lift about 5 cm and stop** -> 19. `n`. (Until t3-c: steps 9-16 for the one snack, then stop.)
- Steady, no hesitation, no exploring. Same order every time.
- **A natural re-grasp is kept** (t3-a; t2 discarded it): a missed or slipped grasp followed by a re-grasp,
  when the episode still ends right (every object in its box or hole) within 180 s, with nothing hit and
  no pause over about 2 s. Note it in notes.txt.
- Discard (`r` while recording, or `r` in the reset right after `n`) when: an object lands outside its box
  or hole (a snack in the wrong hole too) or drops on the way, anything is hit, a pause over about 2 s inside
  the task, the robot jumps (clutch slip), a spoken "Camera error" or a frame-loss warning. The 180 s
  episode limit (t3-d; 120 s until t3-c) ends and keeps an episode: discard it if the task was not finished.

## Recovery demonstrations (t3-a; recorded in stage 2 only, t3-b)
The pilot policy (act_t1) closed beside an object, lifted the empty gripper, carried it to the box and
opened it there: the gripper state is 0 (closed) with or without an object, so only the `grip` image shows
an empty gripper, and no kept demonstration showed a miss. A recovery episode shows one:
1. approach the object **deliberately 1-3 cm off** (direction from the table below) -> 2. down -> 3. close ->
4. **lift empty**, 5 or 10 cm (table) -> 5. **stop 0.5-1 s** (the `grip` camera sees the empty fingers and
the object on the table; not over 2 s) -> 6. **open fully** -> 7. above the object again (where it is now,
if the miss pushed it) -> 8. down, close, and the rest as a normal episode.
- One miss per episode (the carrot or the snack). Never carry the empty gripper to a box.
- 16 in all (stage 2), one cell each of:

  | miss on | off direction (`top` view) | lift |
  |---|---|---|
  | carrot | up, down, left, right | 5 cm and 10 cm |
  | snack | up, down, left, right | 5 cm and 10 cm |

  (2 objects x 4 directions x 2 lifts = 16), the snack kind alternating as in normal episodes.
- **As recorded** (t3-c, session real_20261029_063325): objects, kinds and lifts as planned (8 carrot / 8 snack,
  8 Oreo / 8 RITZ, lifts 5-9 cm and 18-23 cm), but the off direction is right in 14 and left in 2 (none up or
  down), 9-23 cm from the grasp that followed; attempts 30 and 39 missed the snack twice. All 16 kept; more
  directions only if the evaluation shows misses elsewhere that the policy does not recover from.
- **Three-object recovery (t3-d, 24):** the u3 policy (run real_20261029_120825) did open and try again after
  a miss, but closed 5-7 cm further each time (10 tries in a row, 51-107 s): it does not go straight to the
  object. So, stricter than above:
  - off by **1-3 cm from the object's edge** (not more), the direction as in the table below;
  - after the stop and the open, go **straight above the object** in one move, then down: no searching;
  - one miss per episode; the rest of the episode as a normal three-object episode.

  | miss on | off direction (`top` view) | lift |
  |---|---|---|
  | carrot | up, down, left, right | 5 cm and 10 cm |
  | Oreo | up, down, left, right | 5 cm and 10 cm |
  | RITZ | up, down, left, right | 5 cm and 10 cm |

  (3 objects x 4 directions x 2 lifts = 24, one each), the cells in a mixed order (not all of one direction
  in a row). notes.txt: `<attempt> recovery <object> <direction> <lift>`.

## Amount and sessions
Two stages (t3-b, user, 2026-10-29):
- **Stage 1, normal:** **48 kept normal episodes** (natural re-grasps included) in **3 sessions of 16**,
  24 Oreo + 24 RITZ -> dataset `t3n` -> train -> evaluate (below), including what the policy does after a
  missed grasp.
- **Stage 2, recovery:** **16 recovery episodes** (the table above) in 1-2 sessions -> dataset `t3r` = the
  stage 1 sessions + these (64) -> train anew with the same settings -> the same evaluation. Same cameras,
  lighting, box marks and placement habits as stage 1 (the recovery sessions come later, so their
  conditions must not differ).
- **As collected** (t3-c, 2026-10-29): stage 1 in one session, real_20261029_042055 (51 kept, 49 converted:
  25 Oreo + 24 RITZ, 9 natural re-grasps) -> dataset **`u1`** (= `t3n`); stage 2 in real_20261029_063325 (16)
  -> `u2`, merged with `u1` (`lerobot-edit-dataset ... merge`) -> **`u3`** (= `t3r`, 65 episodes; recovery =
  dataset episodes 49-64).
- **notes.txt** in every session folder (`data/demos/<session>/notes.txt`), one line per kept attempt:
  attempt number, snack kind, `normal` / `recovery <object> <direction> <lift>` / `regrasp` (natural).
  The dataset does not record it; it is needed to train without the recovery episodes and for the paper.
- Same lighting for all sessions (blinds / lamps as on the first session; note the time of day).
- After each session: `python -m rb5_lerobot.convert <session> --check-only` and look at the
  report (left-out episodes, image time error, gripper following, start pose spread) before the next one.

- **Stage 3, three objects (t3-d, user, 2026-10-29):** **50 normal three-object episodes** (natural re-grasps
  included) + **24 three-object recovery episodes** (table above), same cameras, lighting, box marks and
  demonstration habits as u1 / u3 -> dataset `u4` = u3 (65) + these (74) = 139 -> train long (100k steps,
  overnight) -> evaluation (below). notes.txt in every session: `<attempt> normal` / `regrasp <object>` /
  `recovery ...` (the kind column is not needed: all three are on the table).

## Cameras
- **D435 (`front`)**: fixed above the workspace. Mark its mount; it must not move between sessions.
- **C270 webcam (`top`)**: **fixed above the workspace** (t2-a, 2026-10-28; was on the gripper as `wrist`),
  looking straight down at the objects' area (the boxes: `front` sees them). Mark its mount; it must not move between sessions.
- **Brio 101 webcam (`grip`)**: on the gripper, sees the fingers and what they hold (cam-5a).
- The two webcams on **different USB ports** (2026-10-28: Brio on its own port, C270 on the hub): each has
  its own 480 Mbit/s. Formats unchanged (C270 MJPG, Brio YUYV). Cables strain-relieved.
- **Exposure, gain and white balance fixed** (cam-2b, cam-5b, `record.cameras` in configs/*.yaml):
  front 16.7 ms / gain 76, top 16.7 ms / gain 14 (t2-b), grip 8.3 ms / gain 152, all 4600 K. Set and
  read back every time the teleop opens the cameras.
- **Brightness check at start**: each camera's mean brightness must be within 114 +/- 15 % (97..131);
  otherwise a typed yes is needed. Reference, measured value and settings go into episodes.json.
- 640x480 @ 30 for all three (the dataset resizes them to 320x240, `configs/policy.yaml`).

## Evaluation (after training, the same for every policy)
- **5 placements** of the three objects (t3-d), chosen and marked before the `u4` training (spread over the
  `top` view, not where demonstrations were placed on purpose, the same for every policy) x 2 = 10 trials
  first (x 2 more = 20 for the paper).
- Start at the ready pose, gripper open.
- **Success**: the carrot in the black box, the Oreo in the large hole and the RITZ in the small hole,
  gripper open, within **180 s** (t3-d; `run.time_limit_s` 180; 120 s until t3-c). Note also each object
  alone (grasped, placed), the order (Oreo before RITZ) and a snack in the **wrong hole** (counted apart from
  a missed or dropped snack: it measures the sorting).
- **After a missed grasp**, note what the policy does: opens and grasps again (recovery), how many tries in a
  row and in which direction it missed, or carries the empty gripper on. (Until t3-c the comparison was u1
  (49) vs u3 (65), one snack per episode.)
- Record every trial on video (the policy runner has no recording yet: to be added), success / failure noted
  per trial.

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
- [ ] Black box and meal box at their marks.

## Rules nobody sees in the data (keep them fixed)
- Gripper orientation: rx, ry from the ready pose, rz follows the azimuth (`yaw.mode: follow_base`). The
  action is only x, y, z + gripper; changing the ready pose orientation or the yaw mode breaks the data.
- Teleop settings frozen (`configs/real.yaml`); LeRobot 0.6.1 (env `lerobot`).
- Dataset format: docs/decisions.md rec-1 (30 Hz; action [x, y, z, gripper]; state 6 joints + gripper).

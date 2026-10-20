# Pipeline guide: collect -> convert -> train -> check -> run (Simulation)

Every step is a command you type. Run them from the repository root (`cd ~/RB_Cobot` first).
Replace `NAME` with the dataset name of the round (round 1: `p1`).

| step | env | moves the robot |
|---|---|---|
| 1 collect | rb5 | yes (Real) |
| 2 check and convert | lerobot | no |
| 3 train | lerobot | no |
| 4 offline check | lerobot | no |
| 5 Simulation run | lerobot | the Simulation robot and the real gripper |

Where the files go (`data/` is not committed):

| folder | what | made by |
|---|---|---|
| `data/demos/real_<time>/` | one collection session: episodes (camera JPEGs), teleop.csv, gripper.csv, episodes.json | step 1 |
| `data/teleop/` | the teleop's own run log (every run, also without `--record`) | step 1 |
| `data/lerobot/NAME/` | the LeRobot dataset (parquet + AV1 videos) | step 2 |
| `data/train/act_NAME_<time>/` | training output: checkpoints, train_config.json | step 3 |
| `data/policy/` | Simulation run logs (CSV + JSON) | step 5 |
| `data/archive/` | old test outputs (scripts 04-21, camera tests, earlier sessions and runs), moved 2026-10-21 | - |

Kept in place for the tests and for comparison: the pilot session `data/demos/real_20261019_141929`, its
dataset `data/lerobot/pilot_front` and checkpoint `data/train/act_pilot_front_20261019_143936`.

The task text is the same in every step. (ACT does not use it, but the dataset and the run log record it.)

## 1. Collect (Real teleop, both cameras)

Checklist: Real mode on the tablet, leader arm and gripper powered, both cameras on USB 3
(`lsusb -t` shows 5000M), lighting as usual, block in the square, gripper open.

```bash
cd ~/RB_Cobot
conda activate rb5
python -m teleop --real --record --task "pick the green block and place it in the box"
```

- Keys: `→` or `n` = save the episode / start the next; `←` or `r` = discard and redo; `Esc` or `q` = end
  the session (an episode in progress is discarded).
- How to demonstrate: docs/data_rules.md ("How to demonstrate"). Round 1: 15 kept episodes.
- The session folder is printed at the end: `data/demos/real_<time>`.

## 2. Check and convert

```bash
conda activate lerobot
S=$(ls -td data/demos/real_* | head -1); echo $S          # the newest session
python scripts/31_convert_to_lerobot.py $S --check-only    # report only
python scripts/31_convert_to_lerobot.py $S --repo-id local/NAME
```

- Look at the report before converting: left-out episodes and why, image time error, frame coverage of
  both cameras, gripper following.
- Several sessions in one dataset: list them all (`... data/demos/real_A data/demos/real_B --repo-id ...`).
- Output: `data/lerobot/NAME`. Building it again: add `--overwrite`.
- Time: about 1.1 x the recorded time per camera (15 episodes x 25 s x 2 cameras: about 15 min).

## 3. Train (ACT, both cameras)

```bash
conda activate lerobot
OUT=data/train/act_NAME_$(date +%Y%m%d_%H%M%S); echo $OUT
lerobot-train --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --dataset.repo_id=local/NAME --dataset.root=data/lerobot/NAME \
  --batch_size=8 --steps=20000 --save_freq=5000 --log_freq=100 --wandb.enable=false \
  --output_dir=$OUT
```

- A log line every 100 steps: `loss` should fall (pilot: 10.5 -> 1.2 in 2000 steps). The step time
  (`updt_s`) times the steps left = time left.
- Checkpoints: `$OUT/checkpoints/005000`, `010000`, ... and `last` (about 600 MB each).
- Keep the laptop on power and awake. Stopped? Resume:
  `lerobot-train --config_path=$OUT/checkpoints/last/pretrained_model/train_config.json --resume=true`
- GPU in use: `nvidia-smi` in another terminal.

## 4. Offline check (no robot)

```bash
python scripts/33_policy_check.py $OUT/checkpoints/last/pretrained_model --repo-id local/NAME
```

- Prediction vs the recorded action on training frames (mm at +1 / +30 / +99 steps, gripper) and the
  inference time. Pilot (6 episodes, 2000 steps): +1 step p50 28 mm. Training frames only: it shows what
  the model learned, not how it does on the robot.

## 5. Simulation run

Checklist: tablet in Simulation mode, the **real arm at the ready pose** (the policy reads the arm from the
front image), block in the square, gripper powered, fingers clear.

```bash
python scripts/35_run_policy.py $OUT/checkpoints/last/pretrained_model --time-limit 30 \
  --task "pick the green block and place it in the box"
```

- Typed answers: brightness (only if out of range) -> ready pose `yes` -> gripper start `yes` -> first
  action distance `yes`; then `g` = run, `space` = hold, `q` = quit.
- Over 100 mm between the policy's action and the target holds (safety). In Simulation the real arm does
  not move, so the image never changes: the robot goes to one point and stays.
- Log: `data/policy/run_sim_<time>.csv` / `.json`.

## A new terminal

`$S` and `$OUT` are lost in a new terminal. Set them again:
`S=data/demos/real_<time>`, `OUT=data/train/act_NAME_<time>` (`ls -t data/demos data/train`).

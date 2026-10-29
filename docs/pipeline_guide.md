# Pipeline guide: collect -> convert -> train -> check -> run (Simulation)

Every step is a command you type. Run them from the repository root (`cd ~/RB_Cobot` first).
Replace `NAME` with the dataset name of the round (round 1: `p1`).

## Code and settings (p2)

| what | code | settings |
|---|---|---|
| teleop and recording | `teleop/` (`python -m teleop`) | `configs/real.yaml` / `sim.yaml` (robot, leader arm, gripper, cameras: `record.cameras`) |
| dataset format (state, action, fps, image resize) | `rb5_lerobot/data_format.py` | none: changing it means changing recording, converter and run together |
| convert | `rb5_lerobot/convert.py` | `configs/policy.yaml` dataset |
| train | `rb5_lerobot/train.py` -> LeRobot's `lerobot-train` | `configs/policy.yaml` train (`act`: any ACT option) |
| offline check | `rb5_lerobot/check.py` | - |
| run (Simulation) | `rb5_lerobot/run.py`, `rb5_lerobot/policy.py` | `configs/policy.yaml` run; hardware from `configs/sim.yaml` |

To change a parameter: edit `configs/policy.yaml` and run the command again (what to redo: the table at the
end). To keep variants side by side: copy it (`configs/policy_<variant>.yaml`) and pass `--config`. Every
output keeps the settings it was made with: `rb5_convert.yaml` in the dataset, `rb5_train.yaml` in the training
folder, the run section in the run JSON.

| step | env | moves the robot |
|---|---|---|
| 1 collect | rb5 | yes (Real) |
| 2 check and convert | lerobot | no |
| 3 train | lerobot | no |
| 4 offline check | lerobot | no |
| 5 Simulation run | lerobot | the Simulation robot and the real gripper |
| 6 Real run (`--real`) | lerobot | **yes (Real)** |

Where the files go (`data/` is not committed):

| folder | what | made by |
|---|---|---|
| `data/demos/real_<time>/` | one collection session: episodes (camera JPEGs), teleop.csv, gripper.csv, episodes.json | step 1 |
| `data/teleop/` | the teleop's own run log (every run, also without `--record`) | step 1 |
| `data/lerobot/NAME/` | the LeRobot dataset (parquet + AV1 videos), rb5_convert.yaml | step 2 |
| `data/train/act_NAME_<time>/` | training output: checkpoints, rb5_train.yaml | step 3 |
| `data/policy/` | Simulation run logs (CSV + JSON) | step 5 |
| `data/cameras/` | camera tests (scripts 30, 34) | - |
| `data/archive/` | old outputs (scripts 04-21, camera tests, earlier sessions and runs), moved 2026-10-21 | - |

Before the p1 collection (2026-10-21) everything from the D405 time went to `data/archive/`, so `data/demos/`
holds only this round's sessions (`data/demos/real_*` picks no old one). The pilot (session
`archive/demos/real_20261019_141929`, dataset `archive/lerobot/pilot_front`, checkpoint
`archive/train/act_pilot_front_20261019_143936`) stays there for `tests/test_policy_obs.py` and comparison.

The task text is the same in every step. (ACT does not use it, but the dataset and the run log record it.)

## 1. Collect (Real teleop, three cameras)

Checklist: Real mode on the tablet, leader arm and gripper powered, D435 on USB 3 (`lsusb -t` shows 5000M),
both webcams connected on different USB ports (`ls /dev/v4l/by-id` shows `usb-046d_0825_...` = top, fixed above
the workspace, and `usb-046d_Brio_101_...` = grip, on the gripper; USB 2 devices), no other program using a
camera (realsense-viewer, ffplay), lighting as usual, objects placed (docs/data_rules.md), gripper open. Cameras:
`record.cameras` in configs/real.yaml (front, top, grip; t2-a; p1 / p2 were recorded with `wrist` on the gripper).

```bash
cd ~/RB_Cobot
conda activate rb5
python -m teleop --real --record --task "pick the green block and place it in the box"
```

- Keys: `→` or `n` = save the episode / start the next; `←` or `r` = discard and redo; `Esc` or `q` = end
  the session (an episode in progress is discarded).
- How to demonstrate: docs/data_rules.md ("How to demonstrate"). Round 1: 50 kept episodes in sessions of
  10-15 (docs/data_rules.md); check each session (`--check-only`, step 2) before the next one.
- The session folder is printed at the end: `data/demos/real_<time>`.

## 2. Check and convert

```bash
conda activate lerobot
S=$(ls -td data/demos/real_* | head -1); echo $S          # the newest session
python -m rb5_lerobot.convert $S --check-only    # report only
python -m rb5_lerobot.convert $S --name NAME
```

- Look at the report before converting: left-out episodes and why, image time error, frame coverage of
  the cameras, gripper following.
- Several sessions in one dataset: list them all (`... data/demos/real_A data/demos/real_B --name ...`).
- Image size and which episodes are left out: `configs/policy.yaml` dataset (images 320x240; the recordings
  stay 640x480). The dataset keeps a copy of the settings it was built with: `rb5_convert.yaml`.
- Output: `data/lerobot/NAME`. Building it again: add `--overwrite`.
- Time: about 1.1 x the recorded time per camera (15 episodes x 25 s x 2 cameras: about 15 min).

## 3. Train (ACT)

```bash
conda activate lerobot
python -m rb5_lerobot.train NAME --dry-run    # the settings and the lerobot-train command, no training
python -m rb5_lerobot.train NAME              # trains; prints the output folder data/train/act_NAME_<time>
python -m rb5_lerobot.train NAME --steps 100000 --wandb   # this run only: 100k steps, curves on wandb.ai
OUT=data/train/act_NAME_<time>                # set it to the printed folder (for steps 4 and 5)
```

- WandB (t3-e): once before, `wandb login` (key from https://wandb.ai/authorize). `--wandb` logs loss, grad
  norm, learning rate and step times to project `train.wandb.project`, run name = the output folder; no
  checkpoint upload (`disable_artifact: true`). The terminal prints the run's address.

- Settings: `configs/policy.yaml` train (batch size, steps, checkpoint interval, ACT options). Another file:
  `--config configs/<copy>.yaml`. The folder keeps `rb5_train.yaml`: the settings, the command and the dataset's
  `rb5_convert.yaml`.
- A log line every `log_freq` steps: `loss` should fall (pilot: 10.5 -> 1.2 in 2000 steps). The step time
  (`updt_s`) times the steps left = time left. Three cameras at 320x240, batch 8: about 0.26 s per step and
  1.8 GB of GPU memory (p2-c trial), so 20000 steps take about 1.5 h.
- Checkpoints: `$OUT/checkpoints/005000`, `010000`, ... and `last` (about 600 MB each).
- Keep the laptop on power and awake. Ctrl+C stops it (the checkpoints so far stay). Continue:
  `python -m rb5_lerobot.train --resume $OUT` (with the settings saved in the checkpoint, not policy.yaml).
  Train longer: `--resume $OUT --steps 100000` (the new total; constant learning rate, so the same as one run).
  A resume continues the same WandB run if the training had `--wandb`.
- GPU in use: `nvidia-smi` in another terminal.

**More data (fine-tune, p3-e).** Convert the old and the new sessions together into a new dataset, then
start from a trained checkpoint:
```bash
python -m rb5_lerobot.convert data/demos/real_A data/demos/real_NEW --name p2
python -m rb5_lerobot.train p2 --from data/train/act_p1_<time>/checkpoints/050000   # new folder, step 0
```
- `--from` = lerobot-train `--policy.path`: the checkpoint's weights and ACT structure; the normalization
  statistics come from the new dataset; a new optimizer and step count (`train.steps`, e.g. 10000-20000).
- Refused when the dataset's cameras, image size, state or action differ from the checkpoint, or when
  `train.act` changes the model structure (chunk_size, dim_model, ...). Changed training options (optimizer_lr,
  kl_weight, dropout, n_action_steps, ...) are passed.
- `--resume` is not for this: it continues the same training (same dataset, statistics and schedule).
- Do not fine-tune on the new sessions alone: the policy forgets what the old ones taught.

## 4. Offline check (no robot)

```bash
python -m rb5_lerobot.check $OUT                 # the last checkpoint, on the dataset it was trained on
python -m rb5_lerobot.check $OUT/checkpoints/005000 --dataset NAME   # another checkpoint or dataset
```

- Prediction vs the recorded action on training frames (mm at +1 / +30 / +99 steps, gripper) and the
  inference time. Pilot (6 episodes, 2000 steps): +1 step p50 28 mm. Training frames only: it shows what
  the model learned, not how it does on the robot. Three cameras at 320x240: inference about 17 ms on the GPU.

## 5. Simulation run

Checklist: tablet in Simulation mode, the **real arm at the ready pose** (the policy reads the arm from the
front image), block in the square, gripper powered, fingers clear.

```bash
python -m rb5_lerobot.run $OUT --task "pick the green block and place it in the box"
```

- Settings: `configs/policy.yaml` run (time limit, hold distance, ensembling, CPU cores); the hardware from
  configs/sim.yaml. The camera images are shrunk to the size the checkpoint was trained on (printed at the
  start; about 2 ms per camera), with the converter's function.
- Typed answers: brightness (only if out of range) -> ready pose `yes` -> gripper start `yes` -> first
  action distance `yes`; then `g` = run, `space` = hold, `q` = quit.
- Inference runs only while running: `g` starts it and the robot follows from the first chunk (about one
  inference later; the RUN line prints how far its first action is); every hold pauses it. The time limit
  counts from each `g`: one program run can hold several attempts (`space`, put the block back, `g`).
- A training folder runs its `checkpoints/last`, printed and logged as its step (e.g. `checkpoints/040000`).
  Do not run while a training (or `--resume`) uses the GPU: Real 071457 had 60 ms inference instead of 22 ms.
- An action further than `run.max_jump_mm` (100 mm) from the target holds (safety). In Simulation the real
  arm does not move, so the image never changes: the robot goes to one point and stays.
- Log: `data/policy/run_sim_<time>.csv` / `.json` (the JSON keeps the run settings and image sizes used).

## 6. Real run (p3-c)

**The real arm follows the policy on its own.** The controller has collision detection Off and tool mass
0 kg. Do a Simulation run of the same checkpoint first.

Checklist: tablet in Real mode allowed, nobody inside the reach of the arm, **hand on the emergency stop**,
the real arm at the ready pose (or let the run move it there), block in the square, gripper powered and open,
fingers clear, the cameras as in the recording (lighting as usual).

```bash
python -m rb5_lerobot.run $OUT --real
```

- Hardware from configs/real.yaml (`--robot-config` for another file) with the teleop's Real checks: Real
  mode reached, TCP offset as expected, the ready pose checked with calc_fk_tcp before moving there, a
  warning (own `yes`) for large wrist turns. Every target goes through the teleop's Real limits (speed,
  acceleration, z_min 19 mm, joint limits): speed caps 100 % of the teleop.
- Typed answers: `REAL` -> brightness (only if out of range) -> ready pose `yes` -> gripper start `yes` ->
  first action distance `yes`; then `g` = run, `space` = hold, `q` = quit (Ctrl+C also stops).
- The state given to the policy is the encoder joints (jnt_ang), as in the Real recordings.
- Log: `data/policy/run_real_<time>.csv` / `.json`.

## What to redo after changing configs/policy.yaml

| changed | redo | effect |
|---|---|---|
| `dataset.image_width / image_height` | convert (`--overwrite` or a new name), train | larger: more GPU memory and slower inference (3 cameras at 640x480: out of memory at batch 8, 50 ms) |
| `dataset.min_*`, `keep_still`, `cut_pauses_s` | convert, train | which episodes and stretches are in the dataset |
| `train.batch_size` | train | memory and time per step |
| `train.steps`, `save_freq` | train | training time, checkpoints |
| `train.wandb.*` | train | whether and where the curves go (WandB) |
| `train.act.*` | train | the model and how it learns |
| `run.*` | nothing (the next run uses it) | the run only |

## A new terminal

`$S` and `$OUT` are lost in a new terminal. Set them again:
`S=data/demos/real_<time>`, `OUT=data/train/act_NAME_<time>` (`ls -t data/demos data/train`).

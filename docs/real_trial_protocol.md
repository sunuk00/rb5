# Real teleop trial protocol (real-1)

**Status (2026-10-16):** not used in the end. The settings were changed step by step from the measured hand
speed and the user's feedback instead (docs/decisions.md, real-1). Kept for comparing settings with
fixed trials later, e.g. before demonstration collection.

Goal: make Real teleop fast enough for demonstration collection, changing one setting at a time and
measuring each change the same way. Summary tool: `scripts/23_trial_summary.py`.

## Task (the same every trial)
The gripper is not connected, so the task is a reach task:
1. `python -m teleop --real`: the RB5 goes to the ready pose (start of every trial).
2. Move above **P**, go down to the agreed height, then lift ("pick").
3. Move above **Q**, go down to the agreed height ("place").
4. Release the clutch, Ctrl+C.

P and Q are taped on the table: both at a radius of about 500 mm, about 300 mm apart (coordinates in
the RB5 base frame: TBD, from the user). Down height: TBD, above the provisional z floor (30 mm).
Pick and place are found from the measured TCP (within `--radius` of P / Q and at or below
`--z-max`), not from the trigger: the trigger read 0 for whole runs (20261014_035701).

## Rules
- Same operator for all trials of a comparison.
- **3 practice trials before the measured trials, for the baseline and again after every setting
  change** (same number each time, so practice affects every setting equally).
- **5 measured trials** per setting. Time limit 120 s per trial (over = `timeout`).
- One run = one trial = one log (`data/teleop/teleop_real_<time>.csv`).
- After each trial, one line in `data/teleop/trials.csv`:
  ```
  run,practice,success,failure_type,note
  teleop_real_20261016_101500,1,1,,practice
  teleop_real_20261016_102200,0,0,missed_P,overshoot when going down
  ```
  `failure_type`: `missed_P`, `missed_Q`, `timeout`, `safety_stop` (the teleop stopped itself),
  `contact` (touched something), `other`; empty when successful.
- Stop the series and go back to the previous setting after any `contact`, `safety_stop`, late cycle
  or anything that felt unsafe.

## Measures (per trial, from the log)
| Measure | Meaning |
|---|---|
| time | first clutch press -> place (successful trials) |
| clutch presses | how often the operator had to re-grip ("dragging") |
| follow % | target path / (hand path x scale): share of the hand motion the robot did |
| dropped mm | hand motion thrown away by the limits |
| speed / joint / range / yaw % | share of engaged cycles cut by each limit: shows the bottleneck |
| slow %, late | singularity / joint-limit slowdowns and late cycles: safety margin of the change |

```
python scripts/23_trial_summary.py data/teleop/teleop_real_2026101*.csv --trials data/teleop/trials.csv \
    --p PX PY --q QX QY --z-max Z --radius 30
```

## Plan (one change per step, each measured as above)
**2026-10-16:** the user applied option B (TCP 200 mm/s, joints 60 deg/s) directly from the measured
hand speed, without the baseline trials (decisions 2026-10-16). The protocol stays available to
compare settings, e.g. before going to option A.

Evidence before the protocol (5 Real runs on 2026-10-14, free motion, not the standard task; scale
1.0, 100 mm/s, C 20 mm): TCP speed limit active in **72-91 %** of engaged cycles, follow **30-79 %**,
dropped hand motion 0.3-3.9 m per run, 4-33 clutch presses, joint speed limit 0-10 %, yaw limit 0 %.
So the TCP speed cap is the main bottleneck, and the joint cap is next.

| Step | Change (real.yaml) | Why | Watch |
|---|---|---|---|
| 0 | none: scale 1.0, 100 mm/s, C 20 mm, joints 0.8 deg/cycle | baseline | - |
| 1 | TCP speed 100 -> 150 mm/s | main bottleneck | joint % rises: at the far / low corners the motion needs up to 0.393 deg/mm, so 150 mm/s needs up to 59 deg/s against the 40 deg/s joint cap |
| 2 | joint speed 0.8 -> 1.2 deg/cycle (60 deg/s) | only if joint % rose clearly in step 1 | slow %, late, how the arm looks |
| 3 | TCP speed 150 -> 200 mm/s (and joints if needed) | only if step 1-2 improved time / presses and stayed safe | same |
| 4 | C lag 20 -> 40 mm | only if dropped mm is still large while speed % is low | the robot keeps moving up to 40 mm after the hand stops (0.4 s at 100 mm/s) |

- Steps 1-3 raise Real limits in `teleop/settings.py` (`REAL_MAX`: TCP 100 mm/s, joints 0.8
  deg/cycle). These are safety limits: each raise is asked and approved separately before it is made.
- The RB5 joint speed specification is still unknown (manual: "joint max 180 deg/s" for the E series).
- Keep a step if the median time or presses improved and nothing in "Rules / stop" happened.
- Collision detection is Off and tool mass is 0 kg: higher speeds raise the risk. Emergency stop in
  reach, a second person watching.

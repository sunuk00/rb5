"""p2-c: train ACT on a dataset made by rb5_lerobot.convert, with the train section of configs/policy.yaml.

Run in the `lerobot` conda env, from the repository root:
    conda activate lerobot
    python -m rb5_lerobot.train p1                                  # dataset data/lerobot/p1 -> data/train/act_p1_<time>
    python -m rb5_lerobot.train p1 --config configs/my_policy.yaml  # another settings file
    python -m rb5_lerobot.train p1 --dry-run                        # print the lerobot-train command only
    python -m rb5_lerobot.train --resume data/train/act_p1_<time>   # continue a stopped training
    python -m rb5_lerobot.train p2 --from data/train/act_p1_<time>/checkpoints/050000   # fine-tune (p3-e)

It builds the lerobot-train command (LeRobot does the training), prints it, and runs it. Each train.act entry
becomes --policy.<name>=<value>. Once lerobot-train has made the output folder, rb5_train.yaml is written into
it: the settings file, the train section, the command, and the dataset's rb5_convert.yaml. A resume continues
with the settings saved in the checkpoint (train_config.json), not with policy.yaml.

--from CHECKPOINT (p3-e) starts a new training (new folder, step 0, new optimizer) from a trained policy's
weights on any dataset with the same inputs (cameras, image size, state and action sizes): lerobot-train
--policy.path. LeRobot then takes the normalization statistics from the new dataset. The ACT structure
(chunk_size, dim_model, ...) is the checkpoint's: a train.act entry that differs from it is passed only if
it is a training setting (learning rate, kl_weight, dropout, ...); a different structure is refused.

Ctrl+C stops the training; the checkpoints saved so far stay (every train.save_freq steps).
"""
import argparse
import copy
import json
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

from rb5_lerobot.config import DEFAULT, ROOT, load_policy_config

DATASETS = ROOT / "data" / "lerobot"
TRAINS = ROOT / "data" / "train"
# ACT options that may change when fine-tuning (--from); every other option is the model's structure
TRAINING_OPTIONS = {"n_action_steps", "temporal_ensemble_coeff", "dropout", "kl_weight", "optimizer_lr",
                    "optimizer_weight_decay", "optimizer_lr_backbone"}


def cli_value(v):
    """A YAML value as lerobot-train expects it on the command line (true / false in lower case)."""
    return str(v).lower() if isinstance(v, bool) else str(v)


def train_command(name, train, output_dir, start=None):
    """The lerobot-train argument list for dataset <name> and the train section (a dict). start: a checkpoint
    (pretrained_model folder) to fine-tune from; then train["act"] holds only the options to override."""
    policy = [f"--policy.path={start}"] if start else ["--policy.type=act"]
    cmd = ["lerobot-train", *policy, f"--policy.device={train['device']}", "--policy.push_to_hub=false",
           f"--dataset.repo_id=local/{name}", f"--dataset.root={(DATASETS / name).relative_to(ROOT)}",
           f"--batch_size={train['batch_size']}", f"--steps={train['steps']}", f"--save_freq={train['save_freq']}",
           f"--log_freq={train['log_freq']}", f"--num_workers={train['num_workers']}", f"--seed={train['seed']}",
           "--wandb.enable=false", f"--output_dir={output_dir}"]
    return cmd + [f"--policy.{k}={cli_value(v)}" for k, v in train["act"].items()]


def fine_tune_problems(checkpoint, dataset_root, act):
    """(problems, overrides): the checkpoint's inputs vs the dataset's features, and the train.act entries
    that differ from the checkpoint's config (structure -> a problem, training options -> overrides)."""
    cfg = json.loads((Path(checkpoint) / "config.json").read_text())
    feats = json.loads((Path(dataset_root) / "meta" / "info.json").read_text())["features"]
    problems = []
    for kind in ("input_features", "output_features"):
        for key, f in cfg[kind].items():
            if key not in feats:
                problems.append(f"the checkpoint needs {key}, the dataset has none")
                continue
            shape = feats[key]["shape"]
            if key.startswith("observation.images."):
                shape = [shape[2], shape[0], shape[1]]      # dataset H, W, C -> policy C, H, W
            if list(shape) != list(f["shape"]):
                problems.append(f"{key}: checkpoint {f['shape']}, dataset {list(shape)}")
    extra = [k for k in feats if k.startswith("observation.images.") and k not in cfg["input_features"]]
    if extra:
        problems.append(f"the dataset has {', '.join(extra)}, the checkpoint does not use them")
    overrides = {}
    for k, v in act.items():
        if k in cfg and cfg[k] == v:
            continue
        if k in TRAINING_OPTIONS:
            overrides[k] = v
        else:
            problems.append(f"train.act.{k}: {v} in the settings, {cfg.get(k)} in the checkpoint (model structure: "
                            f"set it to the checkpoint's value, or train without --from)")
    return problems, overrides


def resume_command(run_dir):
    """The lerobot-train argument list that continues run_dir from its last checkpoint."""
    cfg = Path(run_dir) / "checkpoints" / "last" / "pretrained_model" / "train_config.json"
    return ["lerobot-train", f"--config_path={cfg}", "--resume=true"]


def run(cmd, output_dir=None, record=None):
    """Run cmd in the repository root; write record (a dict) as output_dir/rb5_train.yaml once the folder exists."""
    if shutil.which(cmd[0]) is None:
        raise SystemExit(f"{cmd[0]} not found: conda activate lerobot")
    print("\n" + " \\\n  ".join(cmd) + "\n", flush=True)
    proc = subprocess.Popen(cmd, cwd=ROOT)
    written = record is None
    try:
        while proc.poll() is None:
            if not written and (ROOT / output_dir).is_dir():
                (ROOT / output_dir / "rb5_train.yaml").write_text(yaml.safe_dump(record, sort_keys=False))
                written = True
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nCtrl+C: waiting for lerobot-train to stop ...")
        proc.wait()
    if not written and (ROOT / output_dir).is_dir():
        (ROOT / output_dir / "rb5_train.yaml").write_text(yaml.safe_dump(record, sort_keys=False))
    return proc.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("name", nargs="?", help="dataset name (data/lerobot/<name>, made by rb5_lerobot.convert)")
    ap.add_argument("--config", default=str(DEFAULT), help="policy settings file (its train section)")
    ap.add_argument("--resume", metavar="RUN_DIR", help="continue this training folder from its last checkpoint")
    ap.add_argument("--from", dest="start", metavar="CHECKPOINT",
                    help="fine-tune: start from this trained policy (a training folder or checkpoints/<step>)")
    ap.add_argument("--dry-run", action="store_true", help="print the command, do not train")
    args = ap.parse_args()

    if args.resume and args.start:
        raise SystemExit("--resume continues a training; --from starts a new one: give one of them")
    if args.resume:
        run_dir = Path(args.resume)
        cmd = resume_command(run_dir)
        if not (ROOT / cmd[1].split("=", 1)[1]).exists():
            raise SystemExit(f"no checkpoint to resume: {cmd[1].split('=', 1)[1]} not found")
        print(f"Resume {run_dir} (settings of its checkpoint, not {args.config})")
        if not args.dry_run:
            sys.exit(run(cmd))
        print(" ".join(cmd))
        return
    if not args.name:
        raise SystemExit("give the dataset name (python -m rb5_lerobot.train NAME) or --resume RUN_DIR")

    _, raw = load_policy_config(args.config)
    root = DATASETS / args.name
    if not (root / "meta" / "info.json").exists():
        have = sorted(p.name for p in DATASETS.iterdir() if p.is_dir()) if DATASETS.is_dir() else []
        raise SystemExit(f"no dataset {root} (made by rb5_lerobot.convert --name {args.name}); have: {have}")
    made = yaml.safe_load((root / "rb5_convert.yaml").read_text()) if (root / "rb5_convert.yaml").exists() else None
    output_dir = (TRAINS / f"act_{args.name}_{datetime.now():%Y%m%d_%H%M%S}").relative_to(ROOT)
    t = raw["train"]
    start = None
    if args.start:
        from rb5_lerobot.policy import resolve_checkpoint
        start = resolve_checkpoint(args.start)
        problems, overrides = fine_tune_problems(start, root, t["act"])
        if problems:
            raise SystemExit(f"NOT STARTING - --from {start}:\n  " + "\n  ".join(problems))
        t = copy.deepcopy(t)
        t["act"] = overrides
    cmd = train_command(args.name, t, output_dir, start)
    print(f"Settings: {args.config}")
    if made:
        d = made["dataset"]
        print(f"Dataset {args.name}: {made['episodes']} episodes, {made['frames']} frames, cameras "
              f"{', '.join(made['cameras'])}, images {d['image_width']}x{d['image_height']}")
    if start:
        print(f"Fine-tune from {start} (its weights and ACT structure; normalization from {args.name})")
    print(f"Training: batch {t['batch_size']}, {t['steps']} steps (checkpoint every {t['save_freq']}), "
          f"ACT {', '.join(f'{k} {v}' for k, v in t['act'].items()) or 'as the checkpoint'}")
    print(f"Output: {output_dir}")
    if args.dry_run:
        print("\n" + " ".join(cmd))
        return
    record = {"made": datetime.now().isoformat(timespec="seconds"), "config_file": str(args.config),
              "train": t, "command": " ".join(cmd), "dataset": args.name, "dataset_made": made,
              "from": str(start) if start else None}
    code = run(cmd, output_dir, record)
    print(f"\nlerobot-train ended (exit code {code}). Checkpoints: {output_dir}/checkpoints/")
    if code != 0:
        print(f"Continue a stopped training: python -m rb5_lerobot.train --resume {output_dir}")
    sys.exit(code)


if __name__ == "__main__":
    main()

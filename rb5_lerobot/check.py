"""rec-5: check a trained policy offline (no robot): predictions on dataset frames and inference time.

Run in the `lerobot` conda env, from the repository root:
    python -m rb5_lerobot.check data/train/act_<name>_<time>                 # its last checkpoint, its dataset
    python -m rb5_lerobot.check <training or checkpoint folder> --dataset p1 [--frames 200] [--two-cameras]
The dataset defaults to the one the training used (rb5_train.yaml in the training folder).

1. Loads the policy and its pre / post processors (normalization) from the checkpoint.
2. For --frames frames spread over the dataset: the predicted action chunk (chunk_size steps) vs the
   recorded actions of the next steps (training data: this shows that the pipeline works and what the
   model learned, not how it does on the robot). Errors in mm (x, y, z) and in gripper fraction.
3. Inference time for one observation (batch 1): the chunk (predict_action_chunk) on the GPU and on the
   CPU, preprocessing included. --two-cameras also times an untrained ACT with two 480x640 cameras (the
   rec-1 format) to see the cost of the wrist camera.
4. --endurance MINUTES (rec-6b): inference on the runner's schedule (one per 1/30 s, at once when late) for
   that long with an untrained ACT and two 480x640 cameras (the dataset format; the weights do not change
   the time), the GPU temperature / SM clock / power / utilization from nvidia-smi every 2 s; prints a row
   per 30 s and saves data/policy/endurance_<time>.csv. Shows whether the laptop GPU slows down when hot.
"""
import csv
import subprocess
from datetime import datetime
import argparse
import time
from pathlib import Path

import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parent.parent


def load(path, device):
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    policy = ACTPolicy.from_pretrained(path)
    policy.config.device = device
    policy.to(device).eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(path),
                                         preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post


def observation(item, keys, device):
    obs = {k: item[k].unsqueeze(0).to(device) for k in keys}
    obs["task"] = [item["task"]]
    return obs


@torch.inference_mode()
def chunk(policy, pre, post, obs):
    policy.reset()
    a = policy.predict_action_chunk(pre(obs))          # (1, chunk, action dim), normalized
    return post(a)[0].float().cpu().numpy()           # unnormalized


def timing(policy, pre, post, obs, device, n):
    for _ in range(5):
        chunk(policy, pre, post, obs)
    times = []
    for _ in range(n):
        if device == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        chunk(policy, pre, post, obs)
        if device == "cuda":
            torch.cuda.synchronize()
        times.append((time.perf_counter() - t0) * 1000)
    return np.percentile(times, [50, 95]), max(times)


def gpu_sample():
    """(temperature C, SM clock MHz, power W, utilization %) from nvidia-smi, or Nones."""
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu,clocks.sm,power.draw,utilization.gpu",
                              "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=2).stdout
        return tuple(float(v) for v in out.strip().split(","))
    except (OSError, ValueError, subprocess.SubprocessError):
        return (None, None, None, None)


def two_camera_act():
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    c = ACTConfig(device="cuda", pretrained_backbone_weights=None,
                  input_features={"observation.state": PolicyFeature(FeatureType.STATE, (7,)),
                                  "observation.images.front": PolicyFeature(FeatureType.VISUAL, (3, 480, 640)),
                                  "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, 480, 640))},
                  output_features={"action": PolicyFeature(FeatureType.ACTION, (4,))})
    p = ACTPolicy(c).to("cuda").eval()
    b = {"observation.state": torch.zeros(1, 7, device="cuda"),
         "observation.images.front": torch.rand(1, 3, 480, 640, device="cuda"),
         "observation.images.wrist": torch.rand(1, 3, 480, 640, device="cuda")}
    return p, b


@torch.inference_mode()
def endurance(minutes, period=1 / 30, window_s=30.0):
    policy, batch = two_camera_act()
    for _ in range(10):
        policy.predict_action_chunk(batch)
    torch.cuda.synchronize()
    out_dir = ROOT / "data" / "policy"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"endurance_{datetime.now():%Y%m%d_%H%M%S}.csv"
    rows, gpu_rows, win = [], [], []
    t0 = next_t = time.monotonic()
    next_gpu, next_print = t0, t0 + window_s
    print(f"{minutes:.0f} min, one inference per {period * 1000:.1f} ms (two 480x640 cameras); a row per {window_s:.0f} s")
    print("   time | infer p50 / p95 / max ms | over 33.3 ms | GPU C | SM MHz | W | util %")
    while time.monotonic() - t0 < minutes * 60:
        now = time.monotonic()
        if now >= next_gpu:
            gpu_rows.append((now - t0, *gpu_sample()))
            next_gpu += 2.0
        a = time.perf_counter()
        policy.predict_action_chunk(batch)
        torch.cuda.synchronize()
        ms = (time.perf_counter() - a) * 1000
        rows.append((now - t0, ms))
        win.append(ms)
        if time.monotonic() >= next_print:
            g = gpu_rows[-1]
            w = np.array(win)
            print(f"  {now - t0:5.0f}s | {np.percentile(w, 50):5.1f} / {np.percentile(w, 95):5.1f} / {w.max():5.1f} | "
                  f"{100 * np.mean(w > period * 1000):5.1f} % | {g[1]:.0f} | {g[2]:.0f} | {g[3]:.1f} | {g[4]:.0f}")
            win = []
            next_print += window_s
        next_t += period
        rest = next_t - time.monotonic()
        if rest > 0:
            time.sleep(rest)
        else:
            next_t = time.monotonic()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "infer_ms", "gpu_t_s", "temp_c", "sm_mhz", "power_w", "util_pct"])
        for i, (t, ms) in enumerate(rows):
            g = gpu_rows[min(i * len(gpu_rows) // len(rows), len(gpu_rows) - 1)]
            w.writerow([f"{t:.3f}", f"{ms:.2f}"] + ["" if v is None else f"{v:.1f}" for v in g])
    ms = np.array([r[1] for r in rows])
    first, last = ms[: len(ms) // 10], ms[-len(ms) // 10:]
    temps = [g[1] for g in gpu_rows if g[1] is not None]
    clocks = [g[2] for g in gpu_rows if g[2] is not None]
    print(f"\n{len(ms)} inferences: p50 {np.percentile(ms, 50):.1f} ms, p95 {np.percentile(ms, 95):.1f}, max {ms.max():.1f}; "
          f"over one 30 Hz step {100 * np.mean(ms > period * 1000):.1f} %")
    print(f"first 10 %: p50 {np.percentile(first, 50):.1f} ms; last 10 %: p50 {np.percentile(last, 50):.1f} ms")
    print(f"GPU temperature {min(temps):.0f} -> {max(temps):.0f} C (last {temps[-1]:.0f}); SM clock {max(clocks):.0f} max, "
          f"{min(clocks):.0f} min, last {clocks[-1]:.0f} MHz")
    print(f"saved {path.relative_to(ROOT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", nargs="?", help="training folder data/train/act_<name>_<time> (its last "
                                                  "checkpoint) or a checkpoint folder")
    ap.add_argument("--dataset", help="dataset name in data/lerobot (default: the one the training used)")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--two-cameras", action="store_true")
    ap.add_argument("--endurance", type=float, metavar="MINUTES", help="only the long inference timing")
    args = ap.parse_args()
    if args.endurance:
        return endurance(args.endurance)
    if not args.checkpoint:
        ap.error("checkpoint needed (except with --endurance)")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from rb5_lerobot.policy import resolve_checkpoint
    args.checkpoint = resolve_checkpoint(args.checkpoint)
    if args.dataset is None:
        made = [p / "rb5_train.yaml" for p in Path(args.checkpoint).parents if (p / "rb5_train.yaml").exists()]
        if not made:
            raise SystemExit("--dataset NAME: the training folder has no rb5_train.yaml to say which dataset it used")
        args.dataset = yaml.safe_load(made[0].read_text())["dataset"]
    print(f"checkpoint {args.checkpoint}, dataset {args.dataset}")
    ds = LeRobotDataset(f"local/{args.dataset}", root=ROOT / "data" / "lerobot" / args.dataset)

    policy, pre, post = load(args.checkpoint, "cuda")
    cfg = policy.config
    keys = list(cfg.input_features)
    print(f"policy: ACT, chunk {cfg.chunk_size}, n_action_steps {cfg.n_action_steps}, inputs {keys}")

    # 2. Predictions vs the recorded actions (training data)
    actions = np.stack(ds.hf_dataset["action"]).astype(float)
    ep = np.array(ds.hf_dataset["episode_index"])
    idx = np.linspace(0, len(ds) - 1, args.frames).astype(int)
    errs = {k: [] for k in (1, 10, 30, 99)}
    grip = []
    for i in idx:
        pred = chunk(policy, pre, post, observation(ds[int(i)], keys, "cuda"))
        for k in errs:
            j = i + k
            if j < len(ds) and ep[j] == ep[i]:
                errs[k].append(np.linalg.norm(pred[k, :3] - actions[j, :3]))
        if i + 1 < len(ds) and ep[i + 1] == ep[i]:
            grip.append(abs(pred[1, 3] - actions[i + 1, 3]))
    print(f"\npredicted chunk vs recorded actions ({len(idx)} frames of the training data):")
    for k, e in errs.items():
        print(f"  step +{k:2d} ({k / ds.fps:4.2f} s ahead): position error p50 {np.percentile(e, 50):6.1f} mm, "
              f"p90 {np.percentile(e, 90):6.1f} mm")
    print(f"  gripper (step +1): error p50 {np.percentile(grip, 50):.3f}, p90 {np.percentile(grip, 90):.3f} (0..1)")
    moves = np.linalg.norm(np.diff(actions[:, :3], axis=0), axis=1)
    print(f"  (for scale: the recorded target moves p50 {np.percentile(moves, 50) * ds.fps:.0f} mm/s)")

    # 3. Inference time
    obs = observation(ds[int(idx[len(idx) // 2])], keys, "cuda")
    (g50, g95), gmax = timing(policy, pre, post, obs, "cuda", 100)
    print(f"\ninference (batch 1, chunk of {cfg.chunk_size}, preprocessing included):")
    print(f"  GPU (A1000): p50 {g50:.1f} ms, p95 {g95:.1f} ms, max {gmax:.1f} ms")
    policy_c, pre_c, post_c = load(args.checkpoint, "cpu")
    obs_c = observation(ds[int(idx[len(idx) // 2])], keys, "cpu")
    (c50, c95), cmax = timing(policy_c, pre_c, post_c, obs_c, "cpu", 20)
    print(f"  CPU:         p50 {c50:.1f} ms, p95 {c95:.1f} ms, max {cmax:.1f} ms")

    if args.two_cameras:
        from lerobot.configs.types import FeatureType, PolicyFeature
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        c2 = ACTConfig(device="cuda", pretrained_backbone_weights=None,
                       input_features={"observation.state": PolicyFeature(FeatureType.STATE, (7,)),
                                       "observation.images.front": PolicyFeature(FeatureType.VISUAL, (3, 480, 640)),
                                       "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, 480, 640))},
                       output_features={"action": PolicyFeature(FeatureType.ACTION, (4,))})
        p2 = ACTPolicy(c2).to("cuda").eval()
        b = {"observation.state": torch.zeros(1, 7, device="cuda"),
             "observation.images.front": torch.rand(1, 3, 480, 640, device="cuda"),
             "observation.images.wrist": torch.rand(1, 3, 480, 640, device="cuda")}
        with torch.inference_mode():
            for _ in range(5):
                p2.predict_action_chunk(b)
            ts = []
            for _ in range(100):
                torch.cuda.synchronize()
                t0 = time.perf_counter()
                p2.predict_action_chunk(b)
                torch.cuda.synchronize()
                ts.append((time.perf_counter() - t0) * 1000)
        print(f"  GPU, two cameras (untrained ACT, model only): p50 {np.percentile(ts, 50):.1f} ms, "
              f"p95 {np.percentile(ts, 95):.1f} ms, max {max(ts):.1f} ms")
    print(f"  GPU memory peak {torch.cuda.max_memory_allocated() / 1e9:.2f} GB")


if __name__ == "__main__":
    main()

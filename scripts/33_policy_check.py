"""rec-5: check a trained policy offline (no robot): predictions on dataset frames and inference time.

Run in the `lerobot` conda env:
    python scripts/33_policy_check.py data/train/<run>/checkpoints/last/pretrained_model \
        --repo-id local/pilot_front [--frames 200] [--two-cameras]

1. Loads the policy and its pre / post processors (normalization) from the checkpoint.
2. For --frames frames spread over the dataset: the predicted action chunk (chunk_size steps) vs the
   recorded actions of the next steps (training data: this shows that the pipeline works and what the
   model learned, not how it does on the robot). Errors in mm (x, y, z) and in gripper fraction.
3. Inference time for one observation (batch 1): the chunk (predict_action_chunk) on the GPU and on the
   CPU, preprocessing included. --two-cameras also times an untrained ACT with two 480x640 cameras (the
   rec-1 format) to see the cost of the wrist camera.
"""
import argparse
import time
from pathlib import Path

import numpy as np
import torch

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


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", help="…/checkpoints/<step>/pretrained_model")
    ap.add_argument("--repo-id", default="local/pilot_front")
    ap.add_argument("--root")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--two-cameras", action="store_true")
    args = ap.parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = Path(args.root) if args.root else ROOT / "data" / "lerobot" / args.repo_id.split("/")[-1]
    ds = LeRobotDataset(args.repo_id, root=root)

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

"""rec-2: write and read back a dummy LeRobotDataset with the rec-1 format (no robot, no cameras).

Run in the `lerobot` conda env (not `rb5`):
    conda activate lerobot
    python scripts/29_lerobot_dummy_dataset.py [--episodes 2] [--seconds 3] [--overwrite]

Format (docs/decisions.md, rec-1):
- 30 fps
- observation.state (7): RB5 joints J1..J6 [deg] + gripper position 0..1
- action (4): sent TCP target x, y, z [mm, base frame] + gripper command open_frac 0..1
- observation.images.front (D435, above the workspace) and .wrist (D405, gripper body): RGB 480x640, video

The values are synthetic (smooth sines, images with a moving square and the frame number) so that a
read-back can be compared with what was written. Then the action chunk that ACT asks for (100 steps)
is loaded with delta_timestamps. View the result with:
    lerobot-dataset-viz --repo-id local/dummy_rec1 --root data/lerobot/dummy_rec1 --episode-index 0
"""
import argparse
import shutil
import time
from pathlib import Path

import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

REPO_ID = "local/dummy_rec1"
ROOT = Path(__file__).resolve().parents[1] / "data" / "lerobot" / "dummy_rec1"
FPS = 30
H, W = 480, 640
TASK = "pick the block and place it in the box"
CHUNK = 100   # ACT default chunk_size

STATE_NAMES = ["j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg", "gripper_pos"]
ACTION_NAMES = ["x_mm", "y_mm", "z_mm", "gripper_open"]
CAMERAS = ["front", "wrist"]

FEATURES = {
    "observation.state": {"dtype": "float32", "shape": (len(STATE_NAMES),), "names": STATE_NAMES},
    "action": {"dtype": "float32", "shape": (len(ACTION_NAMES),), "names": ACTION_NAMES},
    **{f"observation.images.{cam}": {"dtype": "video", "shape": (H, W, 3),
                                     "names": ["height", "width", "channels"],
                                     "info": {"is_depth_map": False}} for cam in CAMERAS},
}

READY_Q = np.array([84.898, 1.808, -116.764, 24.718, -90.463, -175.743])   # configs/real.yaml ready_pose


def synthetic(ep, i):
    """State, action and images for frame i of episode ep."""
    t = i / FPS
    s = np.sin(2 * np.pi * 0.3 * t + ep)
    grip = 0.5 + 0.5 * np.cos(2 * np.pi * 0.2 * t)
    state = np.append(READY_Q + 5.0 * s, grip).astype(np.float32)
    action = np.array([50.0 + 100.0 * s, -450.0 + 50.0 * s, 200.0 - 30.0 * s, grip], dtype=np.float32)
    images = {}
    for k, cam in enumerate(CAMERAS):
        img = np.zeros((H, W, 3), dtype=np.uint8)
        img[..., 0] = np.linspace(0, 255, W, dtype=np.uint8)[None, :]   # red ramp left -> right
        img[..., 2] = 60 + 100 * k                                       # front / wrist differ in blue
        x = int((W - 80) * (0.5 + 0.5 * s))
        img[200:280, x:x + 80, 1] = 255                                  # green square that moves
        img[20:40, 20:20 + min(i, 600), :] = 255                         # white bar = frame number
        images[f"observation.images.{cam}"] = img
    return state, action, images


def write(episodes, frames):
    ds = LeRobotDataset.create(repo_id=REPO_ID, fps=FPS, features=FEATURES, root=ROOT,
                               robot_type="rb5_850", use_videos=True)
    for ep in range(episodes):
        t0 = time.monotonic()
        for i in range(frames):
            state, action, images = synthetic(ep, i)
            ds.add_frame({"observation.state": state, "action": action, **images, "task": TASK})
        t1 = time.monotonic()
        ds.save_episode()
        print(f"episode {ep}: {frames} frames added in {t1 - t0:.2f} s, saved (video encoding) in "
              f"{time.monotonic() - t1:.2f} s")
    # A discarded episode (the `d` key in rec-3): frames added, then cleared, nothing saved
    for i in range(10):
        state, action, images = synthetic(99, i)
        ds.add_frame({"observation.state": state, "action": action, **images, "task": TASK})
    ds.clear_episode_buffer()
    ds.finalize()


def check(episodes, frames):
    ds = LeRobotDataset(REPO_ID, root=ROOT)
    problems = []
    print(f"\nread back: {ds.num_episodes} episodes, {ds.num_frames} frames, fps {ds.fps}, "
          f"cameras {ds.meta.camera_keys}")
    if ds.num_episodes != episodes or ds.num_frames != episodes * frames:
        problems.append(f"expected {episodes} episodes / {episodes * frames} frames (discarded one excluded)")

    # Values: state and action exactly, images close (lossy video)
    for idx in (0, frames - 1, frames, ds.num_frames - 1):
        item = ds[idx]
        ep, i = int(item["episode_index"]), int(item["frame_index"])
        state, action, images = synthetic(ep, i)
        ds_err = float(np.abs(item["observation.state"].numpy() - state).max())
        da_err = float(np.abs(item["action"].numpy() - action).max())
        img = item["observation.images.front"]          # float 0..1, channels first
        ref = torch.from_numpy(images["observation.images.front"]).permute(2, 0, 1).float() / 255
        di = float((img - ref).abs().mean())
        print(f"  index {idx}: episode {ep} frame {i} t {float(item['timestamp']):.3f} s | state err "
              f"{ds_err:.1e}, action err {da_err:.1e} | front image {tuple(img.shape)} mean abs err {di:.4f} "
              f"| task '{item['task']}'")
        if ds_err > 1e-4 or da_err > 1e-3:
            problems.append(f"index {idx}: state / action differ from what was written")
        if di > 0.03:
            problems.append(f"index {idx}: image differs more than video compression would explain")

    # ACT-style action chunk: the next CHUNK actions, padded at the episode end
    chunk_ds = LeRobotDataset(REPO_ID, root=ROOT,
                              delta_timestamps={"action": [k / FPS for k in range(CHUNK)]})
    item = chunk_ds[frames - 10]    # 10 frames before the end of episode 0
    pad = item["action_is_pad"]
    print(f"  action chunk at frame {frames - 10}: shape {tuple(item['action'].shape)}, "
          f"padded steps {int(pad.sum())} (expected {CHUNK - 10})")
    if tuple(item["action"].shape) != (CHUNK, len(ACTION_NAMES)) or int(pad.sum()) != CHUNK - 10:
        problems.append("action chunk shape or padding is not as expected")

    size_mb = sum(f.stat().st_size for f in ROOT.rglob("*") if f.is_file()) / 1e6
    print(f"  on disk: {size_mb:.1f} MB for {ds.num_frames / FPS:.0f} s of data ({ROOT})")
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--seconds", type=float, default=3.0, help="length of each episode")
    ap.add_argument("--overwrite", action="store_true", help=f"delete {ROOT} first")
    args = ap.parse_args()

    if ROOT.exists():
        if not args.overwrite:
            raise SystemExit(f"{ROOT} exists; use --overwrite to replace it")
        shutil.rmtree(ROOT)
    frames = int(round(args.seconds * FPS))
    write(args.episodes, frames)
    problems = check(args.episodes, frames)
    print("\nOK" if not problems else "\nPROBLEMS:\n- " + "\n- ".join(problems))
    print(f"view: lerobot-dataset-viz --repo-id {REPO_ID} --root {ROOT} --episode-index 0")


if __name__ == "__main__":
    main()

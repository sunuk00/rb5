"""Tests for rb5_lerobot.config (configs/policy.yaml) and rb5_lerobot.data_format (p2-b). No LeRobot needed.
Run: python -m unittest discover tests"""
import copy
import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from rb5_lerobot.config import DEFAULT, check_policy_config, load_policy_config  # noqa: E402
from rb5_lerobot.data_format import ACTION_LEN, STATE_LEN, resize_image  # noqa: E402
from rb5_lerobot.train import resume_command, train_command  # noqa: E402

RAW = yaml.safe_load(DEFAULT.read_text())


def problems_with(change):
    d = copy.deepcopy(RAW)
    change(d)
    return check_policy_config(d)


class TestPolicyConfig(unittest.TestCase):
    def test_repository_file_loads(self):
        cfg, raw = load_policy_config()
        self.assertEqual((cfg.dataset.image_width, cfg.dataset.image_height), (320, 240))
        self.assertEqual(cfg.train.act.chunk_size, 100)
        self.assertEqual(cfg.run.servo_cores, (0, 5))
        self.assertIn("act", raw["train"])                       # the raw dict keeps the act section

    def test_unknown_and_missing_keys(self):
        self.assertIn("dataset.image_size: unknown key", problems_with(lambda d: d["dataset"].update(image_size=1)))
        self.assertIn("run.max_jump_mm: missing", problems_with(lambda d: d["run"].pop("max_jump_mm")))
        self.assertIn("train.batch_size: 0 is below 1", problems_with(lambda d: d["train"].update(batch_size=0)))

    def test_act_section_is_free_but_checked(self):
        self.assertEqual(problems_with(lambda d: d["train"]["act"].update(dropout=0.2)), [])   # any ACT option
        self.assertIn("train.act: expected a section of ACT options (e.g. chunk_size: 100)",
                      problems_with(lambda d: d["train"].pop("act")))
        self.assertTrue(problems_with(lambda d: d["train"]["act"].update(dim_model=[1, 2]))[0].startswith("train.act.dim_model"))
        self.assertIn("train.act: n_action_steps (120) must not exceed chunk_size (100)",
                      problems_with(lambda d: d["train"]["act"].update(n_action_steps=120)))

    def test_run_relations(self):
        p = problems_with(lambda d: d["run"].update(policy_cores=[4, 11]))
        self.assertTrue(any("overlap" in x for x in p))
        p = problems_with(lambda d: d["run"].update(stop_stale_s=0.4))
        self.assertTrue(any("stop_stale_s" in x for x in p))

    def test_real_run_settings(self):
        # p3-c: python -m rb5_lerobot.run --real loads configs/real.yaml with the Real checks; its cameras are
        # the recording's (the policy is given the same devices and fixed settings)
        from teleop.cameras import camera_defs
        from teleop.settings import load_settings
        s, _ = load_settings(ROOT / "configs" / "real.yaml", real=True)
        names = [d[0] for d in camera_defs(s.record, ["front", "wrist", "grip"])]
        self.assertEqual(names, ["front", "wrist", "grip"])


class TestTrainCommand(unittest.TestCase):
    def test_command_from_the_train_section(self):
        t = copy.deepcopy(RAW["train"])          # fixed values: the user may change configs/policy.yaml
        t.update(batch_size=8, steps=20000, save_freq=5000)
        t["act"].update(chunk_size=100, use_vae=True, optimizer_lr=1e-5)
        cmd = train_command("p1", t, "data/train/act_p1_x")
        self.assertEqual(cmd[0], "lerobot-train")
        for part in ("--policy.type=act", "--dataset.repo_id=local/p1", "--dataset.root=data/lerobot/p1",
                     "--batch_size=8", "--steps=20000", "--save_freq=5000", "--output_dir=data/train/act_p1_x",
                     "--policy.chunk_size=100", "--policy.use_vae=true", "--policy.optimizer_lr=1e-05",
                     "--wandb.enable=false", "--policy.push_to_hub=false"):
            self.assertIn(part, cmd)
        t = copy.deepcopy(RAW["train"]); t["act"]["dropout"] = 0.2; t["act"]["use_vae"] = False
        cmd = train_command("p1", t, "out")
        self.assertIn("--policy.dropout=0.2", cmd)
        self.assertIn("--policy.use_vae=false", cmd)

    def test_fine_tune(self):
        # p3-e: --from -> --policy.path (no --policy.type); only changed training options are passed
        import json
        import tempfile
        from rb5_lerobot.train import fine_tune_problems
        cmd = train_command("p2", {**RAW["train"], "act": {"optimizer_lr": 5e-6}}, "out", start="ck/pretrained_model")
        self.assertIn("--policy.path=ck/pretrained_model", cmd)
        self.assertFalse(any(c.startswith("--policy.type") for c in cmd))
        self.assertIn("--policy.optimizer_lr=5e-06", cmd)
        tmp = Path(tempfile.mkdtemp())
        ck, ds = tmp / "ck", tmp / "ds"
        (ds / "meta").mkdir(parents=True)
        ck.mkdir()
        cfg = {"input_features": {"observation.state": {"shape": [7]}, "observation.images.front": {"shape": [3, 240, 320]}},
               "output_features": {"action": {"shape": [4]}}, "chunk_size": 100, "optimizer_lr": 1e-5, "kl_weight": 10.0}
        (ck / "config.json").write_text(json.dumps(cfg))
        feats = {"observation.state": {"shape": [7]}, "observation.images.front": {"shape": [240, 320, 3]},
                 "action": {"shape": [4]}}
        (ds / "meta" / "info.json").write_text(json.dumps({"features": feats}))
        self.assertEqual(fine_tune_problems(ck, ds, {"chunk_size": 100, "optimizer_lr": 1e-5}), ([], {}))
        self.assertEqual(fine_tune_problems(ck, ds, {"optimizer_lr": 5e-6}), ([], {"optimizer_lr": 5e-6}))
        p, _ = fine_tune_problems(ck, ds, {"chunk_size": 50})
        self.assertTrue(p[0].startswith("train.act.chunk_size: 50 in the settings, 100 in the checkpoint"))
        feats["observation.images.front"]["shape"] = [480, 640, 3]
        feats["observation.images.grip"] = {"shape": [240, 320, 3]}
        (ds / "meta" / "info.json").write_text(json.dumps({"features": feats}))
        p, _ = fine_tune_problems(ck, ds, {})
        self.assertIn("observation.images.front: checkpoint [3, 240, 320], dataset [3, 480, 640]", p)
        self.assertIn("the dataset has observation.images.grip, the checkpoint does not use them", p)

    def test_resume(self):
        self.assertEqual(resume_command("data/train/act_p1_x"), [
            "lerobot-train", "--config_path=data/train/act_p1_x/checkpoints/last/pretrained_model/train_config.json",
            "--resume=true"])


class TestDataFormat(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual((STATE_LEN, ACTION_LEN), (7, 4))

    def test_resize(self):
        rng = np.random.default_rng(0)
        img = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
        small = resize_image(img, 320, 240)
        self.assertEqual((small.shape, small.dtype), ((240, 320, 3), np.uint8))
        self.assertTrue(np.array_equal(small, resize_image(img.copy(), 320, 240)))   # deterministic
        self.assertIs(resize_image(img, 640, 480), img)                               # same size: unchanged
        flat = np.full((480, 640, 3), 77, dtype=np.uint8)
        self.assertTrue((resize_image(flat, 320, 240) == 77).all())                   # a flat image stays flat


if __name__ == "__main__":
    unittest.main()

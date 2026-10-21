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

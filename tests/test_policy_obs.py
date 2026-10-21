"""rec-6b: the policy sees at run time what it saw in training (env lerobot; skipped without LeRobot or data).

Training path: a frame of the LeRobotDataset (images decoded from the AV1 video, float CHW 0..1; state)
-> the checkpoint's preprocessor. Run path: the same frame as the runner gets it (the camera image the
recording saved, HWC uint8 RGB; the state values) -> observation_batch() -> the same preprocessor.
Checked: keys and number of cameras, shapes, dtypes, value ranges before normalization, the color order
(the run path image is close to the training one and its R - B map correlates with the training one's:
about 0.98; swapped channels give about -0.98), the state
order and values (names, exact values, inside the dataset's range in degrees).

Data (in data/archive/): dataset local/pilot_front (front only, Real pilot 141929, episode 0), its session
folder for the images as recorded, and the rec-5 checkpoint. Run: conda run -n lerobot python -m unittest tests.test_policy_obs
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
ARCHIVE = ROOT / "data" / "archive"   # the pilot moved there before the p1 collection (2026-10-21)
DATASET = ARCHIVE / "lerobot" / "pilot_front"
SESSION = ARCHIVE / "demos" / "real_20261019_141929"
CKPTS = sorted(ARCHIVE.glob("train/act_pilot_front_*/checkpoints/last/pretrained_model"))
HAVE = importlib.util.find_spec("lerobot") is not None and DATASET.exists() and SESSION.exists() and CKPTS


@unittest.skipUnless(HAVE, "needs env lerobot, data/archive/lerobot/pilot_front, its session and a rec-5 checkpoint")
class TestTrainRunObservation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.policies.act.configuration_act import ACTConfig  # noqa: F401  (registers the type)
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import make_pre_post_processors
        from PIL import Image
        from teleop.policy import STATE_NAMES, observation_batch, policy_inputs
        cls.Image, cls.observation_batch, cls.STATE_NAMES = Image, staticmethod(observation_batch), STATE_NAMES
        cls.ds = LeRobotDataset("local/pilot_front", root=DATASET)
        cls.ckpt = CKPTS[-1]
        cfg = PreTrainedConfig.from_pretrained(cls.ckpt)
        cfg.device = "cpu"
        cls.cfg = cfg
        cls.pre, _ = make_pre_post_processors(cfg, pretrained_path=str(cls.ckpt),
                                              preprocessor_overrides={"device_processor": {"device": "cpu"}})
        cls.cameras, cls.state_len = policy_inputs(cls.ckpt)
        # the frames of episode 0 as the converter built them (same rules -> same JPEG per dataset frame)
        spec = importlib.util.spec_from_file_location("convert", ROOT / "scripts" / "31_convert_to_lerobot.py")
        cv = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cv)
        s = cv.Session(SESSION)
        args = types.SimpleNamespace(min_clutch_s=1.0, min_path_mm=30.0, keep_still=False, cut_pauses=0.0,
                                     cameras=["front"])
        cls.frames, _ = cv.episode_frames(s, s.meta["attempts"][0], args, [])

    def batches(self, i):
        """(training batch, run batch) for dataset frame i, both before and after the preprocessor."""
        item = self.ds[i]
        train = {k: v.unsqueeze(0) for k, v in item.items()
                 if k.startswith("observation.") and hasattr(v, "unsqueeze")}
        train["task"] = [item["task"]]
        fr = self.frames[i]
        images = {c: np.asarray(self.Image.open(fr[f"observation.images.{c}"]).convert("RGB")) for c in self.cameras}
        run = self.observation_batch(images, fr["observation.state"], item["task"], self.cameras)
        return train, run, self.pre(dict(train)), self.pre(dict(run))

    def test_episode_frames_match_the_dataset(self):
        ep0 = int(self.ds.meta.episodes[0]["length"])
        self.assertEqual(len(self.frames), ep0)

    def test_keys_cameras_shapes_dtypes(self):
        train, run, train_n, run_n = self.batches(100)
        obs_keys = lambda b: sorted(k for k in b if k.startswith("observation."))   # noqa: E731
        self.assertEqual(obs_keys(run), obs_keys(train))
        self.assertEqual(sorted(k for k in self.cfg.input_features), obs_keys(run))   # what the policy expects
        self.assertEqual(self.cameras, ["front"])
        for k in obs_keys(train):
            self.assertEqual(tuple(run[k].shape), tuple(train[k].shape), k)
            self.assertEqual(run[k].dtype, train[k].dtype, k)
            self.assertEqual(tuple(run_n[k].shape), tuple(train_n[k].shape), k)
        self.assertEqual(tuple(run["observation.images.front"].shape), (1, 3, 480, 640))

    def test_image_range_and_color_order(self):
        for i in (0, 300, len(self.frames) - 1):
            train, run, train_n, run_n = self.batches(i)
            a, b = train["observation.images.front"], run["observation.images.front"]
            self.assertGreaterEqual(float(b.min()), 0.0)
            self.assertLessEqual(float(b.max()), 1.0)
            self.assertGreater(float(b.max()), 0.5)              # 0..1, not 0..1/255
            same = float((a - b).abs().mean())
            swapped = float((a - b.flip(1)).abs().mean())       # R and B exchanged
            self.assertLess(same, 0.02, f"frame {i}: mean abs difference {same:.4f} (video compression only)")
            self.assertGreater(swapped, 3 * same, f"frame {i}: RGB {same:.4f} vs BGR {swapped:.4f}")
            # the stronger check: the R - B map of both images correlates (swapped channels: it anti-correlates)
            ra, rb = (a[0, 0] - a[0, 2]).flatten().numpy(), (b[0, 0] - b[0, 2]).flatten().numpy()
            self.assertGreater(float(np.corrcoef(ra, rb)[0, 1]), 0.9, f"frame {i}: R - B correlation")
            na, nb = train_n["observation.images.front"], run_n["observation.images.front"]
            self.assertLess(float((na - nb).abs().mean()), 0.15)   # after normalization (std about 0.16)

    def test_state_order_units_values(self):
        names = self.ds.meta.features["observation.state"]["names"]
        self.assertEqual(list(names), self.STATE_NAMES)
        self.assertEqual(self.state_len, len(self.STATE_NAMES))
        stats = self.ds.meta.stats["observation.state"]
        for i in (0, 300, len(self.frames) - 1):
            train, run, train_n, run_n = self.batches(i)
            np.testing.assert_allclose(run["observation.state"].numpy(), train["observation.state"].numpy(), atol=1e-6)
            np.testing.assert_allclose(run_n["observation.state"].numpy(), train_n["observation.state"].numpy(), atol=1e-5)
            st = run["observation.state"].numpy()[0]
            lo, hi = np.asarray(stats["min"]), np.asarray(stats["max"])
            self.assertTrue(np.all((st >= lo - 1e-3) & (st <= hi + 1e-3)))
            self.assertGreater(np.abs(st[:6]).max(), 6.3)           # at least one joint beyond 2 pi: degrees
            self.assertTrue(0.0 <= st[6] <= 1.0)                    # gripper fraction


if __name__ == "__main__":
    unittest.main()

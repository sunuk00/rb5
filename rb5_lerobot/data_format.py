"""The dataset format (rec-1, p2-b), shared by the converter and the policy run: one definition of the state, the
action, the frame rate and the image resize.

These are not parameters in configs/policy.yaml: changing them means changing the recording, the converter and
the run together. The image size is a parameter (policy.yaml dataset.image_width / image_height); the run takes
it from the checkpoint, so an older checkpoint keeps its own size.
"""
import numpy as np
from PIL import Image

FPS = 30                        # the cameras' rate; the converter's time grid
RECORDED_SIZE = (640, 480)      # width, height of the recorded images (teleop/cameras.py)
STATE_NAMES = ["j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg", "gripper_pos"]   # joints, gripper 0 closed .. 1 open
ACTION_NAMES = ["x_mm", "y_mm", "z_mm", "gripper_open"]   # TCP target in the base frame, gripper command 0 .. 1
STATE_LEN, ACTION_LEN = len(STATE_NAMES), len(ACTION_NAMES)


def resize_image(img, width, height):
    """HxWx3 uint8 RGB -> height x width x 3 uint8, the same way in the converter and in the run. PIL bilinear
    (when shrinking, PIL widens the filter to the scale, so it averages like an area filter: no aliasing). The
    image itself when it already has that size."""
    if img.shape[1] == width and img.shape[0] == height:
        return img
    return np.asarray(Image.fromarray(img).resize((width, height), Image.Resampling.BILINEAR))

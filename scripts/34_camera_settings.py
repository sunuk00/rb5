"""cam-2a / cam-4b: find fixed exposure / gain / white balance for the cameras. Cameras only, no robot.

The cameras are the ones of configs/sim.yaml record (front_serial, wrist_serial): a RealSense by its serial
number, a UVC webcam by its /dev/ path (cam-4b: the C270 on the gripper). For each, 640x480 RGB @ 30:
  1. auto exposure and auto white balance: mean brightness, frame-to-frame noise, a sample image. Its mean
     is the target, unless --target gives one (the start check compares every camera with brightness_ref)
  2. fixed exposure at each candidate (default 8.3 and 16.7 ms: multiples of 1/120 s, so 60 Hz lighting
     does not flicker), auto white balance off at 4600 K: the gain is searched (bisection) until the mean
     brightness matches the target; noise and an image for each
  3. at the less noisy candidate's gain: white balance candidates (default 4000 / 4600 / 5200 K); the gray
     table should come out gray, so R/G and B/G near 1 are better; an image for each
At the end auto exposure and auto white balance are switched on again. A RealSense returns to auto at a
replug; the webcam keeps whatever was written last until it is unplugged (the teleop writes its values
at every open, so either is fine).

Exposure units differ: the D435 RGB sensor takes 0.1 ms steps, the D405 stereo module microseconds, the
webcam 0.1 ms steps; this script works in microseconds and converts.

Usage (env rb5):
    python scripts/34_camera_settings.py
    python scripts/34_camera_settings.py --camera wrist --target 114
    python scripts/34_camera_settings.py --exposures-us 8333 16667 25000 --wb 4200 4600 5000
Output: data/cameras/settings_<time>/ (PNG per setting) and the table printed.
"""
import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.cameras import UVCCamera  # noqa: E402
from teleop.settings import load_settings  # noqa: E402

MEASURE_FRAMES = 6


class RealSense:
    """Color stream of a RealSense; the options on its color sensor."""
    settle_s = 0.6      # after an option change, frames before measuring (applied within a few frames)

    def __init__(self, serial):
        import pyrealsense2 as rs
        self.rs = rs
        dev = next((d for d in rs.context().query_devices() if d.get_info(rs.camera_info.serial_number) == serial), None)
        if dev is None:
            raise SystemExit(f"camera {serial} not found")
        self.sensor = next(s for s in dev.query_sensors()
                           if any(p.stream_type() == rs.stream.color for p in s.get_stream_profiles()))
        self.name = self.sensor.get_info(rs.camera_info.name)
        # device units per microsecond: D435 RGB 0.1 ms steps, D405 stereo module microseconds
        self.scale = 0.01 if self.name == "RGB Camera" else 1.0
        r = self.sensor.get_option_range(rs.option.gain)
        self.gain_range = (r.min, r.max)
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.pipe = rs.pipeline()
        self.pipe.start(cfg)

    def frame(self):
        return np.asanyarray(self.pipe.wait_for_frames().get_color_frame().get_data()).copy()

    def auto(self, on):
        self.sensor.set_option(self.rs.option.enable_auto_exposure, 1 if on else 0)
        self.sensor.set_option(self.rs.option.enable_auto_white_balance, 1 if on else 0)

    def set(self, exposure_us=None, gain=None, wb=None):
        if exposure_us is not None:
            self.sensor.set_option(self.rs.option.exposure, max(1.0, round(exposure_us * self.scale)))
        if gain is not None:
            self.sensor.set_option(self.rs.option.gain, gain)
        if wb is not None:
            self.sensor.set_option(self.rs.option.white_balance, wb)

    def close(self):
        self.pipe.stop()


class Webcam:
    """UVC webcam (teleop UVCCamera for the stream, v4l2-ctl for the controls)."""
    settle_s = 1.0      # the C270 took longer than 0.6 s to settle after leaving auto exposure (cam-4a)

    def __init__(self, path):
        self.cam = UVCCamera(path, 640, 480, 30)
        self.name = "UVC webcam"
        line = next(l for l in self.cam.ctl("-l").splitlines() if l.split()[:1] == ["gain"])
        fields = dict(f.split("=") for f in line.split(":", 1)[1].split() if "=" in f)
        self.gain_range = (int(fields["min"]), int(fields["max"]))

    def frame(self):
        while True:
            got = self.cam.read()
            if got is not None:
                return got[0]

    def auto(self, on):
        # auto_exposure 3 = aperture priority (the webcam's auto), 1 = manual; the variable frame rate stays
        # off so the auto measurement is also at 30 fps
        self.cam.ctl(f"--set-ctrl=auto_exposure={3 if on else 1},exposure_dynamic_framerate=0,"
                     f"white_balance_automatic={1 if on else 0}")

    def set(self, exposure_us=None, gain=None, wb=None):
        values = {"exposure_time_absolute": None if exposure_us is None else max(1, round(exposure_us / 100.0)),
                  "gain": None if gain is None else int(gain),
                  "white_balance_temperature": None if wb is None else int(wb)}
        values = {k: v for k, v in values.items() if v is not None}
        if values:
            self.cam.ctl("--set-ctrl=" + ",".join(f"{k}={v}" for k, v in values.items()))

    def close(self):
        self.cam.close()


def grab(cam, settle_s=None, n=MEASURE_FRAMES):
    t_end = time.time() + (cam.settle_s if settle_s is None else settle_s)
    while time.time() < t_end:
        cam.frame()
    frames = [cam.frame() for _ in range(n)]
    imgs = np.stack(frames).astype(float)
    mean = float(imgs.mean())
    noise = float(np.std(imgs[1:] - imgs[:-1]) / np.sqrt(2))        # frame-to-frame noise of a still scene
    rgb = imgs.mean(axis=(0, 1, 2))
    return mean, noise, rgb, frames[-1]


def save(img, out, name):
    Image.fromarray(img).save(out / f"{name}.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", choices=["front", "wrist"], action="append", help="default: both")
    ap.add_argument("--target", type=float, help="mean brightness to match (default: the auto one)")
    ap.add_argument("--exposures-us", type=float, nargs="+", default=[8333.0, 16667.0])
    ap.add_argument("--wb", type=float, nargs="+", default=[4000.0, 4600.0, 5200.0])
    args = ap.parse_args()
    r = load_settings(ROOT / "configs" / "sim.yaml")[0].record
    cameras = {"front": r.front_serial, "wrist": r.wrist_serial}
    out = ROOT / "data" / "cameras" / f"settings_{datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True)

    for name in args.camera or list(cameras):
        serial = str(cameras[name])
        cam = Webcam(serial) if serial.startswith("/dev/") else RealSense(serial)
        g_min, g_max = cam.gain_range
        print(f"\n== {name} ({cam.name}, {serial}); gain {g_min:.0f}..{g_max:.0f}")
        try:
            cam.auto(True)
            auto_mean, noise, rgb, img = grab(cam, settle_s=2.0)
            save(img, out, f"{name}_auto")
            print(f"  auto:                         mean {auto_mean:5.1f}  noise {noise:4.2f}  R/G {rgb[0] / rgb[1]:.3f} "
                  f"B/G {rgb[2] / rgb[1]:.3f}")
            target = auto_mean if args.target is None else args.target
            if args.target is not None:
                print(f"  target mean {target:.1f} (--target)")
            cam.auto(False)
            cam.set(wb=4600)
            best = None
            for exp_us in args.exposures_us:
                cam.set(exposure_us=exp_us)
                lo, hi = g_min, g_max
                for _ in range(8):   # bisection on the gain: brightness rises with the gain
                    g = round((lo + hi) / 2)
                    cam.set(gain=g)
                    m, _, _, _ = grab(cam)
                    lo, hi = (g, hi) if m < target else (lo, g)
                g = round((lo + hi) / 2)
                cam.set(gain=g)
                m, noise, rgb, img = grab(cam)
                sat = float((img.min(axis=2) >= 250).mean())
                save(img, out, f"{name}_exp{exp_us / 1000:.1f}ms_gain{g}_wb4600")
                print(f"  exposure {exp_us / 1000:5.1f} ms, gain {g:3d}:    mean {m:5.1f}  noise {noise:4.2f}  "
                      f"R/G {rgb[0] / rgb[1]:.3f} B/G {rgb[2] / rgb[1]:.3f}  saturated {100 * sat:.1f} %"
                      + ("  (gain at its max: darker than the target)" if g >= g_max - 1 and m < target - 5 else ""))
                if best is None or noise < best[2]:
                    best = (exp_us, g, noise)
            exp_us, g, _ = best
            cam.set(exposure_us=exp_us, gain=g)
            for wb in args.wb:
                cam.set(wb=wb)
                m, noise, rgb, img = grab(cam)
                save(img, out, f"{name}_exp{exp_us / 1000:.1f}ms_gain{g}_wb{wb:.0f}")
                print(f"  white balance {wb:.0f} K (exposure {exp_us / 1000:.1f} ms, gain {g}): mean {m:5.1f}  "
                      f"R/G {rgb[0] / rgb[1]:.3f} B/G {rgb[2] / rgb[1]:.3f}")
        finally:
            cam.auto(True)
            cam.close()
    print(f"\nimages: {out.relative_to(ROOT)} (auto exposure and auto white balance switched back on)")


if __name__ == "__main__":
    main()

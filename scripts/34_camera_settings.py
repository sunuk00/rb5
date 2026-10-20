"""cam-2a: find fixed exposure / gain / white balance for the RealSense cameras. Cameras only, no robot.

For each camera (front D435, wrist D405), 640x480 RGB @ 30:
  1. auto exposure and auto white balance as they are now: mean brightness (the target), frame-to-frame
     noise, a sample image
  2. fixed exposure at each candidate (default 8.3 and 16.7 ms: multiples of 1/120 s, so 60 Hz lighting
     does not flicker), auto white balance off at 4600 K: the gain is searched (bisection) until the mean
     brightness matches the auto one; noise and an image for each
  3. at the brighter-noise-wise better candidate's gain: white balance candidates (default 4000 / 4600 /
     5200 K); the gray table should come out gray, so R/G and B/G near 1 are better; an image for each
At the end auto exposure and auto white balance are switched on again (the options are not stored in
the camera either: a replug also returns to auto).

Exposure units differ: the D435 RGB sensor takes 0.1 ms steps, the D405 stereo module microseconds; this
script works in microseconds and converts.

Usage (env rb5):
    python scripts/34_camera_settings.py
    python scripts/34_camera_settings.py --exposures-us 8333 16667 25000 --wb 4200 4600 5000
Output: data/cameras/settings_<time>/ (PNG per setting) and the table printed.
"""
import argparse
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pyrealsense2 as rs
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
CAMERAS = {"front": "943222070221", "wrist": "130322273474"}
SETTLE_S = 0.6      # after an option change, frames before measuring (the sensor applies it within a few frames)
MEASURE_FRAMES = 6


def color_sensor(serial):
    for d in rs.context().query_devices():
        if d.get_info(rs.camera_info.serial_number) == serial:
            for s in d.query_sensors():
                if any(p.stream_type() == rs.stream.color for p in s.get_stream_profiles()):
                    return s
    raise SystemExit(f"camera {serial} not found")


def exposure_scale(sensor):
    """Device units per microsecond: D435 RGB 0.1 ms steps, D405 stereo module microseconds."""
    return 0.01 if sensor.get_info(rs.camera_info.name) == "RGB Camera" else 1.0


def grab(pipe, settle_s=SETTLE_S, n=MEASURE_FRAMES):
    t_end = time.time() + settle_s
    while time.time() < t_end:
        pipe.wait_for_frames()
    frames = [np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data()).copy() for _ in range(n)]
    imgs = np.stack(frames).astype(float)
    mean = float(imgs.mean())
    noise = float(np.std(imgs[1:] - imgs[:-1]) / np.sqrt(2))        # frame-to-frame noise of a still scene
    rgb = imgs.mean(axis=(0, 1, 2))
    return mean, noise, rgb, frames[-1]


def save(img, out, name):
    Image.fromarray(img).save(out / f"{name}.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--exposures-us", type=float, nargs="+", default=[8333.0, 16667.0])
    ap.add_argument("--wb", type=float, nargs="+", default=[4000.0, 4600.0, 5200.0])
    args = ap.parse_args()
    out = ROOT / "data" / "cameras" / f"settings_{datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True)

    for cam, serial in CAMERAS.items():
        s = color_sensor(serial)
        scale = exposure_scale(s)
        g_range = s.get_option_range(rs.option.gain)
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        pipe = rs.pipeline()
        pipe.start(cfg)
        print(f"\n== {cam} ({s.get_info(rs.camera_info.name)}, {serial}); gain {g_range.min:.0f}..{g_range.max:.0f}")
        try:
            s.set_option(rs.option.enable_auto_exposure, 1)
            s.set_option(rs.option.enable_auto_white_balance, 1)
            target, noise, rgb, img = grab(pipe, settle_s=2.0)
            save(img, out, f"{cam}_auto")
            print(f"  auto:                         mean {target:5.1f}  noise {noise:4.2f}  R/G {rgb[0] / rgb[1]:.3f} "
                  f"B/G {rgb[2] / rgb[1]:.3f}")
            s.set_option(rs.option.enable_auto_exposure, 0)
            s.set_option(rs.option.enable_auto_white_balance, 0)
            s.set_option(rs.option.white_balance, 4600)
            best = None
            for exp_us in args.exposures_us:
                s.set_option(rs.option.exposure, max(1.0, round(exp_us * scale)))
                lo, hi = g_range.min, g_range.max
                for _ in range(7):   # bisection on the gain: brightness rises with the gain
                    g = round((lo + hi) / 2)
                    s.set_option(rs.option.gain, g)
                    m, _, _, _ = grab(pipe)
                    lo, hi = (g, hi) if m < target else (lo, g)
                g = round((lo + hi) / 2)
                s.set_option(rs.option.gain, g)
                m, noise, rgb, img = grab(pipe)
                save(img, out, f"{cam}_exp{exp_us / 1000:.1f}ms_gain{g}_wb4600")
                print(f"  exposure {exp_us / 1000:5.1f} ms, gain {g:3d}:    mean {m:5.1f}  noise {noise:4.2f}  "
                      f"R/G {rgb[0] / rgb[1]:.3f} B/G {rgb[2] / rgb[1]:.3f}"
                      + ("  (gain at its max: darker than auto)" if g >= g_range.max - 1 and m < target - 5 else ""))
                if best is None or noise < best[2]:
                    best = (exp_us, g, noise)
            exp_us, g, _ = best
            s.set_option(rs.option.exposure, max(1.0, round(exp_us * scale)))
            s.set_option(rs.option.gain, g)
            for wb in args.wb:
                s.set_option(rs.option.white_balance, wb)
                m, noise, rgb, img = grab(pipe)
                save(img, out, f"{cam}_exp{exp_us / 1000:.1f}ms_gain{g}_wb{wb:.0f}")
                print(f"  white balance {wb:.0f} K (exposure {exp_us / 1000:.1f} ms, gain {g}): mean {m:5.1f}  "
                      f"R/G {rgb[0] / rgb[1]:.3f} B/G {rgb[2] / rgb[1]:.3f}")
        finally:
            s.set_option(rs.option.enable_auto_exposure, 1)
            s.set_option(rs.option.enable_auto_white_balance, 1)
            pipe.stop()
    print(f"\nimages: {out.relative_to(ROOT)} (auto exposure and auto white balance switched back on)")


if __name__ == "__main__":
    main()

"""RGB from all connected RealSense cameras at the same time (cam-1). No robot, no leader arm.

For each camera (found by serial number) one pipeline streams only the color stream (RGB8), each in its
own thread. Every frame logs: camera, frame number, camera timestamp [ms] and its domain, the frame
metadata timestamps when the driver provides them (sensor / frame / backend / time of arrival), and the
PC time when the thread got the frame (time.monotonic and time.time). The D405 has no separate RGB
sensor: its color comes from the stereo module; the color sensor is found by its stream profiles.

Summary per camera: achieved fps, frame interval (camera clock and PC clock) p5 / p50 / p95 / max, missed
frames (frame number gaps), camera clock vs PC clock (offset, drift in ppm, residual), receive delay
when the camera timestamp is in the PC clock (global / system time domain). Between cameras: the time
difference of the nearest frames. Also the process CPU use.

auto_exposure_priority (option of the color sensor, only while running, not saved): on = the sensor may
lower the frame rate to expose longer in the dark; off = keep the frame rate.

Output: data/cameras/cam_<time>.csv (one row per frame), a sample PNG per camera, the summary printed.
Usage:
    python scripts/25_camera_rgb_test.py                                   # 640x480 @ 30 for 60 s
    python scripts/25_camera_rgb_test.py --res 640x480 --fps 60 --duration 60 --ae-priority off
"""
import argparse
import csv
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pyrealsense2 as rs

ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = ROOT / "data" / "cameras"
META = {"sensor_ts_us": rs.frame_metadata_value.sensor_timestamp,
        "frame_ts_us": rs.frame_metadata_value.frame_timestamp,
        "backend_ts_ms": rs.frame_metadata_value.backend_timestamp,
        "arrival_ms": rs.frame_metadata_value.time_of_arrival}
HEADER = (["camera", "serial", "frame", "cam_ts_ms", "domain", "pc_mono_s", "pc_wall_ms"] + list(META))


def color_sensor(dev):
    for s in dev.query_sensors():
        if any(p.stream_type() == rs.stream.color for p in s.get_stream_profiles()):
            return s
    return None


def grab(name, serial, pipe, stop, rows, sample, lock):
    saved, count = False, 0
    while not stop.is_set():
        try:
            frames = pipe.wait_for_frames(1000)
        except RuntimeError:
            continue
        mono, wall = time.monotonic(), time.time() * 1000.0
        f = frames.get_color_frame()
        if not f:
            continue
        meta = [f.get_frame_metadata(k) if f.supports_frame_metadata(k) else "" for k in META.values()]
        row = [name, serial, f.get_frame_number(), f"{f.get_timestamp():.3f}", str(f.get_frame_timestamp_domain()).split(".")[-1],
               f"{mono:.6f}", f"{wall:.3f}"] + meta
        with lock:
            rows.append(row)
        count += 1
        if not saved and count > 60:   # after about 2 s of auto exposure
            sample[serial] = np.asanyarray(f.get_data()).copy()
            saved = True


def pct(x, ps=(5, 50, 95)):
    return [float(np.percentile(x, p)) for p in ps]


def summarize(name, r, warmup):
    t_pc = np.array([float(x[5]) for x in r])
    keep = t_pc >= t_pc[0] + warmup
    r = [x for x, k in zip(r, keep) if k]
    t_pc, cam = t_pc[keep], np.array([float(x[3]) for x in r])
    wall = np.array([float(x[6]) for x in r])
    fn = np.array([int(x[2]) for x in r])
    dom = r[0][4]
    dpc, dcam = np.diff(t_pc) * 1000.0, np.diff(cam)
    gaps = np.diff(fn)
    span = t_pc[-1] - t_pc[0]
    print(f"{name}: {len(r)} frames in {span:.1f} s -> {(len(r) - 1) / span:.2f} fps; domain {dom}")
    print(f"   interval camera clock p5/p50/p95/max {'/'.join(f'{v:.2f}' for v in pct(dcam))}/{dcam.max():.2f} ms; "
          f"PC clock {'/'.join(f'{v:.2f}' for v in pct(dpc))}/{dpc.max():.2f} ms")
    print(f"   missed frames (frame number gaps): {int(np.sum(gaps[gaps > 1] - 1))} in {int(np.sum(gaps > 1))} gaps; "
          f"repeated: {int(np.sum(gaps == 0))}")
    a, b = np.polyfit(t_pc * 1000.0, cam, 1)
    res = cam - (a * t_pc * 1000.0 + b)
    print(f"   camera clock vs PC: drift {(a - 1.0) * 1e6:+.0f} ppm, residual std {res.std():.2f} ms, p99 |res| "
          f"{np.percentile(np.abs(res), 99):.2f} ms")
    if dom in ("global_time", "system_time"):
        delay = wall - cam
        print(f"   receive delay (PC time when read - camera time) p5/p50/p95 {'/'.join(f'{v:.1f}' for v in pct(delay))} ms")
    for k, i in (("sensor_ts_us", 7), ("arrival_ms", 10)):
        have = sum(1 for x in r if x[i] != "")
        print(f"   metadata {k}: {'yes' if have else 'not available'} ({have} frames)")
    return t_pc


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--res", default="640x480")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--warmup", type=float, default=2.0, help="seconds left out of the statistics")
    parser.add_argument("--ae-priority", choices=("on", "off", "keep"), default="keep")
    parser.add_argument("--allow-usb2", action="store_true")
    args = parser.parse_args()
    w, h = (int(v) for v in args.res.split("x"))

    devices = list(rs.context().query_devices())
    if not devices:
        raise SystemExit("no RealSense camera found")
    cams = []
    for d in devices:
        name, serial = d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)
        usb = d.get_info(rs.camera_info.usb_type_descriptor)
        print(f"{name} {serial}: USB {usb}, firmware {d.get_info(rs.camera_info.firmware_version)}")
        if not usb.startswith("3") and not args.allow_usb2:
            raise SystemExit(f"{name} is on USB {usb}: reconnect it to a USB 3 port / cable (or --allow-usb2)")
        s = color_sensor(d)
        if s is not None and args.ae_priority != "keep" and s.supports(rs.option.auto_exposure_priority):
            s.set_option(rs.option.auto_exposure_priority, 1.0 if args.ae_priority == "on" else 0.0)
        aep = (s.get_option(rs.option.auto_exposure_priority)
               if s is not None and s.supports(rs.option.auto_exposure_priority) else "n/a")
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, args.fps)
        cams.append((name.replace("RealSense ", ""), serial, cfg, aep))

    rows, sample, lock, stop = [], {}, threading.Lock(), threading.Event()
    pipes, threads = [], []
    for name, serial, cfg, aep in cams:
        pipe = rs.pipeline()
        pipe.start(cfg)
        print(f"{name}: color {w}x{h} @ {args.fps}, auto_exposure_priority {aep}")
        pipes.append(pipe)
        threads.append(threading.Thread(target=grab, args=(name, serial, pipe, stop, rows, sample, lock), daemon=True))
    cpu0, t0 = os.times(), time.monotonic()
    for t in threads:
        t.start()
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(2.0)
    cpu1, t1 = os.times(), time.monotonic()
    for p in pipes:
        p.stop()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = OUT_DIR / f"cam_{stamp}_{w}x{h}_{args.fps}.csv"
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows([HEADER] + sorted(rows, key=lambda x: float(x[5])))
    print(f"\nsaved {path.relative_to(ROOT)} ({len(rows)} frames)")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for serial, img in sample.items():
            png = OUT_DIR / f"cam_{stamp}_{serial}.png"
            plt.imsave(png, img)
            print(f"sample image {png.relative_to(ROOT)}")
    except ImportError:
        pass

    print(f"\nprocess CPU {100.0 * ((cpu1.user - cpu0.user) + (cpu1.system - cpu0.system)) / (t1 - t0):.0f} % of one core\n")
    times = {}
    for name, serial, _, _ in cams:
        r = [x for x in rows if x[1] == serial]
        if len(r) < 10:
            print(f"{name}: only {len(r)} frames")
            continue
        times[name] = summarize(name, r, args.warmup)
    if len(times) == 2:
        (na, ta), (nb, tb) = times.items()
        ta = ta[(ta >= tb[0]) & (ta <= tb[-1])]   # only while both cameras were running
        idx = np.clip(np.searchsorted(tb, ta), 1, len(tb) - 1)
        near = np.minimum(np.abs(tb[idx] - ta), np.abs(tb[idx - 1] - ta)) * 1000.0
        print(f"\n{na} vs {nb}: nearest frame by PC time p50/p95/max {np.percentile(near, 50):.1f} / "
              f"{np.percentile(near, 95):.1f} / {near.max():.1f} ms (cameras are not synchronized)")


if __name__ == "__main__":
    main()

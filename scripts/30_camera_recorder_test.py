"""rec-3a: camera recorder process alone (teleop/cameras.py). No robot, no leader arm.

Records episodes the way the teleop will: the camera process streams all the time, start_episode /
stop_episode(keep) mark the episodes, a discarded episode's files are deleted. Meanwhile this process
runs a 50 Hz loop like the teleop and counts late cycles, to see whether the camera process disturbs it.

Usage (env rb5):
    python scripts/30_camera_recorder_test.py                         # front + wrist, 2 x 20 s kept, 1 x 3 s discarded
    python scripts/30_camera_recorder_test.py --seconds 60 --episodes 1
    python scripts/30_camera_recorder_test.py --camera front=943222070221   # one camera
    python scripts/30_camera_recorder_test.py --fake                   # synthetic frames, no cameras

Output: data/cameras/rec_<time>/episode_<n>/ (cam_<name>/*.jpg, cam_<name>.csv), summary printed.
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop.cameras import CameraRecorder  # noqa: E402

CAMERAS = {"front": "943222070221", "wrist": "130322273474"}   # D435 above the workspace, D405 on the gripper
PERIOD = 0.02          # the teleop loop
LATE_MS = 5.0          # a cycle that starts this much after its time counts as late


def loop_for(seconds, rec):
    """50 Hz loop with a little numpy work; returns (cycles, late, max lateness ms)."""
    n, late, worst = 0, 0, 0.0
    t_next = time.monotonic()
    end = t_next + seconds
    a = np.random.rand(60, 60)
    while t_next < end:
        t_next += PERIOD
        delay = t_next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        lateness = (time.monotonic() - t_next) * 1000.0
        worst = max(worst, lateness)
        late += lateness > LATE_MS
        n += 1
        np.linalg.inv(a + np.eye(60))   # stands in for the planning work
        for err in rec.poll_errors():
            print(f"  camera error: {err}")
    return n, late, worst


def summarize(ep_dir, name, seconds):
    path = ep_dir / f"cam_{name}.csv"
    if not path.exists():
        return f"{name}: no files"
    rows = list(csv.DictReader(open(path)))
    if len(rows) < 2:
        return f"{name}: {len(rows)} frames"
    t = np.array([float(r["t_capture_s"]) for r in rows])
    dt = np.diff(t) * 1000.0
    w = np.array([float(r["write_ms"]) for r in rows])
    size = sum(f.stat().st_size for f in (ep_dir / f"cam_{name}").iterdir()) / 1e6
    return (f"{name}: {len(rows)} frames, {(len(rows) - 1) / (t[-1] - t[0]):.2f} fps | interval p50/p95/max "
            f"{np.percentile(dt, 50):.1f}/{np.percentile(dt, 95):.1f}/{dt.max():.1f} ms | JPEG write p50/p95/max "
            f"{np.percentile(w, 50):.1f}/{np.percentile(w, 95):.1f}/{w.max():.1f} ms | {size:.0f} MB "
            f"({size / seconds:.1f} MB/s, {size * 1000 / len(rows):.0f} kB/frame)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", action="append", help="name=serial (repeat); default: front and wrist")
    ap.add_argument("--episodes", type=int, default=2, help="kept episodes")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--quality", type=int, default=95, help="JPEG quality")
    ap.add_argument("--fake", action="store_true", help="synthetic frames instead of the cameras")
    args = ap.parse_args()
    cams = ([tuple(c.split("=", 1)) for c in args.camera] if args.camera else list(CAMERAS.items()))

    out = ROOT / "data" / "cameras" / f"rec_{datetime.now():%Y%m%d_%H%M%S}"
    rec = CameraRecorder(cams, quality=args.quality, fake=args.fake)
    t0 = time.monotonic()
    problems = rec.open()
    if problems:
        raise SystemExit("; ".join(problems))
    print(f"camera process ready in {time.monotonic() - t0:.1f} s: {', '.join(f'{n}={s}' for n, s in cams)}")

    n, late, worst = loop_for(3.0, rec)
    print(f"no episode, 3 s: loop {n} cycles, late {late}, worst {worst:.1f} ms")
    plan = [(i, args.seconds, True) for i in range(args.episodes)] + [(args.episodes, 3.0, False)]
    cpu0, t_cpu0 = rec.cpu_seconds(), time.monotonic()
    for i, seconds, keep in plan:
        ep_dir = out / f"episode_{i:03d}"
        rec.start_episode(ep_dir)
        n, late, worst = loop_for(seconds, rec)
        t_stop = time.monotonic()
        stats = rec.stop_episode(keep)
        print(f"\nepisode {i} ({seconds:.0f} s, {'kept' if keep else 'discarded'}): loop {n} cycles, late {late}, "
              f"worst {worst:.1f} ms; stop took {(time.monotonic() - t_stop) * 1000:.0f} ms")
        print(f"  recorder: {stats}")
        if keep:
            for name, _ in cams:
                print("  " + summarize(ep_dir, name, seconds))
        else:
            left = list(ep_dir.glob("cam_*")) if ep_dir.exists() else []
            print(f"  files left after discard: {len(left)}")
    cpu1, t_cpu1 = rec.cpu_seconds(), time.monotonic()
    rec.close()
    if cpu0 is not None and cpu1 is not None:
        print(f"\ncamera process CPU while recording: {100 * (cpu1 - cpu0) / (t_cpu1 - t_cpu0):.0f} % of one core")
    print(f"files: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

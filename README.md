# RB_Cobot
RB5-850 + Allegro Hand V4 test repository: robot API tests and RB-Y1 leader arm teleoperation, as the
groundwork for teleoperated demonstrations -> imitation learning (ACT) -> residual RL.

Status (2026-10-16): leader arm -> RB5 teleoperation works on the real robot (real-1, settings frozen).
Next: gripper, demonstration recording. Platform: Linux only for now (Windows: not yet, see below).

- `teleop/` - leader arm -> RB5 teleop. Run from the repository root: `python -m teleop` (Simulation,
  `configs/sim.yaml`) or `python -m teleop --real` (`configs/real.yaml`). Guide: `docs/teleop_guide.md`
- `configs/` - `robot.yaml` (robot IP and speed bar, shared with the scripts via `scripts/config.py`),
  `real.yaml` (real-1 final) and `sim.yaml` (the same values, checked by a test)
- `scripts/` - numbered experiment and analysis scripts (01-24), kept as they were run; list in
  `scripts/README.md`
- `tests/` - automated tests with fake devices: `python -m unittest discover -s tests`
- `assets/` - leader arm URDF (`leader_arm/`, copy from rby1-sdk, Apache 2.0) and the leader arm udev
  rule (`udev/`; see `docs/teleop_guide.md`)
- `environment.yml` - conda env `rb5`: `conda env create -f environment.yml`
- `docs/` - guides and records; list in `docs/README.md`:
  `teleop_guide.md` (how to run), `real_first_run_checklist.md` (before every Real run),
  `decisions.md` (every decision and measurement), `hardware.md` (lab hardware facts)

## Windows
Not ready yet: the teleop uses Linux-only parts (CPU pinning with `os.sched_setaffinity`, the FTDI
`latency_timer` read from `/sys`, the `/dev/ttyUSB0` port name, the udev rule), `environment.yml`
installs `evdev` (Linux only), and Python 3.10 on Windows sleeps in about 15 ms steps, which a 20 ms
control loop cannot rely on. rbpodo and dynamixel-sdk themselves have Windows packages.

## Leader arm latency without the udev rule (after every reboot or replug)
```
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

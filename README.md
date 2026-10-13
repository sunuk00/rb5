# RB_Cobot
RB5-850 + Allegro Hand V4 test repository: robot API tests and RB-Y1 leader arm teleoperation.

- `scripts/` - numbered experiment scripts (01-22), kept as they were run
- `teleop/` - leader arm -> RB5 teleop (position, yaw, gripper signal). Run from the repository root:
  `python -m teleop` (Simulation by default; `--real` for the real robot)
- `configs/` - `robot.yaml` (robot IP and speed bar, shared by the teleop and the scripts via `scripts/config.py`),
  teleop settings `sim.yaml` and conservative `real.yaml`
- `environment.yml` - conda env `rb5`: `conda env create -f environment.yml`
- `assets/` - leader arm URDF (`leader_arm/`, copy from rby1-sdk, Apache 2.0) and the leader arm udev
  rule (`udev/`, not installed yet; see `docs/teleop_guide.md`)
- `tests/` - automated tests with fake devices: `python -m unittest discover tests`
- `docs/teleop_guide.md` - how to run the teleop (setup, checklist, controls, stop, troubleshooting)
- `docs/real_first_run_checklist.md` - what must be done before the first Real run
- `docs/decisions.md` - decisions and measurements behind every value


echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
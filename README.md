# RB_Cobot
RB5-850 + 3D-printed parallel gripper (Dynamixel MX-106T) test repository: robot API tests and RB-Y1 leader arm teleoperation, as the
groundwork for teleoperated demonstrations -> imitation learning (ACT) -> residual RL.

Status (2026-10-28): the imitation learning pipeline works end to end on the real robot: leader arm teleop
with a gripper and 3 cameras -> recorded demonstrations -> LeRobot dataset -> ACT training -> the policy
runs on the RB5 (pick a block, place it in a box). Platform: Linux only for now (Windows: not yet, see below).

Pipeline (commands, settings and the code map: `docs/pipeline_guide.md`):
```
python -m teleop --real --record                       # env rb5: demonstrations -> data/demos/<session>/
python -m rb5_lerobot.convert SESSION ... --name NAME  # env lerobot: -> LeRobot dataset
python -m rb5_lerobot.train NAME [--from CKPT]         # ACT training (lerobot-train); --from: fine-tune
python -m rb5_lerobot.check TRAINING                   # offline check on recorded episodes
python -m rb5_lerobot.run TRAINING [--real]            # the policy drives the robot
```

- `teleop/` - leader arm -> RB5 teleop. Run from the repository root: `python -m teleop` (Simulation,
  `configs/sim.yaml`) or `python -m teleop --real` (`configs/real.yaml`). Guide: `docs/teleop_guide.md`
- `rb5_lerobot/` - demonstrations -> LeRobot dataset -> ACT training -> policy runs (env `lerobot`):
  `convert.py`, `train.py`, `check.py`, `run.py` + `policy.py`; the dataset format in `data_format.py`
- `configs/` - `robot.yaml` (robot IP and speed bar, shared with the scripts via `scripts/config.py`),
  `real.yaml` (real-1 final) and `sim.yaml` (the same values, checked by a test): teleop, gripper, cameras;
  `policy.yaml`: dataset, training and policy run settings
- `scripts/` - numbered experiment and analysis scripts (01-34), kept as they were run; list in
  `scripts/README.md`
- `tests/` - automated tests with fake devices: `python -m unittest discover -s tests`
- `assets/` - leader arm URDF (`leader_arm/`, copy from rby1-sdk, Apache 2.0) and the leader arm udev
  and gripper U2D2 udev rule (`udev/`; see `docs/teleop_guide.md`)
- `environment.yml` - conda env `rb5`: `conda env create -f environment.yml`; `environment_lerobot.yml` - env
  `lerobot` (LeRobot 0.6.1)
- `docs/` - guides and records; list in `docs/README.md`:
  `pipeline_guide.md` (demonstrations -> ACT -> policy run), `data_rules.md` (how demonstrations are collected),
  `teleop_guide.md` (how to run), `real_first_run_checklist.md` (before every Real run),
  `decisions.md` (every decision and measurement), `hardware.md` (lab hardware facts)

## Windows
Not ready yet: the teleop uses Linux-only parts (CPU pinning with `os.sched_setaffinity`, the FTDI
`latency_timer` read from `/sys`, the `/dev/serial/by-id` port names, the udev rule), `environment.yml`
installs `evdev` (Linux only), and Python 3.10 on Windows sleeps in about 15 ms steps, which a 20 ms
control loop cannot rely on. rbpodo and dynamixel-sdk themselves have Windows packages.

## Leader arm latency without the udev rule (after every reboot or replug)
```
echo 1 | sudo tee /sys/bus/usb-serial/devices/$(basename $(readlink -f /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT62AJW7-if00-port0))/latency_timer
```

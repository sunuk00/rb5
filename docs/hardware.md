# Hardware

Facts about the lab setup, with where each one comes from. TBD = not confirmed yet.
Decisions and measurements behind them are in `docs/decisions.md`.

## RB5 cobot
- Model: Rainbow Robotics RB5-850 (N), 6 DoF. Control box / UI version 8.4.4.
- Control box IP 10.0.2.6 (set in `configs/robot.yaml`). No SSH or monitor on the control box.
- Two TCP channels from the PC (rbpodo): command channel port 5000 (`rb.Cobot`), data channel
  port 5001 (`rb.CobotData`, robot state such as `sdata.jnt_ang`).
- Units: joint angles in degrees, TCP positions in mm (17).
- Joint limits: J3 +/-165 deg (user, teach pendant, 2026-10-12); others +/-360 deg (manual). The
  controller's own setup limits were not read.
- Base frame (manual Appendix F): origin at the center of the base surface, +Z up, +Y toward the base
  cable connector. **TBD:** where +X points on the real robot (check with `scripts/22_base_axis_check.py`).
- TCP offset set on the controller: (0, -187, -40) mm from the flange, orientation 0 (17). Who set
  it and what it should be: **TBD** (lab).
- Controller state: **tool mass 0 kg**; collision detection was Off and must be **ON before every Real
  run** (checklist, 2026-10-16). Treat any Real-mode motion as high risk. Do not change controller
  settings without asking (CLAUDE.md).
- The speed bar scales move_j (19), not move_servo_* (06). The scripts and the teleop use 0.1.
- Real arm following move_servo_l: about 80 ms behind the command, joint error within 0.23 deg
  (run 133743).

## PC (UPC)
- All user code, logs and data run here and talk to the control box over TCP.
- Network: 10.0.2.100/24 on a USB Ethernet adapter (NetworkManager connection `rb5`).
- CPU: Intel i7-13700H; cores 0-11 are the performance cores. The teleop pins itself to them
  (`runtime.cpu_cores`), which brought the planning time p99 from 8-9 ms to 5-6 ms.
- GPU: NVIDIA RTX A1000 6GB Laptop (GA107, compute 8.6, 35 W power limit) next to the Intel iGPU.
  Driver `nvidia-driver-595-open` 595.99.02 (reinstalled 2026-10-19), kernel 7.0.0-38, Secure Boot off.
  Checked 2026-10-19 with PyTorch 2.11.0+cu130 (`rby1` env): CUDA available, fp32 matmul 3.0 TFLOPS,
  ResNet18 training step (batch 16, 480x640, bf16) 205 ms, 1.6 GB.

## RB-Y1 leader arm
- Two 7-joint arms with a handle (tool) each. Only the right arm is used for the RB5 teleop.
- Dynamixel bus, Protocol 2.0, 2 Mbps, through an FTDI FT232H USB-serial adapter (serial FT62AJW7).
  Port: `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT62AJW7-if00-port0` (by serial number, since
  grip-1: with the gripper U2D2 also connected, `/dev/ttyUSB0` / `1` depend on the plug order).
- **FTDI latency_timer must be 1 ms** (default 16 ms makes each read about 16 ms, 11). The udev rule
  `assets/udev/99-rb-cobot-usb-serial.rules` sets it and adds `/dev/rby1_leader_arm`; **not installed yet**.
- IDs: right arm joints 0-6, left arm 7-13, right tool 128 (0x80), left tool 129 (0x81).
- Motors (model number from ping, 11): XM540-W150 for IDs 0-3 and 7-10, XM430-W210 for IDs 4-6 and
  11-13.
- Angles: rad = raw / 4096 x 2 pi (rby1-sdk). Tool raw value at address 132: button = bits 8-15
  (1 pressed, 0 released), trigger = bits 16-31 (0-1000; which end is "closed" is TBD).
- Kinematics: URDF in `assets/leader_arm/model.urdf` (rby1-sdk v0.10.0). The teleop uses the wrist
  center (Link_6R origin). Base frame: x forward, y left, z up (checked with a ruler, 20).
- The teleop (`python -m teleop`) only reads the leader arm (no torque). Scripts 14, 15 and 16 write
  torque (the arm moves or holds by itself).

## Cameras (Intel RealSense)
- D435 (fixed camera), serial 943222070221, firmware 5.17.3.10; D405 (wrist camera), serial 130322273474,
  firmware 5.17.0.10. Both must be on USB 3 (`lsusb -t` 5000M): the D435 first came up as USB 2.1 (only
  640x480 @ 30 and 1280x720 @ 15 for color) until it was reconnected (2026-10-17).
- Mounted (2026-10-19): D435 fixed above the workspace looking down (`front`); D405 on the gripper body
  (`wrist`, turns with J6). Recording: `teleop/cameras.py` (own process, JPEG), script 30. Receive delay
  (PC wall clock at arrival - camera global_time): D435 about 34 ms, D405 about 9.5 ms.
- SDK: system librealsense 2.58.4 (Intel apt repository, tools and udev rules); Python `pyrealsense2`
  2.58.4 in `rb5`. The DKMS kernel module (1.3.33) is not built for kernel 7.0, but frame metadata
  (sensor timestamp, time of arrival) works.
- The D405 has no separate RGB sensor: color comes from its stereo module (no auto_exposure_priority).
- Measured (cam-1): see docs/decisions.md 2026-10-17.
- **The D405 broke (2026-10-21).** Wrist camera since then: Logitech C270 webcam (046d:0825, serial 6E259540,
  USB 2) on the gripper, `/dev/v4l/by-id/usb-046d_0825_6E259540-video-index0`; YUYV 640x480 @ 30 (29.8 fps
  measured), no camera clock (kernel buffer time, about 33 ms before arrival). Settings with `v4l2-ctl`,
  written once the stream runs (cam-4a, cam-4b). USB autosuspend off with `assets/udev/99-rb-cobot-webcam.rules`
  (cam-4c).

## Gripper
- 3D-printed parallel gripper (built by a team member, in progress), one Dynamixel MX-106T (TTL, 3-pin).
  The Allegro Hand V4 is not used (user, 2026-10-17).
- Own bus: U2D2 (USB) -> U2D2 Power Hub Board -> MX-106T. The U2D2 does not power the motor (e-Manual).
  U2D2: FTDI FT232H (0403:6014, the same ids as the leader arm adapter), serial FT62AJAX; port
  `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT62AJAX-if00-port0` (`scripts/config.py` GRIPPER_PORT),
  `/dev/gripper_u2d2` with the udev rule.
- The motor (script 27 and DYNAMIXEL Wizard, 2026-10-17): **MX-106(2.0)**, model 321, firmware v45,
  **ID 2, 1,000,000 bps**, Protocol 2.0, operating mode 3 (position), position limits 2388-3853 (about
  129 deg, set before we got it), current limit 2047 (6.88 A), voltage limits 9.5-16.0 V, return delay
  250 (500 us).
- Gripper positions (measured by hand, grip-2): **closed 3968, open 2196** (155.7 deg). Closed is 127 ticks
  from the single-turn wrap (4095). Settings (grip-2): mode 5, Goal Current 150 (0.50 A), profile
  velocity 200 (45.8 rpm), acceleration 100: full stroke about 0.7 s; holding an object at 0.50 A, 10.2 V.
  Since 2026-10-17 (teleop): velocity 210 (= the motor's Velocity Limit), acceleration 200; Goal Current 200 was
  tried on 2026-10-19 and reverted to 150 (no faster stroke, supply dips to 9.5 V).
  While grasping an object (detected): Goal Current 250 (0.84 A), squeeze 60 ticks past the stop point.
  The fingers can close to about 4098 (130 ticks past 3968) when nothing is between them. Measured top
  speed at 10.5 V only about 42 rpm (12 V would allow more). The gripper port waits 10 ms longer for
  replies than dynamixel_sdk's default (thread switching, decisions 2026-10-17).
- Leader arm control: **left** trigger, released 0 = closed, fully pulled 1000 = fully open (user, 2026-10-17).
- MX-106 (e-Manual): 10.0-14.8 V, **12 V recommended**; standby 100 mA; stall 8.4 N.m at 12 V, **5.2 A**
  (11.1 V 4.8 A, 14.8 V 6.3 A); 45 rev/min no load at 12 V; factory ID 1, 57,600 bps.
  Power: 12 V, at least 5 A (the stall current), separate from the leader arm supply. Power Hub: 3.5-24 V,
  max 10 A, one power input only. Plug and unplug cables only with the power off.
- Firmware decides the protocol: model number 320 = Protocol 1.0 firmware (below v39; modes wheel /
  joint / multi-turn, torque control without position), 321 = MX-106(2.0) (v39 or above; operating
  modes 0 current, 1 velocity, 3 position, 4 extended position, **5 current-based position** (Goal Current
  limits the current, "ideal for grippers"), 16 PWM). This motor has the 2.0 firmware: no update needed.

## Notes
- M808 warning on the controller: the PC firewall is not the cause (ufw inactive).

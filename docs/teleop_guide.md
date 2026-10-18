# Teleop guide (leader arm -> RB5)

`python -m teleop` (`teleop/__main__.py`) lets the RB-Y1 leader arm's handles drive the RB5 tool.
Settings: `configs/real.yaml` (real-1 final, 2026-10-16) and `configs/sim.yaml` (the same values, for
practice in Simulation). Every value and why it was chosen: `docs/decisions.md`.

Platform: Linux (tested on Ubuntu with conda env `rb5`). Windows is not supported yet (see README).

## 1. One-time setup

### Python
```
conda env create -f environment.yml      # env rb5: rbpodo, dynamixel-sdk, numpy, PyYAML, ...
conda activate rb5
python -m unittest discover -s tests     # must end with OK (and no "skipped")
```

### Network
The PC talks to the RB5 control box at 10.0.2.6 (`configs/robot.yaml`): PC address 10.0.2.100/24 on
the Ethernet adapter connected to the control box.

### Leader arm USB: 1 ms latency (udev rule)
The leader arm's FTDI adapter needs `latency_timer = 1` (the default 16 ms makes each read about 16 ms,
script 11); the teleop checks it and refuses otherwise. The rule also creates `/dev/rby1_leader_arm`.
```
sudo cp assets/udev/99-rb-cobot-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Unplug and replug the leader arm USB cable, then check:
```
ls -l /dev/rby1_leader_arm                                         # -> ttyUSB0 (or another number)
cat /sys/bus/usb-serial/devices/$(basename $(readlink -f /dev/rby1_leader_arm))/latency_timer   # -> 1
```
The rule matches this adapter's serial number (`FT62AJW7`); another adapter needs its own line (find
the serial with `udevadm info -a -n /dev/ttyUSB0 | grep serial`). Without the rule, set it by hand
after every reboot or replug: `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSBn/latency_timer`, with
`ttyUSBn` from `ls -l /dev/serial/by-id/` (FT62AJW7 = leader arm).
The settings use the adapter's serial number, `leader.port:
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT62AJW7-if00-port0`, which needs no rule: with the
gripper's U2D2 also connected, `/dev/ttyUSB0` may be the gripper.

### Another PC
`runtime.cpu_cores: [0, 11]` pins the teleop to this laptop's performance cores (i7-13700H). On another
CPU set the right cores or `null` (no pinning).

## 2. Before each run
- [ ] Leader arm powered (12 V) and its USB cable connected (leader torque is not used).
- [ ] Gripper: 12 V on the U2D2 Power Hub, U2D2 USB connected (or run with `--no-gripper`).
- [ ] RB5 control box on; the tablet may stay connected (the teleop sets the mode itself).
- [ ] **Real only:** the required items at the top of `docs/real_first_run_checklist.md` (collision
      detection ON, emergency stop in reach, a second person, the workspace clear).

## 3. Run
```
conda activate rb5
python -m teleop            # Simulation (configs/sim.yaml)
python -m teleop --real     # REAL robot (configs/real.yaml); asks you to type REAL
python -m teleop --no-gripper   # without the gripper (Simulation or --real)
```
What happens:
1. The settings are checked (unknown keys, ranges, relations, Real limits). Any problem: it does not start.
2. Checks: latency_timer, leader arm ping (right arm joints, right handle, left handle), gripper (read
   only: model, hardware error, >= 10 V, position inside closed / open), RB5 mode, robot not moving, no
   controller errors, TCP offset (Real: equal to the expected value).
3. Ready pose (85.116, 4.300, -119.566, 31.666, -87.544, 53.521) deg: TCP about r 450, z 152, azimuth
   -83 deg (the workspace is at the robot's side since 2026-10-18), tool down. The current / target joints and the change are shown; type `yes` to move (5 deg/s). Skipped
   when already there.
4. Checks at the ready pose: DH model vs controller (<= 5 mm), inside the range, singularity and joint
   limit margins.
5. Gripper: the writes are listed (mode 5, speed, force cap 0.50 A, goal = present position, torque
   on); type `yes`. It moves in Simulation too. It holds still until the left trigger points at its
   position (catch), then follows.
6. Teleop until Ctrl+C. At the end the gripper torque goes off (mode 3): **put a held object down
   first**. The log is saved as `data/teleop/teleop_<sim|real>_<time>.csv` with a copy of
   the settings (`..._settings.yaml`) and the gripper thread log (`..._gripper.csv`). Summary of runs: `python scripts/23_trial_summary.py <logs>`; full report: `python scripts/24_run_report.py <log>`.

## 4. Controls
| input | effect |
|---|---|
| **right** button held | clutch: the RB5 follows the hand, relative to the moment of the press |
| ... hand forward / back | the arm extends / retracts along the direction it points (radius) |
| ... hand up / down | height |
| ... hand left / right | nothing (J1 stays locked) |
| **left** button held (with the right one) | rotate mode: radius and height stay; moving the hand left / right turns the RB5 base (J1), 100 mm = 30 deg (left = counterclockwise seen from above) |
| right button released | the RB5 holds its pose; move the hand freely, press again to continue |
| twist the handle | not used: the gripper turns with J1, RB5 J6 stays still |
| **left** trigger | gripper: released (0) = closed, fully pulled (1000) = fully open, in proportion; independent of the clutch |
| right trigger | not used (logged when `gripper.trigger: right`) |

Speeds: radial / vertical up to 250 mm/s, sideways (turning J1) up to 360 mm/s, joints J1 75 / others
60 deg/s, acceleration 2.5 m/s^2 (smooth starts and stops). If the hand is faster, the robot keeps up to
20 mm (rotate mode: 15 deg) of lag and catches up after the hand stops; more than that is dropped, so
the hand and the robot drift apart a little: release and press the clutch again to re-align.

Range (absolute, base frame): radius 250-800 mm (the inner radius rises toward the floor: 350 at z <= 0,
300 at z 50, 250 at z >= 100; going down close in pushes the robot outward), height -50..465 mm (the outer radius falls from 800 to
650 between z 265 and 465), azimuth -160..133 deg. Releasing the left button or switching modes never
makes the robot jump.

Other settings modes (not used now): `mapping.left_button: vertical` (left button = x, y fixed, only z,
x 1.5), `mapping.lock_azimuth: false` (left / right moves the robot sideways),
`yaw.mode: leader_j6` (twist the handle to turn the tool). Change real.yaml and sim.yaml together.

## 5. Stop
- **Ctrl+C**: normal stop (the target is held for 0.3 s, then speed 0, then wait until stopped).
- **Emergency stop button** on the robot / tablet at any time.
- It stops by itself with a message when: a singularity or joint limit margin falls below its stop
  value, 3 leader joint reads fail in a row, 3 late cycles in a row, the RB5 state is stale, or the
  controller reports an error (error stops use task_stop).

## 6. When something goes wrong
| message | meaning / what to do |
|---|---|
| `FTDI latency_timer is 16 ms` | install the udev rule or set it by hand (section 1) |
| `leader arm IDs [...] did not answer the ping` | leader power, USB cable, port name in the settings |
| `... was requested but the controller reports ... mode` | set the mode on the tablet, or check that no other program changed it |
| `the robot is moving` | wait, or stop the program running on the tablet |
| `TCP radius / z / azimuth ... is outside` | the start pose is outside the range; let the teleop move to the ready pose |
| `J... from its limit` / `J5` / `shoulder` / `J3 ... from straight` at start | the ready pose is not valid for these settings |
| `range limit` events | the hand asks for a target outside the range; move back inside |
| `joint speed limit` events | the hand is faster than the joints allowed; move a little slower |
| `late: ... ms since last send` | one cycle took too long (CPU); a single one is harmless, 3 in a row stop the teleop |
| `stopping: predicted ... < ...` | a singularity or joint limit was close; restart from the ready pose |
| clutch released by itself | 3 failed handle reads in a row; check the handle cable |
| the robot slows down near the base and the floor | joint limit watch (J3 margin below 15 deg); normal there |

The CSV has one row per 20 ms cycle (leader, targets, tcp_ref, joints, margins, limit flags, timing,
modes). For speeds and accelerations from the RB5 state, divide by the real sample times
(`t_s - state_age_ms`): the state arrives 13-22 ms apart (decisions 2026-10-16).

# Teleop guide (leader arm -> RB5)

`teleop.py` maps the RB-Y1 leader arm's right handle to the RB5 tool: position (wrist center),
yaw (handle twist = leader J6), and a gripper signal (trigger, logged only). Simulation is the
default. Real mode is not cleared yet: see `docs/real_first_run_checklist.md`.

## 1. One-time setup

### udev rule (stable name + 1 ms latency)
The leader arm's FTDI adapter needs `latency_timer = 1` (default 16 ms makes each read about 16 ms,
script 11). The rule also creates `/dev/rby1_leader_arm`.

```
sudo cp udev/99-rby1-leader-arm.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Then unplug and replug the leader arm USB cable, and check:
```
ls -l /dev/rby1_leader_arm                                         # -> ttyUSB0 (or another number)
cat /sys/bus/usb-serial/devices/$(basename $(readlink -f /dev/rby1_leader_arm))/latency_timer   # -> 1
```
The rule matches the adapter's serial number (`FT62AJW7`); another adapter needs its own line
(find the serial with `udevadm info -a -n /dev/ttyUSB0 | grep serial`).
After installing it, set `leader.port: /dev/rby1_leader_arm` in `configs/sim.yaml`.

Without the rule, set it by hand after every reboot or replug:
```
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### Python
conda env `rb5` (rbpodo, dynamixel-sdk, numpy, PyYAML). Tests: `python -m unittest discover tests`.

## 2. Before each run (Simulation)
- [ ] Leader arm powered (12 V) and its USB cable connected; leader torque is not needed.
- [ ] latency_timer is 1 (the teleop checks it and refuses otherwise).
- [ ] RB5 control box on; the tablet may stay connected. The teleop sets Simulation mode itself.
- [ ] `configs/sim.yaml` is the one you want (scale, speed, range, ready pose).
- [ ] Right handle: you know the controls (below).

## 3. Run
```
conda activate rb5
python teleop.py                 # Simulation, configs/sim.yaml
python teleop.py --config other.yaml
```
The process pins itself to the performance cores (`runtime.cpu_cores: [0, 11]` in the settings;
planning p99 5-6 ms instead of 8-9 ms, decisions 2026-10-12). No `taskset` needed.

What happens:
1. The settings are checked (unknown keys, ranges, relations). Any problem: it does not start.
2. Checks: latency_timer, leader arm ping (joints 0-6 and the handle), RB5 mode = Simulation,
   robot not moving, no controller errors, TCP offset without orientation offset.
3. Ready pose (D: tool down, wrist outside, TCP r 500, z about 102): the current / target joints and
   the change are shown; type `yes` to move (move_j at 30 x speed bar 0.1 = 3 deg/s). Skipped when
   already there.
4. Checks at the ready pose: DH model vs controller (<= 5 mm), TCP inside the range, singularity and
   joint limit margins.
5. Teleop until Ctrl+C. The log is saved as `data/teleop_sim_<time>.csv` with
   `..._settings.yaml` (a copy of the settings used).

## 4. Controls (right handle)
| input | effect |
|---|---|
| button held | clutch: the RB5 follows the hand's motion (relative to the moment of the press) |
| button released | the RB5 holds its pose; move the hand freely, press again to continue |
| twist the handle (leader J6) | RB5 tool yaw about the vertical axis (same angle) |
| trigger | gripper signal `grip_close` = 1 - trigger / 1000 (trigger reads 0 pulled, 1000 released: pulled = closed); logged only |

Scale 1.5 in Simulation: the RB5 moves 1.5 times the hand motion. Lower the hand vertically: the
leader's wrist center also moves toward the body when the hand comes down (decisions 2026-10-12).

## 5. Stop
- **Ctrl+C**: normal stop (the target is held for 0.3 s, then speed 0, then wait until stopped).
- Stops by itself with a message when: a singularity or joint limit margin falls below its stop value,
  3 leader joint reads fail in a row, 3 late cycles in a row, the RB5 state is stale, or the controller
  reports an error (error stops use task_stop).

## 6. When something goes wrong
| message | meaning / what to do |
|---|---|
| `FTDI latency_timer is 16 ms` | install the udev rule or set it by hand (section 1) |
| `leader arm IDs [...] did not answer the ping` | leader power, USB cable, port name in the settings |
| `Simulation was requested but the controller reports Real mode` | set the mode on the tablet, or check that no other program changed it |
| `the robot is moving` | wait, or stop the program running on the tablet |
| `TCP radius ... is outside` | the start pose is outside the range; let the teleop move to the ready pose first |
| `J... from its limit` / `J5` / `shoulder` / `J3 ... from straight` at start | the ready pose is not valid for these settings |
| `range limit` events | the hand asks for a target outside the cylinder (r, z, azimuth); move back inside |
| `joint speed limit` events | the hand is faster than the joints allowed; move slower |
| `late: ... ms since last send` | a cycle took too long (CPU); check `runtime.cpu_cores` in the settings |
| `stopping: predicted ... < ...` | a singularity or joint limit was close; restart from the ready pose |
| clutch released by itself | 3 failed handle reads in a row; check the handle cable |

Analysis of a run: the CSV has one row per 20 ms cycle (targets, tcp_ref, joints, margins, limit
flags, timing, yaw, gripper). See `docs/decisions.md` for what each limit means.

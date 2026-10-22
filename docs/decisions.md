# Decisions

## Contents

Entries below are in date order; this list groups them by topic. Open items: [PENDING](#pending).

**Leader arm (RB-Y1 scripts 14, 20, leader signal)**
- [2026-10-10 — 14: align the leader arm to the RB-Y1 READY pose at startup](#2026-10-10--14-align-the-leader-arm-to-the-rb-y1-ready-pose-at-startup)
- [2026-10-10 — 14: one dynamixel_sdk PortHandler for the leader arm](#2026-10-10--14-one-dynamixel_sdk-porthandler-for-the-leader-arm)
- [2026-10-10 — 14: leader arm torque constant chosen per ID from the actual model (differs from Rainbow)](#2026-10-10--14-leader-arm-torque-constant-chosen-per-id-from-the-actual-model-differs-from-rainbow)
- [2026-10-10 — 14: leader arm holds its pose while the clutch is released (rule changed)](#2026-10-10--14-leader-arm-holds-its-pose-while-the-clutch-is-released-rule-changed)
- [2026-10-10 — 14: leader wrist joints always torque off after the alignment](#2026-10-10--14-leader-wrist-joints-always-torque-off-after-the-alignment)
- [2026-10-11 — 14: leader thread + Fast Sync Read; --period 0.02 (default) / 0.01; C++ not needed](#2026-10-11--14-leader-thread--fast-sync-read---period-002-default--001-c-not-needed)
- [2026-10-12 — 20: leader arm FK with rby1_sdk.dynamics; tip = wrist center](#2026-10-12--20-leader-arm-fk-with-rby1_sdkdynamics-tip--wrist-center)
- [2026-10-12 — Leader handle point instead of the wrist center: small effect, not applied](#2026-10-12--leader-handle-point-instead-of-the-wrist-center-small-effect-not-applied)
- [2026-10-12 — Leader deadband (hysteresis) against the wrist shake](#2026-10-12--leader-deadband-hysteresis-against-the-wrist-shake)

**RB5 API probes (17-19)**
- [2026-10-12 — 17/18: TCP pose sources and the move_servo_l probe](#2026-10-12--1718-tcp-pose-sources-and-the-move_servo_l-probe)
- [2026-10-12 — 18 results: move_servo_l convention confirmed; wrist singularity near J5 = 0](#2026-10-12--18-results-move_servo_l-convention-confirmed-wrist-singularity-near-j5--0)
- [2026-10-12 — 18 rz away from J5 = 0: wrist singularity hypothesis supported](#2026-10-12--18-rz-away-from-j5--0-wrist-singularity-hypothesis-supported)
- [2026-10-12 — 19: move_j speed is scaled by the speed bar](#2026-10-12--19-move_j-speed-is-scaled-by-the-speed-bar)
- [2026-10-12 — 19: --pose mode (all joints), shortest equivalent angles, RB5 joint limits](#2026-10-12--19---pose-mode-all-joints-shortest-equivalent-angles-rb5-joint-limits)

**Ready pose**
- [2026-10-12 — RB5 ready pose proposal for pick and place (not applied)](#2026-10-12--rb5-ready-pose-proposal-for-pick-and-place-not-applied)
- [2026-10-12 — Ready pose: wrist folded inward in A (user); proposal C with the wrist outside](#2026-10-12--ready-pose-wrist-folded-inward-in-a-user-proposal-c-with-the-wrist-outside)
- [2026-10-12 — Ready pose C applied (Simulation); runs pinned to performance cores](#2026-10-12--ready-pose-c-applied-simulation-runs-pinned-to-performance-cores)
- [2026-10-12 — Object on the RB5 table (z 0): ready pose D, cylinder r 450-550 mm](#2026-10-12--object-on-the-rb5-table-z-0-ready-pose-d-cylinder-r-450-550-mm)
- [2026-10-12 — Teleop: ready pose from the real RB5, scale 1.5, J3 +/-165, range around the new pose](#2026-10-12--teleop-ready-pose-from-the-real-rb5-scale-15-j3--165-range-around-the-new-pose)
- [2026-10-12 — Real ready-pose move 3 deg/s](#2026-10-12--real-ready-pose-move-3-degs)

**Mapping, range and safety limits (21 -> teleop)**
- [2026-10-12 — 21: leader -> RB5 position mapping, first Simulation results](#2026-10-12--21-leader---rb5-position-mapping-first-simulation-results)
- [2026-10-12 — 21: cylinder range; range excess was being dropped (also in 13)](#2026-10-12--21-cylinder-range-range-excess-was-being-dropped-also-in-13)
- [2026-10-12 — 21: three singularities watched, r_max 500 mm, C mode (catch-up up to a lag)](#2026-10-12--21-three-singularities-watched-r_max-500-mm-c-mode-catch-up-up-to-a-lag)
- [2026-10-12 — 21: default is C with a 30 mm lag](#2026-10-12--21-default-is-c-with-a-30-mm-lag)
- [2026-10-12 — 21: --scale 0.5 Simulation result (104723, C 30 mm, 200 mm/s)](#2026-10-12--21---scale-05-simulation-result-104723-c-30-mm-200-mms)
- [2026-10-12 — 21: joint limit watch, r_min 375 mm; pick positions r 400-500 mm recommended](#2026-10-12--21-joint-limit-watch-r_min-375-mm-pick-positions-r-400-500-mm-recommended)
- [2026-10-12 — 21 from ready pose D: depth reached; vertical-hand habit halves the backward drift](#2026-10-12--21-from-ready-pose-d-depth-reached-vertical-hand-habit-halves-the-backward-drift)
- [2026-10-12 — 21: joint speed limit per joint (Simulation)](#2026-10-12--21-joint-speed-limit-per-joint-simulation)
- [2026-10-12 — Sim: r_max 600 -> 800 mm (reach x 750)](#2026-10-12--sim-r_max-600---800-mm-reach-x-750)
- [2026-10-12 — Real: range as wide as Simulation (user request)](#2026-10-12--real-range-as-wide-as-simulation-user-request)
- [2026-10-12 — Sim z upper bound +200 (same as Real)](#2026-10-12--sim-z-upper-bound-200-same-as-real)
- [2026-10-12 — z upper bound +400 with an outer radius that falls with height (Sim and Real)](#2026-10-12--z-upper-bound-400-with-an-outer-radius-that-falls-with-height-sim-and-real)

**Timing and CPU**
- [2026-10-12 — 21: cause of the late cycles (offline test)](#2026-10-12--21-cause-of-the-late-cycles-offline-test)
- [2026-10-12 — 21: one Jacobian per cycle; late-cycle cause is CPU power saving + hybrid cores](#2026-10-12--21-one-jacobian-per-cycle-late-cycle-cause-is-cpu-power-saving--hybrid-cores)

**Yaw and gripper**
- [2026-10-12 — Yaw mapping: RB5 yaw change = leader J6 change (method B)](#2026-10-12--yaw-mapping-rb5-yaw-change--leader-j6-change-method-b)
- [2026-10-12 — Yaw mapping Simulation result (115603, ready pose D, scale 0.5, C)](#2026-10-12--yaw-mapping-simulation-result-115603-ready-pose-d-scale-05-c)
- [2026-10-12 — Gripper signal from the leader trigger: rules](#2026-10-12--gripper-signal-from-the-leader-trigger-rules)
- [2026-10-12 — Gripper direction fixed: grip_close = 1 - trigger / 1000](#2026-10-12--gripper-direction-fixed-grip_close--1---trigger--1000)

**Teleop code and settings**
- [2026-10-12 — Teleop cleanup: teleop.py + teleop/ modules + configs + tests](#2026-10-12--teleop-cleanup-teleoppy--teleop-modules--configs--tests)
- [2026-10-12 — Sim: speed 400 mm/s and lag 50 mm; Real: provisional values for a first test](#2026-10-12--sim-speed-400-mms-and-lag-50-mm-real-provisional-values-for-a-first-test)

**Real runs (2026-10-12)**
- [2026-10-12 — Real runs 132310 / 133219; faster Real settings; measured joints logged](#2026-10-12--real-runs-132310--133219-faster-real-settings-measured-joints-logged)
- [2026-10-12 — Real run 133743 (50 mm/s, C 20 mm) and z upper bound +200](#2026-10-12--real-run-133743-50-mms-c-20-mm-and-z-upper-bound-200)
- [2026-10-12 — Real run 134902 and TCP speed 100 mm/s](#2026-10-12--real-run-134902-and-tcp-speed-100-mms)
- [2026-10-12 — Real run 135230 (100 mm/s) and ready move 5 deg/s](#2026-10-12--real-run-135230-100-mms-and-ready-move-5-degs)
- [2026-10-12 — Real joint speed limit 0.8 deg/cycle (40 deg/s)](#2026-10-12--real-joint-speed-limit-08-degcycle-40-degs)

**Real teleop speed (real-1)**
- [2026-10-16 — real-1: trial protocol for faster Real teleop](#2026-10-16--real-1-trial-protocol-for-faster-real-teleop)
- [2026-10-16 — Real speed limits to option B: TCP 200 mm/s, joints 1.2 deg/cycle (60 deg/s)](#2026-10-16--real-speed-limits-to-option-b-tcp-200-mms-joints-12-degcycle-60-degs)
- [2026-10-16 — Low-pass filter on the leader wrist center (tau 30 ms) against the shake at 200 mm/s](#2026-10-16--low-pass-filter-on-the-leader-wrist-center-tau-30-ms-against-the-shake-at-200-mms)
- [2026-10-16 — Ready pose moved to r 500, z 200 (Real and Sim); range kept in absolute terms](#2026-10-16--ready-pose-moved-to-r-500-z-200-real-and-sim-range-kept-in-absolute-terms)
- [2026-10-16 — Cylindrical mapping trial (Sim and Real): a hand swing about the leader base center turns J1](#2026-10-16--cylindrical-mapping-trial-sim-and-real-a-hand-swing-about-the-leader-base-center-turns-j1)
- [2026-10-16 — Cylindrical mapping reverted to cartesian after one Real run](#2026-10-16--cylindrical-mapping-reverted-to-cartesian-after-one-real-run)
- [2026-10-16 — Vertical mode on the left handle button (x, y fixed, only z)](#2026-10-16--vertical-mode-on-the-left-handle-button-x-y-fixed-only-z)
- [2026-10-16 — Vertical mode z x 2 (vertical_z_scale)](#2026-10-16--vertical-mode-z-x-2-vertical_z_scale)
- [2026-10-16 — TCP floor 30 -> -50 mm (nothing mounted on the flange)](#2026-10-16--tcp-floor-30----50-mm-nothing-mounted-on-the-flange)
- [2026-10-16 — Faster J1 (sideways 300 mm/s), RB5 J6 still (tool follows the base), vertical z x 1.5](#2026-10-16--faster-j1-sideways-300-mms-rb5-j6-still-tool-follows-the-base-vertical-z-x-15)
- [2026-10-16 — Left button = J1 rotate mode; azimuth range +/-120](#2026-10-16--left-button--j1-rotate-mode-azimuth-range--120)
- [2026-10-16 — Azimuth lock: J1 turns only in the rotate mode](#2026-10-16--azimuth-lock-j1-turns-only-in-the-rotate-mode)
- [2026-10-16 — Target acceleration limit 2.5 m/s^2 against the shake when the limits switch](#2026-10-16--target-acceleration-limit-25-ms2-against-the-shake-when-the-limits-switch)
- [2026-10-16 — Rotate mode: 15 deg catch-up, target kept on the circle, re-anchor at the stop point](#2026-10-16--rotate-mode-15-deg-catch-up-target-kept-on-the-circle-re-anchor-at-the-stop-point)
- [2026-10-16 — Range in absolute base-frame values; ready pose = the user's pose at r 450](#2026-10-16--range-in-absolute-base-frame-values-ready-pose--the-users-pose-at-r-450)
- [2026-10-16 — Joint-limit slow threshold 20 -> 15 deg](#2026-10-16--joint-limit-slow-threshold-20---15-deg)
- [2026-10-16 — Azimuth range widened on the minus side: -160 .. 133 deg](#2026-10-16--azimuth-range-widened-on-the-minus-side--160--133-deg)
- [2026-10-16 — J1 a bit faster (sideways 360 mm/s, J1 75 deg/s); acceleration limit moved to the last stage](#2026-10-16--j1-a-bit-faster-sideways-360-mms-j1-75-degs-acceleration-limit-moved-to-the-last-stage)
- [2026-10-16 — Radial / vertical speed cap 200 -> 250 mm/s](#2026-10-16--radial--vertical-speed-cap-200---250-mms)
- [2026-10-16 — Joint speed limit on the step only (the robot's lag is no longer added)](#2026-10-16--joint-speed-limit-on-the-step-only-the-robots-lag-is-no-longer-added)
- [2026-10-16 — real-1 final settings frozen; Simulation = Real](#2026-10-16--real-1-final-settings-frozen-simulation--real)
- [2026-10-16 — real-1 closed: summary](#2026-10-16--real-1-closed-summary)
- [2026-10-16 — Cleanup before the gripper: cylindrical mapping and the script-21 comparisons removed](#2026-10-16--cleanup-before-the-gripper-cylindrical-mapping-and-the-script-21-comparisons-removed)
- [2026-10-16 — Gripper signal moved to teleop/gripper.py](#2026-10-16--gripper-signal-moved-to-teleopgripperpy)
- [2026-10-16 — Loop: CSV row and status line out of run()](#2026-10-16--loop-csv-row-and-status-line-out-of-run)

**Cameras**
- [2026-10-17 — cam-1: RealSense D435 + D405 RGB test (pyrealsense2 2.58.4)](#2026-10-17--cam-1-realsense-d435--d405-rgb-test-pyrealsense2-2584)

**Demonstration recording**
- [2026-10-17 — What jnt_ref is: the controller's joint reference, 60 ms behind the sent target](#2026-10-17--what-jnt_ref-is-the-controllers-joint-reference-60-ms-behind-the-sent-target)
- [2026-10-17 — Open (step 3): demonstration recording rate 50 Hz vs 30 Hz](#2026-10-17--open-step-3-demonstration-recording-rate-50-hz-vs-30-hz)
- [2026-10-19 — rec-1: demonstration format (30 Hz; action x, y, z, gripper; state 6 joints + gripper; keyboard episodes)](#2026-10-19--rec-1-demonstration-format-30-hz-action-x-y-z-gripper-state-6-joints--gripper-keyboard-episodes)
- [2026-10-19 — rec-2: separate conda env lerobot (LeRobot 0.6.1, Python 3.12); dummy dataset with the rec-1 format](#2026-10-19--rec-2-separate-conda-env-lerobot-lerobot-061-python-312-dummy-dataset-with-the-rec-1-format)
- [2026-10-19 — rec-3a: camera recorder in its own process (JPEG per frame + time CSV)](#2026-10-19--rec-3a-camera-recorder-in-its-own-process-jpeg-per-frame--time-csv)
- [2026-10-19 — rec-3b: --record in the teleop (LeRobot keys, sessions, episodes.json, 120 s limit)](#2026-10-19--rec-3b---record-in-the-teleop-lerobot-keys-sessions-episodesjson-120-s-limit)
- [2026-10-19 — rec-3c: spoken messages while recording (spd-say, as LeRobot)](#2026-10-19--rec-3c-spoken-messages-while-recording-spd-say-as-lerobot)
- [2026-10-19 — rec-4a: converter (sessions -> LeRobotDataset, script 31)](#2026-10-19--rec-4a-converter-sessions---lerobotdataset-script-31)
- [2026-10-19 — rec-4b: replay in Simulation through the teleop limits (TargetFollower, script 32); rbpodo in env lerobot](#2026-10-19--rec-4b-replay-in-simulation-through-the-teleop-limits-targetfollower-script-32-rbpodo-in-env-lerobot)
- [2026-10-19 — Real pilot 141929: the D405 dropped off USB; camera stall detection; converter coverage check, --cameras](#2026-10-19--real-pilot-141929-the-d405-dropped-off-usb-camera-stall-detection-converter-coverage-check---cameras)
- [2026-10-19 — rec-5: ACT trial training on the pilot (front only): the flow works; inference 17 ms GPU](#2026-10-19--rec-5-act-trial-training-on-the-pilot-front-only-the-flow-works-inference-17-ms-gpu)
- [2026-10-21 — Data rules for the first ACT dataset (docs/data_rules.md)](#2026-10-21--data-rules-for-the-first-act-dataset-docsdata_rulesmd)
- [2026-10-21 — cam-2a: fixed camera exposure candidates measured (script 34)](#2026-10-21--cam-2a-fixed-camera-exposure-candidates-measured-script-34)
- [2026-10-21 — cam-2b: fixed camera settings in the teleop; brightness check at start](#2026-10-21--cam-2b-fixed-camera-settings-in-the-teleop-brightness-check-at-start)
- [2026-10-21 — cam-3: a saturated camera image refuses the start (red, no typed yes)](#2026-10-21--cam-3-a-saturated-camera-image-refuses-the-start-red-no-typed-yes)
- [2026-10-21 — cam-4a: C270 webcam as the wrist camera (the D405 broke)](#2026-10-21--cam-4a-c270-webcam-as-the-wrist-camera-the-d405-broke)
- [2026-10-21 — cam-4b: wrist webcam settings (8.3 ms, gain 78); settings written once the stream runs](#2026-10-21--cam-4b-wrist-webcam-settings-83-ms-gain-78-settings-written-once-the-stream-runs)
- [2026-10-21 — cam-4c: webcam USB autosuspend off (udev rule)](#2026-10-21--cam-4c-webcam-usb-autosuspend-off-udev-rule)
- [2026-10-21 — Grasp not visible in the gripper state (test A)](#2026-10-21--grasp-not-visible-in-the-gripper-state-test-a)
- [2026-10-21 — cam-5a: three cameras (grip = Brio on the gripper); C270 in MJPG; missing-rows check](#2026-10-21--cam-5a-three-cameras-grip--brio-on-the-gripper-c270-in-mjpg-missing-rows-check)
- [2026-10-21 — cam-5b: Brio autosuspend off; wrist gain 86, grip gain 152; teleop with three cameras](#2026-10-21--cam-5b-brio-autosuspend-off-wrist-gain-86-grip-gain-152-teleop-with-three-cameras)
- [2026-10-21 — p2: learning code in rb5_lerobot/, its settings in configs/policy.yaml (p2-a: the move)](#2026-10-21--p2-learning-code-in-rb5_lerobot-its-settings-in-configspolicyyaml-p2-a-the-move)
- [2026-10-21 — range.z_min_mm -50 -> 19 mm for the mounted gripper](#2026-10-21--rangez_min_mm--50---19-mm-for-the-mounted-gripper)
- [2026-10-21 — rec-6 policy runner: decisions (two processes, temporal ensembling by time, 100 % speed, keyboard toggle)](#2026-10-21--rec-6-policy-runner-decisions-two-processes-temporal-ensembling-by-time-100--speed-keyboard-toggle)
- [2026-10-21 — rec-6a: policy runner structure (teleop/policy.py) with a fake policy and robot](#2026-10-21--rec-6a-policy-runner-structure-teleoppolicypy-with-a-fake-policy-and-robot)
- [2026-10-21 — rec-6b: train/run observations match; inference 33 ms on the GPU power limit; GC freeze; ACT runner script](#2026-10-21--rec-6b-trainrun-observations-match-inference-33-ms-on-the-gpu-power-limit-gc-freeze-act-runner-script)
- [2026-10-21 — rec-6b fixes after the first Simulation runs: late check right before the send; servo and policy on separate cores](#2026-10-21--rec-6b-fixes-after-the-first-simulation-runs-late-check-right-before-the-send-servo-and-policy-on-separate-cores)
- [2026-10-21 — rec-6c: gripper in the policy runner (shared start code, no catch)](#2026-10-21--rec-6c-gripper-in-the-policy-runner-shared-start-code-no-catch)
- [2026-10-21 — p1: first round of the whole pipeline with the current setup; every step typed by the user](#2026-10-21--p1-first-round-of-the-whole-pipeline-with-the-current-setup-every-step-typed-by-the-user)

**Gripper**
- [2026-10-17 — grip-1: MX-106T gripper on its own U2D2; ports by adapter serial; read-only scan (27)](#2026-10-17--grip-1-mx-106t-gripper-on-its-own-u2d2-ports-by-adapter-serial-read-only-scan-27)
- [2026-10-17 — grip-2: left trigger -> gripper alone (script 28), current-based position mode](#2026-10-17--grip-2-left-trigger---gripper-alone-script-28-current-based-position-mode)
- [2026-10-17 — grip-3a: gripper code in teleop/gripper.py (Motor, GripCommand, GripperWorker thread)](#2026-10-17--grip-3a-gripper-code-in-teleopgripperpy-motor-gripcommand-gripperworker-thread)
- [2026-10-17 — grip-3b: gripper in the teleop (left trigger, own thread, Simulation and Real)](#2026-10-17--grip-3b-gripper-in-the-teleop-left-trigger-own-thread-simulation-and-real)

**Teleop tuning after the gripper**
- [2026-10-17 — Inner radius by height: r_min 350 / 300 / 250 at z 0 / 50 / 100](#2026-10-17--inner-radius-by-height-r_min-350--300--250-at-z-0--50--100)
- [2026-10-17 — Gripper: longer reply timeout on its port (thread switching); a bit faster (210 / 200)](#2026-10-17--gripper-longer-reply-timeout-on-its-port-thread-switching-a-bit-faster-210--200)
- [2026-10-18 — Ready pose turned to azimuth -83 deg (workspace moved to the side)](#2026-10-18--ready-pose-turned-to-azimuth--83-deg-workspace-moved-to-the-side)
- [2026-10-19 — Gripper Goal Current 150 -> 200 (faster start; grip force +33 %)](#2026-10-19--gripper-goal-current-150---200-faster-start-grip-force-33-)
- [2026-10-19 — Ready pose = the user's pose with J6 set for the gripper camera](#2026-10-19--ready-pose--the-users-pose-with-j6-set-for-the-gripper-camera)
- [2026-10-19 — Gripper grasp squeeze: goal past the stop point so the grip force is the Goal Current cap](#2026-10-19--gripper-grasp-squeeze-goal-past-the-stop-point-so-the-grip-force-is-the-goal-current-cap)
- [2026-10-19 — Gripper: Goal Current 250 only while grasping; squeeze 150 -> 60 ticks](#2026-10-19--gripper-goal-current-250-only-while-grasping-squeeze-150---60-ticks)
- [2026-10-19 — Gripper closed for now (grip-5): state and known limits](#2026-10-19--gripper-closed-for-now-grip-5-state-and-known-limits)

**Repository structure (2026-10-14)**
- [2026-10-14 — Leader arm URDF committed under assets/](#2026-10-14--leader-arm-urdf-committed-under-assets)
- [2026-10-14 — environment.yml for the rb5 env](#2026-10-14--environmentyml-for-the-rb5-env)
- [2026-10-14 — Teleop logs in data/teleop/](#2026-10-14--teleop-logs-in-datateleop)
- [2026-10-14 — Teleop entry point moved into the package: python -m teleop](#2026-10-14--teleop-entry-point-moved-into-the-package-python--m-teleop)
- [2026-10-14 — One robot file: configs/robot.yaml](#2026-10-14--one-robot-file-configsrobotyaml)
- [2026-10-14 — config.py moved to scripts/](#2026-10-14--configpy-moved-to-scripts)
- [2026-10-14 — Scripts 15, 20, 21 also use the assets/ URDF](#2026-10-14--scripts-15-20-21-also-use-the-assets-urdf)
- [2026-10-14 — udev rule moved to assets/udev/](#2026-10-14--udev-rule-moved-to-assetsudev)

## 2026-10-10 — 14: align the leader arm to the RB-Y1 READY pose at startup

**Decision:** At startup, script 14 moves the real leader arm to the RB-Y1 READY arm pose
(example 35 `READY_POSE["M"]`) in 6 s with a 0.5 Nm current limit, then holds it with that weak
torque. Each arm's torque is switched off at that arm's first clutch press. During teleoperation the
only write to the leader arm is torque off (added to CLAUDE.md safety rules).

**Reason:** Both robots then start from the same joint pose, so the first clutch press maps a
matching pose.

**Rejected alternatives:**
- Move the RB-Y1 simulator to the leader arm's current pose (keeps the leader read-only). Rejected
  by choice; some leader poses are outside the RB-Y1 joint limits (e.g. left J6 180.2 deg vs +/-155).
- Full example 35 behavior (gravity-compensation torque while the button is held, position hold when
  released). Needs the dynamics model in the loop; out of scope for now.
- Switch the torque off right after alignment: the arm drops and the aligned pose is lost at once.

## 2026-10-10 — 14: one dynamixel_sdk PortHandler for the leader arm

**Decision:** Alignment writes, teleop reads and torque off all use one `dynamixel_sdk` PortHandler,
opened once and closed at the end.

**Reason:** `rby1_sdk.DynamixelBus` (used by example 35) has no Python method to close the port;
it is closed only by the C++ destructor. Mixing it with dynamixel_sdk would open the port twice or
depend on garbage collection.

**Rejected alternatives:** `rby1_sdk.DynamixelBus` for alignment then dynamixel_sdk for teleop;
`rby1_sdk.DynamixelBus` for everything (its fast sync read path was not measured).

## 2026-10-10 — 14: leader arm torque constant chosen per ID from the actual model (differs from Rainbow)

**Decision:** The goal current for a torque is `torque / Kt * 1000 / 2.69` (rby1-sdk
`dynamixel_bus.cpp:501`), with Kt chosen from the model number read by ping:
1130 (XM540-W150) -> 1.6591 Nm/A, 1030 (XM430-W210) -> 1.3043 Nm/A. Unknown model -> no write.

**Difference from Rainbow:** rby1-sdk `src/upc/leader_arm.cpp:12-13` and example 35 (line 282) use
`{1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043}` per arm by ID. On this leader arm the
ping (script 11) returned 1130 for IDs 0-3 and 7-10 and 1030 for IDs 4-6 and 11-13, so Rainbow gives
IDs 3 and 10 (elbows, XM540-W150) the XM430-W210 value. With 0.5 Nm requested that would be raw 142
instead of 112, about 0.63 Nm on those motors (+27 %).

**Evidence:**
- Model number -> name: `dynamixel_easy_sdk/control_table/dynamixel.model` in the installed
  dynamixel-sdk 4.1.0 (`1030 xm430_w210.model`, `1130 xm540_w150.model`).
- Addresses 11 / 64 / 102 / 116 / 132 are the same in `xm430_w210.model` and `xm540_w150.model`.
- Kt values: not found in any local file. 1.6591 = 7.3 / 4.4 and 1.3043 = 3.0 / 2.3 match the
  stall torque / stall current of XM540-W150 and XM430-W210 at 12 V in the ROBOTIS e-Manual
  (from memory, not verified in this repo). Both models use 2.69 mA per current unit.

**Rejected alternatives:** keep Rainbow's per-ID array (wrong for IDs 3 and 10 on this hardware).

## 2026-10-10 — 14: leader arm holds its pose while the clutch is released (rule changed)

**Decision:** Per arm, on clutch press: torque off (move freely by hand). On clutch release
(including "released" after 3 failed button reads): goal position = the joint angles just read,
then torque on, so the arm holds that pose with the startup current limit (0.5 Nm). Operating mode
(5) and goal current are not written again during teleop. CLAUDE.md safety rule changed from
"the only allowed write is torque off" to these two transition writes.

**Reason:** With torque off after the first press, the leader arm dropped whenever the button was
not held, so the pose could not be kept between clutch presses (same idea as example 35 lines
1198-1203: position hold while the button is released).

**Order:** the goal position must be written before torque on; otherwise the motor moves to the
stored goal (the READY pose or an older hold pose).

**Assumption to verify:** torque off does not clear operating mode or goal current (general
Dynamixel behavior, not checked in a document). Check that the arm holds after the first release.

**Rejected alternatives:** example 35's current control with gravity-compensation torque while the
button is held (needs the dynamics model in the loop); writing every cycle (not needed for holding).

## 2026-10-10 — 14: leader wrist joints always torque off after the alignment

**Decision:** Wrist joints (IDs 4-6, 11-13) have torque only during the 6 s startup alignment and are
switched off right after it. On clutch release only the shoulder and elbow joints (IDs 0-3, 7-10)
hold their pose.

**Reason:** While holding, right wrist IDs 4 and 6 oscillated by +/-0.18-0.26 deg (2-3 encoder
ticks) and changed direction on 90-97 of 158 cycles (`data/14_leader_rby1_20261010_112745.csv`);
the user reported strong vibration. The wrists are light, so letting them hang is acceptable.

**Notes:** Position gains read before (11:30) and after (11:32) a leader arm power cycle are the
same: D 6500 (XM540-W150) / 4000 (XM430-W210), I 0, P 800. So these are the power-up values of these
motors, not written by any program (our code never writes address 80). The power cycle was real:
goal current changed from 112 / 142 (ours) to 1000. Operating mode 5 persisted (EEPROM). The cause of
the vibration is not confirmed (high D gain is still only a guess). The left shoulder (ID 7) drifting while held (7.4 -> 18.2 deg in
3.3 s) is a separate open issue.

**Rejected / deferred:** raise the shoulder current limit (B, on hold until the left/right
difference is understood); D gain experiment (C, not now); example 35 gravity compensation (D).

## 2026-10-11 — 14: leader thread + Fast Sync Read; --period 0.02 (default) / 0.01; C++ not needed

**Decision:** During teleop a LeaderThread owns the leader port: Fast Sync Read of the 14 joints +
both buttons, as fast as it can, and it runs the clutch writes requested by the main loop. The main
loop only computes targets and calls `send_command`. `--period 0.02` (50 Hz) stays the default;
`--period 0.01` (100 Hz) is an option to test.

**Measurements (latency_timer 1 ms, 500 reads each, 0 failures):**

| Read of 14 joints | mean | p99 |
|---|---|---|
| dynamixel_sdk GroupSyncRead (previous 14) | 4.65 ms | 7.31 ms |
| dynamixel_sdk Fast Sync Read | 2.05 ms | 4.42 ms |
| rby1_sdk (C++) group_fast_sync_read | 2.14 ms | 4.66 ms |
| rby1_sdk get_motor_states (pos+vel+current) | 5.32 ms | 9.09 ms |
| rby1_sdk read_button_status x2 | 2.22 ms | 5.86 ms |

Fast Sync Read and Sync Read values agree within 1 tick. 100 Hz on the simulator, 300 cycles, arms
holding their pose:

| | single loop (fast sync + buttons + send) | thread split |
|---|---|---|
| main loop work mean / p99 | 8.93 / 14.72 ms | 4.13 / 9.86 ms |
| send interval mean / max | 10.52 / 21.94 ms | 10.00 / 16.50 ms |
| intervals over 15 ms | 21 / 299 | 10 / 299 |
| leader update (thread) | - | 4.53 ms mean (about 220 Hz), age at send 2.4 ms mean |

**Why no C++ (rby1_sdk) reads:** C++ fast sync read (2.14 ms) is no faster than Python Fast Sync
Read (2.05 ms). The time is spent on the bus (USB-serial + 2 Mbps transaction), not in Python. The
leader thread's reads were as fast as alone (1.94 ms), so the GIL was not a problem.

**Late-send limit:** 50 Hz keeps the tested 30 ms (1.5 x period). 100 Hz uses 20 ms (2 x period)
because `send_command` alone sometimes takes about 10 ms (p99 9.8 ms in both layouts). A cycle is
"late" if the gap before sending OR the end-to-end interval between commands (which includes a slow
`send_command`) is over the limit; targets are frozen only when it is known before sending. Every
cycle logs `late`, `late_before_send`, `late_send`, `send_interval_ms`, `send_call_ms` and the RB-Y1
joint angles, so jumps at late cycles can be checked. 3 late cycles in a row stop the run.

**Overrun policy changed for 14:** after an overrun the next cycle starts at once (the 20 ms limit was
measured this way). The 07/08 rule "wait a full period after an overrun" would add up to one more
period after every slow `send_command` (about 20-25 ms gaps at 100 Hz). RB-Y1 commands carry
`minimum_time`, so two commands close together are not the jump risk that RB5 `move_servo_j` had.

**Clutch debounce is time based:** 40 ms of pressed reads to engage, 60 ms of failed reads to release
(2 / 3 cycles at 50 Hz as before, 4 / 6 at 100 Hz). Only new leader samples are counted.

**Rejected alternatives:** single loop with Fast Sync Read (7 % of intervals over 15 ms at 100 Hz);
a separate sender thread (does not remove the slow `send_command` calls); rby1_sdk C++ reads (no speed
gain, and its port cannot be closed from Python).

## 2026-10-12 — 17/18: TCP pose sources and the move_servo_l probe

**Facts from 17 (read-only, Simulation, `scripts/17_tcp_read_compare.py`):** `get_tcp_info`, `tcp_pos`
and `calc_fk_tcp(jnt_ref)` equal `tcp_ref` (difference 0.000). So `get_tcp_info` returns the current
TCP pose (not the TCP setting), and `calc_fk_tcp` / `tcp_ref` share the base frame and mm / deg units.

**TCP offset:** a TCP offset of (0, -187, -40) mm, orientation offset 0, is set on the controller.
Derived as R^T (tcp - tfc) with R = Rz(rz) Ry(ry) Rx(rx) (ZY'X''); an XYZ order gives non-round
numbers. Who set it, and when, is not known. This is separate from the tool mass (0 kg).

**Rotation convention source:** Rainbow rb_cobot_docs, UI Script 1.10 Units ("Z-Y'-X'' Euler angle
(Degree)") and 7.35 Servo L ("Desired Cartesian posture in mm,deg unit (ZY'X'' Euler)"). The t1 / t2 /
gain / alpha text of 7.35 is identical to 7.34 Servo J. The frame of `move_servo_l` is not stated.

**Decision (18):** probe `move_servo_l` in Simulation with step targets, one experiment per run, in
the order hold -> z +1 mm -> rz +1 deg (t1 = send period 0.02 s, t2 0.1, gain 1, alpha 1; 1.0 s of
the same target; end with move_speed_j(0) as in 13).

**Interpretation rule:** "no motion" in the hold experiment cannot be told apart from "command
ignored". The hold result is read together with the z +1 mm experiment as the positive control: only
if z moves by the expected 1 mm does hold's "no motion" show that the command is an absolute pose in
the base frame.

**Rejected alternatives:** ramping the step over 10 cycles (not needed in Simulation; a step shows
the response time directly).

## 2026-10-12 — 18 results: move_servo_l convention confirmed; wrist singularity near J5 = 0

**Results (Simulation, user runs, `data/18_servo_l_{hold,z,rz}_20261012_0742*.csv`):**
hold: no change. z +1 mm: change (0, 0, +1.000). rz +1 deg: rz +1.000, position change 0, but joints
moved up to 6.06 deg (J4 +6.06, J6 -4.98) and the 3 deg joint limit stopped the run. Start J5 = 3.68 deg.

**Conclusion:** `move_servo_l(pnt[x, y, z, rx, ry, rz], ...)` is an absolute TCP pose in the base frame,
mm and deg, ZY'X'' Euler (R = Rz Ry Rx), and an orientation change rotates about the TCP (the TCP
position stays fixed). The hold result counts because z (positive control) moved exactly 1 mm.

**Wrist singularity evidence:**
- Rainbow analytic IK (rb_cobot_docs, technical_docs/analytic_inverse_kinematics) computes
  `th6 = atan2(.../sin(th5), .../sin(th5))`: undefined at th5 = 0 (and 180). The same page also
  writes `R = Rz * Ry * Rx`, and the tool axis is the flange Y axis (`P05 = P06 + d6*Y06`), which fits
  the -187 mm Y component of the TCP offset.
- The manual only describes singularities in general terms (area above/below the base, "rapid joint
  speeds or cause the robot to stop" for Cartesian motion, manual 1.7 Robot Work Area); it does not
  name J5 = 0.
- Model check with the RB5-850 DH table (technical_docs/dh_parameters): FK matches 17's TFC within
  2 mm in position and 0.0002 in rotation matrix elements. Joint change needed for rz +1 deg at the 18
  start pose (J5 = 3.68): J4 +8.0, J6 -6.2, J2 -3.8 deg (observed before the stop: J4 +6.06, J6 -4.98).
  Max joint change by J5: 0 deg -> singular, 1 deg -> 32, 3.68 -> 8.0, 10 -> 2.4, 20 -> 1.0, 45 -> 1.0,
  90 -> 1.5 deg.

**Observations to remember:**
- The TCP reached rz +1 at 80 ms while J4 and J6 kept moving in opposite directions: near J5 = 0,
  J4 + / J6 - barely changes the orientation, so a large joint motion is a nearly invisible TCP motion.
- The safety check is done once per cycle before sending, so the joints went 1.8 deg past the 3 deg
  limit (4.24 at the check -> 6.06 final) after the stop. In 20 ms a joint moved up to 2 deg
  (about 100 deg/s) for a 1 deg TCP command. A real-robot Cartesian teleop needs a joint speed limit
  and a guard on J5 near 0.

## 2026-10-12 — 18 rz away from J5 = 0: wrist singularity hypothesis supported

**Result (Simulation, user run, `data/18_servo_l_rz_20261012_075518.csv`):** start J5 = 31.26 deg (not
45, see below). rz +0.9995 deg, position change < 0.001 mm, max joint change 0.838 deg, no safety stop,
within 0.01 mm / 0.01 deg at 60 ms, max joint step per cycle 0.26 deg (at J5 = 3.68: about 2 deg).

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| measured change [deg] | +0.716 | -0.731 | +0.838 | +0.091 | +0.257 | -0.231 |
| DH model prediction | +0.729 | -0.748 | +0.843 | +0.095 | +0.245 | -0.222 |

The large J4 / J6 motion at J5 = 3.68 deg (J4 +6.06, J6 -4.98 before the stop) is gone at
J5 = 31 deg, and the DH model predicts both runs. Conclusion: the 18 rz joint motion was caused by
the wrist singularity at J5 = 0.

## 2026-10-12 — 19: move_j speed is scaled by the speed bar

**Observation:** 19 moved J5 by 41.3 deg with move_j(10 deg/s, 20 deg/s^2) and speed bar 0.1. The
expected 4.1 s was wrong: after 30 s J5 was at 31.26 deg (27.6 deg, about 0.9 deg/s average) and the
script sent task_stop. This fits an effective speed of 10 x 0.1 = 1 deg/s (plus acceleration time).
02 moved only 2 deg, so it was not noticed there. The UI Script docs (5.41 set_speed_bar) only say
"Set the overall speed control bar" and do not state the scaling; it is inferred from this run.
Servo commands are not scaled (06).

**Consequence:** with the speed bar at 0.1 (CLAUDE.md), a move_j speed argument of v gives about
0.1 v. Timeouts and expected durations must use the scaled speed.

## 2026-10-12 — 20: leader arm FK with rby1_sdk.dynamics; tip = wrist center

**Decision:** Leader arm FK uses `rby1_sdk.dynamics` on `refs/rby1-sdk/models/leader_arm/model.urdf`
(as example 35), with all 14 joints passed to `make_state`. The "tip" is the Link_6R / Link_6L origin,
the intersection of the three wrist axes (center of the J5 motor axis from outside). A handle tip
offset is not in the URDF; it is to be measured and added later. Orientation is shown as ZY'X'' Euler
(same as the RB5). Left arm joints are read too (read-only) for the left-right distance check.

**Reason:** Rainbow's own URDF parser, same code path as example 35; a numpy evaluation of the URDF
chain is used only as a startup self-test (max difference 6e-8 over 50 random poses).

**Pitfall found:** with only the 7 right-arm joints in `make_state`, `set_q` silently ignored J3..J6
(J0..J2 applied, no error). The script stops if the self-test fails.

**Ruler check (user, 2026-10-12):**
- V2 left-right wrist center distance, arms hanging: ruler 302 mm, screen 305 mm (pass, +/-5 mm).
- V3 right arm, J2 near 0, elbow -20 -> -94 deg: screen dx +181.9, dy -18.5, dz +169.1 mm; measured
  height change about 169 mm (measured in inches, converted) (pass).
- With J2 about -100 deg, bending the elbow moves the wrist sideways (-y), on screen and on the arm.
- Leader Base frame confirmed: x forward, y left, z up.
- Not done: V1 (link lengths).
- V4 (handle tilt with a phone inclinometer) skipped by choice. The orientation output is to be
  checked instead by a functional test in the orientation mapping step: compare the wrist rotation
  directions of the leader arm and the RB5.

**Rejected alternatives:** direct numpy FK as the main path (kept as the self-test only).

## 2026-10-12 — 21: leader -> RB5 position mapping, first Simulation results

**Setup:** `scripts/21_leader_rb5_position.py`, leader wrist center -> RB5 TCP position, orientation
fixed, clutch + B (drop the excess) + speed limit, move_servo_l at 50 Hz, J5 watch with the DH model.
Leader FK is the numpy URDF chain (20's self-test reference) so that it runs in the `rb5` env.

**Results (user runs, Simulation):**

| run | speed limit | RB5 path / hand path | speed limited | joint speed limited | range limited |
|---|---|---|---|---|---|
| 092533, 093034 | 50 mm/s | 22 % / 26 % | 86 % / 83 % | 0 % / 0.2 % | 7 % / 8 % |
| 094517 | 200 mm/s (lead limit 8 mm) | 38 % (54 % if only the 200 mm/s cap) | 52 % | 52 % | 9 % |

- Hand speed while engaged: p50 143-206 mm/s, p90 343-570 mm/s. The slow segment (16 % speed
  limited) followed 0.90 of the hand. With B, fast-out / slow-back moves shift the hand-robot offset
  (segment 5: hand net 10 mm, RB5 49 mm).
- Direction: cos(hand step, target step) median 1.000, no reversed step.
- "J1 barely turns": at the 092533 start pose J1 moves 0.318 deg per mm of TCP x but only 0.018 per mm
  of y. J1 follows the wrist point P05, not the TCP; with the orientation fixed and the 187 mm TCP
  offset, P05 sat about 60 deg away from the TCP direction. With a fixed orientation the joint motion
  of a pure translation does not depend on the TCP offset (TCP and flange give the same numbers).
- 094517 ended in a shoulder singularity: P05 horizontal radius 135.7 -> 110.8 mm (d4 = 110.7), up to
  14.2 deg of joint motion per mm. Only the joint speed limit held it (actual max 1.57 deg/cycle).

**Decisions:**
- `--speed-mm-s` (default 50, at most 200 in Simulation); the lead limit is 2 x the step (a fixed
  2 mm froze every 4 mm step).
- Shoulder singularity watch: margin = P05 horizontal radius - d4 (Rainbow IK
  `th1 = atan2(...) - acos(d4 / r)`). Below 50 mm: warning and slower (0.25x at 20 mm); below 20 mm:
  stop; start needs >= 50 mm. Thresholds chosen from 094517: at a 25 mm margin 0.67 deg/mm, at 8 mm
  1.1 deg/mm, at 1.7 mm 2.4 deg/mm.
- Per-cycle timing columns (leader read, plan, send call, work, real send interval) and a late check
  right before sending; the new target is applied only if that check passes.

**Late check gap (also in 13 and 08):** the late check measured "last send -> start of the next
cycle's check". After an overrun the loop waits a full period (08 rule), so the real send gap was
40-50 ms while `late` stayed 0. 21 had 6 / 6 / 27 such cycles (all while engaged, cause not found;
planning is 1.8 ms mean, not related to button read failures). 13 and 08 use the same check and have
the same blind spot; their logs' `late = 0` does not prove there were no 40 ms gaps. Not fixed in
13 / 08.

## 2026-10-12 — RB5 ready pose proposal for pick and place (not applied)

**Proposal A:** q = (180, -15, -90, -165, 90, 0) deg. Tool (flange -Y) points straight down.
Controller `calc_fk_tcp`: TCP (417.9, 110.7, 196.5) mm, (rx, ry, rz) = (90, 0, -90); DH model z
differs by 2 mm. Flange (377.9, 110.7, 381.6), elbow above the wrist.
- Margins: J5 90 deg from singular; shoulder P05 radius 393.8 mm -> margin 283.1 mm; elbow J3 90 deg
  from 0 / 180; J3 limit (+/-150 or +/-165 by model, manual 1.6) 60 deg left.
- Max joint motion per 1 mm of TCP move: x 0.168, y 0.152, z 0.141 deg (TCP and flange the same,
  orientation fixed). For comparison, the 21 start pose: x 0.318, y 0.140, z 0.440.
- J1 = 180 puts the tool on the base +x side; J1 only rotates the setup and does not change the
  margins. The physical +x direction is not checked yet (22).
- Alternative B (180, -15, -90, 15, -90, 0): same height, TCP x 559 mm, arm more stretched, shoulder
  margin 498.8 mm.
- Note: the current Simulation start pose has J3 = -160.6 deg, beyond +/-150 (so this RB5 has the
  +/-165 limit, or the value needs checking).

## 2026-10-12 — 19: --pose mode (all joints), shortest equivalent angles, RB5 joint limits

**Decision:** 19 gets `--pose J1..J6` (move_j of all joints, Simulation only) and `--speed` (default 10,
at most 30 deg/s; the controller scales it by the speed bar 0.1, so 30 -> 3 deg/s). For each joint the
target is the equivalent angle (target + k x 360) inside the joint limit closest to the current angle.
Per-joint change limit: 60 deg (--joint), 180 deg (--pose).

**Joint limits (evidence):** rb_cobot_docs manual 1.6 Robot Operating Range: J1, J2, J4, J5, J6
+/-360 deg, J3 "+/-165 / +/-150" (the page does not say which model has which). Appendix System
Specifications: "Joint Range +/-360 (Elbow: +/-165)", written for the E series. This robot is an
RB5-850 (N), so J3 uses the smaller +/-150 in 19. The 21 start pose had J3 = -160.6 deg, which hints
that this arm allows +/-165, but that is not confirmed. The controller's setup joint limits are not
read by any script.

**Ready pose A from the current Simulation pose (180, 1.1, -156.9, 140.8, -2.5, 43.9):** target
(180, -15, -90, 195, 90, 0): J4 195 instead of -165 (54 deg instead of 306 deg). Largest change J5
92.5 deg, about 31 s at 3 deg/s.

## 2026-10-12 — 21: cylinder range; range excess was being dropped (also in 13)

**Decision:** the box range (+/-150 mm per axis) is replaced by a cylinder piece about the base z axis:
r 300..650 mm, z = start z +/-150 mm, azimuth = start azimuth +/-90 deg (Simulation defaults; before
Real, the z lower bound is set from the table height and the gripper length). A target outside is
clamped in r, z and azimuth separately. The script refuses to start if the TCP radius is outside.

**Bug found and fixed:** the range clamp was applied to the new target only. While the target sat at
the boundary, desired kept moving out, desired - target grew, and the speed limit cut it and B dropped
it, so the range excess was dropped after all (fake test: the hand came back 4 mm instead of 82 mm
before the robot moved inward, and the final position was off). Now desired is clamped into the
range first, then the speed / joint limits and B apply. Fake test after the fix: inward motion starts
after 84 mm of hand return (82 mm + one step) and the final position equals the start. **13 has the
same structure** (range clamp after the speed limit), so its "range excess is not dropped" note is
wrong in effect. Not fixed in 13.

**Singularity margins over the range** (pose A orientation, tool down, DH model, 120 grid points,
IK walked from pose A): J5 stays at 90 deg; shoulder margin >= 151 mm (at r 300); |J3| <= 124 deg.
New: the **elbow singularity** (arm stretched, J3 -> 0): at r >= 550 J3 comes within 17-20 deg of
straight and the joint motion rises to 0.65-0.72 deg/mm (0.19-0.31 deg/mm at r 300-500). At z +150
the model IK did not converge for r 600-650 (out of reach or nearly stretched). Decision on r_max or
an elbow watch is pending.

## 2026-10-12 — 21: three singularities watched, r_max 500 mm, C mode (catch-up up to a lag)

**The three RB5 singularities and how 21 watches them** (all on the joint angles predicted by the DH
model for the next step, q + J^-1 dx; slow = warning and the speed limit falls linearly to 0.25x;
stop = the step is not sent and servo mode ends normally; start needs the slow margin):

| singularity | condition | measure | slow / stop | evidence |
|---|---|---|---|---|
| wrist | J4 and J6 axes line up | J5 distance from 0 / 180 deg | 20 / 10 deg | Rainbow IK divides by sin(th5); 18 rz test |
| shoulder | wrist point P05 on the cylinder of radius d4 about the J1 axis | P05 horizontal radius - d4 | 50 / 20 mm | Rainbow IK acos(d4 / r); 094517 end (14 deg/mm) |
| elbow | arm stretched (J3 -> 0; also folded, J3 -> 180) | J3 distance from 0 / 180 deg | 20 / 10 deg | cylinder grid: 0.65-0.72 deg/mm at r >= 550 |

The joint speed limit (1 deg/cycle predicted) stays as the last guard for anything not covered.

**r_max 500 mm** (was 650) for the Simulation cylinder: over r 300-500 the grid has <= 0.31 deg/mm and
the elbow >= 42 deg from straight in the ready pose. Fake test from pose A with the range off: elbow
slow from 19.3 deg (r 681), stop at 9.7 deg predicted (r 690).

**Bug fix recap:** desired is clamped into the range before the speed limit (entry above); 13 still has
the old order.

**C mode:** `--max-lag-mm L` (0..50, default 0 = B). The part cut by the speed / joint / singularity
limits is kept while the lag (clamped desired - target) is at most L; only the part beyond L is
dropped (leader start shifted). Release drops the lag. Range excess is never dropped (it is outside
the lag because desired is clamped first). Fake tests at 1 mm/cycle:

| test | B | C (30 mm) |
|---|---|---|
| hand 100 mm at 5 mm/cycle, then still | robot 20 mm, stops at once | robot 50 mm, moves 30 mm in 30 cycles (0.6 s) after the hand stops |
| slow hand 50 mm | 50 mm | 50 mm |
| release right after the fast move | stays at 20 mm | stays at 20 mm (lag dropped) |
| fast down 100 / slow up 100 (hand net 0) | robot net +80 mm | robot net +50 mm |

Correction to the C plan: C does not bound the hand-robot offset to 30 mm. The offset grows by what
is dropped (hand travel beyond the speed-limited travel + L); C only reduces it by L per fast move.

**Side effect seen in the fake tests:** the old 21 start pose (J3 = -160.6) is 19.4 deg from folded,
so the elbow watch slows it and the start check would refuse it. Use the ready pose A.

## 2026-10-12 — 21: default is C with a 30 mm lag

**Decision:** `--max-lag-mm` defaults to 30 (C). `--max-lag-mm 0` gives B.

**Reason (Simulation, 200 mm/s, user runs; hand speeds differ between runs, so not a controlled test):**

| run | mode | hand speed p50 | RB5 path / hand path | dropped / hand path | hand-robot offset at segment end (median / max) |
|---|---|---|---|---|---|
| 102624 | B | 246 mm/s | 44 % | 38 % | 83 / 233 mm |
| 102731 | C | 287 mm/s | 45 % | 26 % | 73 / 246 mm |
| 103206 | B | 93 mm/s | 57 % | 15 % | 11 / 126 mm |
| 103353 | C | 143 mm/s | 66 % | 8 % | 34 / 157 mm |

C drops less than B (38 -> 26 % at similar hand speed; 8 % for C even with a faster hand than the
second B run). The path ratio gain is small when the hand is above the speed limit most of the time,
because the 30 mm lag fills quickly. Robot motion while the hand was slower than 20 mm/s: C 11 mm,
B 14 mm (a 0.3 s hand stop was too rare to measure the after-motion directly). A controlled B / C
comparison was skipped by choice.

**Before Real:** C keeps moving up to the lag after the hand stops (fake test: 30 mm in 0.6 s at
50 mm/s). The lag length must be chosen again for the real robot.

**Also seen:** the range limit (r 300-500, z +/-150) was active in 28-56 % of engaged cycles, the
largest limiter; late cycles 0.6-5.2 %, all while engaged, after planning spikes up to 27 ms (cause
under test).

## 2026-10-12 — 21: cause of the late cycles (offline test)

**Test (no robot, no leader arm):** `plan_step` from pose A with a 5 mm step, 4 mm/cycle, C 30 mm,
1000-1500 calls.

| condition | p50 | p99 | max | > 12 ms |
|---|---|---|---|---|
| 20 ms cycle (sleep), numpy default threads | 7.3 ms | 16.8-18.3 ms | 21-26 ms | 53-71 / 1500 |
| 20 ms cycle (sleep), OPENBLAS/OMP/MKL threads = 1 | 7.3 ms | 16.0-16.1 ms | 21-23 ms | 60-64 / 1500 |
| no sleep (busy loop) | 2.05 ms | 3.3 ms | 5.5 ms | 0 / 1000 |

**Conclusion:** numpy (OpenBLAS) threads are not the cause. The same work takes 3.6x longer with
spikes when the loop sleeps between cycles. The PC runs the `powersave` CPU governor on an
i7-13700H (6 performance cores up to 4.8-5.0 GHz, 8 efficiency cores up to 3.7 GHz), so after each
sleep the core is likely clocked down or the thread lands on an efficiency core (not separated yet).
Also, `plan_step` computes the DH Jacobian at the same q four times (0.47 ms each).

**Options (not decided):** compute the Jacobian once per cycle (code only); pin the process to the
performance cores (`taskset`); switch the governor to `performance` while testing (PC setting, needs
sudo).

## 2026-10-12 — 21: one Jacobian per cycle; late-cycle cause is CPU power saving + hybrid cores

**Cause (offline tests, recorded in the entry above, now with core pinning):** the PC uses the
`powersave` governor on an i7-13700H (performance cores cpu0-11 up to 4.8-5.0 GHz, efficiency cores
cpu12-19 up to 3.7 GHz). The same planning work is several times slower right after the 20 ms sleep
than in a busy loop (clock-down after idle), and slower again when it runs on an efficiency core.
numpy thread count has no effect.

**Change:** `plan_step` computes the DH Jacobian once per cycle (it was computed 4 times at the same q).
Fake-test results are unchanged (only differences come from the new C default in tests that do not
set the mode).

**Offline planning time after the change (20 ms cycle with sleep, 1000 calls):**

| condition | p50 | p99 | max | > 12 ms |
|---|---|---|---|---|
| before (4 Jacobians), default cores | 7.3 ms | 16-18 ms | 21-26 ms | 53-71 / 1500 |
| one Jacobian, default cores | 2.4-3.6 ms | 7.9-9.0 ms | 8.9-11.5 ms | 0 |
| one Jacobian, `taskset -c 0-11` (performance cores) | 3.3-3.5 ms | 5.1-6.3 ms | 5.8-8.0 ms | 0 |
| one Jacobian, `taskset -c 12-19` (efficiency cores) | 5.8-6.0 ms | 7.8-9.7 ms | 8.5-13.6 ms | 0-1 |
| one Jacobian, busy loop (no sleep) | 0.85 ms | 1.7-2.8 ms | 2.4-4.0 ms | 0 |

With the leader read (p99 about 7 ms) the expected work p99 is about 16 ms (default cores) or about
13 ms (performance cores) against the 20 ms period. Governor change (`performance`): user's decision
after a Simulation run with these changes.

## 2026-10-12 — 21: --scale 0.5 Simulation result (104723, C 30 mm, 200 mm/s)

| | C runs at scale 1 (102731 / 103353) | scale 0.5 (104723) |
|---|---|---|
| speed limited | 74 % / 37 % | 14 % |
| joint speed limited | 35 % / 20 % | 9 % |
| range limited | 28 % / 56 % | 38 % (desired r < 300: 33 %, z outside: 18 %, r > 500: 1 %) |
| RB5 path / hand path | 45 % / 66 % | 39 % = 78 % of the scaled hand path (0.5 x hand) |
| dropped / hand path | 26 % / 8 % | 0 % |
| offset at segment end (vs scaled hand) | - | median 13 mm, all of it the range residual |

At scale 0.5 almost nothing is dropped; what is lost is the range limit, mostly the inner radius
(the hand pulled toward the base) and z.

## 2026-10-12 — Ready pose: wrist folded inward in A (user); proposal C with the wrist outside

**User report:** in A (J4 = 195) the wrist folds in toward the base, which makes picking hard; the
end effector should face outward.

**Proposal C (not applied):** q = (180, 4, -111, 17, -90, 0), the J5 = -90 branch (J2 + J3 + J4 = -90
keeps the tool down). Controller `calc_fk_tcp`: TCP (415.9, 110.7, 196.8), (rx, ry, rz) = (90, 0, 90),
the same place as A. The wrist point P05 is outside the TCP (radius 469 vs 430; in A 394 vs 432).
Margins: J5 90 deg, shoulder 358 mm, elbow 69 deg; 0.156 deg/mm (A: 0.168).
Over the cylinder (75 points, r 300-500): shoulder >= 182 mm, elbow >= 31.7 deg, <= 0.26 deg/mm, but
at r 300, z -150 J3 reaches -148.3 deg, near the +/-150 limit used in 19.
The tool is rotated 180 deg about its axis compared with A (rz +90 vs -90); J6 = 180 would keep A's
hand rotation but moves the TCP by the 40 mm z offset.

## 2026-10-12 — Ready pose C applied (Simulation); runs pinned to performance cores

**Decision:** the RB5 ready pose for the leader position mapping is C = (180, 4, -111, 17, -90, 0) deg,
J6 = 0 (tool rotated 180 deg about its axis compared with A). Move with
`taskset -c 0-11 python scripts/19_move_joint_sim.py --pose 180 4 -111 17 -90 0 --speed 30`.
J1 = 180 is for Simulation; for Real it is set again from the table direction.

**Reason:** the grasp is top-down (approach from above). The tool must point down, and the wrist must
not fold in toward the base as in A (J4 = 195, wrist point inside the TCP radius), which got in the
way of picking. C keeps the tool down with the wrist outside (wrist point radius 469 mm vs TCP 430 mm),
at the same TCP place as A, with more shoulder margin (358 vs 283 mm) and slightly less joint motion
per mm (0.156 vs 0.168 deg/mm).

**Known limit of C:** low and inner targets bend J3 toward its limit (+/-150 used): at z = start - 150
and r = 300 the margin is 1.7 deg (grid over the cylinder, worst azimuth; the margin does depend on
the azimuth, see the next entry). Joint limit watch
and the inner radius are decided next.

**Performance cores:** from the next run, scripts are started with `taskset -c 0-11` (offline: planning
p99 5-6 ms instead of 8-9 ms). The CPU governor stays `powersave` unless the user decides otherwise.

## 2026-10-12 — 21: joint limit watch, r_min 375 mm; pick positions r 400-500 mm recommended

**Joint limit watch:** margin = limit - |predicted joint| with J3 +/-150 (until the user checks +/-150
vs +/-165 on the teach pendant), other joints +/-360. Below 20 deg: warning and slower (0.25x at
10 deg); below 10 deg: stop; start needs >= 20 deg. Same scheme as the singularity watches.
**r_min 375 mm** (was 300) for the cylinder.

**Joint limit margin over the cylinder, pose C orientation (tool down, wrist outside), worst of
azimuth -90..+90 deg, DH model IK walked from C** [deg, the smallest is always J3]:

| z - start z \ r [mm] | 300 | 325 | 350 | 375 | 400 | 450 | 500 |
|---|---|---|---|---|---|---|---|
| +150 | 42.9 | 44.6 | 46.5 | 48.5 | 50.8 | 56.0 | 62.1 |
| +100 | 34.9 | 36.7 | 38.6 | 40.8 | 43.2 | 48.5 | 54.6 |
| +50 | 27.4 | 29.2 | 31.3 | 33.6 | 36.1 | 41.6 | 47.9 |
| 0 | 20.3 | 22.3 | 24.5 | 27.0 | 29.6 | 35.5 | 42.0 |
| -50 | 13.6 | 15.8 | 18.3 | 20.9 | 23.8 | 30.0 | 36.7 |
| -100 | 7.3 | 9.9 | 12.6 | 15.5 | 18.6 | 25.2 | 32.3 |
| -150 | 1.7 | 4.6 | 7.7 | 10.9 | 14.2 | 21.2 | 28.6 |

The margin depends on the azimuth (the tool orientation is fixed in the base frame, so it turns
relative to the arm): at r 375, z -150 it is 13.7 / 17.3 / 10.9 deg at azimuth -90 / 0 / +90 deg
(an earlier note said "independent of azimuth"; that was wrong, the table values are the worst case).

Other measures over r 400-500 in pose C (75-point grid): shoulder margin >= 281 mm, elbow >= 44 deg
from straight, <= 0.21 deg/mm.

**Recommendation: pick positions at r 400-500 mm.** In that band nothing stops over the whole z range;
at r >= 450 there is no joint limit slowdown either; at r 400 the slowdown starts below z - 100 mm.
r 375-400 is reachable but slows down for z <= -50 and may stop at z -150 (10.9 deg at azimuth +90).
If the teach pendant shows +/-165 for J3, every value above gains 15 deg.

**Fake tests:** from pose C with the range off, straight down: slowdown from margin 20.0 deg
(dz -279 mm), no stop. Default range, inward then down to r 375 / z -150 at azimuth 0: margin 17.4 deg,
38 slow cycles, matching the IK value 17.3. Old tests (old start pose, J3 -160.6, TCP r 367) give the
same results with the joint limit watch and r_min relaxed; with the real settings that pose is refused.

## 2026-10-12 — Object on the RB5 table (z 0): ready pose D, cylinder r 450-550 mm

**Decision:** objects are on the same table as the RB5 (TCP z 0 = base bottom). Ready pose
D = (182, -11.0, -110.1, 31.1, -90, -2) deg: controller `calc_fk_tcp` TCP (483.3, 129.0, 102.2) mm,
r 500.2, (rx, ry, rz) = (90, 0, 90): tool down, wrist outside, same tool rotation as C, 100 mm above
the object height. Margins at D: J3 limit 39.9 deg, elbow 70 deg, shoulder 428 mm, J5 90 deg.
Move: `taskset -c 0-11 python scripts/19_move_joint_sim.py --pose 182 -11.0 -110.1 31.1 -90 -2 --speed 30`.

**Cylinder:** r 450-550 mm (was 375-500), z = start z +/-150 (-50..252 mm from D), azimuth +/-90 deg.
J3 limit margin at the object height z 0 by r (worst azimuth, DH IK): r 450 18.5, 475 22.2, 500 26.1,
525 30.0, 550 34.1 deg; at z -50 and r 450 it is 16.4 deg (slow, no stop). With r_max 550 the
worst spot stays at the inner low corner; the outer side kept elbow >= 35 deg in the grid.

**Before Real (must do):** set the z lower bound again from the table height and the gripper length.
With D the current lower bound is z -50 mm, below the table surface (z 0); in Real that would drive the
tool into the table. The TCP offset itself (0, -187, -40) is still being checked by the lab.

## 2026-10-12 — 21 from ready pose D: depth reached; vertical-hand habit halves the backward drift

**Run 113617 (Simulation, scale 0.5, C 30 mm, 200 mm/s, taskset 0-11, user moved the hand vertically):**
- Deepest TCP z -48.0 mm (start 102.2, dz -150.2): the z lower bound, below the object height z 0.
  The first descent already reached z -19. The depth goal is met.
- At the deepest point only the range limit was active: z lower bound and r_min 450 (desired r < 450
  in 100 % of that segment). Joint limit margin >= 23.6 deg, elbow >= 53.6 deg, no singularity or
  speed limits.
- Hand |dx| / |dz| over the 10 descents of more than 30 mm: 0.45-0.9 (mostly about 0.5). The previous
  run (112319, no habit) was mostly >= 1 (up to 2.3). The hand still drifts toward the body, so the
  robot is pushed to r_min 450 during 6 of 10 descents; z keeps going down because r and z are clamped
  separately.
- Late cycles 0; work p99 8.8 ms, max 11.0 ms.

## 2026-10-12 — Leader handle point instead of the wrist center: small effect, not applied

**Offline analysis (logged leader joints, no robot, no code change):** handle point = Link_6R origin +
R6 (0, 0, -135) mm, taken from the URDF Link_6R inertial origin (0.00089, -0.00028, -0.13533) m (the
handle's center of mass; the URDF has no handle frame). Wrist FK matches the logged leader position
within 0.005 mm.

| run | descents (> 30 mm) | median hand \|dx\|/\|dz\|, wrist center | handle point |
|---|---|---|---|
| 112319 (no habit) | 20 | 0.70 | 0.71 |
| 113617 (vertical habit) | 10 | 0.59 | 0.45 |

The handle tilted 30-130 deg from straight down and turned by 15-55 deg during the descents, so the
handle point also picks up wrist rotation (its dz is 1.1-1.6x the wrist dz). Effect: none without
the habit, about 25 % with it. Decision: keep the wrist center; next step is orientation (yaw) mapping.

## 2026-10-12 — Yaw mapping: RB5 yaw change = leader J6 change (method B)

**Decision:** for top-down grasping only the rotation about the base z axis is mapped. RB5 yaw =
start + (leader J6 now - leader J6 at the clutch press), applied as rz in move_servo_l (rx, ry fixed,
the tool keeps pointing down; servo_l rotates about the TCP, 18). Limits: 90 deg/s (1.8 deg/cycle),
+/-90 deg around the start rz, lead 3.6 deg, C lag 15 deg (B with `--max-lag-mm 0`). The joint
predictions (singularities, joint limits, joint speed) include the yaw. `--no-yaw` keeps the start
orientation.

**Reason (offline, logged leader joints of 112319 and 113617, no robot):**

"Fake yaw" at the lowest point of each descent (> 30 mm, no twist intended):

| run | (A) twist about base z of R_now R_start^T | (B) leader J6 change |
|---|---|---|
| 112319 (20 descents) | median 6.7, max 28.0 deg | median 3.0, max 26.5 deg |
| 113617 (10 descents) | median 4.1, max 64.1 deg | median 0.4, max 10.6 deg |

|A - B| by handle tilt from straight down (all engaged samples, both runs):

| tilt | samples | median | p90 | max |
|---|---|---|---|---|
| 0-30 deg | 390 | 2.5 | 5.8 | 9.1 |
| 30-60 deg | 1244 | 2.5 | 10.5 | 28.2 |
| 60-90 deg | 1515 | 3.4 | 23.1 | 67.3 |
| 90-120 deg | 893 | 4.8 | 28.2 | 57.5 |
| > 120 deg | 134 | 9.9 | 23.2 | 45.1 |

The handle was tilted 30-120 deg in 86 % of the samples (held nearly horizontal). B is quieter in the
descents; A follows the true rotation about vertical but picks up wrist bending (e.g. -64 deg when J4
and J5 moved -26 / +59 deg). With the handle near vertical the two agree. Accepted trade-off of B: a
J6 twist with a tilted handle is a rotation about a non-vertical axis but still drives the RB5 yaw;
"twist the handle = turn the gripper" is the operating rule.

**Fake tests (DH fake robot following position and orientation, scale 0.5, ready pose D):** twist
+/-45 deg -> target and TCP yaw +/-45.00 deg, position change 0.0005 mm; descend 100 mm without twist
-> yaw 0.0000; pick mimic (down + 30 deg) -> yaw 30.00 at the bottom and after returning; fast twist
90 deg in 10 cycles -> C 10.8 then 25.8 (+15 lag), B 10.8; range +120 -> 90.00, back to 60.00;
release freezes yaw; `--no-yaw` 0; J6 crossing +/-180 -> +20.00. Note: the yaw step was 1.08 deg/cycle,
not 1.8: at D a yaw step is mostly J6 (0.92 deg per deg), so the joint speed limit (1 deg/cycle
predicted) caps the yaw at about 54 deg/s.

## 2026-10-12 — Yaw mapping Simulation result (115603, ready pose D, scale 0.5, C)

Segments classified from the data (no time marks given): twist only (|dJ6| > 15 deg, hand < 40 mm),
descend (hand dz < -30 mm, |dJ6| < 10 deg), combined.

- **Twist only (3 segments, handle tilt 12-17 deg):** RB5 yaw followed the leader J6 within the limits:
  +119.1 deg -> +90.0 (range limit), -149.1 -> -108.7, +32.1 -> +3.7 (back-and-forth twists faster than
  the yaw limit; the part beyond the 15 deg lag was dropped). TCP moved 1.6-7.6 mm, which is the
  hand's own motion x 0.5. The true twist about vertical (A) had the same sign as dJ6 (106 / -134 /
  33 deg), so +J6 is counterclockwise seen from above, the same as +rz on the RB5.
- **Descend without twist (6 segments):** yaw change 0.0-2.5 deg, max 3.1 deg during the segment
  (A would have given 7-14 deg). Method B works as intended here.
- **Combined (2 segments):** dJ6 -20.7 / +30.5 deg -> yaw -20.65 / +30.50 deg while moving 41-53 mm.
- Yaw tracking: |target - tcp| yaw p99 1.09 deg. Late cycles 0, work p99 9.0 ms.
- Speed: leader J6 p50 44, p90 75, max 92 deg/s during twisting; RB5 yaw max 48 deg/s because the
  joint speed limit (1 deg/cycle predicted for any joint) caps J6. Yaw speed limited 32 %, joint speed
  limited 37 % of engaged cycles; 110 deg of yaw dropped in total.

## 2026-10-12 — 21: joint speed limit per joint (Simulation)

**Decision:** the predicted joint motion limit is per joint: J1-J3 1 deg/cycle (50 deg/s), wrist J4-J6
2 deg/cycle (100 deg/s), Simulation only. The step (position and yaw together) is scaled by the
smallest factor over the joints that exceed their limit. **Real values are to be set later from the
RB5 specification** (the manual gives "Velocity Joint: Max 180 deg/s, Tool: 1 m/s" for the E series;
per-joint values for this RB5-850 (N) still have to be found).

**Reason:** in 115603 the RB5 yaw reached only 48 deg/s because a yaw step at ready pose D is mostly J6
(0.92 deg per deg) and the common 1 deg/cycle limit capped it. Fake test after the change: fast twist
-> 1.80 deg/cycle (the 90 deg/s yaw limit), C 18.0 then 33.0 deg (+15 lag), B 18.0. Old fake tests
unchanged.

## 2026-10-12 — Gripper signal from the leader trigger: rules

**Signal:** grip_cmd = clip(trigger / 1000, 0, 1) (Rainbow example 35 lines 1173-1176 use trigger / 1000
as the gripper target). Open / closed state with hysteresis: above 0.6 one state, below 0.4 the other,
in between the previous state is kept. Trigger = bits 16-31 of the tool's raw value at address 132
(rby1-sdk `dynamixel_bus.cpp:57`), read together with the clutch button.

**Measured range (user, script 12):** the trigger is continuous: 0 = fully closed, the value rises as it
is opened, 1000 = fully open. Logged data so far: 115603 had only 0 and 1000; earlier runs only 0.
The signal is named `grip_open` (0 closed .. 1 open) to keep the direction explicit; state open above
0.6, closed below 0.4.

**Implemented in 21 (logging only):** columns `trigger_raw`, `grip_open`, `grip_state_open`,
`tool_read_failed`; events on state changes; status line. Fake tests: hysteresis (650 -> open,
450 stays open, 350 -> closed), 4 failed reads keep the value with a warning at the 3rd, state changes
while the clutch is released and while engaged.

**The gripper works independently of the clutch:** the gripper can be closed or opened while the robot
holds its position.

**Failure rule - the two signals fail differently on purpose:**
- Gripper: on a failed tool read, keep the previous value. Reason: an object in the gripper must not be
  dropped because of a lost packet.
- Clutch: after 3 failed reads in a row, treat it as released. Reason: motion must stop when the input
  is not known.

## 2026-10-12 — Teleop cleanup: teleop.py + teleop/ modules + configs + tests

**Decision:** the experiment scripts stay as they are. The teleop is `teleop.py` with modules in
`teleop/` (settings, kinematics, safety, mapping, leader, rb5, preflight, loop, log), settings in
`configs/sim.yaml` (values tested with 21) and `configs/real.yaml` (conservative, TBD values as null:
z lower bound, expected TCP offset, ready pose), tests with `unittest` in `tests/` (no new packages).
Simulation is the default; `--real` needs a valid real.yaml and typing REAL. At start the teleop
checks the leader (latency, ping), the RB5 (mode, not moving, errors, TCP offset), moves to the ready
pose after showing the change and a typed "yes" (skipped when already there; Real: clear-area note and
a warning for wrist turns over 90 deg), then checks the start pose.

**Behavior equal to 21:** plan_step gives the same results as 21 on 600 random cases (including near
the wrist singularity, the J3 limit and the elbow), and a combined script through both loops gives the
same targets, yaw and flags.

**Changes from 21:** gripper signal is `grip_close = trigger / 1000` (pulled = closed, untouched = open;
example 35 maps trigger / 1000 to a gripper target whose open / closed end depends on the gripper and
its GRIPPER_DIRECTION flag, so the source does not settle it). Optional absolute z floor
(`range.z_min_abs_mm`). Settings are checked (unknown keys, ranges, stop < slow, t1 = period).

**udev rule:** `udev/99-rby1-leader-arm.rules` sets the latency timer on the usb-serial device and
matches vendor / product / serial (FT62AJW7) on its parent USB device (checked with `udevadm info -a`;
`udevadm verify` passes). Not installed yet (needs sudo).

**Next feature candidate:** left handle button switches the scale (precise 0.5 / fast 1.5).

## 2026-10-12 — Teleop: ready pose from the real RB5, scale 1.5, J3 +/-165, range around the new pose

**User requests:** run with plain `python teleop.py` (no taskset), the RB5 should go farther (scale
about 1.5), and use the ready pose the user set on the real RB5. J3 limit confirmed +/-165 on the
teach pendant.

**Changes:**
- The process pins itself to `runtime.cpu_cores` ([0, 11], the performance cores) from the settings.
- `mapping.scale` 1.5 in sim.yaml.
- Ready pose (178.969, 8.514, -138.775, 46.535, -87.239, 56.265) deg, read from jnt_ang in Simulation
  mode (the encoder of the real arm). Controller FK: TCP (356.7, 84.6, 64.6) mm, r 366.6, rpy
  (84.2, 3.7, 145.2): the tool is 6.9 deg from straight down.
- J3 limit +/-165 in both settings files. With +/-150 the new pose had only 11.2 deg margin (refused).
- Range (sim.yaml) around the new pose: r 350-600 mm, z start -100 / +150 mm (about -37 .. 213),
  azimuth +/-90 deg. The z range is now asymmetric (`z_below_mm`, `z_above_mm`). DH grid with the
  pose's orientation over r 330-600, z start -150 .. +150, azimuth +/-90: elbow >= 25 deg, shoulder
  >= 198 mm, J5 >= 83 deg, all reachable; J3 (+/-165) margin is lowest at the inner low corner
  (10.2 deg at r 330, z -87). At the ready pose: J5 87.2 deg, shoulder 252.8 mm, elbow 41.2 deg,
  J3 26.2 deg.
- The teleop now checks the ready pose with the start checks (controller FK) before moving there.
- Note: with pose D the forward reach was only 50 mm (r 500 -> r_max 550); that, more than the scale,
  limited "how far forward" the RB5 went.

**Tests:** fixed to the pose D era values (same as 21) in `tests/fixtures.py`, plus a test that the
ready pose in sim.yaml passes the start checks with sim.yaml.

**Open:** the trigger read 1000 with the trigger apparently untouched (earlier logs: 0). With
`grip_close = trigger / 1000` that means "closed" at rest. To be checked by the user (released /
pulled values) before the direction is changed.

## 2026-10-12 — Gripper direction fixed: grip_close = 1 - trigger / 1000

**Fact (user):** the trigger reads 0 when it is closed (pulled) and 1000 when fully open (released).
This matches the 1000 read with the trigger untouched (dry run with the real leader arm). Earlier logs
had 0 at rest; the reason is not known.

**Decision:** `grip_close = 1 - trigger / 1000`: pulled = closed (1), released = open (0), as the user
wants. Hysteresis and the failure rule are unchanged (closed above 0.6, open below 0.4, keep the
value on a failed read). Replaces `grip_close = trigger / 1000` from the cleanup step 2.

## 2026-10-12 — Sim: speed 400 mm/s and lag 50 mm; Real: provisional values for a first test

**Why the RB5 stopped near x 500 (runs 130330 scale 1.5, 130757 scale 2.0):** r 600 (x about 590) was
reached only at the z upper bound (z 215); with z <= 100 the max x was 476 / 491 mm. The hand was
faster than 200 mm/s / scale in 50 % / 73 % of the engaged cycles (p50 134-172 mm/s), so C dropped
4.0 / 1.8 m of hand motion per run; one press moved the RB5 0.13-0.97 x the hand instead of 1.5-2.0.
Reaching forward also raised the leader wrist (desired z up to 370), clipped at the z upper bound.

**Sim change:** `mapping.speed_mm_s` 200 -> 400 (schema maximum raised to 400; Real stays capped at
50), `mapping.max_lag_mm` 30 -> 50 (schema maximum). The per-joint speed limit stays as the guard.
The user's own edit `mapping.scale: 2.0` is kept.

**Real (user wants to test now; table height, gripper length and TCP can come later):**
- `ready_pose.joints_deg` = the pose the user set on the real RB5, read again from jnt_ang:
  (178.969, 8.514, -138.775, 46.535, -87.239, 56.265); TCP (356.7, 84.6, 64.6), r 366.6.
- `range.z_min_abs_mm` 30.0 and `tcp.expected_offset_mm` (0, -187, -40) are PROVISIONAL placeholders:
  the floor keeps the TCP at z >= 30 mm, the TCP check refuses the start if the controller's TCP
  changes. Both are to be replaced (table height + gripper length + margin; the lab's TCP).
- Range r 340-420, z start -40 / +50 (floor 30 -> 30 .. 112.6 mm), azimuth +/-30; speeds unchanged
  (20 mm/s, yaw 15 deg/s, joints 0.2 deg/cycle = 10 deg/s, B). DH grid over the range (27 points):
  J5 >= 84.3 deg, shoulder >= 215.7 mm, elbow >= 34.0 deg, joint limit >= 19.0 deg (inner low corner,
  slow but no stop), <= 0.257 deg/mm (5.1 deg/s at 20 mm/s).
- Not done and not changed by the code: collision detection is Off and tool mass is 0 kg on the
  controller; move_servo_l has not been used on the real robot before (18 was Simulation).

## 2026-10-12 — Sim: r_max 600 -> 800 mm (reach x 750)

**Why x stayed below 600 at scale 2.0:** the cylinder r_max 600; the TCP azimuth is 13.4 deg, so
x = r cos(13.4) <= 584. x 750 needs r 771 at that azimuth.

**Check (DH model, the ready pose orientation kept, worst of azimuth -30 / 0 / +30, z -37 .. 213):**
every point up to r 850 is reachable. Elbow distance from straight: r 600 >= 70, r 700 >= 70, r 750
>= 58, r 800 >= 43, r 850 >= 19.5 deg (only at the top z 213: slowdown). Joint limit margins grow with
r (>= 55 deg). Decision: r_max 800 (no slowdown anywhere in the range).

## 2026-10-12 — Real: range as wide as Simulation (user request)

**Decision:** real.yaml range r 350-800 mm, z start -100 / +150 mm, azimuth +/-90 deg (same as
sim.yaml), keeping the PROVISIONAL floor z >= 30 mm. Was r 340-420, z -40 / +50, azimuth +/-30.

**Check (DH model, real ready pose orientation, 36 points over r 350-800, z 30 .. 214.6, azimuth
-90 / 0 / +90):** all reachable; J5 >= 83.4 deg, shoulder >= 218.1 mm, elbow >= 35.5 deg, joint limit
>= 20.5 deg (inner low corner r 350, z 30: just above the slow threshold); <= 0.393 deg/mm, i.e.
15.7 deg/s at 40 mm/s (joint limit 20 deg/s).

**Risk noted:** collision detection is still Off and the TCP / floor values are provisional; the wider
range lets the real arm sweep +/-90 deg of azimuth and reach r 800.

## 2026-10-12 — Real ready-pose move 3 deg/s

**Decision:** real.yaml `ready_pose.move_speed_deg_s` 10 -> 30 and the Real cap (REAL_MAX) 10 -> 30, so the
move_j to the ready pose runs at 30 x speed bar 0.1 = 3 deg/s (same as Simulation; was 1 deg/s, too
slow to wait for). The speed bar stays 0.1 (CLAUDE.md). The change display, the typed "yes" and the
wrist-turn warning before the move are unchanged.

## 2026-10-12 — Real runs 132310 / 133219; faster Real settings; measured joints logged

**Runs (real robot, scale 1.0, 40 mm/s, B):** 29 s and 122 s, no late cycles (send interval max 26 ms,
work p99 9.2 ms), no singularity or joint limit slowdown (J5 >= 84, shoulder >= 232 mm, elbow >= 41,
J3 >= 26 deg). The wide-range run reached x 330-661, r 350-663, z 65-215, yaw -27..+16 deg. The robot
target speed sat at 40 mm/s (speed limited 83-88 %, joint speed limited 27-34 %) while the hand moved
at p50 126-184 mm/s: the RB5 moved 16 % of the hand path and B dropped 1.9 / 8.7 m. Tool read failures
2-3 %. The trigger read 0 for the whole of both runs (gripper "closed" 100 %), unlike the 837-1000 read
at rest before: to be checked.

**Decision (user):** real.yaml `speed_mm_s` 50 (Real max), `joint_speed_deg_per_cycle` 0.5 (Real max,
25 deg/s; 50 mm/s x 0.393 deg/mm = 20 deg/s), `max_lag_mm` 20 (C: up to 20 mm, 0.4 s, after the hand
stops; yaw stays B since yaw.max_lag_deg is 0).

**Logging:** the state reader keeps the measured joints (jnt_ang); the CSV adds `jnt_ang_0..5`, their
DH-model TCP `meas_tcp_x/y/z` (absolute about 2 mm off the controller FK) and `target_minus_meas_mm`.
The logged tcp_ref / jnt_ref are the controller's reference (command) values; jnt_ang shows how the
real arm actually followed. In Simulation jnt_ang stays at the real arm's pose.

## 2026-10-12 — Real run 133743 (50 mm/s, C 20 mm) and z upper bound +200

**Run 133743 (66 s):** first run with the measured joints. jnt_ref - jnt_ang <= 0.145 deg (p50 0.03);
the measured TCP follows the target about 80 ms late (4 cycles; error 3.9 mm mean at 0 delay, 2.0 mm
when shifted, 2.0 mm while holding = the DH model offset). Measured joint speeds J1-J4 10-12 deg/s,
J6 24 deg/s. RB5 path 32 % of the hand path (16 % before), dropped 2.6 m, speed limited 91 %, joint
speed limited 10 %, range limited 52 % - all of it the z upper bound (start + 150 = 215 mm; reaching
forward raises the leader wrist). No late cycles, no singularity / joint limit slowdown. Tool read
failures 5.2 % (higher than before). The tool raw value was only 256 / 0: the trigger bits stayed 0.

**Decision:** real.yaml `range.z_above_mm` 150 -> 200 (z up to 264.6). DH grid with the real ready
pose orientation, r 350-800, azimuth +/-90: at +200 every point is reachable, elbow >= 25 deg (at r
800), J5 >= 84 deg, shoulder >= 218 mm, joint limit >= 48 deg; at +250 r 800 is out of reach.

## 2026-10-12 — Sim z upper bound +200 (same as Real)

sim.yaml `range.z_above_mm` 150 -> 200, same as real.yaml (user). Same ready pose and orientation as
Real, so the Real grid applies: reachable over r 350-800 at +200, elbow >= 25 deg at r 800.

## 2026-10-12 — Real run 134902 and TCP speed 100 mm/s

**Run 134902 (206 s, z top +200):** reached x 253-680, r 350-725, z 63-265, yaw -32..+22 deg. Range
limited 29 % (z top 19 %, r < 350 9 %; was 52 %). Speed limited 94 %, RB5 path 31 % of the hand path
(hand p50 120 mm/s). Measured tracking as before: jnt_ref - jnt_ang <= 0.139 deg, measured TCP about
80 ms behind the target. No late cycles, no singularity / joint limit slowdown. Tool read failures
0.8 %. Trigger bits 0 for the whole run again (third Real run).

**Decision (user):** Real TCP speed cap (REAL_MAX) 50 -> 100 mm/s and real.yaml `speed_mm_s` 100. The
joint speed limit stays 0.5 deg/cycle (25 deg/s): near the ready pose (about 0.22 deg/mm) 100 mm/s
fits, at the far / low corners (up to 0.393 deg/mm) the joint limit caps the TCP at about 64 mm/s.
With the 80 ms delay the real arm will trail the target by about 8 mm at 100 mm/s.

## 2026-10-12 — Real run 135230 (100 mm/s) and ready move 5 deg/s

**Run 135230 (68 s):** RB5 target at 100 mm/s in 63 % of the engaged cycles (60-95 mm/s in 21 %);
RB5 path 50 % of the hand path (31 % at 50 mm/s), dropped 1.9 m; speed limited 89 %, joint speed
limited 32 % (the 25 deg/s joint limit at far poses), range limited 24 % (all z top +200). Reached x
357-705, r 367-710, z 65-265. Measured: jnt_ref - jnt_ang <= 0.149 deg; measured TCP about 80 ms
behind (6.9 mm p50 while moving at 100 mm/s, 2.1 mm after shifting by 80 ms). Measured joint speeds up
to 23.4 deg/s. No late cycles, no singularity / joint limit slowdown; tool read failures 0.9 %. The
measured TCP speed shows peaks up to 171 mm/s (p99 125), probably from sampling (jnt_ang read every
20 ms from a 10 ms data stream), not checked. Trigger bits 0 again (4th Real run).

**Decision (user):** ready-pose move_j speed argument 30 -> 50 (5 deg/s with speed bar 0.1); schema
maximum 30 -> 60 and Real cap 30 -> 50. Speed bar unchanged.

## 2026-10-12 — Real joint speed limit 0.8 deg/cycle (40 deg/s)

**Decision (user):** real.yaml `joint_speed_deg_per_cycle` 0.5 -> 0.8 on all joints and the Real cap
0.5 -> 0.8. In 135230 the joint speed limit was active in 32 % of the engaged cycles (measured joints
up to 23.4 deg/s against 25). With 0.8, 100 mm/s fits everywhere in the range (worst 0.393 deg/mm x
100 = 39.3 deg/s). The RB5 specification for per-joint speeds is still to be found (manual: joint max
180 deg/s for the E series).

## 2026-10-12 — z upper bound +400 with an outer radius that falls with height (Sim and Real)

**Decision (user, option B):** both settings files: `range.z_above_mm` 200 -> 400 (about 463 mm) and a new
`range.r_max_by_z` table [[200, 800], [250, 775], [300, 725], [350, 700], [400, 650]] (start z + dz,
r_max). The cylinder is clamped in z first, then in r at that height; between table points r_max is
linear, below the first point it is the first value, above the last the last value. The low part of the
range is unchanged (r up to 800 below +200).

**Reason:** at +250 the arm could not reach r 800. Largest radius with every watch above its slow
threshold (DH grid with the ready pose orientation, azimuth -90 / 0 / +90): +200 800, +250 775, +300 725,
+350 700, +400 650, +450 600 mm. Option A (one smaller r_max for all heights) would have given up
forward reach at low height.

**Check of the sloped edge (9 heights from +200 to +400 incl. the interpolated ones, 3 azimuths):** all
reachable; J5 >= 83.4 deg, shoulder >= 518 mm, elbow >= 23.3 deg (at +250, r 775), joint limit >= 84 deg.
Tests: range clamp with the table, settings checks of the table (heights increasing, r within r_min..r_max).

**Real:** the real arm can now go up to about 463 mm; the space above has to be clear.

## 2026-10-12 — Leader deadband (hysteresis) against the wrist shake

**Observation (Real run 141108, user felt a slight motor shake):** the yaw target reversed direction in
18 % of its steps (position 3-6 %), and 79 of 81 reversals were exactly one leader encoder tick
(0.087-0.088 deg = 360 / 4096): the leader J6 flickered between two counts and the RB5 J6 followed.
Measured RB5 J6 had the largest fast wobble (p99 0.38 deg vs 0.11-0.19 for J1-J4). The log is at 50 Hz,
so vibration above 25 Hz cannot be seen in it.

**Decision (user):** a hysteresis on the leader values before the mapping (`LeaderFilter`): a new value is
taken only when it moved at least the deadband from the last taken value. `yaw.leader_deadband_deg`
0.15 (one tick ignored, two ticks pass), `mapping.leader_deadband_mm` 0.3. Both settings files. The CSV
adds `leader_used_x/y/z_mm` and `leader_j6_used_deg` (what the mapping used).

**Replay on 141108:** leader J6 reversals 81 -> 1; position reversals 71 / 197 / 127 -> 59 / 182 / 118
(x / y / z: mostly real hand tremor above 0.3 mm). Largest difference raw - used: 0.088 deg, 0.30 mm.
If the shake is still felt, next options: servo alpha < 1 (controller smoothing, adds delay) or a
low-pass on the position.

## 2026-10-14 — Leader arm URDF committed under assets/

**Problem:** `teleop.py` and `tests/test_core.py` read the leader URDF from `refs/rby1-sdk/`, which is
gitignored, so the teleop and the tests did not run from a fresh clone.

**Decision:** an unmodified copy at `assets/leader_arm/model.urdf` (rby1-sdk v0.10.0, Apache 2.0) with
`LICENSE`, `NOTICE` and a README (source, version, SHA-256). Only the URDF; the meshes are not used.
Scripts 15, 20 and 21 keep their `refs/` paths (kept as they were run).

**Rejected:** a git submodule of rby1-sdk (whole SDK for one 12 kB file); installing the rby1-sdk
Python package and reading its model (extra dependency, and the path inside the package is not
confirmed).

## 2026-10-14 — environment.yml for the rb5 env

**Decision:** `environment.yml` lists Python 3.10 and only the packages the code imports, pinned to
the versions in the lab PC's `rb5` env (rbpodo 0.16.14, dynamixel-sdk 4.1.0, numpy 2.2.6, PyYAML
6.0.3, matplotlib 3.10.9, evdev 1.9.3), all through pip. Checked: a new env built from the file
imports them all and runs the 43 tests (OK).

**Rejected:** `pip freeze` / `conda env export` of the whole env (it also lists the system ROS 2
packages and Jupyter, which the code does not use); unpinned versions (a later numpy or rbpodo could
change behavior without notice). rby1_sdk (scripts 14, 15, 20, env `rby1`) is only noted, not added.

## 2026-10-14 — Teleop logs in data/teleop/

**Decision:** `logging.dir: data/teleop` in both settings files; the 34 earlier teleop files
(`teleop_sim_*`, `teleop_real_*`, with their settings copies) were moved there. Script outputs stay in
`data/` (named by script number). `data/` is not committed. Demonstration data will get its own folder
once its format (e.g. LeRobot) is chosen.

## 2026-10-14 — Teleop entry point moved into the package: python -m teleop

**Problem:** `teleop.py` next to the `teleop/` package had the same name (confusing to read; Python
picks the package on import).

**Decision (user):** `teleop.py` -> `teleop/__main__.py` (git mv, history kept), relative imports,
`ROOT` is the repository root. Run from the repository root: `python -m teleop` (`--real`,
`--config` unchanged). README, teleop_guide and the package docstring updated; earlier entries in
this file still say `teleop.py`.

**Rejected:** renaming to `run_teleop.py` (still a second top-level name for the same thing).

## 2026-10-14 — One robot file: configs/robot.yaml

**Problem:** the robot IP was in `config.py`, `configs/sim.yaml` and `configs/real.yaml`.

**Decision (user):** `configs/robot.yaml` (ip, speed_bar) is the only place. `load_settings` adds it as
the `robot` section and checks it with the same schema (speed bar <= 0.1); a settings file that still
has a `robot` section is refused. `config.py` reads the same file and keeps `ROBOT_IP` and `SPEED`, so
the 14 scripts that import it are unchanged. Sim and Real now share one speed bar (both were 0.1).
Scripts 01 and 02 still have the IP written in them (kept as run).

**Rejected:** deleting `config.py` and editing the 14 scripts (they are kept as run); `config.py`
reading `sim.yaml` (the IP would still be in two settings files).

## 2026-10-14 — config.py moved to scripts/

Only the scripts use it (`import config`). In `scripts/` Python finds it without any change to the
scripts, because the folder of the script being run is on the import path. The repository root
keeps only folders, README and environment.yml. `scripts/README.md` now lists, per script, how to
run it, its options and what it moves.

## 2026-10-14 — Scripts 15, 20, 21 also use the assets/ URDF

Changes the earlier "scripts keep their refs/ paths" (user request): all code now reads
`assets/leader_arm/model.urdf`; `refs/` is reference only. The example 35 diff was regenerated.

## 2026-10-14 — udev rule moved to assets/udev/

`udev/99-rby1-leader-arm.rules` -> `assets/udev/` (user request: no single-file folder at the root).
Still not installed; the install commands in `docs/teleop_guide.md` use the new path.

## 2026-10-16 — real-1: trial protocol for faster Real teleop

**Problem (user):** Real teleop is too slow for demonstration collection; the operator drags the robot
with many clutch presses.

**Decision (user):** measure first, then change one setting at a time (`docs/real_trial_protocol.md`):
a fixed reach task P -> Q (gripper not connected), **3 practice trials before every setting (baseline
included) then 5 measured trials**, manual results in `data/teleop/trials.csv`, measures from the logs
with `scripts/23_trial_summary.py`. Baseline = the current real.yaml (scale 1.0, 100 mm/s, C 20 mm),
which already contains the scale 0.5 -> 1.0 change of the original request. Pick and place are found
from the measured TCP near P / Q, not from the trigger, because the trigger read 0 for whole runs
(20261014_035701).

**Evidence for the plan (5 Real runs 2026-10-14, free motion):** TCP speed limit active in 72-91 % of
engaged cycles, follow 30-79 %, joint speed limit 0-10 %, yaw limit 0 %. Plan: TCP 150, then joints 1.2
deg/cycle if needed, then 200, then C 40 mm; every raise of a Real limit is approved separately.

**Rejected:** comparing against scale 0.5 / B again (extra Real runs; the user chose the current
settings as the baseline); trigger as the pick / place marker (unreliable).

## 2026-10-16 — Real speed limits to option B: TCP 200 mm/s, joints 1.2 deg/cycle (60 deg/s)

**Request (user):** the robot should keep up with the hand instead of being dragged with the clutch,
without the coordinate-based trial protocol first.

**Hand speed (5 Real runs 2026-10-14, about 160 s with the clutch engaged, scale 1.0;
`leader_used_*` x scale per cycle):**

| | median | p90 | p95 | p99 | max |
|---|---|---|---|---|---|
| hand speed x scale [mm/s] | 121 | 278 | 360 | 515 | 689 |
| joint speed needed to follow [deg/s] (estimate) | 30 | 78 | 103 | 185 | 511 |
| leader J6 [deg/s] | 0 | 18 | 26 | 44 | 89 |

Share of the clutch time with the hand faster than 100 / 150 / 200 / 300 mm/s: 63 / 36 / 22 / 8 %.
Joint estimate = predicted joint motion per mm of each step (`max_dq_pred_deg` / target step, steps
> 0.3 mm) x hand speed; share above 40 / 60 / 90 deg/s: 35 / 18 / 7 %.

**Options shown:** A 300 mm/s, 80 deg/s, yaw 45 deg/s (about 92 % of the hand time); B 200 mm/s,
60 deg/s, yaw 30 (about 78 %); C 500 mm/s, 180 deg/s, yaw 90 (about 99 %, at the manual's joint
maximum).

**Decision (user): B.** `REAL_MAX` mapping.speed_mm_s 100 -> 200, `REAL_MAX_JOINT_SPEED_DEG_PER_CYCLE`
0.8 -> 1.2, and the same values in real.yaml. Yaw 30 deg/s, scale 1.0, C 20 mm, range and watches
unchanged. A only after B has been checked on the real robot.

**Reasons for B over A:** collision detection is Off and the tool mass is 0 kg on the controller, so
nothing stops the arm on contact; the kinetic energy of the moving arm grows with the square of the
speed: 200 mm/s is 4x the energy at 100 mm/s, 300 mm/s would be 9x. B removes most of the dragging
(hand faster than the cap 22 % of the time instead of 63 %) with the smaller step.

**Condition:** collision detection ON before every Real run (user turns it on with the lab; required
item at the top of `docs/real_first_run_checklist.md`). First run at the new speeds: slow, small
motions near the ready pose first.

## 2026-10-16 — Low-pass filter on the leader wrist center (tau 30 ms) against the shake at 200 mm/s

**Observation (Real run 20261016_021514, option B, user felt a shake):** compared with the 100 mm/s
runs, the target acceleration jitter rose from a median of 0.13-0.22 to 1.23 m/s^2 and the measured
joint acceleration p99 from 482-638 to 823 deg/s^2 (J2-J4 fast wobble p99 0.11-0.15 deg). The hand
itself was not jerkier (acceleration p99 3.7-4.4 m/s^2 in all runs). The jitter was larger when the
speed limit was **not** active (1.66 vs 0.41 m/s^2): the target then follows the hand step by step,
and the hand step per cycle jumps (e.g. 3.3, 4.0, 3.6 mm) because one leader encoder tick moves the
wrist center by about 0.5 mm (J0, J1) / 0.25 mm (J2, J3). At 100 mm/s the cap (2 mm per cycle) hid it.
The log is at 50 Hz, so vibration above 25 Hz cannot be seen.

**Decision (user, item 1 of 4):** a first-order low-pass on the wrist center after the deadband,
`mapping.leader_filter_tau_s` 0.03 s in sim.yaml and real.yaml (0 = off). J6 / yaw is not filtered.
Replay of 021514 through `LeaderFilter`: hand acceleration jitter median 1.74 -> 0.65 m/s^2, p99 4.4 ->
2.1; delay about 30 ms (largest filtered - raw distance 19 mm at the fastest hand motion). Offline
alternatives: 20 ms -> 0.77, 60 ms -> 0.47, 100 ms -> 0.38 m/s^2.

**Rejected / later:** servo alpha < 1 (controller filter; 06: alpha 0.5 added 19 ms and the settling
took 213 ms instead of 81 ms); an acceleration limit on the target (does not remove jitter below the
limit). Next items, one at a time after a Real check of this one: option A speeds (300 mm/s, 80 deg/s,
yaw 45 deg/s), C lag 20 -> 50 mm (275 mm of a forward reach was dropped in 021514), yaw range +/-45 ->
+/-90 (yaw at the range end 43 % of the clutch time).

## 2026-10-16 — Ready pose moved to r 500, z 200 (Real and Sim); range kept in absolute terms

**Problem (user, Real run 20261016_022416):** going straight down to place was hard. The hand asked for
z down to -130 mm, but the descents ran into r_min (350 mm): the leader wrist center moves back by
0.5-0.9 x the descent (it arcs about the shoulder), and the ready pose was at r 367, only 17 mm from
r_min (e.g. press 22: hand dz -394, dx -264, robot r 692 -> 350). The z floor (30 mm) was not the
cause for placing: the place height is 150 mm above the table (base on the table, so z 150).

**Why the ready pose was at r 367:** it is the pose the user set by hand on the real RB5 on
2026-10-12, read from jnt_ang and used as is; r_min was lowered from 450 to 350 so that it fitted
the range. The 17 mm gap to r_min was not pointed out then.

**Decision (user):** ready pose (182.920, -0.156, -107.813, 24.418, -87.677, 52.334) deg in sim.yaml
and real.yaml: DH TCP (486.5, 115.5, 200.0), r 500, azimuth 13.4 deg, same tool orientation as the
2026-10-12 pose (rpy 84.2, 3.7, 145.2). Found by damped least-squares IK marched from the old pose
(same arm configuration). The range is relative to the start z, so it was re-expressed to stay the
same in absolute terms (start z about 202 with the controller's +2 mm): z_above 400 -> 263 (top about
465), r_max_by_z heights -137 mm, z_below 100 -> 172 in Real (floor 30 = z_min_abs) and -> 237 in Sim
(floor about -35 as before).

**Margins (DH, Real thresholds slow 20 deg / 50 mm):** at the pose J5 87.7 deg, shoulder 386 mm, elbow
72.2 deg, joint limit 57.2 deg (J3; was 26.2); vertical line at r 500 from z 30 to 400: at least 87.7 /
386 / 58.0 / 43.0; place height z 150 over r 350-800, azimuth +/-60: at least 83.2 / 219 / 47.7 / 32.7.
Room to r_min 150 mm (was 17). Largest joint change from the old pose 31 deg (J3), about 6 s at 5 deg/s.

**Candidates computed:** r 450 / 475 / 500 x z 150 / 200 / 250, all with margins >= 1.6x the slow
thresholds; r 500 has the most J3 margin and room to r_min, z 200 is 50 mm above the place height.

**Next if descending is still awkward:** a "vertical mode" switched by the left handle button
(plan to be proposed separately). Cylindrical mapping for J1: on hold.

## 2026-10-16 — Cylindrical mapping trial (Sim and Real): a hand swing about the leader base center turns J1

**Problem (user):** turning the RB5 base (J1) is hard. Run 022416: up to 45 deg of J1 per clutch press
for about 240 mm of sideways hand motion, 99 deg in 30 presses; with the cartesian mapping 1 deg of J1
needs 7 mm of sideways motion at r 400 (10.5 mm at r 600).

**Decision (user, trial, to be reverted if it feels worse):** `mapping.mode: cylindrical` in sim.yaml
and real.yaml. Each cycle the leader motion is split about a vertical pivot axis (leader frame) into an
angle, a radius and a height change; the RB5 target turns about the base z axis by the same angle
(1:1), radius and height change x scale. Angle = sideways motion / max(hand radius,
`min_pivot_radius_mm` 150), so the gain stays bounded near the axis. The state is integrated per cycle;
B / C dropped parts are taken off the desired position (same meaning as the cartesian reference shift);
a late cycle that is not applied keeps its motion for the next cycle. Limits, range, watches, yaw and
servo are unchanged. Revert: `mode: cartesian` (identical to the mapping before; the old tests run in
cartesian).

**Pivot:** `pivot_xy_mm` [0, 0] = leader base center. From the 10-14 / 10-16 Real logs the hand was
205 mm or more from it in 95 % of the clutch time (sideways gain r_RB5 / r_hand median 1.6x, p95 2.4x);
about the right shoulder (0, -150) it was 87 mm (gain 2.3x median, 5.1x p95), too jumpy.

**Replay of 022416 (mapping only, no limits):** J1 swing per press median 7.3 -> 25.1 deg, max 36 -> 79
deg, total 411 -> 815 deg (2.0x).

**Expected side effects:** sideways motions are larger, so the speed cap (200 mm/s) is hit more often;
a straight sideways hand motion becomes an arc. Not addressed: the descent drift (the hand moves back
0.5-0.9 x the descent); about the base center that is a radius decrease and is mapped as one. Ready
pose r 500 removed the r_min stops (025735: 0 cycles at r_min), but the user still finds descending
awkward: next candidate is the "vertical mode" on the left handle button.

## 2026-10-16 — Cylindrical mapping reverted to cartesian after one Real run

**Run 031043 (cylindrical, Real):** J1 swing per clutch press median 9.1 -> 21.5 deg (vs 025735), but
the sideways speed the hand asked for rose to 127 mm/s median / 237 mm/s p90, the speed cap was hit in
65 % of the clutch time (29 %), joint speed limit 11 % (3 %), 2627 mm of hand motion dropped (115),
17 clutch presses (5). User: "J1 is easier, but it does not go where I want".

**Decision (user):** `mapping.mode: cartesian` in both files (the code stays, `cylindrical` turns it on
again). Next: make descents for pick and place easier, and try leader J2 (shoulder yaw) -> RB5 J1 with
the RB5 J6 no longer following the leader J6.

## 2026-10-16 — Vertical mode on the left handle button (x, y fixed, only z)

**Problem (user):** going straight down to pick / place is hard, also after the ready pose moved to
r 500. Analysis of 21 descents >= 60 mm (021514, 022416, 025735): hand -111 mm in z and -103 mm in x
(median); the leader elbow (J3) changed 51 deg, shoulder pitch / roll / yaw only 3 / 1 / 5 deg; the
elbow dominated in 90 % of the descents. At the typical pose +10 deg of elbow moves the wrist center
25 mm down and 18 mm back, so the drift is the arm's geometry, not a habit or a setting.

**Decision (user):** `mapping.vertical_button: true` (Sim and Real). While the **left** handle button is
held, the desired x and y stay at the target's x and y and only z follows the hand (x, y fully fixed,
user's choice); yaw still follows. Every switch (button or clutch) re-anchors the references, so the
RB5 does not jump. The left button uses the same debounce as the clutch (2 reads to press, 3 failed
reads = released, a button held at start must be released once). The left tool (ID 0x81) is read once
per cycle (read only; pinged at start only when the mode is on); the leader read was 4.9 ms p50 / 5.5
ms p99 before. CSV: `left_button_raw`, `vertical_mode`.

**Rejected:** "elbow motion -> z only" (reaching forward also extends the elbow, so forward motion
would break); slow x, y instead of fixed (user chose fixed: simpler to predict).

## 2026-10-16 — Vertical mode z x 2 (vertical_z_scale)

**Run 032657 (vertical mode, Real):** in the vertical mode x and y stayed fixed (0.000 mm change) and
the 17 switches caused no jump (0.0 mm); left button read failures 1 of 11869; leader read p50 / p99
4.9 / 5.5 -> 5.9 / 8.5 ms, work p99 10.4 ms, no late cycle; target jitter median 0.29 m/s^2. The user
found descending easier. 57 clutch presses (half shorter than 1 s), 31 of them with the vertical mode;
some vertical segments show the hand going up overall while the robot went down (re-gripping to go
further). The user said the many presses were mostly for turning J1, which was fine when moving slowly.

**Decision (user):** `mapping.vertical_z_scale` 2.0 (Sim and Real): in the vertical mode z follows the
hand x scale x 2, so a 170 mm descent needs 85 mm of hand motion. The B / C reference shift uses the
same gain. Normal mode unchanged.

## 2026-10-16 — TCP floor 30 -> -50 mm (nothing mounted on the flange)

**Request (user):** go lower for pick and place (run 032657: 8 of 9 s of range limit were the 30 mm
floor, all in the vertical mode).

**Finding:** the TCP offset (0, -187, -40) points 187 mm *down* from the flange center in the ready
orientation (the flange faces sideways, flange y is up), so at TCP z 30 the flange center is at about
220 mm. The 30 mm floor protected a tool tip at the TCP, but nothing is mounted (user, 2026-10-16).

**Decision (user):** `range.z_min_abs_mm` -50 in Real (flange center then about 140 mm above the table)
and `z_below_mm` 252 in Real and Sim (start z about 202 -> floor -50 in both). DH check at TCP z -50 over
r 350-800, azimuth +/-90 (ready orientation): all reachable; J5 >= 83 deg, shoulder >= 218 mm, elbow >=
29.5 deg, joint limit >= 14.5 deg at the inner low corner (r 350, azimuth -75): below the 20 deg slow
threshold there, so the teleop slows down in that corner (stop at 10).

**Must be raised again when a tool is mounted** (tool tip height + margin); also in
`docs/real_first_run_checklist.md`.

## 2026-10-16 — Faster J1 (sideways 300 mm/s), RB5 J6 still (tool follows the base), vertical z x 1.5

**Run 033917 (vertical z x 2, floor -50, Real):** while J1 turned (> 5 deg/s) its speed was 16 deg/s
median, 28 p90, 42 max, and the TCP speed cap was active 46 % of that time (032657: 59 %); sideways
target speed p90 197 mm/s against the 200 cap (about 25 deg/s of J1 at r 450); joint speed limit only
5 %. Vertical mode z speed median 122 mm/s, 40 % at the cap. Measured RB5 J6 range 140 deg: leader-J6
yaw (to the +/-45 end) plus J6 turning back whenever J1 turned (the tool kept its world orientation).
TCP reached z -52. User: J1 should turn faster, RB5 J6 should stay still, vertical a bit slower.

**Decisions (user):**
- `mapping.speed_tangential_mm_s` 300 in Real (400 in Sim, same as its speed): the per-cycle step is
  scaled so that (t / 300)^2 + (o / 200)^2 <= 1, t = sideways about the base z axis, o = radial and
  vertical; direction kept. New Real max 300 (`REAL_MAX`). Lead limit uses the larger step.
- `yaw.mode: follow_base` (Sim and Real): the tool yaw equals the azimuth change of the target, so the
  gripper turns with the arm; the leader J6 is not used and the yaw speed / range limits do not apply.
  Fake robot (straight sideways hand motion, 40 deg twist): J1 +4.6 deg, J6 -0.28 deg; with the old
  world-held tool J6 turned back by as much as J1 (-4.29 for 4.29). `leader_j6` keeps the old mapping.
- `mapping.vertical_z_scale` 2.0 -> 1.5.

## 2026-10-16 — Left button = J1 rotate mode; azimuth range +/-120

**Run 034808 (sideways 300 mm/s, tool follows the base, Real):** RB5 J6 range 6 deg (was 140); J1
speed-limited 21 % of its turning time (46 %), up to 50 deg/s; dropped 36 mm (1433). J1 turned from 90
to 183 deg: 9 s at the azimuth limit -90 (the hand asked 20 deg more), 7 s at r_min (a sideways hand
motion arcs toward the body), J3 joint-limit slowdown 13 % (margin >= 16.6 deg) in that inner corner.
Vertical mode used 6 s. User: the left button would be more useful for turning J1 than for going down.

**Decision (user):** `mapping.left_button` off / vertical / rotate (replaces `vertical_button`), now
`rotate` in Sim and Real: while held, the radius and z of the target stay and only the azimuth changes,
by `rotate_deg_per_mm` 0.3 x the leader's sideways (leader y) motion (100 mm = 30 deg; the cartesian
mapping needed about 236 mm at r 450). The tool still follows the base, so only J1 turns. Same speed
limits; a dropped arc piece shifts the leader y reference by its angle (B / C as elsewhere). The
vertical mode stays available (`left_button: vertical`). CSV column `vertical_mode` -> `left_mode_active`.
`range.azimuth_half_deg` 90 -> 120 (Sim and Real): with the tool turning with J1 the other joints are the
same as at +/-90 (DH grid r 350-800, z -50..400: J5 87.2, shoulder 236 mm, elbow 31.7, joint limit 16.7 deg,
all at the inner low corner); J1 then spans about 63-303 deg (limit +/-360). The space at the sides and
behind the RB5 must be clear.

## 2026-10-16 — Azimuth lock: J1 turns only in the rotate mode

**Run 035549 (rotate mode, Real):** 7 rotate segments turned J1 by 225 deg in 9 s (up to 60.6 deg in one),
with z unchanged (0.000 mm) and the radius within 0.9 mm (a speed-limited step is a chord, slightly
inside the circle); J1 speed 24 / 35 / 48 deg/s (p50 / p90 / max), speed cap active 41 % of that time.
Range limit 1 s (was 15), J3 slowdown 1 % (13). But J1 also turned 150 deg in the normal mode, by
accident. User: without the left button J1 should stay.

**Decision (user):** `mapping.lock_azimuth: true` (Sim and Real). Without a left-button mode the
azimuth stays at its value at the last re-anchor; leader forward / back (leader x) x scale moves the
target along the arm's current direction (radius), up / down (z) x scale moves z, sideways is not used
(user's choice "along the arm": after turning J1, forward still extends the arm). B / C: the radial and
z parts of a dropped vector shift the leader x / z reference. With the tool following the base, the
normal mode now moves J2-J5 in the arm's vertical plane only.

## 2026-10-16 — Target acceleration limit 2.5 m/s^2 against the shake when the limits switch

**Run 040257 (current operation, Real):** the user finds this operation the best for pick and place so
far. J1 locked in the normal mode (0.087 deg in total), descents nearly vertical (radius change 0.13 x
the descent, was 0.5-0.9), RB5 J6 6 deg, work p99 11.1 ms, no late cycle. But the joint acceleration
p99 was the highest so far (1295 deg/s^2, target 5.7 m/s^2 p99), and of the top 1 % of joint
accelerations 97 % were next to the TCP speed limit switching and 82 % next to the joint speed limit
(0 % next to a mode switch): the target velocity jumped from the hand's to the cap in one cycle.

**Decision (user, item 1 of the refinement list):** `mapping.accel_mm_s2` 2500 (Sim and Real; 0 = off).
In plan_step, before the speed limits, the step may differ from the previous cycle's step by at most
a x dt^2 (1.0 mm at 20 ms), and is at most (-a + sqrt(a^2 + 8 a d)) / 2 for a remaining distance d, so
it can still stop in time without overshoot (the continuous sqrt(2 a d) overshot 1.9 mm in the test).
The cut part goes to the C lag / is dropped like the other limits. A release, a late or a stale cycle
still freezes the target at once (the previous step is reset to zero). CSV column `accel_limited`; no
terminal message for it.

**Replay of 040257 through TeleopLoop (Real settings, fake robot, same hand data):**

| accel limit | target accel p99 / max | joint accel p99 / max | desired - target p50 / p95 | dropped |
|---|---|---|---|---|
| off | 4.0 / 20.6 m/s^2 | 971 / 2530 deg/s^2 | 0.3 / 51.9 mm | 2161 mm |
| 2.5 m/s^2 | 2.5 / 2.8 | 412 / 605 | 5.7 / 29.8 mm | 2526 mm |
| 4.0 m/s^2 | 4.0 / 4.5 | 579 / 977 | 3.5 / 36.8 mm | 2354 mm |

Cost of 2.5: about 5 mm more lag behind the hand (median) and 17 % more dropped motion. The test found
a missing event label for the new flag that would have stopped the teleop at the first limited cycle.

**Remaining list:** rotate-mode catch-up in degrees (1457 mm dropped while rotating), radial / vertical
cap 250 mm/s, asymmetric azimuth range (-120 reached, +120 unused), radius kept exactly on the circle in
the rotate mode (1.2 mm chord drift).

## 2026-10-16 — Rotate mode: 15 deg catch-up, target kept on the circle, re-anchor at the stop point

**Run 041233 (acceleration limit 2.5 m/s^2, Real).** Correction first: the "measured joint acceleration
p99" in the earlier entries (823-1295 deg/s^2) assumed RB5 samples 20 ms apart; they are 13-22 ms apart
(the state arrives every ~10 ms and the loop takes the latest), which inflated J1 in fast turns. With
the logged sample times: 040257 -> 041233 joint acceleration p99 533 -> 409 deg/s^2 (J1 493 -> 318), the
commanded azimuth acceleration p99 757 -> 310, target jitter p99 5.7 -> 2.5 m/s^2. Also: clutch presses
longer (median 0.9 -> 1.7 s, none < 0.5 s), normal-mode speed cap 40 -> 14 %, dropped 845 -> 386 mm; but
1477 mm dropped while rotating (25 % at the cap), and J1 moved 1.23 deg in the normal mode: after the
left button was released the target still had its turning speed, passed the anchor and came back.

**Decisions (user, items 1 + 2, and 4 with them):**
- `mapping.rotate_max_lag_deg` 15 (Sim and Real; the fixture uses 0): in the rotate mode the C lag is
  r x 15 deg (about 118 mm at r 450) instead of max_lag_mm, so a fast swing is finished after the hand
  stops; only the part beyond 15 deg is dropped. B stays B.
- In the rotate mode the applied target is put back on the circle (anchor radius and z) every cycle:
  the steps are straight chords (radius drift was up to 1.2 mm; now 0.001 mm in the replay).
- Every re-anchor (mode switch, clutch press) uses the point where the target stops from its last step
  at the acceleration limit, v (v - a) / (2 a) further along it, matching the discrete stopping rule.
  A clutch release still freezes at once (its step is reset to zero).

**Replay of 041233 through TeleopLoop (Real settings, fake robot):** rotate lag 0 / 15 deg: dropped while
rotating 3720 / 507 mm (-86 %), J1 turned in the rotate mode 460 / 798 deg (more of what the hand
asked); in the normal mode 1.24 deg of J1 motion within 0.3 s after the 9 releases (slowing down to the
stop point, no return) and 0.12 deg later (same in both); one release turned back by 0.05 deg (about
0.4 mm). Not changed: the 250 mm/s radial / vertical cap (the hand exceeded 200 mm/s only 8 % of the
normal time); the asymmetric azimuth range is still open.

## 2026-10-16 — Range in absolute base-frame values; ready pose = the user's pose at r 450

**Run 042300 (rotate catch-up, Real):** dropped while rotating 1477 -> 212 mm, radius drift in the rotate
mode 0.00 mm, J1 in the normal mode 0.076 deg, joint acceleration p99 (sample times) 409 -> 331 deg/s^2,
no range limit, no late cycle (slow hand motion this run: normal-mode radial speed p90 57 mm/s).

**Question (user):** use the pose the user set on the real RB5 as the ready pose (it looks better)?
Read from the data channel (read only): (171.53, 14.59, -127.82, 29.54, -87.31, 55.63), DH TCP r 383,
z 152, azimuth 5.3, same tool tilt. It passes the start checks, but (1) it is only 33 mm from r_min
(like the r 367 pose before) and (2) the range was relative to the start pose, so its z top would drop
465 -> 417, the r_max_by_z heights by 48 mm and the azimuth range by 8 deg.

**Decisions (user):**
- The range is absolute in the base frame: `z_min_mm`, `z_max_mm`, `r_max_by_z` [z, r_max] and
  `azimuth_min_deg` / `azimuth_max_deg` replace z_below / z_above / z_min_abs / dz pairs / azimuth_half.
  Values = the space in effect: z -50 .. 465, r_max 800 / 775 / 725 / 700 / 650 at z 265 / 315 / 365 /
  415 / 465, azimuth -107 .. 133 (was 13.4 +/- 120). The start check now also requires the ready pose
  inside the z and azimuth range. Changing the ready pose no longer moves the range.
- Ready pose (173.650, 4.300, -119.566, 31.666, -87.544, 53.521) in Real and Sim: the user's pose with
  the same azimuth, height and tool orientation (0.001 deg), moved out to r 450 (IK marched from it):
  DH TCP (448.1, 41.4, 152.3), 100 mm to r_min; margins J5 87.5, shoulder 336 mm, elbow 60.4, J3 45.4
  deg; vertical line z -50 .. 465 at r 450: J5 87.5, shoulder 336, elbow 47.2, J3 32.2. Largest joint
  change from the user's pose 10.3 deg (J2), from the previous ready pose 11.8 deg (J3).

## 2026-10-16 — Joint-limit slow threshold 20 -> 15 deg

**Run 043303 (new ready pose, absolute range, Real):** start TCP (447.8, 41.3, 154.3), r 449.8, as set.
J1 lock 0.099 deg, rotate radius drift 0.00 mm, no late cycle. Faster and deeper than before (vertical
speed asked p90 191 mm/s, floor -50 reached): normal-mode speed cap 24 %, 754 mm dropped. New: the
joint-limit slowdown was active 19.9 % of the clutch time (13 s), all J3, all at r 350-372 and z -50 ..
-5 (the inner low corner; going down from r 450 pulled the radius in by 0.26 x the descent). The J3
margin there was 16.7 deg, the smallest anywhere in the range (DH grid with the tool following the base:
16.7 at r 350, z -50).

**Decision (user, option A):** `safety.joint_limit_slow_deg` 15 in Real and Sim (stop stays 10). Inside
the range the J3 margin cannot go below 16.7, so the stop is unreachable there and the 20 deg slowdown
only slowed the corner. Replay of 043303 (TeleopLoop, Real settings, fake robot): slowdown 13.0 s -> 0 s,
smallest margin 17.4 deg either way. The test fixture keeps 20 (script 21's value).

**Rejected:** a higher r_min near the floor (would block reaching that corner); leaving it.

## 2026-10-16 — Azimuth range widened on the minus side: -160 .. 133 deg

**Run 044024 (J3 slow threshold 15, Real):** joint-limit slowdown 0 % (was 19.9), J3 margin >= 16.6 deg;
dropped 293 mm in total (the lowest of the active runs; normal 199, rotate 93 while turning J1 716 deg);
J1 lock 0.070 deg, rotate radius drift 0.00, joint acceleration p99 (sample times) 417 deg/s^2, work
p99 10.6 ms, no late cycle. The azimuth used was -107.0 .. 14.4 deg: 1.7 s at the -107 limit with the
hand asking up to 49 deg more; the + side beyond 14 was not used.

**Decision (user):** widen only the minus side (moving the hand to the right in the rotate mode turns
clockwise seen from above): `range.azimuth_min_deg` -107 -> -160 in Real and Sim, max stays 133 (width
293 deg). From the ready pose (azimuth 5.3) that is 165 deg clockwise (nearly behind the robot) and
128 deg counterclockwise. The tool turns with J1, so the other joints keep their margins; J1 spans about
8 .. 301 deg (limit +/-360). That side and the back of the RB5 must be clear (checklist).

**Rejected:** shifting the whole range to -160 .. 80 (user keeps the + side).

## 2026-10-16 — J1 a bit faster (sideways 360 mm/s, J1 75 deg/s); acceleration limit moved to the last stage

**Run 044916 (azimuth -160 .. 133, Real):** the whole new range was used (-160 .. 120). Faster operation
than before (normal-mode speed cap 52 %, hand above 200 mm/s 30 %). User: J1 turns a little slowly. In
the rotate mode the hand asked J1 p50 / p90 / p99 36 / 89 / 121 deg/s, the robot did 35 / 42 / max 49:
capped by the sideways speed (300 mm/s = 49 deg/s at r 350, 37 at r 459) and the J1 joint cap (60 deg/s;
the joint speed limit was active 50 % of the rotate time). Target jitter p99 6.9 m/s^2 in the rotate mode
(2.5 in the normal mode), 92 % of the top 1 % in the rotate mode: the joint limit factor came after the
acceleration limit and changed from cycle to cycle, because the joint prediction includes the robot's
lag, which jitters with the 13-22 ms state timing. 21 s of "range limited" in the rotate mode were the
rotate circle lying exactly on r_min (350): every straight step cut inside it and was pushed back.

**Decisions (user, all three):**
1. `mapping.speed_tangential_mm_s` 300 -> 360 and J1 joint cap 1.2 -> 1.5 deg/cycle (75 deg/s; J2-J6 stay
   1.2) in Real; `REAL_MAX` tangential 360, joint 1.5. Sim unchanged.
2. The acceleration limit is the last stage of plan_step (after the speed and joint limits), so a switching
   limit cannot make the step jump. While it slows down a cap can be exceeded for a few cycles.
3. The rotate circle is at least r_min + 1 mm.

**Replay of 044916 (TeleopLoop, Real settings, fake robot).** With the state delayed by a random 1-2
cycles (like the real 13-22 ms timing; harsher than reality): target accel p99 rotate / normal 10.0 / 9.7
-> 2.5 / 2.5 m/s^2; J1 in the rotate mode p90 41 -> 58 deg/s, max 61 -> 77; dropped while rotating 2261
-> 1535 mm; a joint above its cap by at most 0.012 -> 0.043 deg/cycle (about 2 deg/s, 36 cycles). Without
the delay: J1 p90 49 -> 58 deg/s, dropped while rotating 765 -> 506 mm, no cap exceeded, range flag in
the rotate mode 20.6 -> 1.5 s. The 6.9 m/s^2 seen on the real robot did not appear in the replay without
the delay (the fake state is exact), which supports the diagnosis.

## 2026-10-16 — Radial / vertical speed cap 200 -> 250 mm/s

**Run 045859 (sideways 360 mm/s, J1 75 deg/s, acceleration limit last, Real):** rotate-mode target jitter
p99 6.9 -> 2.5 m/s^2 (same as the normal mode), robot J1 in the rotate mode p90 / max 46 / 55 deg/s (42 /
49), dropped while rotating 1174 -> 287 mm, rotate range-flag flicker gone; the sideways cap (360 mm/s =
46 deg/s at r 450) was active 71 % of the rotate time, the J1 joint cap 16 %. J1 in the normal mode 0.75
deg (slowing down after faster releases, no return). Now the normal mode is the bottleneck: speed cap
73 % of the normal time, hand above 200 mm/s 28 %, 1167 mm dropped there (80 % of all drops); radial /
vertical speed asked p90 199 / 194 mm/s.

**Decision (user, option A):** `mapping.speed_mm_s` 200 -> 250 in Real, `REAL_MAX` 250 (kinetic energy
1.56x at the cap). Sideways 360 and the joint caps unchanged. Replay of 045859 (TeleopLoop, Real settings,
fake robot, no state delay): normal-mode speed cap 73 -> 65 %, dropped in the normal mode 1129 -> 606 mm,
no joint cap exceeded, target jitter p99 2.5 m/s^2. Far / low corners need up to 0.393 deg/mm, so the
J2-J6 joint caps (60 deg/s) can bind there at 250 mm/s (98 deg/s needed); not seen in the replay.

## 2026-10-16 — Joint speed limit on the step only (the robot's lag is no longer added)

**Run 050415 (radial / vertical 250 mm/s, Real):** normal-mode speed cap 73 -> 59 %, dropped there 1167 ->
873 mm (the hand was faster: above 200 mm/s 36 %). Target jitter p99 2.5 m/s^2, no late cycle, J3
slowdown 0. New bottleneck: the joint speed limit was active 34 % of the time in both modes (was 10 %),
mostly J4 / J3 / J2 in the normal mode; 652 of the 873 mm dropped there happened with it active. Yet the
measured J1 stayed below its cap (max 56 of 75 deg/s).

**Cause:** the per-joint speed check predicted the joint motion of "the robot's lag behind the target +
this cycle's step". The reported state is about one cycle old (real lag while moving fast: 0.54 steps
median, 1.01 p99, at most 6.7 mm), so at a steady speed that is about two steps, and the joints were held
near half their caps. The lag came over from script 21 together with the pose predictions (where the
robot will be after catching up); no reason was recorded for using it in the speed check.

**Decision (user):** the joint speed limit uses this cycle's step only (position and yaw scaled by the same
factor, f = min(cap / dq)). The pose checks (singularity, joint limit watches, stop) still predict the
pose after catching up. The script-21 comparison test sets the joint caps out of reach on both sides (all
else still compared one to one); a new test checks that the lag does not change the joint decision.

**Replay of 050415 (TeleopLoop, Real settings, fake robot, state reported one cycle late = the real p99):**
previous / new rule: joint limit active 63 / 2 %; dropped in the normal mode 1387 / 560 mm, while rotating
781 / 223 mm; the fake robot's true joint speeds J1 max 41 / 59 deg/s, J2-J4 max 34 / 60 deg/s (caps 75 /
60), never above a cap; target jitter p99 2.5 / 2.5 m/s^2. Real lag + a full step stays below the 14.4 mm
lead limit in 0 % of the fast cycles of 050415, so the lead limit should not start acting. (A harsher model
with a random 1-2 cycle delay triggers the lead limit; the real data does not show that delay.)

## 2026-10-16 — real-1 final settings frozen; Simulation = Real

**Run 051339 (joint speed limit on the step only, Real):** joint speed limit 34 -> 0 %; measured joint
speeds within the caps (max J1 64 / J2 54 / J3 47 / J4 63 deg/s against 75 / 60; no sample more than 5 %
above); lead limit 0 %; target jitter p99 2.5 m/s^2; descent drift 0.05 x the descent. One late cycle (a
single 12.7 ms planning step, send gap 30.2 ms against the 30 ms limit; that cycle froze the target as
designed). The hand was faster again (above 200 mm/s 54 % of the normal time, rotate asks p50 54 deg/s),
so the speed caps are what is left: normal-mode cap 72 %, rotate 83 %.

**Decision (user):** no further speed increase; wrap up real-1. Step 1: the Real settings are frozen as
"real-1 final" (header in real.yaml: change a value only with a decisions entry), and configs/sim.yaml
is generated from real.yaml so Simulation practice feels like the real robot (before: scale 2.0, 400 mm/s,
lag 50 mm, other joint caps and yaw values). Only `mode` and `tcp.expected_offset_mm` (not checked in
Simulation) differ; tests/test_settings.py fails if anything else drifts. The test fixture pins the
older Simulation values its tests were written for.

**Final operation and values (Real and Sim):** clutch = right button; normal mode: J1 locked, leader
forward / back -> along the arm (radius), up / down -> z, scale 1.0, sideways unused; left button held:
rotate mode, radius and z fixed, 0.3 deg of J1 per mm of sideways hand motion, 15 deg catch-up; the tool
turns with J1 (RB5 J6 still). Speeds: radial / vertical 250 mm/s, sideways 360 mm/s, joints J1 75 / J2-J6
60 deg/s (on the step), acceleration 2.5 m/s^2 (last stage), C lag 20 mm. Leader: deadband 0.3 mm / 0.15
deg, low-pass 30 ms. Range (absolute): r 350-800, z -50..465 (outer radius 800 -> 650 between z 265 and
465), azimuth -160..133 deg. Ready pose (173.650, 4.300, -119.566, 31.666, -87.544, 53.521). Watches:
J5 / shoulder / elbow slow 20 deg / 50 mm / 20 deg, joint limit slow 15 (stop 10 / 20 / 10 / 10).

## 2026-10-16 — real-1 closed: summary

**Goal (user):** make Real teleop easy enough for pick-and-place demonstrations (it was too slow and the
operator had to drag the robot with many clutch presses).

**Start (2026-10-14, run 035701):** scale 1.0, 100 mm/s, joints 40 deg/s, cartesian mapping, the leader J6
turned the tool, ready pose r 367 (17 mm from r_min), range relative to the start pose, z floor 30 mm.
Speed cap active 72 % of the clutch time, 1540 mm of hand motion dropped, target jitter p99 3.5 m/s^2.

**End (run 051339, settings frozen):** see "real-1 final settings frozen" for every value. In short:
- Normal mode: J1 locked; forward / back = along the arm, up / down = height; 250 mm/s.
- Left button: rotate mode, J1 only, 0.3 deg/mm, 15 deg catch-up, sideways up to 360 mm/s.
- The tool turns with J1 (RB5 J6 still).
- Joint caps applied to the step (J1 75, J2-J6 60 deg/s); acceleration 2.5 m/s^2 as the last stage.
- Leader low-pass 30 ms; absolute range with azimuth -160..133; ready pose at r 450; J3 slowdown at 15 deg.

Measured: joint speeds up to the caps and not above, target jitter p99 2.5 m/s^2, lag p99 0.9 steps,
J1 locked to 0.09 deg in the normal mode, descents nearly vertical (0.05 radius change per mm), no
slowdowns, one late cycle in 83 s. The hand is now about 2.5x faster than at the start (radial /
vertical asked p90 235 / 252 mm/s), and the caps are again what limits it (by choice: no further raise).

**What helped most (in order of effect on the operator):**
1. Speed caps step by step with the hand speed measured from the logs (B, then sideways, then 250).
2. Modes that keep the hand's natural arcs out of the other axes: J1 lock + rotate mode, the tool following the base.
3. Moving the ready pose away from r_min.
4. Smoothness: leader low-pass, acceleration limit (last stage).
5. Removing the robot's lag from the joint speed check: it held the joints at half their caps.

**Tried and reverted:** cylindrical mapping (J1 easier, but "does not go where I want"; speed cap 65 %);
vertical mode on the left button (worked: x, y fixed; replaced by the rotate mode, which the user found
more useful; still available as `left_button: vertical`).

**Lessons recorded in the entries:**
- Measure the RB5 state with its real sample times (13-22 ms apart).
- A replay needs the real state delay to reproduce problems, but a delay model harsher than the logs
  invents new ones.
- Values relative to the start pose move with every new ready pose.
- A limit later in the chain can undo a smoothing stage earlier in it.
- Adding a new flag means checking every place that uses the flags (a missing label would have stopped
  the teleop).

**Tools:** `scripts/23_trial_summary.py` (trial summary), `scripts/24_run_report.py` (full run report,
`--compare`), `docs/real_trial_protocol.md` (trial protocol, not needed in the end: the user chose
changes from the measured hand speed).

## 2026-10-16 — Cleanup before the gripper: cylindrical mapping and the script-21 comparisons removed

**Decision (user, cleanup list items 6, 3, 4):**
- The cylindrical mapping (`mapping.mode`, `pivot_xy_mm`, `min_pivot_radius_mm`) is removed from the code,
  the settings and the tests. It was tried once (031043) and reverted; it was the most complex part of the
  Mapper (an integrated desired position and a pending state). The vertical mode and `yaw.mode: leader_j6`
  stay (small, settings only).
- The tests that compared plan_step and the loop with scripts/21 (`TestSameAs21.test_random_steps`,
  `TestLoopSameAs21`) are retired: the teleop has deliberately diverged from 21 (joint check on the step,
  acceleration limit, modes), and keeping them needed workarounds (joint caps out of reach, pinned
  values). The behaviour tests stay; the joint-check test moved to `TestPlanStep`. 66 -> 58 tests (the 8
  removed: 6 cylindrical / 21 tests in test_core and test_loop, by name).
- Small: unused imports in the tests; stale comments in the settings (C lag time at 250 mm/s; the yaw
  values are used only with `leader_j6`); the mapping docstring no longer says "without changes in
  behavior" from 21.

## 2026-10-16 — Gripper signal moved to teleop/gripper.py

**Decision (user, cleanup list item 1):** `GripSignal` (leader trigger -> grip_close with hysteresis)
moved unchanged from `teleop/leader.py` to a new `teleop/gripper.py`, so the gripper has its own module
before the Allegro Hand is connected (its commands will go there). leader.py keeps the leader arm only
(joints, handles, FK, filter, clutch). The class is byte-identical to before; the module notes that the
trigger direction is not confirmed on the hardware (PENDING). 58 tests pass.

## 2026-10-16 — Loop: CSV row and status line out of run()

**Decision (user, cleanup list item 2):** in `teleop/loop.py` the CSV row is built by a module-level
`csv_row(...)` next to `CSV_HEADER` (named arguments, same order as the header), and the status line by
`TeleopLoop._status_line(...)`. No change in behaviour: five fake-device scenarios (normal, fast with C,
rotate + azimuth lock + acceleration limit, vertical, read failures and gripper) give byte-identical rows
and stop messages before and after (the wall-clock timing columns excluded; the baseline itself was
checked to be repeatable). The status line was run once with the status enabled. 58 tests pass.

## 2026-10-17 — cam-1: RealSense D435 + D405 RGB test (pyrealsense2 2.58.4)

**Setup:** `pyrealsense2==2.58.4.10922` in `rb5` (same version as the system librealsense 2.58.4: the
tools, udev rules and Python behave the same; 2.59.1 is newer but not mixed in); no OpenCV yet (the test
needs numpy only, PNG via matplotlib). The D435 first enumerated as USB 2.1 (480 Mbps; color only up to
640x480 @ 30 / 1280x720 @ 15); after it was reconnected both cameras are at 5000 Mbps (same hub, its USB 3
side). `scripts/25_camera_rgb_test.py`: one pipeline per camera (color RGB8 only), one thread each; per
frame the frame number, camera timestamp and domain, metadata timestamps and the PC receive time.

**Results (60 s each, 2 s warm-up left out, both cameras at once, no robot):**

| setting | D435 fps | D405 fps | missed frames | PC interval p95 / max | CPU (one core) |
|---|---|---|---|---|---|
| 640x480 @ 30 | 29.98 | 29.98 | D405 1 | 33.7-33.8 / 36.9 (66.6 at the miss) ms | 10 % |
| 640x480 @ 60 | 59.53 | 59.89 | 0 | 17.1-17.2 / 17.9 ms | 17 % |
| 1280x720 @ 30 | 29.98 | 30.00 | 0 | 33.9-34.1 / 35.1 ms | 18 % |
| 640x480 @ 30, D435 ae priority on | 29.76 | 29.99 | 0 | 34.0 / 34.6 ms | 9 % |

- Timestamps are in the `global_time` domain (the camera clock mapped to the PC clock by librealsense):
  drift within +/-6 ppm, residual 0.2-0.4 ms std. Metadata (sensor timestamp, time of arrival) available
  for every frame without the DKMS module.
- PC read time - camera time: D435 33.6 ms at 30 fps, 17.2 at 60, 35.1 at 1280x720; D405 9.5 / 9.4 / 17.4.
  The D435's is about one frame period, so its timestamp is likely taken earlier in the frame than the
  D405's (not confirmed in a document).
- The D435's "60 fps" is 59.53 fps (16.80 ms): records must use the timestamps, not frame counts.
- The cameras are not synchronized: the nearest frames differ by up to half a frame period (p50 4-8 ms,
  max 8.6 ms at 60 fps, 16.8 ms at 30 fps).
- auto_exposure_priority: the D435 default is off (keeps 30 fps); on gave 29.76 fps in this room light.
  The D405 color sensor does not have the option.

**Candidates for demonstrations:** 640x480 @ 30 (the common LeRobot setting, ACT / ALOHA used 480x640
images; lowest load) or 640x480 @ 60 (closer to the 50 Hz teleop; ALOHA records at 50 Hz). Both work with
little CPU; 1280x720 also works at 30 fps but costs storage. To decide with the recording format.

## 2026-10-17 — What jnt_ref is: the controller's joint reference, 60 ms behind the sent target

**Question (user):** is `jnt_ref` (data channel) the target or the measurement, how late is it, and can it
be the joint action for ACT? Logs only: Real runs 051339 and 045859 (real-1 final settings), no motion.

**Method (`scripts/26_jnt_ref_check.py`):** every sent TCP target (position + start rpy with rz + yaw) is
solved with our DH model (IK error < 1e-5 mm); targets are placed at their send time, RB5 states at their
receive time (t_s + leader_read_ms - state_age_ms); the delay is the shift with the smallest RMS while
moving (65 % of the samples). A model-free check compares the Cartesian sent target with `tcp_ref` and with
the FK of both joint signals.

**Results (051339; 045859 the same within 2 ms):**

| compared | delay | RMS while moving |
|---|---|---|
| tcp_ref vs sent target | +24 ms | 0.20 mm |
| FK(jnt_ref) vs sent target | +60 ms | 2.1 mm (= the DH model error) |
| jnt_ref vs IK(sent target) | +60 ms | 0.20 deg (J1-J6 0.06 / 0.26 / 0.18 / 0.37 / 0.00 / 0.02) |
| jnt_ang vs IK(sent target) | +64 ms | 0.20 deg |
| jnt_ang vs jnt_ref | +6 ms | 0.03 deg (max 0.18) |

- Our DH model differs from the controller by about 2 mm when the robot is still (FK(jnt_ref) vs tcp_ref,
  max 4.9 mm); this is the floor of any comparison through our IK (0.37 deg median at rest).
- Within one packet, `tcp_ref` is about 40 ms ahead of `jnt_ref`: tcp_ref is the Cartesian command (our
  target about one cycle later; at rest equal to it, 0.0000 mm in 045859), jnt_ref is the joint reference
  after the controller's servo interpolation (t1 / t2) and IK.
- The encoders follow jnt_ref within 6 ms and 0.03 deg RMS; at rest 0.002 deg median.

**Conclusion:** jnt_ref is on the command side, not a measurement: the controller's smoothed joint
trajectory, equal to the IK of what we sent about 60 ms later (within our DH model error). It can be a
joint action for ACT: action(t) = jnt_ref(t + 60 ms) is the joint form of the command at t; the measured
joints are almost the same (6 ms, 0.03 deg), so action and observation differ mainly by that shift. But:
(1) running a joint-action policy needs `move_servo_j` and joint-space safety checks (our limits, range
and watches work on the Cartesian target); (2) the exact command is the sent target itself (no model, no
delay to estimate), and it is what our safety pipeline takes. Proposal for the recording format (step 3):
record all of them (sent target + yaw, tcp_ref, jnt_ref, jnt_ang, timestamps); main action = the sent TCP
target (x, y, z, yaw) + gripper, with jnt_ref(t + 60 ms) as the alternative joint action; observation
state = jnt_ang (or jnt_ref).

## 2026-10-17 — Open (step 3): demonstration recording rate 50 Hz vs 30 Hz

Candidates for the recording format step. **Decided 2026-10-19: 30 Hz (see the rec-1 entry).**

| | 50 Hz | 30 Hz |
|---|---|---|
| robot data | native (the teleop loop is 50 Hz) | resampled from 50 Hz (nearest or interpolated) |
| cameras | 30 fps: each frame used 1.7 times (duplicates); 60 fps: nearest frame per 20 ms tick, 8.4 ms max error, D435 delivers 59.53 fps (cam-1) | 30 fps: each frame once, natural for both cameras |
| LeRobot | video stored at the dataset fps: 50 fps video from 30 / 60 fps cameras means resampling | the most common setting in LeRobot datasets (640x480 @ 30) |
| ACT | ALOHA recorded and ran at 50 Hz | fewer steps per second: the same chunk covers more time |
| policy at run time | outputs at 50 Hz, same as the teleop | outputs at 30 Hz; the 50 Hz servo loop holds or interpolates between them |
| size | 1.7x frames | smaller |
| timing | jnt_ref / command delays (24 / 60 ms) are 1.2 / 3 ticks | 0.7 / 1.8 ticks |

Things to measure before deciding: CPU and disk with two cameras + the teleop together; how smooth a 30
Hz action stream is after interpolation to the 50 Hz servo loop. Cameras are not synchronized, so frame
choice by timestamp (half a frame period max) holds for both.

## 2026-10-17 — grip-1: MX-106T gripper on its own U2D2; ports by adapter serial; read-only scan (27)

**Hardware (user):** the end effector is a 3D-printed parallel gripper (built by a team member) with one
Dynamixel MX-106T (TTL, 3-pin). The Allegro Hand V4 is not used.

**Decisions:**
- **Own bus:** U2D2 + U2D2 Power Hub Board, not on the leader arm's bus. The leader arm bus is read
  every cycle at 2 Mbps (Protocol 2.0); a second device there would share its time and its IDs (the
  MX-106 factory ID 1 is a leader joint ID).
- **Ports by adapter serial number** (`/dev/serial/by-id/...`): both adapters are expected to be FTDI
  FT232H (0403:6014), so `/dev/ttyUSB0` / `1` depend on the plug order. With the old `/dev/ttyUSB0`,
  scripts 14 / 16 (which write torque settings to the leader IDs 0-13) could have written to the
  gripper motor. Leader: `configs/*.yaml` `leader.port` and `scripts/config.py` `LEADER_PORT` (scripts
  11-14, 16, 20, 21). The by-id link exists without any rule; the udev rule is still needed for the
  latency timer (it reset to 16 ms after a replug, 2026-10-17).
- **udev rule** renamed `assets/udev/99-rb-cobot-usb-serial.rules` (leader arm + gripper). The gripper
  lines are written but commented: the U2D2 was not connected yet, so its serial is unknown.
- **Read-only scan first** (`scripts/27_gripper_scan.py`): ping / read only, Protocol 2.0 and 1.0, refuses
  the leader arm's port. What decides the plan is the firmware: model number 320 = Protocol 1.0 firmware
  (no current-based position mode), 321 = MX-106(2.0) (operating mode 5, current-based position:
  Goal Current limits the current, "ideal for grippers", e-Manual).
- **Firmware 1.0 -> 2.0 update** only after the user checks the motor's other uses in the lab.
- **Power** (e-Manual): 12 V recommended (10.0-14.8 V), stall 5.2 A at 12 V -> a 12 V supply of at least
  5 A, separate from the leader arm's supply; a current-limited bench supply for the first tests.

**Scan result (script 27, read-only, 2026-10-17):** U2D2 = FT232H serial FT62AJAX (`/dev/ttyUSB1` this time).
One motor: MX-106(2.0) (model 321), firmware v45, ID 2, 1,000,000 bps (not the factory ID 1 / 57,600: set
before), operating mode 3 (position), position limits 2388-3853, current limit 2047 (6.88 A), return
delay 500 us. No firmware update needed (the user also checked with DYNAMIXEL Wizard). At the scan:
torque enable 1 (not written by us), position 2388 (= the min limit), hardware error status 1 = input
voltage (9.2 V < min 9.5 V; supply to be changed to 12 V). Reading 10 bytes (current, velocity,
position) took 16.0 ms p50 because the U2D2 latency_timer was still 16 ms.
Second scan after the udev rule (user installed it) and the supply change: latency_timer 1 on both
adapters, `/dev/gripper_u2d2` -> ttyUSB0 and `/dev/rby1_leader_arm` -> ttyUSB1 (the order swapped, as
expected); the same read 2.0 ms p50 / 2.9 ms max, 0 failed. Torque enable 0, position 2223 (below the min
limit 2388; torque off, so not refused). **Input voltage still 9.1 V** and the voltage error bit still set:
the 12 V does not reach the motor yet.
Third scan after the user fixed the supply: **input voltage 10.8 V, hardware error 0**, read 2.0 ms p50 /
2.9 ms max, 0 failed. Inside the e-Manual range (10.0-14.8 V) but 1.2 V below the 12 V recommended at
almost no load (about 30 mA): check the voltage under load in grip-2 (it must stay >= 10 V).

**Plan after grip-1:** grip-2 the gripper alone (mode 5, low Goal Current, open / closed positions,
trigger direction); grip-3 a gripper thread in the teleop (latest goal / latest state shared, the loop
never waits for the gripper bus), compared in Simulation by `work_ms` and late cycles.

**Rejected:** the leader arm bus (shared time and IDs); `/dev/ttyUSBn` names (plug order); updating the
firmware right away (other lab users).

## 2026-10-17 — grip-2: left trigger -> gripper alone (script 28), current-based position mode

**Request (user):** move the gripper with the leader trigger 0-1000 to check it works; then the left
trigger instead of the right (like the left button for RB5 J1); trigger only, no button.

**Setup (script 28):** left trigger, released 0 = closed, fully pulled 1000 = fully open (user), in
proportion: goal = closed + trigger / 1000 x (open - closed). Open / closed measured by hand with torque
off: **closed 3968, open 2196** (155.7 deg; closed is 127 ticks from the 4095 wrap). Operating mode 5
(current-based position): Goal Current 100 (0.34 A) caps the force; the motor's position limits
(2388-3853) are not used in mode 5 (e-Manual), so every goal is clipped to open..closed. Profile velocity
100 (22.9 rpm), acceleration 50. At the end always torque off and mode 3 (user: back to 3).

**Problems found on the way (fixed):**
- Run 1: the leader arm FTDI disconnected (kernel `usb 1-9.2: USB disconnect`, ttyUSB1 -> ttyUSB2) before
  the first loop read; torque off / mode restore worked, but the error showed as "Ctrl+C" and no log was
  saved -> any error now stops with its reason and the log is always saved. The leader adapter, the U2D2
  and the D405 were on one VIA USB 2.0 hub; the user moved the leader adapter.
- Run 2: the catch (hold until the trigger points within 5 % of the gripper position) was skipped by a
  fast trigger (643 -> 779 in one 20 ms read, window 662-762), so it only caught when the user pressed
  the button and moved slowly; it looked like "button needed". Now it also catches when the trigger
  passes over the position (replayed on the log: catches at the first pull, button 0).
- A run that did not finish left the motor in mode 5 with torque on (no log; probably the terminal
  closed or a double Ctrl+C), and the restore used "the mode at the start" (would have kept 5) ->
  always mode 3 at the end, SIGTERM / SIGHUP clean up like Ctrl+C, a second Ctrl+C is ignored during the
  cleanup, torque left on is turned off with the confirmed writes (no DYNAMIXEL Wizard needed).

**Result (run 101652, 34 s, trigger only, button never pressed):**

| | value |
|---|---|
| loop | 20.0 ms period (p99 20.1), work p50 6.8 / max 10.6 ms, 0 failed reads |
| tracking when settled | 0-2 ticks (closed 3968 +0, open +1); in proportion in between |
| full stroke (156 deg) | 1.22-1.28 s, limited by the profile velocity (max 23.8 rpm); 420 ms behind the trigger while moving |
| current | p50 40, p95 138, max 366 mA (cap 336, 4 short peaks); 0 mA holding closed (nothing inside) |
| voltage | 10.3-10.7 V (run 1: 10.1 V); no hardware error |

**Decisions:** left trigger for the gripper; trigger only (no button); keep the 5 % catch with the pass-over
rule; mode 3 at the end of every run.

**Grasp run (102033, 77 s, Goal Current 100):** an object held at about 3190-3290 with the current at
the cap (336 mA) and 10.2 V (min 10.1 V). The current also reached the cap while closing without an
object (51.4-51.9 s), so the cap slowed the closing. User: grip a little weak, closing too slow.
-> defaults: **Goal Current 150 (0.50 A)**, **profile velocity 200 (45.8 rpm, about the no-load maximum;
the velocity limit 210 is checked at the start)**, profile acceleration 100 (0 -> 45.8 rpm in about 0.13 s).
More current means more voltage drop: at 0.34 A the supply already fell to 10.1 V, so 0.50 A may hit the
10 V stop until the supply gives 12 V.

**Cleanup bug (found after run 102033):** the motor was again in mode 5 with torque on although the log
was saved. Cause: Ctrl+C in the middle of a serial transaction (about 7 of every 20 ms) leaves
dynamixel_sdk's `port.is_using = True`, so every later packet is refused ("port is in use") and the
end writes fail. Reproduced read-only (flag set -> read None; cleared -> read OK). Fix: the cleanup
clears the flag and the input, tries each write 3 times, reads torque and mode back and warns when they
are wrong. grip-3 (gripper thread) needs the same cleanup.

**Run 102450 (new defaults, 24 s):** full stroke 0.66-0.76 s (was 1.22-1.28), max 42.8 rpm, 240 ms behind
the trigger while moving; settled 0-4 ticks; the current reached the 504 mA cap only while accelerating
(short peaks to 709 mA when braking at the closed end), no sustained grasp in this run; position up to
4001 (33 ticks past the closed 3968, inertia when reversing); voltage 10.2-10.8 V; the cleanup read back
torque 0 / mode 3 (checked again with script 27).

**Run 102724 (grasp, 64 s, Goal Current 150):** three holds of 0.9-2.4 s at about 3440-3515 (the object),
current 502 mA = the cap (504), voltage 10.2-10.3 V (no 10 V stop), temperature max 36 C; one hold with
the trigger partly pulled (goal 3663) still pressed at the cap because the goal was past the object.
Strokes and tracking as in 102450. Catch: started open (2202) with the trigger released, so it waited
until the first full pull (19.6 s; expected). Cleanup read back torque 0 / mode 3.

**Closed (2026-10-17, user: "grip force enough"):** Goal Current 150 (0.50 A), profile velocity 200,
profile acceleration 100, left trigger only, catch with pass-over, mode 3 at the end.

**Open (next):** the supply gives only 10.6 V at rest (10.2 V while gripping), it should be 12 V; stroke speed for pick and place (1.25 s now; profile velocity is the
knob); in the teleop (grip-3) `teleop/gripper.py` assumes the right trigger and "pulled = closed" (the
opposite of this): change it with the thread.

## 2026-10-17 — grip-3a: gripper code in teleop/gripper.py (Motor, GripCommand, GripperWorker thread)

**Decision:** the MX-106 code of script 28 moves to `teleop/gripper.py` so the teleop (grip-3b) and script 28
use the same code:
- `Motor`: read, write, state block, `reset_port`, `end_state` (torque off, mode 3, retried, read back).
- `check_motor` (model 321, no hardware error, voltage) and `start_motor` (the confirmed writes; the
  position must not move with the mode change).
- `GripCommand`: open fraction (trigger / 1000) -> goal, clipped to the measured range, catch (5 % or
  pass-over). Pure logic.
- `GripperWorker`: a thread that owns the gripper bus at its own 50 Hz. The caller only calls `set(frac)`
  and `latest()` / `events()` (short lock, no waiting). It writes the goal when it changed by 2 ticks or
  more, reads the state, keeps one row per cycle (`ROW_HEADER`, with the age of the trigger value).
  It stops by itself on a hardware error, > 70 C, 3 failed reads or any error, leaving the motor as it
  is; `finish()` stops the thread and runs `end_state`.
- Script 28 now reads the trigger in its loop and hands it to the worker (same behaviour, same checks;
  the CSV columns are the worker's rows). The old `GripSignal` stays until grip-3b replaces it in the loop.

**Teleop decisions for grip-3b (user approved the recommendations, 2026-10-17):**
1. A gripper fault (read failures, hardware error, U2D2 lost) stops only the gripper thread and shows an
   event; the arm teleop continues. Rejected: stopping the whole teleop (the arm would stop suddenly
   because of the gripper).
2. Input voltage < 10 V during the teleop: warning event only (`low_voltage_stops=False`); turning the
   torque off would drop the object. Script 28 still stops.
3. At the teleop end: torque off and mode 3, like grip-2. The object must be put down first (checklist).
4. The gripper moves in Simulation mode too; on by default, `--no-gripper` to turn it off.

**Why a thread:** one gripper bus transaction takes about 2 ms (script 27, p50) and a cycle has 2-3 of
them (goal write, state read, error read; grip-2 work up to about 10 ms); in the teleop loop that would
add to every cycle. On its own U2D2 and thread it runs
in parallel with the leader read (the bus I/O releases the GIL).

**Checked:** 14 new tests in `tests/test_gripper.py` with a fake motor (proportional mapping, clipping,
catch window and pass-over, deadband, write only on change, stops on failed reads / hardware error /
temperature / low voltage or warning only, an exception still cleans up, start-write order, refusal when
the position moved, `end_state` clears the busy flag); all 72 tests pass. Script 28 not run on the
hardware yet in this step.

## 2026-10-17 — grip-3b: gripper in the teleop (left trigger, own thread, Simulation and Real)

**Request (user):** move the gripper with `python -m teleop --real`. Implemented for both modes (decision 4
of grip-3a: the gripper moves in Simulation too, `--no-gripper` turns it off).

**Decision:**
- Settings `gripper:` replaced (sim.yaml = real.yaml): `enabled`, `port` (U2D2 by serial), `trigger: left`,
  `trigger_full`, `closed_ticks` 3968 / `open_ticks` 2196, `goal_current` 150 (max 300), `profile_velocity`
  200 (max 210), `profile_acceleration` 100, `catch_frac`, `min_voltage`, `max_temperature`, `fail_stop`.
  Checked: at least 50 ticks between open and closed, port not the leader's.
- `__main__`: the gripper read-only checks run with the leader checks (refused before the RB5 moves); its
  writes run after the ready pose, after a typed `yes`, right before the loop. At the end (also SIGTERM /
  SIGHUP; a second Ctrl+C is ignored during the cleanup): RB5 stop first, then the gripper torque off and
  mode 3, read back.
- Loop: the trigger of `gripper.trigger`'s handle (left: the left tool read that already runs for the
  rotate mode, so no extra leader read) -> `worker.set(trigger / 1000)`; never waits. Gripper events are
  shown; a stopped gripper thread does not stop the arm. `GripSignal` removed.
- CSV: `trigger_raw` (now the trigger value of the gripper's handle) + `grip_open_frac, grip_following,
  grip_goal, grip_position, grip_current_mA, grip_voltage_V, grip_hw_error, grip_age_ms, grip_stopped`
  (replace `grip_close`, `grip_state_closed`); the gripper thread's own rows in `<stem>_gripper.csv`.

**Not done here:** the Simulation comparison with and without the gripper (script 24) and a run on the
hardware. The gripper did not answer the read-only check at the time (no reply at any baud rate: supply
off?).

**Checked:** 73 tests pass (new: the trigger goes to the gripper thread independent of the clutch, a
stopped gripper does not stop the arm, blank columns without a gripper, settings relations).

## 2026-10-17 — Inner radius by height: r_min 350 / 300 / 250 at z 0 / 50 / 100

**Problem (user, Real run 20261017_140618):** the RB5 felt held back; it should come closer in along x.
The target sat on r_min (350 mm) for 33 % of the engaged time (8.8 of 27 s); the hand asked for r p10 235,
median 318 mm, at z p50 69 (133433: 30 % of the time, z p50 183).

**Why not just lower r_min:** low down the folded arm brings J3 to its limit (+/-165). J3 margin, DH IK
from the ready pose with its tool orientation (yaw follow_base, so the same at every azimuth):

| z \ r | 250 | 300 | 350 |
|---|---|---|---|
| -50 | 2 (stop) | 9 (stop) | 17 |
| 0 | 6 (stop) | 12 (slow) | 19 |
| 50 | 12 (slow) | 17 | 23 |
| >= 100 | >= 18 | >= 22 | >= 28 |

J5, shoulder and elbow margins are not limiting for r >= 250.

**Decision (user):** `range.r_min_by_z: [[0, 350], [50, 300], [100, 250]]` with `r_min_mm: 250` (linear
between the points, 350 below z 0, 250 above z 100; never below r_min_mm), in sim.yaml and real.yaml.
Along this boundary (5 mm steps): J3 margin >= 16.7 deg (no slowdown), elbow >= 31.7 deg, shoulder
>= 136 mm, J5 >= 86 deg.

**Implementation:** `CylinderRange.r_min_at(z)` (like `r_max_at`), used by the clamp, the rotate circle
(1 mm outside r_min at its height), the start-pose check and the report's r_min split (script 24).
Settings: heights increasing, each r in r_min_mm..r_max_mm, inner below outer at every table height.

**Effect to know:** going down while close in pushes the target outward (the inner radius rises).

**Rejected:** r 200 above z 150 (J3 22 deg, possible in the model) because r 250 is already close to the
robot base and the model has no robot body; one lower r_min for every height (stops low down).

**Checked:** 74 tests pass (inner radius clamp, settings relations). Not run on the robot yet: Simulation
first, then Real with a slow first approach to check the gap to the base by eye.

**Fix after the first runs (2026-10-17, user approved):** in Sim 20261017_142123 J2 went over its cap
(60 deg/s; up to about 73 deg/s from 20 ms samples, 1.69 % of the samples over 1.05 x cap) while going down
fast along the sloped inner radius. Cause: `plan_step` applies the speed, joint speed and acceleration
limits, then the final `cyl.clamp` pushes the target outward, and that push was not checked. The region
is not convex at the z 0 corner (the straight step between two inside points crosses the inner radius),
so the push is large there; the old fixed r_min had the same gap with a smaller push. Now the clamped
step is checked against the joint speed and the speed caps; if it is over, the step is shortened by
bisection (8 steps) until the clamped one fits (`joint_speed_limited` set). Replay of the run's desired
positions through `plan_step` with an ideal robot: 6 cycles over a cap (J2 1.24 x) before, 0 after;
Real 142316 unchanged (0 before and after); plan time unchanged. Test: the Sim case from real.yaml.
Real 142316 itself: r_min 20 % of the engaged time (was 35 %), none above z 100, r down to 261;
joint speeds within the caps.

## 2026-10-17 — Gripper: longer reply timeout on its port (thread switching); a bit faster (210 / 200)

**Problem (Real 20261017_143135):** the gripper thread stopped at 66.2 s after 3 failed state reads in a
row (the hardware error read of the same cycles answered); the arm continued as decided (grip-3). Range
part of that run: r_min 3 % of the engaged time (was 20 %, 35 %), joint speeds within the caps.

**Cause (read-only test, the gripper did not move):** a thread reads the state at 50 Hz while the main
thread runs `plan_step` like the teleop loop. Without the planning: 0 of 3001 failed; with it (phases
aligned, the worst case): 2859 failed. Python switches threads every 5 ms (GIL); dynamixel_sdk waits
about 4.3 ms for a reply (latency_timer 1). The 32-byte state reply comes in two USB pieces; if the loop
is planning in between, the reply is already there but counted as a timeout. The 12-byte error reply
comes in one piece. Leader joint reads: 0 failures in every run (they run in the main thread).

**Decision (user approved):** the gripper port's packet timeout is 10 ms longer
(`REPLY_TIMEOUT_EXTRA_MS`, `add_reply_margin` in `teleop/gripper.py`); the leader port is unchanged. Cost
only when the motor really does not answer. Same test after the change: 0 of 3001 failed (+6 ms and
+12 ms by hand: 0 too). Rejected: a shorter thread switch interval for the whole process (affects every
thread, more switching); Return Delay 0 alone (0.5 ms, not enough).

**Gripper a bit faster (user):** profile velocity 200 -> 210 (the motor's Velocity Limit; more needs a
motor setting change) and acceleration 100 -> 200 (0 -> 48 rpm in about 0.07 s instead of 0.13 s), in
configs/*.yaml and script 28. Measured: top speed at 10.5 V only 38-42 rpm (so the velocity raise helps
little; 12 V is the bigger gain), and the current while speeding up was 212-337 mA, below the 504 mA cap,
so the acceleration setting limited the start.

**Late cycles in 143135:** 9 (5 single, 4 within 63.45-63.65 s while planning took 9-11 ms instead of
about 5 and the gripper thread slowed too); not at the range limit; cause not found. Watch next runs.

## 2026-10-18 — Ready pose turned to azimuth -83 deg (workspace moved to the side)

**Request (user):** the workspace moved to the side; the home (teleop ready pose) should be where the robot
is now. Read-only state: jnt_ref (81.740, -10.618, -148.051, 74.914, -87.178, -303.121), DH TCP r 352,
z -52, azimuth -83.3, the same wrist shape as the 2026-10-16 ready pose.

**Why not that pose as is:** it is at the inner / floor corner of the range (2 mm to r_min, tcp_ref z
-49.9 with the floor at -50: the start check would pass by 0.1 mm), J3 limit margin 16.9 deg (slowdown below
15), and the first hand motion down or in would be clamped (the 2026-10-16 reason to move the pose out).

**Decision (user, option A):** the 2026-10-16 ready pose turned by J1 only to azimuth -83.25 deg:
(85.116, 4.300, -119.566, 31.666, -87.544, 53.521) in sim.yaml and real.yaml. The tool turns with J1
(follow_base), so the margins are the same: J5 87.5, shoulder 336 mm, elbow 60.4, J3 limit 45.4 deg;
DH TCP (52.9, -446.9, 152.3), r 450, z 152, 200 mm to r_min and the floor. From the current pose the move
is about 9 s, largest change J4 -43 deg, no wrist warning, start checks pass (computed). J6: the move
picks the nearest equivalent, so from -303 it goes to -306.5 (the wrist stays one turn wound as now).

**Rejected:** option B (r 420, z 60 at the same azimuth: closer to the working height but less margin,
J3 33 deg); the current pose itself (above).

**Open:** the azimuth range (-160 .. 133, absolute) was set for the old workspace; from the new ready pose
there are 77 deg to the minus side and 216 to the plus side. Recentre it if the side workspace needs more
room on the minus side (Real limit change: its own decision). The ready pose is the start of every
demonstration: fix it before recording (camera framing too).

## 2026-10-19 — Gripper Goal Current 150 -> 200 (faster start; grip force +33 %)

**Analysis (Real 20261019_084105, gripper only; 20261017_144815):** trigger -> gripper starts moving
about 40 ms (loop sampling 0-20 ms, leader read about 7 ms, gripper thread p50 12 ms, motor). A full
stroke (141-156 deg) pulled in 80-140 ms reaches 90 % after about 640 ms and the end after 700-740 ms. Top
speed 42 rpm = the motor at about 10.5 V; while speeding up the current was 474-504 mA = the Goal Current
cap (504 mA), so the cap (not profile acceleration 200) set the start. The felt delay is the motion time.

**Decision (user, option 3 of: smaller opening, 12 V, higher Goal Current, write on arrival, return delay
0):** Goal Current 200 (0.67 A) in configs/*.yaml and script 28 (still within the 300 schema limit). In
mode 5 the same value caps the gripping force, so it grows by a third: the user watches the printed
gripper and says at once if it is too strong. Expected gain: tens of ms at the start; the top speed
stays voltage-bound (12 V is still open).

**Result and revert (Real 20261019_095845, user approved):** full strokes 90 % at 640-660 ms, end 700-720
ms, start 40 ms: the same as at 150, so no measurable gain; the supply dipped to 9.5 V (the motor's minimum
input voltage; 2.7 s below 10 V) against 9.8-9.9 V at 150. Back to 150 in configs/*.yaml and script 28.
The user's finding from pick and place: the grip force itself is too weak, more than the speed.

## 2026-10-19 — Ready pose = the user's pose with J6 set for the gripper camera

**Request (user):** the robot was left with J6 where the camera (mounted on the gripper body) sees best;
use that pose as the home. Read-only state: jnt_ref (84.898, 1.808, -116.764, 24.718, -90.463, -175.743),
tcp_ref (67.2, -499.4, 146.2, rpy 89.7, -0.5, 179.2).

**Decision (user, method 1):** `ready_pose.joints_deg` = that pose in sim.yaml and real.yaml. J5 -90.5
makes the J6 axis vertical (within 0.5 deg) with the gripper hanging along it, so J6 turns the gripper
and its camera about the vertical. DH TCP r 504, z 144, azimuth -82.3: room in 254, out 296, down 194, up
321 mm; margins J5 89.5, shoulder 355 mm, elbow 63.2, J3 limit 48.2 deg; start checks pass (computed).
From the 2026-10-18 pose the move turns J6 +130.7 deg (wrist warning) and takes about 27 s.

**J6 during the teleop: kept as is (yaw follow_base).** The camera is on the gripper body, so it turns
with J6 and always sees the gripper the same way; the teleop keeps the start tilt (J5 stays within about
1 deg) and J6 moves by up to about 10 deg over a run, because J1 differs from the TCP azimuth by the
110.7 mm shoulder offset (d4) and J6 makes up the difference (run 105154: correlation -0.94). Rejected:
yaw tied to J1 (J6 fixed; only needed for a camera before J6) and a yaw fixed in the room (J6 would turn
as much as J1).

## 2026-10-19 — Gripper grasp squeeze: goal past the stop point so the grip force is the Goal Current cap

**Problem (user, pick and place):** the grip is too weak, more than the speed. **Cause (Real 095845):**
with the trigger released (goal 3968) a thin object stopped the fingers at 3951-3968, 17 ticks (1.5 deg)
short. In mode 5 the current is the position error x the position gain (P gain 850, about 21 mA per tick
here) and Goal Current only caps it, so the holding current was 363 mA (p50) against the 672 mA cap; that
is why Goal Current 200 did not grip harder.

**Decision (user, option A):** `GripperWorker` squeezes when an object stops the fingers: the goal
written becomes the stop point + `gripper.squeeze_ticks` (150 ticks = 13 deg) toward closed, so the error
is large and the current goes to the Goal Current cap (the configured force, 0.50 A at 150). It ends when
the trigger goal opens back to the object (then it follows the trigger) or when the fingers close on to
the squeeze goal (the object is gone; then it follows the trigger goal again). Without an object the
fingers reach the goal and nothing changes (no constant squeezing of the empty printed fingers).
Settings `gripper.squeeze_ticks` (0 = off, max 300); script 28 uses the same 150. Mode 5 takes goals past
4095 (e-Manual: Goal Position +/-1,048,575 in modes 4 and 5, position limits not used, Present Position
reset to one turn only when switching to mode 3 or on reboot).

**Detection:** stopped (|velocity| < 1 rpm) for 3 cycles, the motor's profile finished (Position
Trajectory 140 within 2 ticks of the goal written) and the position at least 8 ticks behind it. A first
rule with the goal alone, replayed on 095845 / 143135, fired at the start of moves (the motor starts about
40 ms later and accelerates, so it looks stopped short) and while following a slow trigger. The e-Manual
calls 140 the "desired position trajectory from profile" and says mode 5 computes its current from the
desired trajectory, but does not state outright that 140 is updated in mode 5: if it is not, condition
(profile finished) fails and the gripper simply never squeezes (test). It is logged
(`position_trajectory`, worker CSV) to check on the hardware. The state tuple has a 6th value (140).

**Rejected:** closed_ticks set inside the real closed position (the empty fingers would press at the cap
whenever the trigger is released); a higher position P gain (a motor setting, changes every motion).
Friction pads on the fingers: recommended in addition (mechanical, user).

**Checked:** 83 tests pass (object -> squeeze, empty close -> none, trigger opened -> release, object
gone -> back to the trigger goal, no squeeze while the motor is starting, none without a profile value,
off with 0). Not run on the hardware yet (the motor did not answer the read-only check: supply off).

## 2026-10-19 — Gripper: Goal Current 250 only while grasping; squeeze 150 -> 60 ticks

**First teleop run with the squeeze (Real 20261019_103804):** Position Trajectory (140) is updated in
mode 5 (while moving it leads the position by about 23 ticks, at rest it equals the goal). One grasp in the
run, detected correctly; no false detections at the starts of moves, none missed. The object stopped the
fingers at 3949 (19 ticks short) already at 437-457 mA; the squeeze raised it to the cap (501-504 mA). So
the weak grip is mainly the 0.50 A cap itself (and friction). When the object slipped, the fingers closed
to 4098, 130 ticks past the hand-measured closed position (3968 is not a mechanical end), then the grasp
ended and the goal went back to 3968. The supply still dipped to 9.5 V at the start of moves at 150.

**Decision (user, A + B):**
- A: `gripper.grasp_current` 250 (0.84 A): `GripperWorker` writes Goal Current 250 when a grasp starts and
  goal_current (150) when it ends, before the goal of that cycle. Moves keep the 150 cap, so the start
  current spikes (which dipped the supply) stay as they were; holding draws a steady current (supply
  model from the logs: about 10.5 - 0.25 V/A, so about 10.3 V at 0.84 A). Settings: at least goal_current,
  at most 300. Worker CSV column `goal_current`.
- B: `squeeze_ticks` 150 -> 60: still far above what reaches the cap (about 22 mA per tick, so 60 ticks
  ask for over 1 A), and an object that slips lets the fingers close 60 ticks instead of 150 before the
  grasp ends (then the cap goes back to 150 too).
- Script 28 uses the same values (`--current` at most the grasp current).
- 12 V supply and friction pads stay open (hardware, user).

**Checked:** 84 tests pass (the cap switches 150 -> 250 -> 150 around a grasp, written before the goal;
settings relation).

## 2026-10-19 — Gripper closed for now (grip-5): state and known limits

**Decision (user):** stop tuning the gripper; LeRobot setup comes first. Reopen when a problem shows up.

**State (configs/*.yaml, script 28 the same):** MX-106(2.0) on its own U2D2 (FT62AJAX), mode 5, left
trigger 0 = closed (3968) .. 1000 = open (2196), catch at start, gripper thread at 50 Hz with a 10 ms longer
reply timeout, profile velocity 210 / acceleration 200, Goal Current 150 (0.50 A) while moving, grasp
detection (motor profile 140 reached, position 8+ ticks behind, stopped 60 ms) -> Goal Current 250 (0.84 A)
and goal = stop point + 60 ticks until the trigger opens back or the object is gone. Torque off and mode 3
at the end. Arm teleop continues if the gripper thread stops.

**Measured:** trigger -> motion about 40 ms; full stroke (156 deg) about 0.7 s (top speed 42 rpm, set by
the 10.5 V supply); no read failures since the timeout change; grasp detection correct in the runs so far
(no false detections at move starts).

**Known limits (not fixed):**
- Grip force under squeeze is eaten by finger / object compliance: Real 105914, the fingers went 46 ticks
  further, leaving 14 ticks of error, about 300 mA instead of the 840 mA cap. Fix options when needed: a
  larger squeeze (about 130) or a goal that follows the position while grasping (preferred).
- Very small or thin objects stop the fingers only 2-4 ticks short: not detected, about 90 mA (Real
  105557). Finger pads (thickness, friction) are the simplest fix.
- The fingers can close to about 4098 (130 ticks past the hand-measured 3968) when an object slips; the
  real contact position is not measured.
- Supply 10.5 V at rest, dips to 9.5-9.7 V at the start of moves (the motor's minimum is 9.5 V): a 12 V
  supply is still needed.
- Return delay 500 us -> 0 (motor setting, ask first) not done.

## 2026-10-19 — rec-1: demonstration format (30 Hz; action x, y, z, gripper; state 6 joints + gripper; keyboard episodes)

**Decision (user, rec-1):** the four recommendations below, plus the clutch state in the raw log and a
converter option that cuts long clutch-released pauses (default: keep them).

**Pipeline:** the teleop records raw data at the native rates (robot 50 Hz, cameras 30 fps), every
sample stamped with the same monotonic clock. A converter builds the LeRobotDataset afterwards, so the
dataset rate and contents can be changed later without recording again.

| item | decision | reason | rejected |
|---|---|---|---|
| dataset rate | 30 Hz (robot data resampled by timestamp) | each 30 fps camera frame used once; the common LeRobot setting; smaller | 50 Hz (ALOHA): camera frames duplicated 1.7x, video resampled to 50 fps |
| action | `[x, y, z, gripper]`: the sent TCP target in base-frame mm (absolute) + the trigger command `open_frac` (0 = closed, 1 = open) | the exact values our safety pipeline (range, speed and joint caps, move_servo_l) executes; no model error, no delay estimate; the gripper thread applies the same grasp logic at run time | joint action (jnt_ref 60 ms ahead): needs move_servo_j and joint-space checks; deltas: drift; yaw: follows x, y with `follow_base` (rx, ry fixed), so it is redundant (kept in the raw log) |
| state | 6 measured joints (`jnt_ang`) + gripper position normalized 0..1 = 7 | measured values; include the wrist pose; ALOHA style | measured TCP: DH model error (about 2 mm) and no wrist information. Gripper current / grasp flag, TCP, tcp_ref, jnt_ref: raw log only |
| images | `front` = D435 fixed above the workspace looking down (position set); `wrist` = D405 on the gripper body; RGB 640x480 @ 30 | the ACT default; both cameras tested in cam-1 | depth: large, not needed now |
| episodes | **changed 2026-10-19 (user): LeRobot's keys.** Recording / reset phases. → or `n`: end the episode (kept) / end the reset and start the next episode; ← or `r`: discard the episode being recorded, or in the reset the one just finished; Esc or `q`: end the session, **an episode in progress is discarded** (LeRobot keeps it). The first episode starts with → (not at launch). Episode limit 60 s (then ended and kept), reset without a time limit | the operator knows LeRobot's keys; the leader buttons are all in use | own `s` / `e` / `d` keys |
| between episodes | **changed 2026-10-19 (user): no home move.** In the reset phase the operator moves the robot with the teleop to a start position of their own choice near the ready pose (about +/-10 cm), gripper open. The start joint pose of each episode is recorded | varied start poses make the policy less tied to one start; the run starts from the ready pose, so it must lie inside the spread; no new motion code | `h` key with move_j to the ready pose |
| clutch released | kept in the dataset (robot still, action constant): the policy learns to wait. The raw log has the clutch state; the converter can cut released stretches longer than a set time (off by default) | user | always cut |
| task text | one per dataset (e.g. "pick the block and place it in the box") | | |

**Checks later:** the 30 Hz action stream interpolated to the 50 Hz servo loop must be smooth (rec-4,
replay in Simulation); CPU, disk, USB and loop timing with two cameras + teleop + gripper (after the
D405 is mounted). Real demonstrations start after the D405 is mounted: the camera set must not change
inside a dataset. The converter's clutch-pause cut also covers the still tail at an episode end (the
operator lets go of the leader to press a key).

## 2026-10-19 — rec-2: separate conda env lerobot (LeRobot 0.6.1, Python 3.12); dummy dataset with the rec-1 format

**Decision:** LeRobot lives in its own conda env `lerobot` (`environment_lerobot.yml`): Python 3.12,
`lerobot[training,intelrealsense,dataset-viz]==0.6.1`, ffmpeg 7.1.1 from conda-forge. The teleop stays
in `rb5` and only writes raw files; the converter (rec-4) and training run in `lerobot`.

**Reason:** LeRobot 0.6 needs Python >= 3.12 (rb5 is 3.10) and pins torch, numpy and pyrealsense2
(2.56.5; rb5 has 2.58.4). Keeping the teleop free of these avoids changing a tested real-time loop.
rby1-lerobot needs `lerobot>=0.4`; we do not use its plugins (own teleop), only its pattern.

**Rejected:** LeRobot 0.4.x on Python 3.10 inside rb5 (older API; the newest dataset format and tools
are in 0.6); recording through `lerobot-record` with a robot plugin (decided earlier: our own loop).

**Checked (script 29, 2026-10-19):**
- Dummy dataset, 2 episodes x 90 frames + 1 discarded: read back 2 episodes / 180 frames, fps 30; state
  and action exact (error 0), images through AV1 video mean abs error 0.0035; task text kept.
- ACT action chunk (delta_timestamps, 100 steps) 10 frames before an episode end: shape (100, 4), 90
  padded steps, as expected.
- Saving an episode (video encoding of 2 cameras) took about 0.3x the episode length on the CPU.
- `lerobot-find-cameras realsense`: D435 943222070221 found (640x480 rgb8 30 fps default, USB 3.2). The
  D405 was not connected (being mounted).
- CUDA available in the env (torch 2.11.0+cu130).

## 2026-10-19 — rec-3a: camera recorder in its own process (JPEG per frame + time CSV)

**Decision:** `teleop/cameras.py` runs the cameras in a separate process (multiprocessing, spawn): per
camera a grab thread (streams all the time) and a writer thread (PIL JPEG quality 95). Episode files:
`cam_<name>/<index>.jpg` + `cam_<name>.csv` (index, frame number, t_capture_s, t_arrival_s, cam_ts_ms,
domain, write_ms). A discarded episode's camera files are deleted. Cameras: `front` = D435 943222070221
(fixed, above the workspace), `wrist` = D405 130322273474 (gripper body), 640x480 RGB @ 30.

**Reason:** a thread in the teleop process shares the GIL with the 50 Hz loop (grip-4 showed what that
does to short timeouts); a process does not, and time.monotonic is the same clock in both. JPEG: raw
frames would be about 55 MB/s for two cameras; the dataset video is lossy (AV1) anyway. t_capture_s
subtracts the receive delay (camera global_time vs PC wall clock): D435 about 34 ms, D405 about 9.5 ms,
so matching by arrival time would shift the cameras by about 24 ms against each other.

**Rejected:** a thread in the teleop process; raw .npy frames; mp4 encoding while recording (more CPU,
one lost frame cannot be found); LeRobot's camera classes (env lerobot, other pyrealsense2 version).

**Checked (script 30, 2026-10-19, both cameras on USB 3.2):**
- 2 x 20 s kept + 3 s discarded: 29.98 / 29.99 fps, interval max 33.4 ms, frame number gaps 0, queue
  drops 0; JPEG write p50 3.5-4.2 ms, max 13 ms; front 69 kB / wrist 59 kB per frame, together 3.9 MB/s
  (a 30 s episode about 115 MB, 50 episodes about 6 GB); discarded episode: no files left.
- Camera process 36 % of one core. A 50 Hz loop in the main process meanwhile: late (> 5 ms) 0 of 2150
  cycles, worst 0.3 ms.
- Before (script 25, 20 s): both cameras together 30 fps, no missed frames, nearest frames 12-17 ms apart
  (not synchronized).

## 2026-10-19 — rec-3b: --record in the teleop (LeRobot keys, sessions, episodes.json, 120 s limit)

**Decision:** `python -m teleop --record [--task ...] [--no-cameras]` (Simulation and Real). `teleop/episodes.py`:
`KeyReader` (terminal in cbreak mode in its own thread; → / n, ← / r, Esc / q, as rec-1 changed) and
`EpisodeRecorder` (recording / reset phases, called once per cycle after the send, never waits). One
session folder per run in `record.dir` (data/demos/<mode>_<time>/): teleop.csv with a new last column
`episode` (attempt number while recording), gripper.csv, settings.yaml (task as used), episodes.json
(rewritten at every change), episode_<attempt>/cam_*. Settings section `record` (dir, task,
episode_time_s **120 (user)**, reset_time_s 0 = no automatic start, front / wrist serials, jpeg_quality).

**Details:**
- Attempt numbers count every start; discarded attempts keep their CSV rows (marked by episodes.json)
  but their camera files are deleted. ← in the reset discards the last kept attempt (its files deleted).
- Cameras open before the RB5 mode switch, so a missing camera refuses the start before anything moves.
- `CameraRecorder.stop_episode` no longer waits (results through `poll()`): writing the last frames takes
  up to about 0.1 s, too long for a 20 ms cycle. Deleting a discarded episode also happens in the camera
  process.
- A camera error discards the episode being recorded and blocks new ones; at the session end the
  recorder waits (up to 10 s) for the camera process to close the files.
- CSV rows are tuples now: Python's garbage collector stops scanning tuples of strings. A one-hour
  session is about 180,000 rows kept in memory until the end; as lists, every full GC pass would walk
  them all inside the loop.

**Rejected:** recording in a separate program next to the teleop (two programs on one leader arm);
writing the robot CSV per episode during the loop (file writes inside the 20 ms cycle).

**Known limit:** the robot CSV is written at the session end (as before); a hard crash of the process
loses the robot data of that session (episodes.json and the camera files are on disk already).

**Checked:** 102 tests (episodes: keys incl. a pseudo terminal, phases, time limit, discard in the reset,
camera error, quit; loop: `episode` column, q ends the loop). Loop timing without the robot (the real
loop with the fake leader / robot of the tests, real 50 Hz sleep, 60 s, 3 episodes kept): without
cameras late 0, interval p99 20.16 ms; with both cameras recording late 0, interval p99 20.14 ms.

## 2026-10-19 — rec-3c: spoken messages while recording (spd-say, as LeRobot)

**Decision:** `record.sounds: true` (Sim and Real): `teleop/episodes.py` `Speaker` speaks "Recording
episode N", "Episode saved", "Time limit, episode saved", "Episode discarded", "Camera error" and, at the
end, "Stop recording" with `spd-say` (installed on this PC), in its own thread; a new message cancels one
still playing (`spd-say -C`). Without spd-say: a note at start, no sound.

**Reason:** LeRobot's lerobot-record speaks the same way (`log_say` -> `spd-say` on Linux); the
operator's eyes stay on the robot. A camera error is the message that matters most. The loop only puts
text in a queue.

## 2026-10-19 — rec-4a: converter (sessions -> LeRobotDataset, script 31)

**Decision:** `scripts/31_convert_to_lerobot.py` (env lerobot) builds one LeRobotDataset anew from the
given session folders (raw files untouched; no appending). 30 fps grid from each kept episode's start:
- action x, y, z: the sent target interpolated linearly at the grid times (send time = cycle start +
  work_ms); gripper: trigger / trigger_full while the gripper thread follows, else (before the catch)
  the position it holds, 0 = closed .. 1 = open (the policy's command then means what the gripper did)
- state: joints at their read time (cycle start + leader_read_ms - state_age_ms): jnt_ang in Real,
  **jnt_ref in Simulation** (jnt_ang there is the physical robot, not the simulated one); gripper
  position 0..1 at its read time (- grip_age_ms)
- images: nearest frame by capture time
- left out: discarded attempts; still episodes (clutch < 1 s or target path < 30 mm; `--keep-still`);
  `--cut-pauses S` (default off) removes clutch-released stretches > S s with an unchanged gripper
  command (seamless: the robot stands still there), keeping their first step

**Checked:** 7 tests on a synthetic session (values at their times, Simulation joints, gripper before /
after the catch, nearest frames, still episodes, pause cut). The Sim session sim_20261019_134705: 2
episodes kept (the still attempt 3 left out), image time error max 6.4 / 14.9 ms, resampling error of
the action 30 -> 50 Hz p50 0.00 / p99 0.43 / max 1.89 mm, dataset 7 MB for 19 s (raw JPEG about 60 MB);
read back with LeRobotDataset. Warnings shown: gripper never caught the trigger in that session.

**Rejected:** zero-order hold of the target (steps of up to one 20 ms cycle); the gripper command as
the raw trigger before the catch (the gripper did not do that); appending sessions to an existing
dataset (a full rebuild is reproducible and takes minutes).

## 2026-10-19 — rec-4b: replay in Simulation through the teleop limits (TargetFollower, script 32); rbpodo in env lerobot

**Decision:** `teleop/follow.py` `TargetFollower`: an absolute desired position (a dataset action, later
a policy output) goes through `plan_step` (range, speed caps with the watches, joint speed,
acceleration, lead) and yaw follow_base, as in the teleop, without the leader and the clutch.
`scripts/32_replay_episode.py` (env lerobot, **Simulation only**) replays one episode: move_j to its
first joints (typed yes), 50 Hz stream of the 30 Hz action interpolated linearly, then compares the
replay's jnt_ref with the recorded joints at the same episode time. The gripper action is not sent.
`rbpodo==0.16.14` added to env lerobot (cp312 wheel; user approved) so that the replay and the policy
runner run where torch / LeRobot are; the teleop stays in env rb5.

**Reason:** the replay checks that the action definition (rec-1) plays the motion back through the same
safety pipeline a policy will use; the follower is the policy runner's core. One process for policy and
robot is simpler than a policy server + robot client across two envs.

**Rejected:** sending the actions straight to move_servo_l (no range / speed checks: a policy's output
must go through them); a separate runner in env rb5 with the dataset exported to CSV (tests the export,
not the dataset).

**Checked (no robot):** 4 tests with the fake DH robot (reaches a target within the speed cap, stays
inside the range, the tool turns with the azimuth, refuses yaw modes other than follow_base); the
script loads episode 0 of local/sim_test (348 x 4 actions) and its interpolation. The Simulation run
itself: by the user.

**Simulation runs (user, 2026-10-19, local/sim_test):**

| | episode 0 (11.6 s) | episode 1 (7.5 s) |
|---|---|---|
| late cycles | 0 | 0 |
| end: tcp_ref vs last action | 0.00 mm | 0.00 mm |
| joints replay vs recording, max (J1..J4) | 2.2 / 1.4 / 1.6 / 1.6 deg | 0.1 / 1.6 / 2.0 / 1.5 deg |
| mean joint error, as is -> after the best time shift | 0.16-0.22 -> 0.04-0.07 deg (replay 30-60 ms later) | 0.02-0.33 -> 0.01-0.08 deg (40 ms later) |
| target vs desired p95 / max | 21 / 22 mm | 10 / 10 mm |
| limit flags (cycles) | accel 239, speed 216, range 1 | accel 156, speed 127, range 1 |

The path is reproduced; the difference is mostly a delay of 30-60 ms. Cause: the 30 Hz action,
interpolated linearly, has its velocity changes bunched at the 30 Hz samples; at those points the
acceleration limit trims the step, and while the motion is at the speed cap (the recording moved
there) the target cannot catch up, so it falls behind by up to 22 mm until the motion slows down.
Accepted for now (the limits stay; the end position is exact). If a policy's motion turns out too
sluggish: a smoother interpolation (e.g. cubic) between the 30 Hz actions.

## 2026-10-19 — Real pilot 141929: the D405 dropped off USB; camera stall detection; converter coverage check, --cameras

**Pilot (user, Real, `--record`, task "test01"):** 238 s, 6 episodes kept (19-36 s). Loop: late 0, interval
p99 20.23 ms, joint speed and range limits 0 %. Gripper trigger used 0..1000, following 100 %, a grasp
detected in episode 5. Front camera: 1 frame lost. Start poses 60-175 mm from the ready pose. Leader
tool reads failed 1.1 % (longest run 11 cycles at 39 s; earlier runs 0.5-0.9 %). TCP down to z 14 mm
(range.z_min_mm is still -50).

**Problem:** at 14:20:52 (19.5 s into episode 1) the kernel logged a USB disconnect of the D405; it came
back at once but as a USB 2 device (480M), so its pipeline only timed out. `_grab` skipped timeouts
silently: episodes 1-5 have no wrist frames after that point and nothing was reported.

**Decisions (user approved):**
- `teleop/cameras.py`: no frame for `STALL_S` = 1 s -> a camera error (once); the recorder then discards
  the episode being recorded, blocks new ones and says "Camera error".
- Converter: an episode whose frames of a camera do not cover it (no frames, start or end missing by more
  than 0.2 s, a gap over 0.2 s) is left out with the reason. `--cameras front` makes a front-only dataset
  (tests only; the rec-1 format has both).
- Hardware (user): D405 cable strain relief, a good USB 3 cable, fewer extensions / hubs; check with
  `lsusb -t` (5000M) and script 30 while moving the gripper.

**Result on the pilot:** both cameras: only episode 0 (the others left out: wrist frames end at 19.5 s /
none). Front only: 6 episodes, 4794 frames (2.7 min) -> `local/pilot_front` (37 MB) for the rec-5 flow
test. Conversion took 176 s (JPEG decoding + AV1 encoding of real images, about 1.1 x real time per
camera; the 0.3 x of rec-2 was synthetic images): 50 episodes x 30 s x 2 cameras about 55 min.

## 2026-10-19 — rec-5: ACT trial training on the pilot (front only): the flow works; inference 17 ms GPU

**Run:** `lerobot-train --policy.type=act --policy.device=cuda --policy.push_to_hub=false
--dataset.repo_id=local/pilot_front --batch_size=8 --steps=2000 --save_freq=1000 --wandb.enable=false`
(ACT defaults: chunk 100, ResNet18 ImageNet weights, VAE, lr 1e-5; 52 M parameters). Data:
`local/pilot_front` = Real pilot 141929, 6 episodes, front camera only, 4794 frames.

**Training:** 2000 steps in 10.5 min, 0.307 s per step (data loading 0.002 s: the GPU is the limit),
GPU memory 2.1 GB, about 3.3 epochs. Loss 10.47 (step 100) -> 1.15 (step 2000), L1 0.64 -> 0.20, KL 0.98
-> 0.10, still falling. Checkpoints at 1000 and 2000, about 600 MB each (model + optimizer state).

**Offline check (script 33, last checkpoint, 200 frames of the training data):** predicted position vs
the recorded action p50 / p90: +1 step 28 / 63 mm, +30 steps 30 / 71 mm, +99 steps 42 / 100 mm;
gripper +1 step 0.04 / 0.20. Underfit after 2000 steps on 6 episodes, as expected; it shows that loading,
normalization, chunk prediction and unnormalization work end to end. Not for the robot.

**Inference time (batch 1, the whole chunk of 100):** GPU 17.3 ms p50 (18.4 max) with one camera, CPU
132 ms p50; GPU with two 480x640 cameras (untrained ACT, model only) 32.5 ms p50 (33.8 max); GPU memory
0.5 GB.

**Consequences for the policy runner (rec-6, to plan):** one inference (about 33 ms with both cameras)
is longer than a 20 ms servo cycle, so it must run outside the servo loop (its own process, as the
cameras), with the servo loop interpolating the latest chunk and never waiting. A chunk covers 3.3 s, so
a new chunk every few hundred ms is enough; inference every 30 Hz step (temporal ensembling) would use
the GPU all the time with two cameras. Full training estimate with both cameras: to be measured (the
ResNet part doubles; roughly 0.5 s per step -> 50 k steps about 7 h).

## 2026-10-21 — Data rules for the first ACT dataset (docs/data_rules.md)

**Decisions (user, all as recommended):** task "pick the green block and place it in the box"; the green
block; block positions random inside a marked 20 x 20 cm square, orientation fixed; start poses within
about 10 cm of the ready pose, gripper open; an episode ends after placing, opening and lifting about
5 cm; 50 kept episodes in sessions of 10-15; camera exposure and white balance fixed; evaluation at 5
positions (corners + center) x 4 trials, success = block in the box and gripper open within 60 s.

**Reason:** a narrow, consistent task for about 50 demonstrations (ACT learns a small range well); the
gripper stays vertical (no orientation change); the block is thick and rigid enough for grasp detection;
fixed camera settings so the images do not change with the lighting; an evaluation fixed before
collecting so every policy is compared the same way.

**Open (before the real collection):** the exposure / white balance values and the code that sets them
(recording and policy runner); `range.z_min_mm` for the mounted gripper; D405 cable; D435 position.

## 2026-10-21 — cam-2a: fixed camera exposure candidates measured (script 34)

**Facts:** both color sensors support auto exposure / auto white balance off, exposure, gain, white
balance (2800-6500 K). Exposure units: D435 RGB 0.1 ms (1-10000), D405 stereo module us (1-165000); gain
D435 0-128, D405 16-248. The options are not stored in the camera (a replug returns to auto), so the code
must set them every time it opens a camera and read them back. With auto, the D405 used about 32 ms of
the 33 ms frame (motion blur on the moving wrist camera: about 8 mm at 250 mm/s); the D435 does not
report its actual exposure (metadata missing without the kernel module).

**Measured 2026-10-21 (lab lighting, gain searched to match the auto brightness):**

| camera | setting | mean (0-255) | noise | R/G, B/G |
|---|---|---|---|---|
| front D435 | auto | 113.9 | 1.74 | 0.909, 0.969 |
| | 8.3 ms, gain 128 (max) | 101.8 (too dark) | 2.58 | |
| | **16.7 ms, gain 76** | 114.0 | 1.95 | 4600 K: 1.091, 1.203; 5200 K: 1.141, 1.137 |
| wrist D405 | auto (about 32 ms) | 114.7 | 1.35 | 0.947, 0.986 |
| | **8.3 ms, gain 61** | 114.3 | 2.51 | 4600 K: 0.984, 0.998 |
| | 16.7 ms, gain 31 | 116.8 | 1.81 | 4600 K: 0.990, 1.007 |

Exposures are multiples of 1/120 s (60 Hz mains: no flicker). The D435 does not reach a neutral gray
with any tested white balance (green is low: slight magenta); the D405 is neutral at 4600 K. Values to
choose: next entry (cam-2b).

## 2026-10-21 — cam-2b: fixed camera settings in the teleop; brightness check at start

**Decision (user):** front D435 16.7 ms / gain 76 / 4600 K, wrist D405 8.3 ms / gain 61 / 4600 K (script 34,
2026-10-21), in `record.front_*` / `record.wrist_*` (sim.yaml = real.yaml). `teleop/cameras.py` switches
auto exposure and auto white balance off and sets these values every time it opens a camera, reads them
back (a mismatch refuses the start, before anything moves), then measures each camera's mean brightness
(10 frames after 1 s). Outside `brightness_ref` 114 +/- `brightness_tolerance` 15 % (97..131): a warning and
a typed yes (user); no -> the teleop stops. episodes.json `cameras`: serial, settings as read back,
brightness, reference, tolerance, ok, and whether a typed yes accepted it. Script 30 uses the same values
(`--auto` for auto). Same-lighting item added to the data collection checklist (docs/data_rules.md).

**Reason:** the options are not stored in the camera (a replug returns to auto), so they must be applied
and checked at every open; with fixed exposure the image brightness follows the room lighting, so a
changed lighting must be noticed before recording.

**Checked:** tests (fake cameras: settings reported, read-back mismatch refuses, brightness limits,
episodes.json). Real cameras (script 30): read back front exposure 16.7 ms (167 x 0.1 ms) / gain 76 /
4600 K, wrist 8.333 ms / 61 / 4600 K; brightness 114.6 / 114.8. 10 s recording: 29.98 / 29.89 fps, front 0
gaps, wrist 1 missed frame (the D405 cable is not fixed yet); JPEG 82 / 69 kB per frame (higher gain, more
noise: 4.5 MB/s together, was 3.9). The options stay set after the program ends until a replug.

## 2026-10-21 — range.z_min_mm -50 -> 19 mm for the mounted gripper

**Decision (user):** `range.z_min_mm` 19.0 in real.yaml and sim.yaml (frozen settings, so this entry).

**Measurement:** the user put the gripper (fingers) at the lowest height it may reach above the table, at
r 485 mm, azimuth -82.3 (below the ready pose), gripper vertical (rx 89.73, ry -0.45). Read only from the data
channel: tcp_ref (89.73 / -0.45 / 179.16 rpy) x 64.65, y -480.24, **z 18.93 mm** (no change over 20 reads);
jnt_ref = jnt_ang (84.33, -7.75, -129.76, 47.27, -90.47, -175.18). The DH model gives z 16.94 from the same
joints (the known about 2 mm difference); the range acts on the targets in the controller's TCP frame, so
the tcp_ref value is the one used. Options were 19 (chosen), 20 (1 mm margin, recommended) and 23.

**Checks:** settings load (Real and Sim); r_min at z 19 is 331 mm (r_min_by_z unchanged), r_max 800; the
ready pose (z 144) is inside; 123 tests pass. The pilot had grasps at TCP z 14-30 mm (one at 14.2, now
below the floor). Measured at one place only: grasps in the pilot at r 667-693 happened at z 18-22, so the
table there is not higher. The floor is a software limit (not safety rated) and does not protect the box
walls; tool mass and TCP offset on the controller are still 0 kg / provisional.

## 2026-10-21 — rec-6 policy runner: decisions (two processes, temporal ensembling by time, 100 % speed, keyboard toggle)

**Decisions (user):**
1. **Two processes:** a policy process (both cameras opened with `teleop/cameras.py`'s fixed settings, ACT
   on the GPU) and the servo process (RB5 at 50 Hz, gripper thread). The servo loop never waits for the
   policy.
2. **Inference every 30 Hz step with temporal ensembling** (not the recommended new chunk every 0.2 s
   without ensembling). One inference with both cameras is 32.5 ms p50 / 33.8 ms max, about one 33.3 ms
   step, so not every step can be met: the ensembling is done by time in the servo process (every chunk
   carries its observation time; at time t the actions of all recent chunks that cover t are averaged with
   ACT's weights exp(-0.01 i), oldest first), so a late or skipped inference only means one chunk fewer.
   LeRobot's ACTTemporalEnsembler (one inference per step, synchronous) is not used. The GPU is busy
   nearly all the time: inference time and laptop GPU throttling to be measured in rec-6b.
3. **Speed caps 100 %** of the teleop (not the recommended 50 %).
4. **Keyboard toggle** for running / holding (not the recommended leader-button deadman): space = hold at
   once, start = a key after a typed yes showing the target and current pose. With 3, the policy moves at
   demonstration speed with nobody holding a button: hold on a stale chunk (0.5 s; 3 s -> stop), on a target
   over 100 mm from the current one, after 60 s; emergency stop within reach. Real runs need their own
   plan and approval; the speed can be revisited then.
5. **dynamixel-sdk 4.1.0 installed in env lerobot** (the gripper thread of `teleop/gripper.py`).

**Steps:** 6a structure with a fake policy and robot (no hardware); 6b real ACT + cameras + Simulation
(timing, chunk age, GPU); 6c gripper and the key controls in Simulation.

## 2026-10-21 — rec-6a: policy runner structure (teleop/policy.py) with a fake policy and robot

**What:** `teleop/policy.py`:
- `ChunkBuffer`: chunks with their observation time; `action_at(t)` = every recent chunk that covers t gives
  its action interpolated between its 30 Hz steps; weights exp(-0.01 i), i = 0 the oldest (ACT); chunks
  that ended before t are dropped; an older or repeated chunk is ignored.
- Policy process (`PolicyClient`, spawn): opens the cameras with `teleop/cameras.py` (fixed settings) and
  keeps the latest frame of each; at most once per 1/30 s: latest frames + the latest robot state (shared
  memory from the servo side) -> policy -> chunk (100 x [x, y, z, gripper]) with t_obs = the front capture
  time, inference time, counter; never waits (a full queue drops the chunk). Policy kinds: `replay` (dataset
  actions as chunks from the first observation, optional simulated inference delay); `act` comes in 6b.
- `PolicyRunner` (servo side, devices passed in): chunks and keys without waiting; the teleop's state checks
  (stale state, late cycles); TargetFollower (the teleop's limits) and the gripper thread while running;
  state published every cycle (Real: jnt_ang, Simulation: jnt_ref, as the converter). Keys: g = run (refused
  until a recent chunk exists), space = hold, q / Esc = quit (`KeyReader` takes a parser now). Holds on: a
  chunk older than 0.5 s, no chunk covering the time, an action over 100 mm from the target, 60 s of running.
  Stops when no chunk has arrived for 3 s or the policy process ended. CSV row per cycle (`RUN_HEADER`).

**Checked:** 13 tests (buffer interpolation, weights, ordering; keys; on a simulated clock with the fake DH
robot: the same target path as a direct replay of the interpolated actions; the same path with chunks
arriving 200 ms late or every second chunk missing; hold 0.5 s after the chunks stop and stop after 3 s;
space / g; a 150 mm jump holds; the time limit; g refused before the first chunk; the real policy process
with fake cameras). Real time (fake robot, real policy process with fake cameras, 10 s): simulated inference
33 ms -> late 0, interval max 20.2 ms, chunk age 68 / 96 ms (p50 / max), 97 chunks ensembled; 200 ms -> late 0,
interval max 20.3 ms, chunk age 330 / 410 ms, 15 chunks ensembled. The path follows in both; at the test range
top the target stopped and the action kept rising: the jump guard held at 101 mm, as designed.

## 2026-10-21 — rec-6b: train/run observations match; inference 33 ms on the GPU power limit; GC freeze; ACT runner script

**Observation check first (user):** `tests/test_policy_obs.py` (env lerobot) feeds frames of
local/pilot_front through the training path (dataset item, AV1-decoded image) and the run path (the JPEG the
recording saved = the camera's rgb8 array, `observation_batch()`, the same steps as LeRobot's
prepare_observation_for_inference), both through the checkpoint's preprocessor. Keys and camera count equal
the policy's input features (front, state); shapes (1, 3, 480, 640) / (1, 7), float32 for both; image range
0..1 in both; image mean difference 0.007 (video compression; R/B swapped would be 0.025) and the R - B maps
correlate 0.98 (swapped: -0.98); state names J1..J6 deg + gripper in the dataset order, values identical
(0 difference), inside the dataset range, degrees. The runner's gripper fraction uses the converter's formula
and the published joints are jnt_ref in Simulation / jnt_ang in Real (test in test_policy).

**Inference endurance (user):** script 33 `--endurance 5`: untrained ACT with two 480x640 cameras on the
runner schedule (1 / 30 s), 8829 inferences. p50 31.6 ms in the first 10 %, 33.1 ms in the last 10 % (+5 %,
reached after about 2 min, then flat); 20.6 % over one 33.3 ms step after the warm-up (one step in four or
five has no new chunk: the ensembling by time keeps going). The GPU sits at its 35 W power limit the whole
time (34.6-34.9 W), 58 -> 80 C, SM clock 1440 -> about 1100 MHz: the slowdown is the power limit, small.
Spikes 92-174 ms, 7 times, every 42.6-43.1 s: Python's full GC pass over the model's objects (about 80 ms;
GC callback times match, none with GC off). **Fix:** `gc.collect(); gc.freeze()` after the model loaded (policy
process) and before the servo loop (script 35): 130 s with GC on, full passes 0.1 ms, max inference 34 ms.

**ACT in the policy process:** kind `act` (`ACTChunkPolicy`: from_pretrained, the checkpoint's pre / post
processors on the GPU, `observation_batch`, predict_action_chunk); 3 warm-up inferences before "ready" (the
first took 233 ms). With the pilot checkpoint and a fake camera: ready in 6.8 s, 147 chunks in 5 s (100 x 4),
inference p50 24.9 ms, max 46.8 ms. The process also reports each camera's settings and start brightness
(the same check as the recording). Its first action from a gray image was 125 mm from the ready pose TCP:
with real images it may differ, and over 100 mm the runner holds (the script shows the distance first).

**Script 35 (Simulation only):** cameras the policy expects with the fixed settings and the brightness check;
Simulation mode and the teleop's checks; move_j to the ready pose (yes); the first chunk's distance (yes); the
PolicyRunner with keys g / space / q; log data/policy/run_sim_<time>.csv + .json. Not run yet (moves the
Simulation robot: the user runs it).

## 2026-10-21 — rec-6b fixes after the first Simulation runs: late check right before the send; servo and policy on separate cores

**Simulation runs (user, pilot checkpoint, front camera, 2 x 30 s running):** the structure worked (holds only
by the time limit; chunk age 102-104 ms p50, about 82 chunks ensembled; the robot moved some tens of mm near
the ready pose, no block reached: the pilot policy is undertrained, expected). Two problems: (1) while
running, the cycle interval reached 40-48 ms 22 and 15 times (a cycle's work over 20 ms, then a full period's
wait), and inference was p95 54-60 ms (offline 34 ms); (2) the runner counted those as late 0-1 times: it
checked the gap only before planning, not right before the send as the teleop does (the difference noted in
6a).

**Fixes (user approved):**
1. `TargetFollower.plan()` / `apply()` (step = both). `PolicyRunner` checks the gap before planning and again
   right before the send; a late cycle keeps the old target (the plan is not applied), 3 in a row stop.
2. Script 35 pins the servo process to cores 0-5 and the policy process to 6-11 (physical performance cores
   0-2 / 3-5 of the i7-13700H), PyTorch CPU threads 4 (`--servo-cores`, `--policy-cores`, `--torch-threads`;
   spec `cpu_cores`, `torch_threads`).
3. CSV columns `late_before_plan`, `work_ms`, `plan_ms`; script 35 prints work / plan times.

**Checked (real cameras + the pilot ACT + the fake robot, 30 s running):** unpinned with PyTorch on all
cores: work 26-37 ms per cycle even before running, stopped after 19 cycles (3 late in a row). Pinned: late 2
in 30 s (not consecutive), work p50 / p99 / max 4.1 / 9.2 / 20.6 ms (the fake robot's send costs a few ms
the real one does not), plan p99 3.0 ms, inference p50 / p95 / max 17.8 / 18.7 / 34.4 ms (one camera), chunk age
p50 / max 85 / 123 ms. Tests: a 35 ms plan is late and not applied; three in a row stop.

**Simulation run after the fixes (user, 043648, 23 s running):** cycle interval p50 / p99 / max 20.00 / 20.05 /
20.18 ms, none over 25 ms (before: 15-22 at 40-48 ms); work p50 / p99 / max 2.1 / 4.7 / 14.4 ms; inference
p50 / p95 / max 17.8 / 18.6 / 29.7 ms, none over one 33.3 ms step; chunk age p50 / p95 / max 84 / 107 / 114 ms; 96
chunks ensembled. One late cycle, handled as intended: a 10.8 ms plan (p99 3.3) put the send 14 ms into the
cycle, 33 ms after the previous send (over 30): that cycle kept the old target, the next was normal (the
cycle-start intervals stayed 20 ms; the check measures the gap between sends, as the teleop). Holds: none
but the start; the pilot policy again moved tens of mm near the ready pose (x 46..67, y -558..-499, z
113..146), as expected. The structure is ready for a real policy; next 6c (gripper).

## 2026-10-21 — rec-6c: gripper in the policy runner (shared start code, no catch)

**What:** the gripper start code moved from `teleop/__main__.py` to `teleop/gripper.py` so the teleop and the
policy runner use the same code: `open_gripper` (read-only checks), `start_lines` (what the writes do, shown
before the typed yes), `start_worker` (start_motor, GripCommand, GripperWorker; torque off again on a failed
start), `GRIPPER_DEADBAND`. The teleop behaves as before. Script 35: the read-only checks before anything
moves (refused if they fail; `--no-gripper` runs without it), the start writes after a typed yes at the ready
pose, the first observation with the gripper position, `PolicyRunner(gripper=worker)`, and at the end (after
the robot stopped) `worker.finish()` with the torque / mode read back.

**No catch for the policy (catch_frac 1.0):** the teleop's catch waits until the trigger comes within 5 % of
the gripper position (a human trigger must not make the gripper jump). A policy that keeps asking for 0.2
while the gripper is open would never be caught and the gripper would never move; the dataset's gripper
action is what the gripper did (the converter uses the held position before a catch), so the runner follows
it at once. It is sent only while running (after g) and not on a late cycle.

**Checked:** tests (the runner sends the policy's gripper action only while running, the state carries the
gripper position with the converter's 0..1 rule; catch 1.0 follows at once while the teleop's 0.05 holds);
all tests pass. The read-only check on the lab gripper: the motor did not answer (model None), as when its
supply was off before: to repeat with the power on. The Simulation run with the gripper: by the user.

**Run log (99ca8ae):** a `grip_pos` column (the gripper position 0..1 given to the policy, the same value) at
the end of the CSV, and a `gripper` entry in the run JSON (ticks at the check and the start, held position,
the first observation's value, stop reason, torque / mode read back).

**Simulation runs with the gripper (user):** 060447 and 070756 never ran: every g was held by the 100 mm
check (122-155 mm). Cause: the scene, not the code. Offline, with the ready-pose state, the policy's first
action follows the arm it sees in the front image (recorded frame: 13 mm from the ready pose; a live frame
with the real arm in another pose, the elbow in front of the camera and no block: 118 mm). In Simulation the
real arm does not move, so the image must show it at the ready pose. 071545 (real arm at the ready pose, block
in the square): first action 79 mm away; 30 s running (g at 36.1 s, space at 62.9 s, g again, time limit at
68.7 s). The target reached the policy's point (65, -553, 85) in about 2 s at the teleop speed cap (250 mm/s;
flags speed / accel on 21 cycles) and stayed there: the image never changes in Simulation, so the policy keeps
asking for the same point. The gripper followed the action (0.85-0.86) at once: 0.988 -> 0.85, mean
|grip_pos - grip_action| 0.001; read back torque 0 / mode 3. Timing: no interval over 25 ms (max 24.2), no
late cycle, inference p50 / p95 17.7 / 19.3 ms, work max 10.0 ms, plan max 6.2 ms. rec-6 is done in
Simulation. For Real (separate plan): the first action may be tens of mm away and is reached at the full
speed cap; consider a lower cap or a smaller jump limit for the first second.

## 2026-10-21 — p1: first round of the whole pipeline with the current setup; every step typed by the user

**Decision (user):** leave the real setup (D435 position, table marks) for later and go through
collect -> train -> run now with the current setup, improving the pipeline from what each round shows.
Every step is a command the user types; the commands are in `docs/pipeline_guide.md`.

**Round 1 (`p1`):** one Real session with both cameras, 15 kept episodes (data_rules.md way of
demonstrating, task "pick the green block and place it in the box") -> `local/p1` -> ACT with both cameras,
20000 steps, batch 8 -> script 33 offline check -> script 35 Simulation run. A practice dataset: not part of
the 50 of data_rules.md (the front camera will move, which changes the scene).

**Why these numbers:** 6 episodes / 2000 steps left the pilot policy at 28 mm (+1 step). 15 episodes give
several block positions (does the policy follow the block?); about 11000 frames at batch 8 make 20000
steps about 15 epochs, enough to tell too little data from too little training. Time to be measured
(two cameras, the pilot was 0.31 s per step with one).

**Known weak point to watch:** at the ready pose the arm hides the block from the front camera, so the
block position must come from the wrist camera. If round 1 shows it, it is the reason to move the D435.

**Not yet tried end to end:** both cameras through convert -> train -> run (the pilot was front only).

## 2026-10-21 — cam-3: a saturated camera image refuses the start (red, no typed yes)

**What happened (p1 collection):** session real_20261021_105618 (51 kept episodes) recorded the D405 (wrist)
saturated in every frame: mean 250-254, 98-100 % of the pixels white; the front camera and the robot data
were fine. The start check measured the wrist at 253.4 (outside 97..131) and a typed yes let it record.
The settings read back were the normal ones (8.3 ms, gain 61); opened again later the camera gave 115.5:
the cause is not known (camera state). With 100 ms exposure / gain 128 the D405 gives the same picture
(253.9, 98 %). A saturated image has no detail to recover, so the session (and the earlier 7-episode session
073857) went to data/archive/demos and the 50 episodes are collected again (user).

**Decision (user):** a saturated image must show a red warning and must not be passable with yes.
`teleop/cameras.py`: `image_level` (mean, fraction of pixels with R, G and B all >= 250), `level_info`,
`saturation_problems` (more than 25 % saturated, `SATURATED_MAX_FRAC`; normal images 0-2 %), `print_red`.
The start measurement (camera process and policy process) records `saturated_frac` with the brightness
(episodes.json, run JSON). The teleop (`--record`) and script 35 refuse in red before anything moves; script 30
prints it in red. The brightness range check with its typed yes stays for the smaller differences.

**Checked:** tests (fake camera at 253: 100 % and refused; one channel at 255 does not count; under 25 %
passes); real cameras: normal 119.7 / 0.7 % (front), 115.5 / 0 % (wrist); wrist at 165 ms / gain 128: 253.9 /
98 %, red text; script 35 with a config at front 100 ms / gain 128: 245.8 / 78 %, "NOT STARTING", exit 1. The
teleop's refusal was not run on the hardware (it opens the leader arm and the gripper first); same code path.

**Not covered:** a camera that saturates during a session (only the start is checked).

## 2026-10-21 — cam-4a: C270 webcam as the wrist camera (the D405 broke)

**What happened:** the D405 (wrist) broke physically (user). Before that, the kernel log showed it switching
between USB 3 and USB 2 and USB errors (-71, -110) through the VIA hub, also on the D435 next to it. The user
mounted a Logitech C270 webcam on the gripper instead and aimed it at the workspace.

**Decision (user approved the plan):** `wrist_serial` may be a device path: a value starting with `/dev/` opens
`UVCCamera`, a number a RealSense (`open_camera()`, used by the camera process and the policy process, so
recording and policy runs open the cameras the same way). Path `/dev/v4l/by-id/usb-046d_0825_6E259540-video-index0`
(stays the same after a replug).
- YUYV 640x480 at 30 fps (29.9 fps measured, no interval over 50 ms). Rejected: MJPG (same rate, but a
  "Corrupt JPEG data" warning on almost every frame in the terminal, and compression on top of our JPEG).
- Settings with `v4l2-ctl` (OpenCV has no control for the variable frame rate): auto exposure, variable frame
  rate and auto white balance off first (the values are inactive under auto), then exposure (0.1 ms steps),
  gain, white balance; read back after 0.5 s of streaming. While streaming the C270 reports the exposure
  0.5 ms higher than written (83 -> 88, 167 -> 172, every value tried), so the exposure may differ by 1 ms;
  gain and white balance must match. The settings stay in the webcam until it is unplugged: written at every
  open (the earlier "30 ms darker than 16.6 ms" test had left the gain at auto's 192).
- Times: the webcam has no clock of its own; OpenCV gives the kernel's buffer time (CLOCK_MONOTONIC ms,
  the clock of time.monotonic()), about 33 ms before the arrival with steady 32 ms intervals: domain `v4l2`,
  `capture_delay_s()` handles every domain. Frame numbers count frame periods between buffer times (a
  missed frame is a gap, as on a RealSense).
- Images converted BGR -> RGB (OpenCV order) like the RealSense rgb8 stream.
- rb5 env: `opencv-python-headless==4.13.0.92` (same as the lerobot env), installed by the user.
- Provisional wrist values: 8.3 ms (same blur as the D405 setting), gain 128, 4600 K. To be set with script 34
  in cam-4b.

**Checked:** tests (webcam controls, read-back tolerance, capture delay for each domain, a missing webcam path
refuses; fake cameras unchanged). Hardware, camera only: applied 8.8 ms / 128 / 4600; 150 frames in 5 s, 0 gaps,
capture interval 32.0 / 36.1 ms (p50 / max), delay 33.1 ms; green block green (RGB order right); both cameras
through the recorder: front 111.2 / 0.1 %, wrist 147.3 / 0 % (first open 131.8): the wrist is outside 97..131
with gain 128 -> cam-4b.

**Not done:** the webcam's own delay from exposure to the buffer time is not known (the buffer time is when the
first USB data arrives); the D405 rule of thumb no longer applies. Fingers are not in the wrist view yet.

## 2026-10-21 — cam-4b: wrist webcam settings (8.3 ms, gain 78); settings written once the stream runs

**Script 34** now takes the cameras from the config (a /dev/ path = webcam), `--camera`, and `--target` (the
mean to match instead of the auto one: the start check compares every camera with brightness_ref 114, and the
webcam's auto gives about 135 on the white table). User run (settings_20261021_132305, wrist, target 114):
auto 135.0; 8.3 ms needs gain 78 (114.6); 16.7 ms gain 12 (113). White balance 4000-5200 K hardly changes
R/G and B/G (0.955-0.977). **Decision (user):** 8.3 ms (blur, as for the D405), gain 78, 4600 K.
Rejected: 16.7 ms (less noise, twice the blur on the moving wrist).

**Problem found with script 30** (rec_20261021_132555): the wrist came out at 127.5, not 114. Reproduced
with fresh processes: the first open in a process gave 127 every time (8 of 8), a second open in the same
process 114; the read-back values were the same in both, and all the v4l2 controls too. Pixel values were
about 1.1 x (sensor side, the same ratio as 147.3 / 131.8 in cam-4a). Writing the settings after the stream
had started gave 114 every time. **Fix:** `UVCCamera` reads `UVC_START_FRAMES` (5) frames before it writes
the settings; no frame -> "gives no frames" refuses the start. The teleop and the policy process each open
the webcam once in a new process, so without the fix every recording and run would have been at about 127.

**Checked:** fresh processes 4 x 114.1-114.2; through the recorder process 114.1 / 0 %; tests pass. The read-
back check could not have caught this: the brightness measurement at the start is what shows it.

## 2026-10-21 — cam-4c: webcam USB autosuspend off (udev rule)

**What happened:** while the cable test (script 30, rec_20261021_133925, cable shaken: 0 gaps, no USB error during
the recording) was checked, the kernel log showed at every webcam open (5 of 5 since 13:34):
`usb 1-7.4: reset high-speed USB device` and `uvcvideo 1-7.4:1.0: reset_resume error -5`. The webcam's
`power/control` was `auto` (suspended 2 s after it is closed); the C270 does not wake from suspend properly
and the kernel resets it. It came back every time, but a failed reset would stop the start.

**Decision (user: cam-4c before the collection):** `assets/udev/99-rb-cobot-webcam.rules` sets
`power/control` to `on` for 046d:0825 (all C270s: there is one; the serial would only matter with two).
Installed by the user with sudo (docs/teleop_guide.md). Rejected: the kernel boot option
`usbcore.autosuspend=-1` (turns it off for every USB device: a system-wide change for one camera);
`usbcore.quirks=046d:0825:b` (reset-resume quirk: keeps the reset, only makes it expected).

## 2026-10-21 — Grasp not visible in the gripper state (test A)

**What happened (user, collecting p1, session real_20261021_135405, 22 kept):** the robot cannot tell whether
the gripper holds the block. In every kept episode the gripper closed to the configured closed position (3967,
goal 3968) with the block, holding at about 20 mA; the grasp detection (squeeze) never fired. By hand the
fingers touch the block at 3517-3575 and touch each other (empty) at 4172 (user: the block is light and
grippy, a touch holds it). The logs show the motor passing 3517-3575 at full speed (48 rpm, no current
rise): the contact does not reach the motor.

**Test A (script 28, `--closed 4150` / `4140`, block and empty closes):** with the block the motor still went
to the goal (4138-4139) at full speed, holding 20-34 mA; empty 4139-4142, 0 mA. Position and current are the
same with and without the block, so the gripper state cannot tell a grasp. Overshoot max 4160 (12 ticks
before the empty touch). `closed_ticks` stays 3968 (nothing changed). **Decision (user):** a camera on the
fingers (cam-5a). Rejected: a deeper closed position (no difference, test A); adding the gripper current to
the state (no difference either).

## 2026-10-21 — cam-5a: three cameras (grip = Brio on the gripper); C270 in MJPG; missing-rows check

**Setup (user):** a Logitech Brio 101 (046d:094d) on the gripper, aimed at the fingers and what they hold:
camera `grip`. The C270 (`wrist`) keeps the view of the block, the D435 (`front`) the workspace.

**USB:** both webcams are USB 2 devices on the main controller's USB 2 bus (bus 1, with the leader arm and the
gripper U2D2; the other USB-C port, on the Thunderbolt controller, holds the robot's Ethernet adapter). Both in
YUYV: the second one is refused (`Not enough bandwidth for altsetting 11`). Tried:
- uvcvideo `quirks=128` (FIX_BANDWIDTH): all three open, but every C270 frame lost its bottom 61 % (pure
  green rows: YUYV zeros) and the brightness check passed it. **Rejected.** (The value is unsigned on this
  kernel: restore with 4294967295, not -1.) The D435 was unaffected after a replug.
- Formats, 20 s each (C270 / Brio): YUYV / MJPG: Brio refused (it asks the most bandwidth in MJPG too);
  **MJPG / YUYV: both 30 fps, 0 gaps, 0 damaged frames**; MJPG / MJPG also works. The 634 JPEG warnings came
  from the C270 (OpenCV's decoder); the Brio's MJPG was clean.
**Decision:** wrist (C270) MJPG, grip (Brio) YUYV. `UVCCamera(fmt)`: MJPG frames are taken as the JPEG bytes
(`CAP_PROP_CONVERT_RGB` 0) and decoded with PIL: no warnings, 4.6 ms per frame. Rejected: moving the Ethernet
adapter (changes the robot link; kept as the fallback).

**Config:** `record.cameras.<front|wrist|grip>` = device, format (RGB / YUYV / MJPG), exposure_us, gain,
white_balance_k (replaces front_* / wrist_*); checks: devices differ, RGB only for a RealSense.
`cameras.camera_defs(record, names)` gives the list for the teleop, script 35 (the cameras the policy expects,
in its order), scripts 30 and 34. The converter takes the cameras the sessions recorded (episodes.json) and
refuses sessions with different cameras.

**Missing-rows check:** `image_level` also returns the fraction of pure-green rows; a start image with more
than 2 % refuses like a saturated one (`refusal_problems`, teleop and script 35, no typed yes).

**Checked:** tests (rb5 154, 4 skipped; lerobot policy, cameras, converter). Hardware, cameras only, 20 s through
the recorder: front 30.03 fps, wrist (MJPG) 29.84, grip 29.98; 0 gaps, 0 dropped, 0 missing rows, no warnings.
Start brightness front 116.2, wrist 108.5, grip 71.4 (provisional gain: cam-5b).

**Next (cam-5b):** Brio USB autosuspend off (it resets at every open, as the C270 did); wrist and grip settings
(script 34); script 30 with all three while the teleop runs the leader arm and the gripper (same USB bus).
The 22 two-camera episodes go to data/archive.

## 2026-10-21 — cam-5b: Brio autosuspend off; wrist gain 86, grip gain 152; teleop with three cameras

- **USB:** the Brio moved to a new USB 3.1 hub (Genesys 05e3) on the USB-C port: its USB 3 half came up on
  the Thunderbolt controller (bus 4), its USB 2 half on the main controller (bus 1). The USB-C ports route
  USB 2 to the main controller, so every USB 2 webcam shares bus 1 whatever the port (bus 3 stays empty);
  moving the Ethernet adapter would not have helped. C270 MJPG + Brio YUYV stays the solution.
- **Autosuspend:** `assets/udev/99-rb-cobot-webcam.rules` has a line for the Brio (046d:094d); installed by the
  user, both read `on`. The check command in docs/teleop_guide.md read the hub above the webcam (two
  `dirname`s): fixed; the C270 had been `on` since cam-4c.
- **Settings (script 34, settings_20261021_152437, target 114, user approved):** wrist 8.3 ms gain 86 (114.1;
  was 78 in YUYV before it was re-aimed); grip 8.3 ms gain 152 (114.6; the Brio is less sensitive; 16.7 ms
  gain 88 gave 106). White balance 4600 K for both (the Brio's auto balance; the C270 hardly reacts to it).
  Rejected for grip: 16.7 ms (twice the blur on the moving wrist).
- **Teleop with three cameras (Simulation, session sim_20261021_153300, leader arm and gripper running):**
  start brightness 115.1 / 106.9 / 107.2, no refusal; kept episode 11.4 s: front 30.07 fps, wrist 29.88, grip
  30.02, 0 gaps, 0 dropped, 0 missing rows. Loop: 0 late cycles in 1164, cycle interval max 20.4 ms, leader read
  p99 8.6 ms, gripper reads normal. Leader tool (trigger) read failures 1.4 % (16, longest 3 cycles); before
  the Brio (real_20261021_135405) 0.9 % (longest 26): not new, a short test.

## 2026-10-21 — p2: learning code in rb5_lerobot/, its settings in configs/policy.yaml (p2-a: the move)

**Why (user):** the learning settings were spread over four places (configs/*.yaml for the hardware,
constants at the top of script 31 for the dataset format, lerobot-train options in docs/pipeline_guide.md plus
LeRobot defaults for training, PolicyRunner defaults and script 35 options for runs), the state / action names
were written twice, and scripts/ is meant for test scripts. The user wants to train, run and change parameters
alone. Measured first (synthetic ACT on the RTX A1000 6 GB): three cameras at 640x480 run out of memory at
batch 8 and infer in 50 ms (one camera: 17 ms); at 320x240 batch 8 takes 2.1 GB, 329 ms per step, 17 ms
inference.

**Decision (user, plan p2-a .. p2-e):** a package `rb5_lerobot/` (env lerobot) for convert / train / check / run,
one settings file `configs/policy.yaml` (dataset, train, run), copies of the settings saved with each
dataset, training and run. Not named `lerobot/`: at the repository root it would hide the installed LeRobot
package. Hardware code stays in teleop/ (shared with the teleop). Recordings stay 640x480; datasets and runs
use a smaller size (p2-b, p2-d).

**p2-a (move only, no behavior change):** `git mv` scripts/31_convert_to_lerobot.py -> rb5_lerobot/convert.py,
scripts/33_policy_check.py -> rb5_lerobot/check.py, scripts/35_run_policy.py -> rb5_lerobot/run.py,
teleop/policy.py -> rb5_lerobot/policy.py (its relative camera imports now `teleop.cameras`). Run as
`python -m rb5_lerobot.<convert|check|run>`. Tests, docs/pipeline_guide.md, docs/data_rules.md and
scripts/README.md point to the new places (older entries in this file keep the old names).
**Checked:** rb5 tests pass (4 skipped); lerobot: policy 22, convert 9, cameras 18 ok; the three modules start;
`convert --check-only` on the three-camera Simulation session gives the same report as before.

**p2-b (settings file, shared format, converter):** `configs/policy.yaml` with sections dataset / train / run
(every key commented: what it changes and what to redo); `rb5_lerobot/config.py` checks it with the teleop's
checker (unknown and missing keys, ranges), except `train.act`, which passes any ACT option on to
lerobot-train (`--policy.<name>`; lerobot-train rejects unknown names), checked for value types and
n_action_steps <= chunk_size; run: core ranges must not overlap, stop_stale_s above stale_s.
`rb5_lerobot/data_format.py`: FPS, the state and action names (were written in the converter and in
policy.py) and `resize_image` (PIL bilinear, which averages when shrinking; the run will use the same function).
The converter takes `--name` (data/lerobot/<name>, repo id local/<name>) and `--config`; its still-episode
limits, pause cut and image size come from the dataset section (the options --repo-id, --root, --cut-pauses,
--min-clutch-s, --min-path-mm, --keep-still are gone); it writes `rb5_convert.yaml` (settings, sessions,
cameras, command) into the dataset. **Checked:** tests (rb5 160, 4 skipped; lerobot policy and convert); the
three-camera Simulation session converted to data/lerobot/p2b_test: front / wrist / grip (3, 240, 320), state
7, action 4, 342 frames. Until p2-d the run does not shrink camera images: a 320x240 checkpoint cannot run yet.

**p2-c (training command):** `python -m rb5_lerobot.train NAME [--config FILE] [--dry-run]` builds the
lerobot-train command from the train section (train.act -> `--policy.<name>=<value>`, booleans in lower case),
prints the settings, the dataset (from its rb5_convert.yaml) and the command, and runs it in the repository
root. lerobot-train refuses an existing output folder, so `rb5_train.yaml` (settings file, train section,
command, the dataset's rb5_convert.yaml) is written once lerobot-train has made it. `--resume RUN_DIR` runs
`lerobot-train --config_path=RUN_DIR/checkpoints/last/pretrained_model/train_config.json --resume=true`
(LeRobot continues in the original folder with the checkpoint's settings). **Checked:** command tests; trial on
data/lerobot/p2b_test (3 cameras, 320x240, batch 8): 40 steps in 16 s, 0.26 s per step, 1.8 GB, loss 41 -> 11;
a 200-step run stopped after step 112 (checkpoints 50, 100), then `--resume` went on from 100 to 200. (A
SIGINT sent to a background job is ignored by Linux, so the stop was tested with SIGTERM; Ctrl+C in a terminal
reaches lerobot-train.)

**p2-d (run and check):** `python -m rb5_lerobot.run <training folder | checkpoint>` and `python -m rb5_lerobot.check`
take a training folder (its last checkpoint) or a step / pretrained_model folder (`resolve_checkpoint`). The run
reads the run section of configs/policy.yaml (`--config`; the hardware file is now `--robot-config`; the options
--time-limit, --servo-cores, --policy-cores, --torch-threads are gone) and passes stale_s, stop_stale_s,
max_jump_mm, time_limit_s and the ensembling coeff to PolicyRunner; the run JSON keeps the run section, the
settings file and the image sizes. `ACTChunkPolicy` shrinks each camera image to the size in the checkpoint's
config (`data_format.resize_image`, as the converter), so an older checkpoint keeps its own size. The check
takes the dataset from the training's rb5_train.yaml (`--dataset` otherwise). **Checked:** tests (checkpoint
folders, image sizes); with the p2b_test checkpoint: 640x480 camera images in -> a (100, 4) chunk; the run-path
image (recorded JPEG shrunk) vs the training-path image (dataset video frame) for three frames and all three
cameras: mean difference 1.4-2.0 / 255 (video compression), R-B maps correlated 0.90-0.98; check: GPU inference
17 ms with three 320x240 cameras; the resize costs about 2.2 ms per camera (p50).

**p2-e (wrap-up):** docs/pipeline_guide.md starts with a table of the code and where each setting lives, and
ends with what to redo after a change in policy.yaml; collection step for three cameras, 50 episodes in sessions
of 10-15 (docs/data_rules.md). CLAUDE.md (not committed: gitignored) lists `rb5_lerobot/` and
`configs/policy.yaml`. p2 is done: convert / train / check / run are `python -m rb5_lerobot.<name>` with one
settings file; tested end to end on a one-episode Simulation dataset (p2b_test, 200 steps). The trial outputs
(data/lerobot/p2b_test, data/train/act_p2b_test_20261021_160351) stay until the user's own trial.

## 2026-10-22 — p3-a: the policy run starts from the newest chunk only

**Problem:** first Simulation run of the p1 policy (data/policy/run_sim_20261022_034730): every `g` held in the
same cycle, "action 117 mm from the target (> 100)"; the robot never moved. The policy wanted to go down toward
the block (y -100, z -64 mm), a plausible direction. Cause: inference runs while waiting, and the ensembling by
time mixed about 96 chunks from the last 3.3 s, the oldest weighted most (ACT weights). Each of them planned
motion from its observation time; the arm stood still, so their value for "now" was where the arm should have
been after up to 3 s of motion. The distance grew from 40 to 117 mm while waiting and then stayed.
**Decision:** on `g` (also after a hold) `ChunkBuffer.keep_newest()` drops every chunk but the newest; the
ensembling then builds up again while the arm moves (as LeRobot, which resets the ensembler at an episode start).
The RUN line prints the first action's distance. max_jump_mm stays 100 mm; no retraining.
**Rejected:** a larger max_jump_mm (removes the safety check, and the jump would be real); a negative coeff
(newest weighted most: changes the ensembling while running too); no inference while waiting (the first chunk
would come 20+ ms after g, and the stale check needs a recent chunk). **Checked:** a runner test with chunks
that plan 60 mm/s up and a 3 s wait: it held at 103 mm without the fix and moves with it; tests rb5 165 OK
(4 skipped), lerobot policy tests 33 OK.

## 2026-10-22 — p3-b: inference only while running (g starts it)

**Context:** Simulation 035528 with p3-a: g started (first action 47 mm away), 60 s running, no late cycle,
inference p50 22 ms; the arm went about 120 mm toward the block in 5 s and stayed (the real arm and the
wrist cameras did not move: in Simulation the images never change). User: g should start the inference.
**Decision:** after its warm-up (and the one chunk shown before the loop) the policy process is paused. g
sends "go" (the buffer is emptied), the run starts with the first chunk that arrives (hold reason "starting"
until then, the target held); every hold (space, stale chunk, jump, time limit) sends "pause" and empties the
buffer. Each go has a number; queued chunks of an earlier go are dropped (PolicyClient.poll). The 3 s stop
applies only while inferring; after the time limit g is refused. ChunkBuffer.keep_newest (p3-a) is replaced by
clear(). **Rejected:** keep inferring while waiting and drop at g (p3-a: works, but the GPU runs for nothing
and the first chunk may still come from before g). **Checked:** tests (no chunk before g; the run starts about
one inference after g, target held while starting; hold pauses, g again goes; a 3 s stop while starting;
time limit refuses g; the policy process with fake cameras: no chunk while paused, chunks after go, none after
pause); rb5 tests OK (4 skipped), lerobot policy tests OK. The p1 model with fake cameras: first chunk 47-72 ms
after go after 0.5-20 s paused (inference 43-69 ms right after a pause, then about 23 ms).

## 2026-10-22 — p3-c: Real policy runs (python -m rb5_lerobot.run --real)

**Decision (user, 2026-10-22):** the policy run may move the real arm: `--real` uses configs/real.yaml with
load_settings(real=True) (the teleop's Real requirements and limits) and asks to type REAL after a note
(collision detection Off, tool mass 0 kg, emergency stop within reach). The same checks as the teleop: Real
mode reached, TCP offset equal to the expected value, the ready pose checked with calc_fk_tcp and the start
checks before move_j (now also in Simulation), "clear the area" and a separate yes for large wrist turns;
before the loop a reminder (hand on the emergency stop). The runner gives the policy the encoder joints
(jnt_ang) as state, as the converter does for Real sessions. Log run_real_<time>; the JSON keeps the mode and
the robot settings file. **Speed:** 100 % of the teleop's Real caps (user; a speed_scale 0.5 for the first
runs was offered and declined). No controller setting changed. **Not done:** no Real run yet (the user runs
it). **Checked:** tests (real.yaml loads with the Real checks and gives the recording's three cameras); rb5 and
lerobot policy tests OK; `--help`. The train-command test no longer reads values from configs/policy.yaml
(the user changes them: steps 30000 broke it).

## 2026-10-22 — p3-d: time limit per g; checkpoints/last logged as its step

**Context:** first Real run of p1 (data/policy/run_real_20261022_071457): four grasp attempts in 60 s (user: 3
of 5 failed, and after a missed grasp the arm still went to the box). The time limit counted the whole program
run, so every g after 60 s was refused. Grasp heights 25 / 19 / 34 / 64 mm vs 19 / 24 / 29 (p10 / 50 / 90) in
the 51 demonstrations; the gripper state is 0.00 one second after closing in the demonstrations too (a grasp
is not visible in it, test A); the demonstrations hold no missed grasp. Inference 60 ms p50 (Simulation: 22):
a `lerobot-train --resume --steps=50000` was training in the same folder on the GPU; the run loaded
checkpoints/last = 040000 but the log said only "last".
**Decision (user: A):** each g starts an attempt with its own time limit (run_time reset at g); g is never
refused for time. resolve_checkpoint replaces checkpoints/last by the step it points to (run and check print and
log it). **Rejected:** a larger run.time_limit_s only (B: one long limit, no per-attempt guard).
**Checked:** tests (two attempts of 2 s each in one run; last -> step); rb5 tests OK (4 skipped), lerobot policy
tests OK. **Next (suggested):** recovery demonstrations (miss, reopen, grasp again), an evaluation per checkpoint
with fixed block positions, runs without a training on the GPU.

## 2026-10-22 — p3-e: fine-tune from a trained policy (python -m rb5_lerobot.train NAME --from CHECKPOINT)

**Decision:** more data is added by converting the old and the new sessions into one new dataset and either
training anew or fine-tuning from a checkpoint (user asked how to continue from the current weights). `--from`
runs lerobot-train with `--policy.path=<pretrained_model>` instead of `--policy.type=act`: LeRobot 0.6.1
(scripts/lerobot_train.py) loads the weights and the policy config, and when not resuming replaces the
normalizer / unnormalizer statistics with the new dataset's; the optimizer, scheduler and step start anew in a
new folder. Before starting, the checkpoint's input / output features are compared with the dataset's
(cameras, image C x H x W, state, action); train.act entries equal to the checkpoint's config are dropped,
differing training options (n_action_steps, temporal_ensemble_coeff, dropout, kl_weight, optimizer_lr,
optimizer_weight_decay, optimizer_lr_backbone) are passed, any other difference (structure) refuses.
rb5_train.yaml records `from`. **Rejected:** `--resume` on a new dataset (keeps the old statistics, schedule
and step); fine-tuning on the new sessions only (forgetting). **Checked:** tests (command, feature and option
checks); `--dry-run`; a 20-step fine-tune of checkpoint 050000 on p1: loss 0.096 at step 5 (a new model starts
at about 41), 0.25 s per step, 1.8 GB; the test folder was removed.

## PENDING

- **14: RB-Y1 self-collision check during teleop.** Example 35 blocks the arm command when the
  nearest link distance is below 2 cm (`detect_collisions_or_nearest_links`, lines 1226-1240).
  Skipped for now (simulator only); needed before any real RB-Y1 use. Measure its cost in the 50 Hz loop.

- **real-1 follow-ups (2026-10-16):**
  - **Gripper:** connected in grip-3b (left trigger, released = closed). Log the gripper state with the
    demonstrations.
  - **Before mounting any tool:**
    - ~~Raise `range.z_min_mm`~~: done, 19 mm for the gripper (2026-10-21).
    - Set the tool mass and the lab's TCP offset (controller settings: ask first).
    - Recheck the ready pose margins with the new TCP.
  - **Base +x direction:** confirm with `scripts/22_base_axis_check.py` and a tape. `frame_yaw_deg` 0 is
    still an assumption, although the operation felt right.
  - **Collision detection sensitivity** with the lab (it is required ON before every Real run).
  - **RB5 joint speed specification** for this model, to replace our own caps (75 / 60 deg/s).
  - **Demonstration recording for ACT / LeRobot (2026-10-19):** rec-1 .. rec-5 done (format, env lerobot,
    `--record`, converter, Simulation replay, ACT trial on the pilot). Open:
    - Hardware before the real collection (user): the D405 broke (2026-10-21); C270 webcam on the gripper
      (cam-4a), grip camera Brio (cam-5a/5b: three cameras, settings set, checked in a Simulation teleop);
      D435 position (the arm covers about 2/3 of its view).
    - Real collection: rules in docs/data_rules.md (2026-10-21); fixed camera settings done (cam-2b; the policy
      runner must open the cameras with the same code) and z_min 19 mm (2026-10-21). Left: the hardware
      items above.
    - **rec-6 policy runner:** decided 2026-10-21 (see its entry): two processes, temporal ensembling by time,
      100 % speed, keyboard toggle (space = hold). 6a done; 6b code done (script 35, Simulation run by the
      user: 043648 clean after the core split); 6c done (gripper; Simulation 071545: the gripper follows the
      policy, the image must show the real arm at the ready pose). Real policy runs need their own plan.
  - **Windows** (only if needed):
    - environment.yml (evdev only on Linux)
    - CPU pinning and the FTDI latency check per OS
    - the port name
    - Python 3.11+ for the sleep resolution
  - **Optional, when the operator wants more speed:** sideways 400 mm/s, radial / vertical 300 mm/s.
    Each raise is a Real limit change and needs its own decision.
  - **Wrist rotation (J6 0..90 deg about the tool axis; on hold, 2026-10-19):** for objects that need the
    fingers turned. User: on hold, ACT first with the gripper orientation fixed (orientation changes
    need many more demonstrations). If resumed: the right trigger (stays where released) 0..1000 ->
    0..90 deg added to the follow_base yaw, only while the clutch is pressed, rate limited (about 45
    deg/s); the action becomes [x, y, z, wrist, gripper], so decide before a dataset is recorded.
    Leader J6 -> RB5 J6 rejected: unintended leader J6 roll per clutch press (Real 095845 / 103804 /
    105914) median 17 / 19 / 30 deg, p90 38 / 45 / 73, max 88 deg -> a deadband of 40+ deg would be needed.
  - **Gripper (closed for now, 2026-10-19; see its entry):** 12 V supply; grip force eaten by compliance
    (goal that follows the position while grasping); thin objects not detected (finger pads); real finger
    contact position; return delay 0 (motor setting, ask first); mounted on the flange: z_min done (19 mm),
    tool mass and TCP offset still to set (ask first).

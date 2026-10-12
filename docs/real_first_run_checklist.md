# Real first run checklist (not done yet)

The teleop refuses `--real` until `configs/real.yaml` has no TBD (null) values and stays within the
Real limits in `teleop/settings.py`. This list is the work to do before the first Real run. Do it as
its own step, with the lab, not as part of a Simulation session.

## Controller and robot (ask before changing any controller setting, CLAUDE.md)
- [ ] **Collision detection**: currently Off. Decide with the lab whether to turn it on and at what
      sensitivity before Real teleop.
- [ ] **Tool mass**: currently 0 kg. Set it for the real tool (gripper / Allegro Hand) if the lab agrees.
- [ ] **TCP offset**: the controller has (0, -187, -40) mm, orientation 0 (17). PROVISIONAL: real.yaml
      expects exactly this (refuses if it changes). Replace with the lab's confirmed value.
- [x] **J3 limit**: +/-165 (user checked on the teach pendant, 2026-10-12); set in both settings files.
- [ ] Joint limits set on the controller (setup page), if any are narrower than the teleop's.

## Workspace
- [ ] **Base +x direction**: confirm with `scripts/22_base_axis_check.py` and a tape (manual
      Appendix F: +Y toward the cable connector, so +X is 90 deg clockwise from it seen from above).
- [x] **Ready pose for Real**: the pose set by the user on the real RB5 (2026-10-12), margins checked.
- [ ] **z lower bound** (`range.z_min_abs_mm`): PROVISIONAL 30 mm. Replace with table height + gripper
      length + margin, in the base frame (z 0 = base bottom). The cylinder never goes below it.
- [x] Range (`range.*`): widened to the Simulation range on user request (2026-10-12): r 350-800,
      z -100 / +150 (floor 30), azimuth +/-90. Keep the area of that range clear.
- [ ] Nothing in the reach of the robot along the ready-pose move (large wrist turns are warned).

## Speeds (conservative in real.yaml; RB5 specification still to be found)
- [ ] Now (2026-10-12): TCP 100 mm/s, yaw 30 deg/s, joints 0.8 deg/cycle (40 deg/s), C 20 mm, ready move 5 deg/s.
- [ ] B mode (`max_lag_mm: 0`): the robot stops as soon as the hand stops.
- [ ] Per-joint speed limits from the RB5 specification (manual lists only "joint max 180 deg/s, tool
      1 m/s" for the E series).

## People and procedure
- [ ] Emergency stop within reach of the operator; a second person watches the robot.
- [ ] Tablet connected; speed bar 0.1.
- [ ] First Real run: short, small motions near the ready pose, scale 0.5, clutch only briefly.
- [ ] After the run: check the CSV (late cycles, limit flags, tcp_ref following) before the next run.

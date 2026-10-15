# Real first run checklist (not done yet)

The teleop refuses `--real` until `configs/real.yaml` has no TBD (null) values and stays within the
Real limits in `teleop/settings.py`. This list is the work to do before the first Real run. Do it as
its own step, with the lab, not as part of a Simulation session.

## Before EVERY Real run (required, from 2026-10-16)
- [ ] **Collision detection is ON** on the controller (check on the tablet). Do not start
      `python -m teleop --real` while it is Off. Required since the Real speed limits were raised to
      TCP 200 mm/s and joints 60 deg/s (decisions 2026-10-16). The teleop does not check this itself.
- [ ] Emergency stop within reach of the operator; a second person watches the robot.
- [ ] Workspace clear (range below; TCP azimuth -160 .. 133 deg about the base z axis since 2026-10-16, so the sides
      and the back of the RB5 too), tablet connected, speed bar 0.1.

## Controller and robot (ask before changing any controller setting, CLAUDE.md)
- [ ] **Collision detection**: was Off. The user turns it on with the lab before the next Real run
      (2026-10-16); sensitivity to be decided with the lab. Then "ON" is checked before every run (above).
- [ ] **Tool mass**: currently 0 kg. Set it for the real tool (gripper / Allegro Hand) if the lab agrees.
- [ ] **TCP offset**: the controller has (0, -187, -40) mm, orientation 0 (17). PROVISIONAL: real.yaml
      expects exactly this (refuses if it changes). Replace with the lab's confirmed value.
- [x] **J3 limit**: +/-165 (user checked on the teach pendant, 2026-10-12); set in both settings files.
- [ ] Joint limits set on the controller (setup page), if any are narrower than the teleop's.

## Workspace
- [ ] **Base +x direction**: confirm with `scripts/22_base_axis_check.py` and a tape (manual
      Appendix F: +Y toward the cable connector, so +X is 90 deg clockwise from it seen from above).
- [x] **Ready pose for Real**: 2026-10-16, the user's pose moved out to r 450 (decisions), margins checked.
- [ ] **z lower bound** (`range.z_min_mm`): -50 mm since 2026-10-16 because **nothing is mounted** on
      the flange (the TCP is 187 mm below the flange center; flange center about 140 mm above the table
      at the floor). **Raise it before mounting any tool**: tool tip height + margin (z 0 = base bottom).
- [x] Range (`range.*`, absolute in the base frame since 2026-10-16): r 350-800, z -50 .. 465 (outer radius
      800 -> 650 between z 265 and 465), azimuth -160 .. 133 deg. Keep that space clear, also above the robot.
- [ ] Nothing in the reach of the robot along the ready-pose move (large wrist turns are warned).

## Speeds (conservative in real.yaml; RB5 specification still to be found)
- [ ] Now (2026-10-16): TCP radial / vertical 250 mm/s, sideways (turning J1) 360 mm/s, joints J1 75 deg/s and
      J2-J6 60 deg/s, acceleration 2.5 m/s^2, C 20 mm (rotate mode 15 deg), ready move 5 deg/s.
- [ ] First run at the new speeds: slow, small motions near the ready pose first; check for controller
      errors and late cycles before moving fast.
- [ ] Per-joint speed limits from the RB5 specification (manual lists only "joint max 180 deg/s, tool
      1 m/s" for the E series).

## People and procedure
- [ ] Emergency stop within reach of the operator; a second person watches the robot.
- [ ] Tablet connected; speed bar 0.1.
- [ ] First Real run: short, small motions near the ready pose, scale 0.5, clutch only briefly.
- [ ] After the run: check the CSV (late cycles, limit flags, tcp_ref following) before the next run.

# Real first run checklist (not done yet)

The teleop refuses `--real` until `configs/real.yaml` has no TBD (null) values and stays within the
Real limits in `teleop/settings.py`. This list is the work to do before the first Real run. Do it as
its own step, with the lab, not as part of a Simulation session.

## Controller and robot (ask before changing any controller setting, CLAUDE.md)
- [ ] **Collision detection**: currently Off. Decide with the lab whether to turn it on and at what
      sensitivity before Real teleop.
- [ ] **Tool mass**: currently 0 kg. Set it for the real tool (gripper / Allegro Hand) if the lab agrees.
- [ ] **TCP offset**: the controller has (0, -187, -40) mm, orientation 0 (17). The lab is checking
      what it should be. Put the confirmed value in `tcp.expected_offset_mm`; the teleop compares it
      with get_tcp_info / get_tfc_info at start (tolerance 1 mm).
- [ ] **J3 limit**: +/-150 or +/-165 on this RB5-850 (N)? Check on the teach pendant; update
      `safety.joint_limit_deg[2]`.
- [ ] Joint limits set on the controller (setup page), if any are narrower than the teleop's.

## Workspace
- [ ] **Base +x direction**: confirm with `scripts/22_base_axis_check.py` and a tape (manual
      Appendix F: +Y toward the cable connector, so +X is 90 deg clockwise from it seen from above).
- [ ] **Ready pose for Real** (`ready_pose.joints_deg`): J1 from the table direction; tool down, wrist
      outside (like pose D); check the margins with the DH model as done for D.
- [ ] **z lower bound** (`range.z_min_abs_mm`): table height + gripper length + margin, in the base
      frame (z 0 = base bottom). The cylinder never goes below it.
- [ ] Range (`range.*`): narrow first (r 470-530, z +/-50, azimuth +/-30 in real.yaml); widen only
      after runs look right.
- [ ] Nothing in the reach of the robot along the ready-pose move (large wrist turns are warned).

## Speeds (conservative in real.yaml; RB5 specification still to be found)
- [ ] TCP 20 mm/s, yaw 15 deg/s, joints 0.2 deg/cycle (10 deg/s), ready move 10 x 0.1 = 1 deg/s.
- [ ] B mode (`max_lag_mm: 0`): the robot stops as soon as the hand stops.
- [ ] Per-joint speed limits from the RB5 specification (manual lists only "joint max 180 deg/s, tool
      1 m/s" for the E series).

## People and procedure
- [ ] Emergency stop within reach of the operator; a second person watches the robot.
- [ ] Tablet connected; speed bar 0.1.
- [ ] First Real run: short, small motions near the ready pose, scale 0.5, clutch only briefly.
- [ ] After the run: check the CSV (late cycles, limit flags, tcp_ref following) before the next run.

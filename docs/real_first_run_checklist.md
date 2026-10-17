# Real run checklist

The teleop refuses `--real` until `configs/real.yaml` has no TBD (null) values and stays within the
Real limits in `teleop/settings.py`. Real teleop has been run since 2026-10-12 (real-1, settings final
on 2026-10-16). The first section is for every run; the rest are the open items with the lab.

## Before EVERY Real run (required)
- [ ] **Collision detection is ON** on the controller (check on the tablet). Do not start
      `python -m teleop --real` while it is Off. The teleop does not check this itself.
- [ ] Emergency stop within reach of the operator; a second person watches the robot.
- [ ] Workspace clear: radius up to 800 mm around the base, height up to about 465 mm, TCP azimuth
      -160 .. 133 deg about the base z axis (the sides and the back of the RB5 too); cables free.
- [ ] Nothing in the way of the ready-pose move (large wrist turns are warned).
- [ ] Tablet connected; speed bar 0.1.
- [ ] **Gripper mounted on the flange?** Then first: raise `range.z_min_mm`, set the tool mass and TCP
      offset (controller settings: ask first). The teleop does not know whether it is mounted.
- [ ] Gripper: 12 V supply on, fingers clear. At Ctrl+C its torque goes off: put a held object down first.
- [ ] After the run: `python scripts/24_run_report.py <log>` (late cycles, limits, joint speeds vs caps).

## Controller and robot (ask before changing any controller setting, CLAUDE.md)
- [ ] **Collision detection sensitivity**: decide with the lab (it is required ON, above).
- [ ] **Tool mass**: 0 kg on the controller. Set it for the real tool (gripper) when mounted.
- [ ] **TCP offset**: the controller has (0, -187, -40) mm, orientation 0 (17). PROVISIONAL: real.yaml
      expects exactly this (refuses if it changes). Replace with the lab's value for the mounted tool.
- [x] **J3 limit**: +/-165 (user checked on the teach pendant, 2026-10-12); set in both settings files.
- [ ] Joint limits set on the controller (setup page), if any are narrower than the teleop's.

## Workspace
- [ ] **Base +x direction**: confirm with `scripts/22_base_axis_check.py` and a tape (manual
      Appendix F: +Y toward the cable connector, so +X is 90 deg clockwise from it seen from above).
- [x] **Ready pose**: 2026-10-16, the user's pose moved out to r 450 (decisions), margins checked.
- [ ] **z lower bound** (`range.z_min_mm`): -50 mm because **nothing is mounted** on the flange (the TCP
      is 187 mm below the flange center; the flange center is about 140 mm above the table at the floor).
      **Raise it before mounting any tool**: tool tip height + margin (z 0 = base bottom).
- [x] Range (`range.*`, absolute in the base frame since 2026-10-16): r 350-800, z -50 .. 465 (outer
      radius 800 -> 650 between z 265 and 465), azimuth -160 .. 133 deg.

## Speeds (real-1 final, 2026-10-16)
- [x] TCP radial / vertical 250 mm/s, sideways (turning J1) 360 mm/s, joints J1 75 / J2-J6 60 deg/s,
      acceleration 2.5 m/s^2, C 20 mm (rotate mode 15 deg), ready move 5 deg/s. Run on the real robot
      (051339): no late cycles but one, joint speeds within the caps, target jitter 2.5 m/s^2 p99.
- [ ] Per-joint speed limits from the RB5 specification (manual lists only "joint max 180 deg/s, tool
      1 m/s" for the E series); the caps above are our own conservative values.
- [x] First Real runs: done 2026-10-12 .. 16 (short and slow first, then step by step).

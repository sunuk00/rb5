# RB_Cobot
RB5-850 + Allegro Hand V4 test repository: robot API tests and RB-Y1 leader arm teleoperation.

- `scripts/` - numbered experiment scripts (01-22), kept as they were run
- `teleop.py` + `teleop/` - leader arm -> RB5 teleop (position, yaw, gripper signal); Simulation by default
- `configs/` - teleop settings (`sim.yaml`, conservative `real.yaml` with TBD values)
- `tests/` - automated tests with fake devices: `python -m unittest discover tests`
- `docs/teleop_guide.md` - how to run the teleop (setup, checklist, controls, stop, troubleshooting)
- `docs/real_first_run_checklist.md` - what must be done before the first Real run
- `docs/decisions.md` - decisions and measurements behind every value

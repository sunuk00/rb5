# RB-Y1 leader arm URDF

`model.urdf` is an unmodified copy of `models/leader_arm/model.urdf` from Rainbow Robotics'
rby1-sdk v0.10.0 (https://github.com/RainbowRobotics/rby1-sdk), Apache License 2.0 (`LICENSE`,
`NOTICE`). SHA-256: `972a38add178ac2c1e8ed5d3c9fa45fb5ae66bf2c6138e1e9891cb8ee961669f`.

It is kept here so the teleop runs from a fresh clone (`refs/` is not committed). Only the joint
origins and axes are used (`teleop/leader.py`, `LeaderModel`); the mesh files it names are not
included.

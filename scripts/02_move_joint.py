import time
import numpy as np
import rbpodo as rb

ROBOT_IP = "10.0.2.6"
DELTA_DEG = 2.0   # Move only joint 6 by 2 degrees
SPEED = 10.0      # deg/s
ACC = 20.0        # deg/s^2

robot = rb.Cobot(ROBOT_IP)            # Command channel (port 5000)
rc = rb.ResponseCollector()
data_channel = rb.CobotData(ROBOT_IP) # Data channel (port 5001)

# 1. Read the current joint angles
q_now = np.array(data_channel.request_data().sdata.jnt_ang, dtype=float)
q_target = q_now.copy()
q_target[5] += DELTA_DEG              # Index 5 corresponds to joint 6

print("Current:", q_now)
print("Target:", q_target)

# 2. Require explicit user confirmation
if input("Joint 6 will move by 2 degrees in Simulation mode. Enter yes to continue: ") != "yes":
    raise SystemExit("Cancelled.")

# 3. Apply safety settings
robot.set_operation_mode(rc, rb.OperationMode.Simulation)  # Set Simulation mode
robot.set_speed_bar(rc, 0.1)          # Limit the speed bar to 10%
robot.flush(rc)                       # Clear previous messages

# 4. Move and poll until the motion is complete
robot.move_j(rc, q_target, SPEED, ACC)
if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(1e-3)
rc.error().throw_if_not_empty()

# 5. Check the result
q_after = np.array(data_channel.request_data().sdata.jnt_ang, dtype=float)
print("After motion:", q_after)
print("Difference:", q_after - q_now)
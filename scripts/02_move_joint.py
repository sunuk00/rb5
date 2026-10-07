import time
import numpy as np
import rbpodo as rb

ROBOT_IP = "10.0.2.6"
DELTA_DEG = 2.0   # 6번 관절만 2도
SPEED = 10.0      # deg/s
ACC = 20.0        # deg/s^2

robot = rb.Cobot(ROBOT_IP)            # 명령 통로 (5000)
rc = rb.ResponseCollector()
data_channel = rb.CobotData(ROBOT_IP) # 데이터 통로 (5001)

# 1. 현재 관절각 읽기
q_now = np.array(data_channel.request_data().sdata.jnt_ang, dtype=float)
q_target = q_now.copy()
q_target[5] += DELTA_DEG              # 인덱스 5 = 6번 관절

print("현재:", q_now)
print("목표:", q_target)

# 2. 사람이 직접 확인해야만 진행
if input("Real 모드로 6번 관절을 2도 움직입니다. 진행하려면 yes: ") != "yes":
    raise SystemExit("취소했습니다.")

# 3. 안전 설정
robot.set_operation_mode(rc, rb.OperationMode.Real)
robot.set_speed_bar(rc, 0.1)          # 속도 10%만
robot.flush(rc)                       # 이전 메시지 비우기

# 4. 이동 + 폴링으로 끝날 때까지 대기
robot.move_j(rc, q_target, SPEED, ACC)
if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(1e-3)
rc.error().throw_if_not_empty()

# 5. 결과 확인
q_after = np.array(data_channel.request_data().sdata.jnt_ang, dtype=float)
print("이동 후:", q_after)
print("차이:", q_after - q_now)
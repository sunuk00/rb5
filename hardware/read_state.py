import rbpodo as rb

ROBOT_IP = "10.0.2.6"

data_channel = rb.CobotData(ROBOT_IP)   # 상태를 읽는 통로
data = data_channel.request_data()       # 현재 상태 한 번 요청

print("관절각:", data.sdata.jnt_ang)
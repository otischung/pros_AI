# config.py

"""
vel, rotate_vel為自走車PID數值, 可於arduino程式碼查看

於ros_receive_and_data_processing/AI_node.py使用
"""
vel = 20.0
rotate_vel = 5.0

ACTION_MAPPINGS = {
    "FORWARD": [vel, vel],  # 前進
    "LEFT_FRONT": [rotate_vel, rotate_vel * 1.2],  # 左前
    "COUNTERCLOCKWISE_ROTATION": [
        -rotate_vel,
        rotate_vel,
    ],  # 左自轉
    "BACKWARD": [-vel, -vel],  # 後退
    "CLOCKWISE_ROTATION": [rotate_vel, -rotate_vel],  # 右自轉
    "RIGHT_FRONT": [rotate_vel * 1.2, rotate_vel],  # 右前
    "STOP": [0.0, 0.0],
}

"""
LIDAR_PER_SECTOR : 原始lidar射線有1800條，除上這個數值讓他變90條

於ros_receive_and_data_processing/AI_node.py使用

底下目前沒用到
LIDAR_RANGE : 設定lidar要有幾個偵測
FRONT_LIDAR_INDICES : 車子偵測前面障礙物的lidar range
LEFT_LIDAR_INDICES : 車子偵測左邊障礙物的lidar range
RIGHT_LIDAR_INDICES : 車子偵測右邊障礙物的lidar range
"""
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(
    range(-15, 0)
)  # 0~16個和-15到0的LIDAR RANGE
LEFT_LIDAR_INDICES = list(range(16, 46))
RIGHT_LIDAR_INDICES = list(range(-45, -15))

"""
NEXT_POINT_DISTANCE : 距離下一個點的距離

於ros_receive_and_data_processing/ros_AI_node.py使用
"""
NEXT_POINT_DISTANCE = 0.5

"""
TARGET_DISTANCE : 判定為成功到達目標的距離

於car_navigation/navigation_process.py使用
"""
TARGET_DISTANCE = 0.3

"""
BODY_WIDTH : 車寬
WHEEL_DIAMETER : 車輪直徑

於ros_receive_and_data_processing/navigation_utils.py使用
"""
BODY_WIDTH = 0.3
WHEEL_DIAMETER = 0.05

"""
FACTOR : rpm與pid的比例關係
rpm數值會除以這個FACTOR數值轉換成PID數值
ex. 車體的RPM與PID之間是10 : 1, 若RPM = 100, PID = RPM / FACTOR, PID = 10

於ros_receive_and_data_processing/navigation_utils.py使用
"""
FACTOR = 10

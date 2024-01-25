#  計算reward的工具
import numpy as np

#  處理偏行角計分 (180 +- 30 可以加分)
def get_yaw_from_quaternion(z, w):
        """从四元数的 z 和 w 分量中提取偏航角（Y 轴旋转）"""
        return np.degrees(2 * np.arctan2(z, w))
    
def get_direction_vector(current_position, target_position):
    current_position.pop()
    target_position.pop()
    """计算从当前位置指向目标位置的向量"""
    return np.array(target_position) - np.array(current_position)

def get_angle_to_target(car_yaw, direction_vector):
    #  計算car與target之間的角度差
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)

    return np.abs(np.degrees(angle_diff)) % 360

#  占比很重
def calculate_angle_point(angle):
    point = 0
    angle_diff = np.abs(angle - 180)
    if 0 <= angle_diff <= 30:
        point = 100 * (1 - (angle_diff / 30))
    else:
        point =  -10 * (1 + (angle_diff / 30))
    return point

#  計算與目標之間的距離和上次的距離比較
def calculate_distance_change(current_distance, threshold):
    #  因為傳輸過快的關係，距離差幾乎趨近0，不要用距離差
    if current_distance > threshold:
        # 距离大于4时，给予惩罚，惩罚随着距离减小而减少
        reward = -(current_distance - 3.5)*15 # distance > 4时，-2是基础惩罚值，随距离增加而减少惩罚
    elif 3 < current_distance <= threshold:
        # 距离在3到4之间，给予小量奖励，奖励随着距离减小而增加
        reward = 7 * (threshold - current_distance)
    else:
        # 距离小于3时，给予较大奖励，特别是在距离接近1.5时，奖励更大
        reward = 8 * (3 - current_distance) ** 2

    return reward


def calculate_lidar_based_reward(lidar_data, safe_distance):
    point = 0
    for lidar_distance in lidar_data:
        if lidar_distance < safe_distance:
            point -= 500

    if min(lidar_data) < 0.4:
        point -= 1000
    elif min(lidar_data) < 0.3:
        point -= 10000

    return point

#  駕車穩定性
def calculate_steering_change(current_steering_angle, previous_steering_angle):
    angle_change = current_steering_angle - previous_steering_angle

    # 处理角度的跳变
    if angle_change > 180:
        angle_change -= 360
    elif angle_change < -180:
        angle_change += 360

    return angle_change

#  防左右擺頭 
def calculate_drive_reward(current_direction, previous_direction):
    point = 0
    if previous_direction == None:
        point = 0
    elif current_direction == previous_direction:
        # 如果连续向同一方向转向，给予奖励
        point = 10
    else:
        # 如果频繁改变转向方向，给予惩罚
        point = -30

    return point

#  輪速評分
def calculate_wheel_speed_reward(left_wheel_speed, right_wheel_speed):
    speed_difference = abs(left_wheel_speed - right_wheel_speed)
    stable_speed_threshold = 5  # 穩定數值

    if speed_difference < stable_speed_threshold:
        # 輪速差異小，平穩駕駛
        reward = 3
    else:
        # 不穩駕駛
        reward = -10

    return reward

#  計算是否一直重複
def check_spinning(action_history, threshold):
    point = 0
    action_history = list(action_history)
    if len(action_history) >= threshold:
        last_actions = action_history[-threshold:]
        if len(set(last_actions)) == 1:
            # 如果最后 'threshold' 个动作都相同，则扣分
            point = -30
        elif len(set(action_history)) == 1:
            # 如果所有动作都相同，则扣更多分
            point = -100
    return point

def get_smallest_lidar_values_with_direction(lidar_data):
    """
    Divide the lidar data into 15 chunks and return the smallest value from each chunk
    along with its direction.
    """
    chunk_size = len(lidar_data) // 8
    result = []
    min_value_list = []

    for i in range(0, len(lidar_data), chunk_size):
        chunk = lidar_data[i:i + chunk_size]
        min_value = min(chunk)
        min_index = i + chunk.index(min_value)
        
        min_value_list.append(min_value)

    return round_to_decimal_places(min_value_list)

#  將list取到小數第三位
def round_to_decimal_places(data_list, decimal_places=3):
    """
    Round the elements of a list to a specified number of decimal places.
    """
    return [round(num, decimal_places) for num in data_list]
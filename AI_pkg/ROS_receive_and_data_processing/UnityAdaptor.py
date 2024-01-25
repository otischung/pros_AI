from utils.adaptor_utils import *
from utils.rotate_angle import *

def transfer_obs(obs):
    car_pos = obs['ROS2CarPosition'][:2]
    target_pos = obs['ROS2TargetPosition'][:2]
    car_quaternion = obs['ROS2CarQuaternion'][2:4]
    lidar_data = obs['ROS2Range']
    left_front_speed = obs['ROS2WheelAngularVelocityLeftFront']
    right_front_speed = obs['ROS2WheelAngularVelocityRightFront']
    left_back_speed = obs['ROS2WheelAngularVelocityLeftBack']
    right_back_speed = obs['ROS2WheelAngularVelocityRightBack']
    angle_diff = calculate_angle_point(car_quaternion[0], car_quaternion[1], car_pos, target_pos)#  輸出面向目標角度
    car_target_distance = cal_distance(car_pos, target_pos)
    
    '''
    這邊宣告自己要回傳的資料
    '''
    state_dict = {
        "car_pos": trans_to_float(car_pos),
        "target_pos": trans_to_float(target_pos),
        "car_target_distance": float(car_target_distance),
        "car_quaternion": trans_to_float(car_quaternion),
        "lidar_data": trans_to_float(lidar_data),
        "relative_coordinates": trans_to_float(round_to_decimal_places(
                                            [target_pos[0]-car_pos[0],target_pos[1]-car_pos[1]])
                                            ),
        "angle_diff": float(angle_diff)
    }
    
    return state_dict

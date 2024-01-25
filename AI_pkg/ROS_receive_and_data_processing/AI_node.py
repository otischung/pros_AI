from std_msgs.msg import Float32MultiArray
from collections import defaultdict
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import json
from std_msgs.msg import String
from ROS_receive_and_data_processing.car_models import *
import sys
import math
from rclpy.node import Node
from ROS_receive_and_data_processing.UnityAdaptor import *
import orjson
import time
from ROS_receive_and_data_processing.config import ACTION_MAPPINGS

class AI_node(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")  # ros2Ai #unity2Ros
        self.real_car_data = {}
        self.data_to_AI = ""
        
        #  goal的數值只會出現在點擊goal的一瞬間, 不然都不送訊號
        self.latest_goal_position = None 
        self.lastest_data = None
        self.real_car_data['ROS2CarPosition'] = []
        self.real_car_data['ROS2CarQuaternion'] = []
        self.real_car_data['ROS2TargetPosition'] = []
        #  四輪的速度
        self.real_car_data['ROS2WheelAngularVelocityLeftBack'] = 0.01
        self.real_car_data['ROS2WheelAngularVelocityLeftFront'] = 0.01
        self.real_car_data['ROS2WheelAngularVelocityRightBack'] = 0.01
        self.real_car_data['ROS2WheelAngularVelocityRightFront'] = 0.01
        self.real_car_data['ROS2Range'] = []
        self.real_car_data['ROS2RangePosition'] = []
        
        #  追蹤每個數據是否更新
        self.data_updated = {
            'amcl': False,
            'goal': False,
            'lidar': False,
        }
        #  接收目標, 車體目前位置, 輪子轉速, lidar的距離和vector
        self.subscriber_amcl = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                                        self.subscribe_callback_amcl, 1)
        self.subscriber_goal = self.create_subscription(PoseStamped, "/goal", self.subscribe_callback_goal, 1)
        self.subscriber_lidar = self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 1)
        self.subscriber_rear = self.create_subscription(String, DeviceDataTypeEnum.car_C_state,
                                                        self.rear_wheel_callback, 1)
        self.subscriber_forward = self.create_subscription(String, DeviceDataTypeEnum.car_C_state_front,
                                                           self.forward_wheel_callback, 1)

        #  publish給前後的esp32驅動車輪
        self.publisher = self.create_publisher(
            String,
            DeviceDataTypeEnum.car_C_control,
            10
        )
        self.publisher_forward = self.create_publisher(
            String,
            "test",  # topic name
            10
        )
        
    #  檢查所有的數據是否更新
    def check_and_get_lastest_data(self):
        '''將訊息轉換成跟虛擬環境的python一樣的格式'''
        # print(self.data_updated.values())
        if all(self.data_updated.values()):
            # 確認所有的數據都更新並publish
            self.data_updated['amcl'] = False
            self.data_updated['lidar'] = False
            self.lastest_data = transfer_obs(self.real_car_data)

    # 取得資料
    def get_latest_data(self):
        return self.lastest_data
    
    #  輸出給車子的writer
    def publish_to_unity(self, action_code):
        '''
        _vel1, _vel3是左側
        _vel2, _vel4是右側
        '''
        action = ACTION_MAPPINGS.get(action_code, "invalid")
        _vel1, _vel2, _vel3, _vel4 = action[0],action[1],action[2],action[3]
        
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_C_control),
            "data": dict(CarCControl(
                target_vel=[_vel1, _vel2]
            ))
        }
        
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()
        self.publisher.publish(control_msg)
        
        control_signal_forward = {
            "type": str(DeviceDataTypeEnum.car_C_control),
            "data": dict(CarCControl(
                target_vel=[_vel3, _vel4]
            ))
        }
        control_msg_forward = String()
        control_msg_forward.data = orjson.dumps(control_signal_forward).decode()

        # Publish the control signal
        self.publisher_forward.publish(control_msg_forward)
        
        #  lidar轉一圈需要0.1多秒, 確保lidar更新到最新data
        time.sleep(0.2)
        
    def publish_to_unity_RESET(self):
        self.publish_to_unity(4)
            
    def reset(self):
        self.lastest_data = None
        
    def update_and_check_data(self, data_type):
        self.data_updated[data_type] = True
        self.check_and_get_lastest_data()

    def subscribe_callback_amcl(self, message):
        pose = self.transfer_car_pose(message)[0]
        quaternion = self.transfer_car_pose(message)[1]
        self.real_car_data['ROS2CarPosition'] = pose
        self.real_car_data['ROS2CarQuaternion'] = quaternion
        self.update_and_check_data('amcl')

    def transfer_car_pose(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return [position.x, position.y, position.z], [orientation.x, orientation.y, orientation.z, orientation.w]

    def subscribe_callback_goal(self, message):  # execute only if there has signal
        target = self.transfer_target_pos(message)
        if target:
            self.real_car_data['ROS2TargetPosition'] = target   
            self.update_and_check_data('goal')

    def transfer_target_pos(self, msg):
        position = msg.pose.position
        return [position.x, position.y, position.z]

    def laser_scan_callback(self, msg):
        '''
        普通lidar all_ranges = 1800
        '''
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges_180 = []
        direction_180 = []
        all_ranges = msg.ranges
        for i in range(len(all_ranges)):
            if i % 36 == 0:  # handle the amount of lidar.
                angle_tmp = angle_min + i * angle_increment
                ranges_180.append(all_ranges[i])
                direction_180.append([math.cos(angle_tmp), math.sin(angle_tmp), 0])
        self.real_car_data['ROS2Range'] = ranges_180
        self.real_car_data['ROS2RangePosition'] = direction_180
        self.update_and_check_data('lidar')

    def rear_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if 'data' in data and 'vels' in data['data']:
                vels = data['data']['vels']
                self.real_car_data['ROS2WheelAngularVelocityLeftBack'] = vels[0]
                self.real_car_data['ROS2WheelAngularVelocityRightBack'] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def forward_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if 'data' in data and 'vels' in data['data']:
                vels = data['data']['vels']
                self.real_car_data['ROS2WheelAngularVelocityLeftFront'] = vels[0]
                self.real_car_data['ROS2WheelAngularVelocityRightFront'] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
            
    

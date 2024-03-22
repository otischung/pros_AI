from avoidance_rule.Simulated_Annealing import ObstacleAvoidanceController
import rclpy
from utils.obs_utils import *
from math import pi
from utils.rotate_angle import calculate_angle_point
import time
from ros_receive_and_data_processing.config import FRONT_LIDAR_INDICES


class NavigationController:
    def __init__(self, node):
        self.node = node
        self.body_length = 0.5
        self.body_width = 0.3
        self.wheel_diameter = 0.05
        self.angle_tolerance = 10

    def reset_controller(self):
        self.data = []
        self.node.publish_to_unity_RESET()

    def calculate_wheel_speeds(self, linear_velocity, angular_velocity):
        # 車軸長度假設為車體的寬度
        L = self.body_width
        # 計算左右輪速度
        v_left = linear_velocity - (L / 2) * angular_velocity
        v_right = linear_velocity + (L / 2) * angular_velocity
        return v_left, v_right

    def speed_to_rpm(self, speed):
        wheel_circumference = pi * self.wheel_diameter  # Wheel circumference in meters
        rpm = (speed / wheel_circumference) * 60  # Convert m/s to RPM
        return rpm

    def rpm_to_pwm(self, rpm):
        max_rpm = 176
        # 將RPM映射到0到100的範圍（或者你的PWM控制範圍）
        pwm = (rpm / max_rpm) * 100
        # 確保PWM值在合理範圍內
        pwm = max(0, min(pwm, 100))
        return pwm

    def action_control(self, angle_diff):
        if abs(angle_diff) > self.angle_tolerance:
            if angle_diff < 0:
                return 4
            else:
                return 2
        else:
            return 0  # forward

    def run(self):
        while rclpy.ok():
            self.node.reset()
            start = time.time()
            car_data = self.node.wait_for_data()
            end = time.time()
            twist_data = car_data["navigation_data"]
            #  用/cmd_vel_nav
            linear_velocity = twist_data.linear.x
            angular_velocity = twist_data.angular.z
            v_left, v_right = self.calculate_wheel_speeds(
                linear_velocity, angular_velocity
            )
            rpm_left = self.speed_to_rpm(v_left)
            rpm_right = self.speed_to_rpm(v_right)
            pwm_left = self.rpm_to_pwm(rpm_left)
            pwm_right = self.rpm_to_pwm(rpm_right)
            stop_signal = self.node.check_signal()

            if car_data["car_target_distance"] < 1:
                self.node.publish_to_unity_RESET()
            else:
                front_clear = all(car_data["lidar_data"][i] > 0.35 for i in FRONT_LIDAR_INDICES)
                #  強制設定若距離牆壁0.2就後退
                if not front_clear:
                    action = 3
                elif stop_signal:
                    action = 6
                elif abs(pwm_right - pwm_left) <= 20.0:
                    action = 0
                elif pwm_right > pwm_left:
                    action = 2
                elif pwm_right < pwm_left:
                    action = 4
                elif pwm_right == pwm_left:
                    action = 0
                elif pwm_right == 0 and pwm_left == 0:
                    action = 6
                self.node.publish_to_unity(action)
from utils.navigation_utils import calculate_wheel_speeds, action_choice
from utils.rotate_angle import calculate_angle_to_target
import threading
from robot_arm.robot_control import RobotArmControl
from csv_store_and_file.csv_store import DataCollector
from ros_receive_and_data_processing.config import TARGET_DISTANCE
import time

class NavigationProcess:
    def __init__(self, node):
        self.node = node
        self.robot_controler = RobotArmControl(
            node,
        )
        self.flag = 1
        # self.data_collector = DataCollector()  # 資料收集

    def robot_action_thread(self):
        self.robot_controler.action()

    """
    接收車子經處理過的state
    """

    def node_receive_data(self):
        self.node.reset()
        return self.node.wait_for_data()

    def process_car_data(self, car_data):
        received_global_plan = car_data["received_global_plan"]
        car_position = car_data["car_pos"][:2]
        pid_left, pid_right = calculate_wheel_speeds(car_data["cmd_vel_nav"])
        # print("pid_left, pid_right : ", pid_left, pid_right)
        return received_global_plan, car_position, pid_left, pid_right

    # 到達終點時要做的事
    def handle_reached_destination(self):
        if self.flag == 1:
            print("end")
            action = "STOP"
            # 讓機械手臂動
            robot_thread = threading.Thread(target=self.robot_action_thread)
            robot_thread.start()
            self.node.publish_to_robot(action, pid_control=False)
            self.flag = 0

    # 未到達終點時做的事
    def handle_action(self, car_data):
        self.flag = 1
        """
        stop_signal == True 時代表 nav2 的 cmd_vel_nav 的 topic 沒訊號, 要改用其他控制方式
        車子接收 nav2 的 cmd_vel_nav 的速度移動快碰到牆壁時會花很長時間重算, 因為要等很久
        , 所以發現他在重算就改用其他控制方式
        """
        stop_signal = self.node.check_signal()
        received_global_plan, car_position, pid_left, pid_right = self.process_car_data(
            car_data
        )
        # 將數值傳送至PID控制器
        if stop_signal is not True:
            self.node.publish_to_robot(
                [pid_left, pid_right],
                pid_control=True,
            )
        else:
            """
            因為 received_global_plan 有時會讀取到空值的關係, 若讀取到空值就讓車子停止
            """
            if received_global_plan == None:
                action = "STOP"
                # print("enter stop area")
                self.node.publish_to_robot(action, pid_control=False)
            else:
                """
                根據車頭面向全域路徑給的路徑距離(config設定的NEXT_POINT_DISTANCE)座標計算角度差，
                用角度差決定要用什麼動作
                """
                angle_to_target = calculate_angle_to_target(
                    car_position, received_global_plan, car_data["car_quaternion"]
                )
                action = action_choice(angle_to_target)
                self.node.publish_to_robot(action, pid_control=False)

    def run(self):
        car_data = self.node_receive_data()
        # 車子距離終點一定的距離時便判定到達目標
        if car_data["car_target_distance"] < TARGET_DISTANCE:
            self.handle_reached_destination()
        else:
            self.handle_action(car_data)

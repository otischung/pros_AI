import rclpy
import threading
from ros_receive_and_data_processing.AI_node import AI_node
from avoidance_rule.rule_base import RuleBasedController
# from car_supervised.lstm_inference import supervised_inference
from car_navigation.navigation_main import NavigationController
from robot_arm.robot_control import RobotArmControl

def init_ros_node():
    """node初始化並開一個thread跑ros node"""
    rclpy.init()
    node = AI_node()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread


def main(mode):
    node, ros_thread = init_ros_node()

    if mode == "1":
        rule_controller = RuleBasedController(
            node,
        )
        rule_controller.run()
    # elif mode == "2":
    #     supervised = supervised_inference()
    #     supervised.lstm_inference(node)
    elif mode == "2":
        navigation_controller = NavigationController(
            node,
        )
        navigation_controller.run()
    elif mode == "3":
        robot_controler = RobotArmControl(
            node,
        )
        robot_controler.action()
    else:
        print("Please type the correct numbers.")

    rclpy.shutdown()
    ros_thread.join()


def print_usage():
    print("modes:")
    print(" 1 -- rule-based.")
    # print(" 2 -- supervised learning inference.")
    print(" 2 -- ros2 navigation.")


if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)

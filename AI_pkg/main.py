import rclpy
import threading
from ROS_receive_and_data_processing.AI_node import AI_node
from avoidance_rule.rule_base import RuleBasedController
from car_supervised.lstm_inference import supervised_inference
def init_ros_node():
    '''node初始化並開一個thread跑ros node'''
    rclpy.init()
    node = AI_node()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread

def main():
    node, ros_thread = init_ros_node()

    supervised = supervised_inference()
    supervised.lstm_inference(node)

    # rule_controller = RuleBasedController(
    #     node, 
    #     './Simulated_Annealing_model/parameters.pkl',
    #     load_parameters=True, 
    #     save_to_csv=True
    # )
    
    # rule_controller.run()
    
    rclpy.shutdown()
    ros_thread.join()


if __name__ == '__main__':
    main()

import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import torch
import torch.nn as nn
import torch.nn.functional as F
from utils.obs_utils import *
from utils.adaptor_utils import *
import math
import numpy as np
import rclpy


class supervised_inference():
    def __init__(self):

        self.input_size = 20 # input dimension 15
        self.hidden_size = 128
        self.num_layers = 3
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.lstm = nn.LSTM(self.input_size, self.hidden_size, self.num_layers, batch_first=True).to(device)
        self.linear = nn.Linear(self.hidden_size, 4).to(device) 

        
        self.model_path = './car_supervised/models/best_model.pth'
        self.model_weights = torch.load(self.model_path, map_location=device)
        self.lstm.load_state_dict(self.model_weights)

        self.linear_path = './car_supervised/models/best_model_linear.pth'
        self.linear_weights = torch.load(self.linear_path, map_location=device)
        self.linear.load_state_dict(self.linear_weights)

        self.lstm.eval()

    def lstm_inference(self, node):
        while rclpy.ok():
            token = list()
            _, real_car_data = wait_for_data(node)
            token = self.get_wanted_features(real_car_data)
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.lstm.to(device).eval()
            with torch.inference_mode():
                test_input = [eval(token)]
                test_input_tensor = torch.tensor(test_input, dtype=torch.float32)
                test_input_tensor = test_input_tensor[:, :]
                test_input_tensor = test_input_tensor.unsqueeze(0).to(device)
                h0 = torch.zeros(self.num_layers, test_input_tensor.size(0), self.hidden_size).to(device)
                c0 = torch.zeros(self.num_layers, test_input_tensor.size(0), self.hidden_size).to(device)
                lstm_output, _ = self.lstm(test_input_tensor, (h0, c0))

                lstm_output_last = lstm_output[:, -1, :]
                predicted_output = self.linear(lstm_output_last)

                probabilities = F.softmax(predicted_output, dim=-1)
                print("Softmax probabilities:", probabilities)

                predicted_class = torch.argmax(predicted_output)
                print("Predicted class (Argmax):", predicted_class.item())

                node.publish_to_unity(predicted_class.item())
    
    def get_wanted_features(self, real_car_data):

        lidar_18 = []
        minimum = 999.0
        # print(len(real_car_data["lidar_data"]))
        for index in range(90):
            minimum = min(real_car_data["lidar_data"][index], minimum)
            if index % 5 == 0:
                lidar_18.append(minimum)
                minimum = 999.0
        
        lidar_18 = trans_to_float(lidar_18)
        lidar_18 = round_to_decimal_places(lidar_18)
        token = str([real_car_data["car_target_distance"]] + [real_car_data["angle_diff"]] + lidar_18)
        return token

import numpy as np
import math

def round_to_decimal_places(data_list, decimal_places=3):
    """
    Round the elements of a list to a specified number of decimal places.
    """
    return [round(num, decimal_places) for num in data_list]

def trans_to_float(data_list):
    return [float(i) for i in data_list]

def cal_distance(car_pos,target_pos):
    car_target_distance = (car_pos[0] - target_pos[0])**2 + (car_pos[1] - target_pos[1])**2
    car_target_distance = round_to_decimal_places([math.sqrt(car_target_distance)])[0]
    return car_target_distance
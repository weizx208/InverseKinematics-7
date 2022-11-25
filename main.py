import math
import numpy as np 
from typing import List
import matplotlib.pyplot as plt
from config_parser import ArmConfigParser

CONFIG_PATH = './resources/configs/armConfig - 3links.yml'

if __name__ == "__main__":    
    arm = ArmConfigParser(path=CONFIG_PATH).parse()    
    targets = [
                np.array([-30, 1, 5]),
                np.array([5, 2, 1]),
                np.array([-10, 4, 6]),
                np.array([-30, 9, 2]),
                np.array([20, 1, 5])
            ]

    for target in targets:
        arm.inverse_kinematics(target=target, joint_id=4, lr=.0002, atol=0.01)
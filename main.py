import os
import numpy as np 
from typing import List
import matplotlib.pyplot as plt
from config_parser import ArmConfigParser

CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'armConfig - 3 joints_constr.yml')

if __name__ == "__main__":    
    arm = ArmConfigParser(path=CONFIG_PATH).parse()    
    
    targets = [
                np.array([-30, 1, 5]),              
                np.array([-10, 4, 6]),
                np.array([20, 1, 5])
            ]

    for target in targets:
        arm.inverse_kinematics(target=target, joints_id=[7, 8, 11], lr=.001, atol=0.01)
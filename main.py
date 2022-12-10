import os
import numpy as np 
from typing import List
import matplotlib.pyplot as plt
from config_parser import ArmConfigParser

# arm robot example
#CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'armConfig - 3 joints.yml')
#JOINTS_IDS = [4]

# hand example
CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'hand.yml')
JOINTS_IDS = [7, 8, 11, 16, 17]

if __name__ == "__main__":    
    arm = ArmConfigParser(path=CONFIG_PATH).parse()    
    
    targets = [
                np.array([-30, 1, 5]),              
                np.array([-10, 4, 6]),
                np.array([20, 1, 5])
            ]

    for target in targets:
        arm.inverse_kinematics(target=target, joints_id=JOINTS_IDS, lr=.0001, atol=0.01)
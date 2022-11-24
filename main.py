import math
import numpy as np 
from typing import List
import matplotlib.pyplot as plt
from config_parser import ArmConfigParser

#CONFIG_PATH = './resources/configs/armConfig - 2links.yml'
CONFIG_PATH = './resources/configs/armConfig - 3links.yml'

if __name__ == "__main__":    
    arm = ArmConfigParser(path=CONFIG_PATH).parse()
    arm.delta_commands[0,:] = np.array([0, 0, -np.pi/4, 0,0,0])
    arm.delta_commands[1,:] = np.array([0, 0, -np.pi/4, 0,0,0])    
    arm.delta_commands[2,:] = np.array([0, 0, -np.pi/4, 0,0,0])    
    arm.forward_kinematics()

    target = np.array([-10, -1, 5])
    arm.inverse_kinematics(target=target, joint_id=3, lr=.001, atol=0.01)
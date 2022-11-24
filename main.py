import math
import numpy as np 
from typing import List
import matplotlib.pyplot as plt
from config_parser import ArmConfigParser

CONFIG_PATH = './armConfig.yml'

if __name__ == "__main__":    
    arm = ArmConfigParser(path=CONFIG_PATH).parse()
    arm.delta_commands[0,:] = np.array([0, 0, -np.pi/4, 0,0,0])
    arm.delta_commands[1,:] = np.array([0, 0, -np.pi/4, 0,0,0])
    
    arm.forward_kinematics()
    plt.figure()
    plt.plot(-10, -1, marker='*')
    arm.plot_config(joints_coords=arm.vertices)
    arm.inverse_kinematics(target=np.array([-10, -1, 0]), lr=.001)
    plt.show()
    print("test")        
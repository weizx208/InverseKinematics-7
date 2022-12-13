import os
import numpy as np 
from config_parser import RobotConfigParser

# 2 joints unconstrained
#CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'armConfig - 2 joints.yml')
#JOINTS_IDS = [2]

# 3 joints unconstrained
CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'armConfig - 3 joints.yml')
JOINTS_IDS = [4]

# 3 joints constrained
#CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'armConfig - 3 joints_constr.yml')
#JOINTS_IDS = [4]

# hand 
#CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'hand.yml')
#JOINTS_IDS = [7, 8, 11, 16, 17]


if __name__ == "__main__":    
    robot = RobotConfigParser(path=CONFIG_PATH).parse()    
    
    targets = [
                np.array([-30, 1, 5]),              
                np.array([-10, 4, 6]),
                np.array([20, 1, 5])
            ]

    for target in targets:
        robot.inverse_kinematics(target=target, joints_id=JOINTS_IDS, lr=.03)
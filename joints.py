import numpy as np
from typing import List, Dict, Any
from actuator import Actuator

class Joint:
    def __init__(self, id: int,
                 actuators: List[Actuator]):
        self.id = id          
        self.actuators = actuators
        # 3 vectors representing the joint world basis
        # defines changes with respect the initial position/rotation 
        # of the joint 
        """
            |x_x|y_x|z_x|
            |x_y|y_y|z_y|
            |x_z|y_z|z_z|
        """
        self.world_basis = np.eye(3)
    
    def update_world_coords_sys(self, new_angles: np.array):   
        for act in self.actuators:
            _ = act.apply(vector=np.zeros(shape=(3, 1)), 
                          delta_commands=new_angles, 
                          father_joint_sys=self.world_basis,
                          child_joint_sys=self.world_basis
                        )     
        

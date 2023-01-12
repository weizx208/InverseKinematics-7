import utils
from typing import Dict
from enum import Enum
import numpy as np
from abc import ABC, abstractmethod


class Actuator(ABC):
    def __init__(self, constraints: Dict[str, float]):
        self.constraints = constraints

    @abstractmethod
    def actuate(self, vector: np.array, delta_commands: np.array,
              father_joint_sys: np.array, child_joint_sys: np.array):    
        pass

class RotaryActuator(Actuator):
    def __init__(self, axis: int, constraints: Dict[str, float], start_angle: float):
        super(RotaryActuator, self).__init__(constraints=constraints)
        self.id = id    
        self.angle = start_angle
        self.axis = axis
        if self.constraints['max'] != np.inf:
            self.constraints['max'] *= np.pi/180
        if self.constraints['min'] != -np.inf:
            self.constraints['min'] *= np.pi/180

    def actuate(self, child_config: Dict[str, np.array], parent_config: Dict[str, np.array], command: np.array) -> None:                        
        self.angle += command[self.axis]   
        temp_angle = min(self.constraints['max'], self.angle)
        temp_angle = max(self.constraints['min'], temp_angle)         
        command[self.axis] += temp_angle - self.angle 

        if child_config["id"] == parent_config["id"]:
            self.angle = temp_angle
        else:
            self.angle -= command[self.axis] - (temp_angle - self.angle) 


        q = utils.axisangle_to_q(v=parent_config["basis"][self.axis,:], theta=command[self.axis])

        # update child joint system
        for i in range(3):
            child_config["basis"][i, :] = utils.qv_mult(q, child_config["basis"][i, :])

        child_config["coords"][:] = utils.qv_mult(q, child_config["coords"] - parent_config["coords"]) + parent_config["coords"]


class ActuatorSelector(Enum):
    ROTARY_ACTUATOR = RotaryActuator
    
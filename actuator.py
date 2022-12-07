import utils
from typing import Dict
from enum import Enum
import numpy as np
from abc import ABC, abstractmethod


class Actuator(ABC):
    def __init__(self, constraints: Dict[str, float]):
        self.constraints = constraints

    @abstractmethod
    def apply(self, vector: np.array, delta_commands: np.array,
              father_joint_sys: np.array, child_joint_sys: np.array):    
        pass


class XAxisMotor(Actuator):
    def __init__(self, constraints: Dict[str, float], start_angle: float):
        super(XAxisMotor, self).__init__(constraints=constraints)
        self.id = id
        self.angle = start_angle
        if self.constraints['max'] != np.inf:
            self.constraints['max'] *= np.pi/180
        if self.constraints['min'] != -np.inf:
            self.constraints['min'] *= np.pi/180

    def apply(self, child_config: Dict[str, np.array], parent_config: Dict[str, np.array], command: np.array) -> None:            
        q = utils.axisangle_to_q(v=parent_config["basis"][0,:], theta=command[0])

        # update child joint system
        for i in range(3):
            child_config["basis"][i, :] = utils.qv_mult(q, child_config["basis"][i, :])
    
        child_config["coords"][:] = utils.qv_mult(q, child_config["coords"] - parent_config["coords"]) + parent_config["coords"]        


class YAxisMotor(Actuator):
    def __init__(self, constraints: Dict[str, float], start_angle: float):
        super(YAxisMotor, self).__init__(constraints=constraints)
        self.id = id
        self.angle = start_angle
        if self.constraints['max'] != np.inf:
            self.constraints['max'] *= np.pi/180
        if self.constraints['min'] != -np.inf:
            self.constraints['min'] *= np.pi/180

    def apply(self, child_config: Dict[str, np.array], parent_config: Dict[str, np.array], command: np.array) -> None:
        q = utils.axisangle_to_q(v=parent_config["basis"][1,:], theta=command[1])

        # update child joint system
        for i in range(3):
            child_config["basis"][i, :] = utils.qv_mult(q, child_config["basis"][i, :])
    
        child_config["coords"] = utils.qv_mult(q, child_config["coords"] - parent_config["coords"]) + parent_config["coords"]


class ZAxisMotor(Actuator):
    def __init__(self, constraints: Dict[str, float], start_angle: float):
        super(ZAxisMotor, self).__init__(constraints=constraints)
        self.id = id    
        self.angle = start_angle
        if self.constraints['max'] != np.inf:
            self.constraints['max'] *= np.pi/180
        if self.constraints['min'] != -np.inf:
            self.constraints['min'] *= np.pi/180

    def apply(self, child_config: Dict[str, np.array], parent_config: Dict[str, np.array], command: np.array) -> None:                 
        q = utils.axisangle_to_q(v=parent_config["basis"][2,:], theta=command[2])

        # update child joint system
        for i in range(3):
            child_config["basis"][i, :] = utils.qv_mult(q, child_config["basis"][i, :])
    
        child_config["coords"][:] = utils.qv_mult(q, child_config["coords"] - parent_config["coords"]) + parent_config["coords"]


class ActuatorSelector(Enum):
    X_AXIS_MOTOR = XAxisMotor
    Y_AXIS_MOTOR = YAxisMotor
    Z_AXIS_MOTOR = ZAxisMotor

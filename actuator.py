import utils
import math
from typing import Dict
from enum import Enum
import numpy as np
from abc import ABC, abstractmethod


class Actuator(ABC):
    def __init__(self, constraints: Dict[str, float]):
        self.constrants = constraints

    @abstractmethod
    def apply(self, vector: np.array, delta_commands: np.array,
              father_joint_sys: np.array, child_joint_sys: np.array):    
        pass


class XAxisMotor(Actuator):
    def __init__(self, constraints: Dict[str, float], start_angle: float):
        super(XAxisMotor, self).__init__(constraints=constraints)
        self.id = id
        self.angle = start_angle

    def apply(self, vector: np.array, delta_commands: np.array, father_joint_sys: np.array, child_joint_sys: np.array):            
        q = utils.axisangle_to_q(v=np.array(father_joint_sys[:, 0]), theta=delta_commands[0])

        # update child joint system
        child_joint_sys[:, 0] = utils.qv_mult(q, child_joint_sys[:, 0])
        child_joint_sys[:, 1] = utils.qv_mult(q, child_joint_sys[:, 1])
        child_joint_sys[:, 2] = utils.qv_mult(q, child_joint_sys[:, 2])
            
        self.angle += delta_commands[0]   
        return utils.qv_mult(q, vector)


class YAxisMotor(Actuator):
    def __init__(self, constraints: Dict[str, float], start_angle: float):
        super(YAxisMotor, self).__init__(constraints=constraints)
        self.id = id
        self.angle = start_angle

    def apply(self, vector: np.array, delta_commands: np.array, father_joint_sys: np.array, child_joint_sys: np.array):            
        q = utils.axisangle_to_q(v=np.array(father_joint_sys[:, 1]), theta=delta_commands[1])

        # update child joint system
        child_joint_sys[:, 0] = utils.qv_mult(q, child_joint_sys[:, 0])
        child_joint_sys[:, 1] = utils.qv_mult(q, child_joint_sys[:, 1])
        child_joint_sys[:, 2] = utils.qv_mult(q, child_joint_sys[:, 2])
        
            
        self.angle += delta_commands[1]   
        return utils.qv_mult(q, vector)


class ZAxisMotor(Actuator):
    def __init__(self, constraints: Dict[str, float], start_angle: float):
        super(ZAxisMotor, self).__init__(constraints=constraints)
        self.id = id    
        self.angle = start_angle

    def apply(self, vector: np.array, delta_commands: np.array, father_joint_sys: np.array, child_joint_sys: np.array):            
        q = utils.axisangle_to_q(v=np.array(father_joint_sys[:, 2]), theta=delta_commands[2])

        # update child joint system
        child_joint_sys[:, 0] = utils.qv_mult(q, child_joint_sys[:, 0])
        child_joint_sys[:, 1] = utils.qv_mult(q, child_joint_sys[:, 1])
        child_joint_sys[:, 2] = utils.qv_mult(q, child_joint_sys[:, 2])
            
        self.angle += delta_commands[2]   
        return utils.qv_mult(q, vector)


class ActuatorSelector(Enum):
    X_AXIS_MOTOR = XAxisMotor
    Y_AXIS_MOTOR = YAxisMotor
    Z_AXIS_MOTOR = ZAxisMotor

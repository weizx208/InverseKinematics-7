import math
from typing import Dict
from enum import Enum
import numpy as np
from abc import ABC, abstractmethod


class Actuator(ABC):
    def __init__(self, n_dims: int, constraints: Dict[str, float]):
        if n_dims not in [2, 3]:
            raise ValueError(f"invalid number of dimensions")

    @abstractmethod
    def apply(self, vector: np.array, delta_commands: np.array):
        pass


class XAxisMotor(Actuator):
    def __init__(self, n_dims: int, constraints: Dict[str, float]):
        super(XAxisMotor, self).__init__(n_dims=n_dims, constraints=constraints)
        self.id = id
        self.x_norm = np.array([1, 0, 0])

    def apply(self, vector: np.array, delta_commands: np.array):    
        q = utils.axisangle_to_q(v=self.x_norm, theta=delta_commands[0])
        return np.array(utils.qv_mult(q, vector))



class YAxisMotor(Actuator):
    def __init__(self, n_dims: int, constraints: Dict[str, float]):
        super(YAxisMotor, self).__init__(n_dims=n_dims, constraints=constraints)
        self.id = id
        self.y_norm = np.array([0, 1, 0])

    def apply(self, vector: np.array, delta_commands: np.array):    
        q = utils.axisangle_to_q(v=self.y_norm, theta=delta_commands[1])
        return np.array(utils.qv_mult(q, vector))


import utils
class ZAxisMotor(Actuator):
    def __init__(self, n_dims: int, constraints: Dict[str, float]):
        super(ZAxisMotor, self).__init__(n_dims=n_dims, constraints=constraints)
        self.id = id
        self.z_norm = np.array([0, 0, 1])

    def apply(self, vector: np.array, delta_commands: np.array):    
        q = utils.axisangle_to_q(v=self.z_norm, theta=delta_commands[2])
        return np.array(utils.qv_mult(q, vector))


class ActuatorSelector(Enum):
    X_AXIS_MOTOR = XAxisMotor
    Y_AXIS_MOTOR = YAxisMotor
    Z_AXIS_MOTOR = ZAxisMotor

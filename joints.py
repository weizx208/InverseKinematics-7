import numpy as np
from typing import List, Dict, Any
from actuator import Actuator

class Joint:
    def __init__(self, id: int,
                 actuators: List[Actuator],
                 angle_config: np.array = np.array([0, 0, 0])):
        self.id = id  
        self.angle_conf = angle_config.astype(np.float)
        self.actuators = actuators
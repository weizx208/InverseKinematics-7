import numpy as np
from typing import List, Dict, Any
from actuator import Actuator
from dataclasses import dataclass


@dataclass
class Joint:
    id: int
    actuators: List[Actuator]
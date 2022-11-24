import yaml 
import numpy as np
from robot import RobotArm
from actuator import ActuatorSelector
from dof import DegreeOfFreedom
from joints import Joint

class ArmConfigParser:
    def __init__(self, path: str):
        with open(path, "r") as stream:
            try:
                self.config = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)       

    def parse(self) -> RobotArm:            
        joints = []
        for joint in self.config["Joints"]:          
            actuators = []
            for act_index, act in enumerate(joint["actuators"]):
                d = ActuatorSelector[act["type"]].value            
                d_inst = d.__new__(d, constraints=act["constraints"],
                                      n_dims=self.config["n_dims"])
                d_inst.__init__(constraints=act["constraints"],
                                n_dims=self.config["n_dims"])
                actuators.append(d_inst)        
            joints.append(Joint(id = joint["id"], 
                                angle_config = np.array(joint["angles"]), 
                                actuators=actuators)
                         )
        return RobotArm(joints=joints, 
                        vertices=np.array(self.config["Geometry"]["vertices"]),
                        edges=self.config["Geometry"]["edges"])        

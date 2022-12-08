import yaml 
from robot import RobotArm
from actuator import ActuatorSelector
from joints import Joint

class ArmConfigParser:
    def __init__(self, path: str):
        with open(path, "r") as stream:
            try:
                self.config = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)       
                exit(-1)

    def parse(self) -> RobotArm:            
        joints = []
        for joint in self.config["Joints"]:          
            actuators = []
            for act in joint["actuators"]:
                d = ActuatorSelector[act["type"]].value
                del act['type']            
                d_inst = d.__new__(d, **act)
                d_inst.__init__(**act)
                actuators.append(d_inst)        
            joints.append(Joint(id = joint["id"], actuators=actuators))
        return RobotArm(joints=joints,  
                        vertices=self.config["Geometry"]["vertices"],
                        edges=self.config["Geometry"]["edges"])        

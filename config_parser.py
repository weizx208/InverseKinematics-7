import yaml 
from robot import Robot
from actuator import ActuatorSelector
from joints import Joint

class RobotConfigParser:    
    def __init__(self, path: str) -> None:
        """
            path: str - Path to configuration file    
        """
        with open(path, "r") as stream:
            try:
                self.config = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)       
                exit(-1)

    def parse(self) -> Robot:
        """            
            return Robot
        
            Parse config file into a Robot class instance 
        """
        joints = []
        for joint in self.config["Joints"]:          
            actuators = []
            for act in joint["actuators"]:
                # Dynamically create actuators instances
                d = ActuatorSelector[act["type"]].value
                del act['type']            
                d_inst = d.__new__(d, **act)
                d_inst.__init__(**act)                
                actuators.append(d_inst)        
            # Instanciate a joint with the set of actuators provided  
            joints.append(Joint(id = joint["id"], actuators=actuators))
        # Create the final instance of the robot 
        return Robot(joints=joints,  
                    vertices=self.config["Geometry"]["vertices"],
                    edges=self.config["Geometry"]["edges"])        

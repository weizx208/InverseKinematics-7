import math
import numpy as np
from abc import ABC
from typing import List
from joints import Joint
import matplotlib.pyplot as plt

class Robot(ABC):    
    def __init__(self, joints: List[Joint],
                 vertices: List[np.array], 
                 edges: List[List[int]]):
        """
            delta_commands: np.array >> each row contains roto-translations of a joint
                                  --------------------------------------------------------
                            J_0 : |  d_angle_x | d_angle_y | d_angle_z | d_x | d_y | d_z |                           
                                  --------------------------------------------------------
                            J_1 : |  d_angle_x | d_angle_y | d_angle_z | d_x | d_y | d_z |   
                                  --------------------------------------------------------
                                                        .....
                                  --------------------------------------------------------
                            J_n : |  d_angle_x | d_angle_y | d_angle_z | d_x | d_y | d_z |   
                                  --------------------------------------------------------
        """ 
        self.joints = joints
        self.vertices = vertices.astype(np.float) 
        self.edges = edges   
        self.delta_commands = np.zeros(shape=(len(joints), 6))
        self.visited_states = [False]*len(self.joints)

class RobotArm(Robot):
    def __init__(self, joints: List[Joint],
                 vertices: np.array, 
                 edges: List[List[int]]):
        super(RobotArm, self).__init__(joints=joints, vertices=vertices, edges=edges)
    
    @staticmethod
    def vertices_distance(pt1: np.array, pt2: np.array):
        np.numpy.linalg.norm(pt1 - pt2, axis=0)
    
    def transform_child_dfs(self, father_joint_id: int,
                            linked_joint_id: int):
        self.visited_states[linked_joint_id] = True
        for v in self.edges[linked_joint_id]: 
            if not self.visited_states[v]:                            
                self.transform_child_dfs(father_joint_id=father_joint_id, 
                                         linked_joint_id=v)
                for act in self.joints[father_joint_id].actuators:
                    self.vertices[v, :] = act.apply(vector=self.vertices[v, :] - self.vertices[father_joint_id, :], 
                                                    delta_commands=self.delta_commands[father_joint_id, :])                                                    
                    self.vertices[v, :] += self.vertices[father_joint_id, :]


    def forward_kinematics(self) -> np.array:                    
        for joint in self.joints:
            # todo: check dof constraints 
            joint.angle_conf += self.delta_commands[joint.id, :3]   
            self.vertices[joint.id, :] += self.delta_commands[joint.id, 3:]           

            # DFS of the linked edges
            self.visited_states = [False]*self.vertices.shape[0]
            self.transform_child_dfs(father_joint_id=joint.id, 
                                     linked_joint_id=joint.id)           

        self.delta_commands = np.zeros(shape=(len(self.joints), 6))  

    def calc_error(self, p1: np.array, p2: np.array):
        return math.sqrt(np.sum((p2-p1)**2))

    def inverse_kinematics(self, target: np.array, lr: float = 10):
        angle_dist = np.pi/180  # equivalent to 1 deg 
        count = 0
        max_count = 1000
        while count < max_count:
            for j in self.joints:         
                ### loop through angles 
                self.delta_commands[j.id, :3] = np.array([angle_dist]*3)           
                self.forward_kinematics()            
                new_error = self.calc_error(self.vertices[-1, :], target)

                self.delta_commands[j.id, :3] = -np.array([angle_dist]*3)           
                self.forward_kinematics()     
                error = self.calc_error(self.vertices[-1, :], target)

                self.delta_commands[j.id, :3] = -np.array([lr*(new_error-error)/angle_dist]*3)
                self.forward_kinematics() 
            count += 1
            
            # plot final config
            self.plot_config(joints_coords = self.vertices, color='r')        
            plt.pause(.1)
        
    
    def plot_config(self, joints_coords: np.array, color: str = 'b') -> None:    
        line_xs = np.linspace(joints_coords[0, 0], joints_coords[1, 0], 100)    
        line_ys = np.linspace(joints_coords[0, 1], joints_coords[1, 1], 100)    
        plt.plot(line_xs, line_ys, color=color)
        line_xs = np.linspace(joints_coords[1, 0], joints_coords[2, 0], 100)    
        line_ys = np.linspace(joints_coords[1, 1], joints_coords[2, 1], 100)    
        plt.plot(line_xs, line_ys, color=color)

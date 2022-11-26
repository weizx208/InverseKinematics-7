import os
import math
import numpy as np
from abc import ABC
from typing import List
from joints import Joint
import matplotlib.pyplot as plt


class Robot(ABC):    
    def __init__(self, n_dims: int, joints: List[Joint],
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
        if n_dims not in [2, 3]:
            raise ValueError(f"invalid number of dimensions")
        self.n_dims = n_dims
        self.joints = joints
        self.vertices = vertices.astype(np.float64) 
        self.edges = edges   
        self.delta_commands = np.zeros(shape=(len(joints), 6))
        self.visited_states = [False]*len(self.joints)
        self.plot = plt.figure().add_subplot(projection='3d')    
   
class RobotArm(Robot):
    def __init__(self, n_dims:int, joints: List[Joint],
                 vertices: np.array, 
                 edges: List[List[int]]):
        super(RobotArm, self).__init__(n_dims=n_dims, joints=joints, vertices=vertices, edges=edges)
    
    @staticmethod
    def vertices_distance(pt1: np.array, pt2: np.array):
        return np.linalg.norm(pt1 - pt2)
    
    def transform_child_dfs(self, father_joint_id: int,
                            linked_joint_id: int, act):
        self.visited_states[linked_joint_id] = True
        for v in self.edges[linked_joint_id]: 
            if not self.visited_states[v]:                                                          
                self.vertices[v, :] = act.apply(vector=self.vertices[v, :] - self.vertices[father_joint_id, :], 
                                                delta_commands=self.delta_commands[father_joint_id, :], 
                                                father_joint_sys=self.joints[father_joint_id].world_basis,
                                                child_joint_sys=self.joints[v].world_basis
                                                )                       
                self.vertices[v, :] += self.vertices[father_joint_id, :]    

                self.transform_child_dfs(father_joint_id=father_joint_id, 
                                        linked_joint_id=v, act=act)

    def forward_kinematics(self) -> np.array:                    
        for joint in self.joints:
            # todo: check dof constraints 
            
            # DFS of the linked edges
            # update father joint basis            
            for act in self.joints[joint.id].actuators:    
                self.visited_states = [False]*self.vertices.shape[0]
                self.transform_child_dfs(father_joint_id=joint.id, 
                                        linked_joint_id=joint.id, act=act)    
                # change coords basis of the father joint
                act.apply(
                            vector=np.zeros(shape=(3, 1)), 
                            delta_commands=self.delta_commands[joint.id, :], 
                            father_joint_sys=joint.world_basis,
                            child_joint_sys=joint.world_basis
                        )                        
            
        self.delta_commands[joint.id, :] = np.zeros(shape=(1, 6))  

    def inverse_kinematics(self, target: np.array, joint_id: int, atol: float = 0.1, lr: float = 10):            
        angle_dist = np.pi/180  # equivalent to 1 deg 
        count = 0
        max_count = 100
        while count < max_count and not np.allclose(self.vertices[joint_id, :], target, atol=atol):
            for j in self.joints:      
                for i in range(self.n_dims+1):   
                    ### loop through angles 
                    self.delta_commands[j.id, i] = angle_dist
                    self.forward_kinematics()                            
                    new_error = self.vertices_distance(pt1=self.vertices[joint_id, :], pt2=target)

                    self.delta_commands[j.id, i] = -angle_dist
                    self.forward_kinematics()     
                    error = self.vertices_distance(pt1=self.vertices[joint_id, :], pt2=target)

                    self.delta_commands[j.id, i] = -lr*(new_error-error)/angle_dist
                    self.forward_kinematics() 
            count += 1
            
            # plot final config
            self.visited_states = [False]*self.vertices.shape[0]
            self.plot.clear()     
            
            self.plot.set_xlim(-20, 20)
            self.plot.set_ylim(-20, 20)
            self.plot.set_zlim(0, 20)
            self.plot.set_xlabel('X')
            self.plot.set_ylabel('Y')
            self.plot.set_zlabel('Z')       
            self.plot.plot(target[0], target[1], target[2], marker='*')
            self.plot_config(vertex_id=0, color='r')                
            plt.pause(0.0001)

             
    def plot_config(self, vertex_id: np.array, color: str = 'b') -> None:           
        self.visited_states[vertex_id] = True
        
        # plot joint basis
        line_xs = np.linspace(self.vertices[vertex_id, 0], self.vertices[vertex_id, 0]+self.joints[vertex_id].world_basis[0, 0], 100)    
        line_ys = np.linspace(self.vertices[vertex_id, 1], self.vertices[vertex_id, 1]+self.joints[vertex_id].world_basis[1, 0], 100)    
        line_zs = np.linspace(self.vertices[vertex_id, 2], self.vertices[vertex_id, 2]+self.joints[vertex_id].world_basis[2, 0], 100)  
        self.plot.plot(line_xs, line_ys, line_zs, color='b')      
        line_xs = np.linspace(self.vertices[vertex_id, 0], self.vertices[vertex_id, 0]+self.joints[vertex_id].world_basis[0, 1], 100)    
        line_ys = np.linspace(self.vertices[vertex_id, 1], self.vertices[vertex_id, 1]+self.joints[vertex_id].world_basis[1, 1], 100)    
        line_zs = np.linspace(self.vertices[vertex_id, 2], self.vertices[vertex_id, 2]+self.joints[vertex_id].world_basis[2, 1], 100)  
        self.plot.plot(line_xs, line_ys, line_zs, color='g')      
        line_xs = np.linspace(self.vertices[vertex_id, 0], self.vertices[vertex_id, 0]+self.joints[vertex_id].world_basis[0, 2], 100)    
        line_ys = np.linspace(self.vertices[vertex_id, 1], self.vertices[vertex_id, 1]+self.joints[vertex_id].world_basis[1, 2], 100)    
        line_zs = np.linspace(self.vertices[vertex_id, 2], self.vertices[vertex_id, 2]+self.joints[vertex_id].world_basis[2, 2], 100)  
        self.plot.plot(line_xs, line_ys, line_zs, color='r')   


        for v in self.edges[vertex_id]: 
            if not self.visited_states[v]:         
                line_xs = np.linspace(self.vertices[vertex_id, 0], self.vertices[v, 0], 100)    
                line_ys = np.linspace(self.vertices[vertex_id, 1], self.vertices[v, 1], 100)    
                line_zs = np.linspace(self.vertices[vertex_id, 2], self.vertices[v, 2], 100)  
                self.plot.plot(line_xs, line_ys, line_zs, zdir='z', color=color)                       
                self.plot_config(vertex_id=v)              

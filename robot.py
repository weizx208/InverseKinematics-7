import numpy as np
from abc import ABC
from typing import List, Dict
from joints import Joint
import matplotlib.pyplot as plt
from skeleton import Skeleton


class Robot(ABC):    
    def __init__(self, joints: List[Joint],
                 vertices: Dict[str, List[float]], 
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
        # create skeleton
        self.skeleton = Skeleton(joints=joints, 
                                 joints_loc=np.array(vertices["coords"]).astype(np.float64), 
                                 joints_angles=np.array(vertices["angles"]).astype(np.float64)*np.pi/180,
                                 edges=edges)        
        self.delta_commands = np.zeros(shape=(len(joints), 6))
        self.plot = plt.figure().add_subplot(projection='3d')       
    
    @staticmethod
    def vertices_distance(pt_list: np.array, target: np.array):
        return np.sum(np.sum((pt_list - target)**2, axis=1))
    
    def apply_kinematics(self, skeleton: Skeleton, joint_id: int ) -> np.array:                            
        skeleton.process_command(joint_id=joint_id, command=self.delta_commands[joint_id, :])
        self.delta_commands[joint_id, :] = np.zeros(shape=(1, 6))  

    def inverse_kinematics(self, target: np.array, joints_id: List[int], atol: float = 0.1, lr: float = 10):            
        angle_dist = np.pi/180  # equivalent to 1 deg 
        count = 0
        max_count = 50
        while count < max_count:
            for j in self.skeleton.joints:     
                for i in range(3):
                    shadow = self.skeleton.get_shadow()                                       
                    error = self.vertices_distance(pt_list=shadow.joints_loc[joints_id, :], target=target)

                    self.delta_commands[j.id, i] = angle_dist
                    self.apply_kinematics(skeleton=shadow, joint_id=j.id)                            
                    new_error = self.vertices_distance(pt_list=shadow.joints_loc[joints_id, :], target=target)

                    self.delta_commands[j.id, i] = -lr*(new_error-error)/angle_dist
                    self.apply_kinematics(skeleton=self.skeleton, joint_id=j.id) 
            count += 1    
            self.refresh_plot(target=target)
            
    def refresh_plot(self, target: np.array) -> None:
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
        self.skeleton.visited[vertex_id] = True
        
        # plot joint basis
        line_xs = np.linspace(self.skeleton.joints_loc[vertex_id, 0], self.skeleton.joints_loc[vertex_id, 0]+self.skeleton.joints_basis[vertex_id][0, 0], 100)    
        line_ys = np.linspace(self.skeleton.joints_loc[vertex_id, 1], self.skeleton.joints_loc[vertex_id, 1]+self.skeleton.joints_basis[vertex_id][0, 1], 100)    
        line_zs = np.linspace(self.skeleton.joints_loc[vertex_id, 2], self.skeleton.joints_loc[vertex_id, 2]+self.skeleton.joints_basis[vertex_id][0, 2], 100)  
        self.plot.plot(line_xs, line_ys, line_zs, color='b')      
        line_xs = np.linspace(self.skeleton.joints_loc[vertex_id, 0], self.skeleton.joints_loc[vertex_id, 0]+self.skeleton.joints_basis[vertex_id][1, 0], 100)    
        line_ys = np.linspace(self.skeleton.joints_loc[vertex_id, 1], self.skeleton.joints_loc[vertex_id, 1]+self.skeleton.joints_basis[vertex_id][1, 1], 100)    
        line_zs = np.linspace(self.skeleton.joints_loc[vertex_id, 2], self.skeleton.joints_loc[vertex_id, 2]+self.skeleton.joints_basis[vertex_id][1, 2], 100)  
        self.plot.plot(line_xs, line_ys, line_zs, color='g')      
        line_xs = np.linspace(self.skeleton.joints_loc[vertex_id, 0], self.skeleton.joints_loc[vertex_id, 0]+self.skeleton.joints_basis[vertex_id][2, 0], 100)    
        line_ys = np.linspace(self.skeleton.joints_loc[vertex_id, 1], self.skeleton.joints_loc[vertex_id, 1]+self.skeleton.joints_basis[vertex_id][2, 1], 100)    
        line_zs = np.linspace(self.skeleton.joints_loc[vertex_id, 2], self.skeleton.joints_loc[vertex_id, 2]+self.skeleton.joints_basis[vertex_id][2, 2], 100)  
        self.plot.plot(line_xs, line_ys, line_zs, color='r')   


        for v in self.skeleton.edges[vertex_id]: 
            if not self.skeleton.visited[v]:         
                line_xs = np.linspace(self.skeleton.joints_loc[vertex_id, 0], self.skeleton.joints_loc[v, 0], 100)    
                line_ys = np.linspace(self.skeleton.joints_loc[vertex_id, 1], self.skeleton.joints_loc[v, 1], 100)    
                line_zs = np.linspace(self.skeleton.joints_loc[vertex_id, 2], self.skeleton.joints_loc[v, 2], 100)  
                self.plot.plot(line_xs, line_ys, line_zs, zdir='z', color=color)                       
                self.plot_config(vertex_id=v)    
        self.skeleton.refresh_visited_state()
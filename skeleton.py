from __future__ import annotations
import numpy as np
import copy
from typing import Dict, List, Any
import matplotlib.pyplot as plt
import utils 
from joints import Joint


class Skeleton:    
    def __init__(self, 
                joints: List[Joint], 
                joints_loc: List[np.array],
                joints_angles: List[np.array],
                edges: List[List[int]]) -> None:
        """        
            The Skeleton is a collection of informations related to the actual geometry of the robot.
        """
        
        super(Skeleton, self).__init__()
        self.joints: List[Joint] = joints
        self.edges: List[List[int]] = edges
        self.edges_t: List[List[int]] = []                
        self.transpose_edges()        
        self.visited: List[bool] = [False]*len(self.joints)          
        # child related to each joint
        self.childs_collection: List[List[int]] = []                
        
        self.__initialize_joints(jnts_strt_loc=joints_loc, jnts_strt_ngls=joints_angles)
                
    def __initialize_joints(self, jnts_strt_loc: List[np.array], 
                                jnts_strt_ngls: List[np.array]):   
        """
            Initialize variables related to skeleton joints
        """     
        for i in range(len(self.joints)):
            # append child related to selected joint
            self.childs_collection.append(self.joint_dfs_traversing(i))
            self.refresh_visited_state()

        # init joints locations 
        for j in self.joints:            
            j.location = jnts_strt_loc[j.id]

        # init joints orientation 
        self.__set_joints_basis_alignement(joints_angles=jnts_strt_ngls) 
    
    def transpose_edges(self):        
        """
            Transpose the adjacency list
        """
        self.edges_t = [[] for i in range(len(self.edges))] 
        for e_index, e in enumerate(self.edges):
            for index in e:                
                self.edges_t[index] += [e_index]
                
    def __get_basis_target_z(self, joint_id: int) -> np.array:
        """
            Define the orientation of the target z axis.
            The target axis have to point to the childs locations. 
        """
        point_to: List[np.array] = []
        for c in self.edges[joint_id]:
            point_to.append(self.joints[c].location)
        
        if len(point_to) == 0:
            point_to = [self.joints[self.edges_t[joint_id][0]].location]
            return - np.mean(np.array(point_to), axis=0) + self.joints[joint_id].location
        return np.mean(np.array(point_to), axis=0) - self.joints[joint_id].location
                
    def __set_joints_basis_alignement(self, joints_angles: List[np.array]):        
        """
            Define joints basis orientations.
        """
        for j in self.joints:            
            z = self.__get_basis_target_z(joint_id=j.id)
            z = z/np.linalg.norm(z)            
            
            j.align_basis_to_target_z(target_z=z)
            j.rotate_basis_by_angles(angles=joints_angles[j.id])
            
    def refresh_visited_state(self):        
        self.visited = [False]*len(self.joints)   

    def get_skeleton_config(self) -> List[Dict[str, Any]]:
        conf = []
        for j in self.joints:
           conf + self.joints[j.id].get_joint_config()
        return conf    
    
    def joint_dfs_traversing(self, parent_id: int) -> List[int]:
        """
            Depth first search trasversing of the skeleton.
        """
        self.visited[parent_id] = True
        childs = []
        
        for j in self.edges[parent_id]: 
            if not self.visited[j]:                                                                                                                        
                childs += [j]+self.joint_dfs_traversing(parent_id=j)
        return childs    

    def process_command(self, joint_id: int, command: np.array) -> None:             
        childs_id = self.childs_collection[joint_id]
        childs_cfgs = [self.joints[c].config for c in childs_id] 
        self.joints[joint_id].apply_command(childs_configs=childs_cfgs, command=command)        
                                            
    def get_shadow(self) -> Skeleton:  
        """
            Get a copy of the Skeleton
        """      
        return copy.deepcopy(self)

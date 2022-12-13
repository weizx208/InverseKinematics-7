from __future__ import annotations
import numpy as np
import copy
from typing import Dict, List, Any
import utils 
from joints import Joint

class Skeleton:    
    def __init__(self, 
                joints: List[Joint], 
                joints_loc: List[np.array],
                joints_angles: List[np.array],
                edges: List[List[int]]) -> None:
        super(Skeleton, self).__init__()
        self.joints: List[Joint] = joints
        self.joints_loc: List[np.array] = joints_loc
        self.edges: List[List[int]] = edges
        self.edges_t: List[List[int]] = []
        self.transpose_edges()
        self.visited: List[bool] = [False]*len(self.joints)     
        self.joints_basis: List[np.array] = [np.eye(3) for _ in range(len(self.joints))]
        self.set_joints_basis_alignemt(joints_angles=joints_angles)    

    def transpose_edges(self):        
        self.edges_t = [[] for i in range(len(self.edges))] 
        for e_index, e in enumerate(self.edges):
            for index in e:                
                self.edges_t[index] += [e_index]
                
    def get_basis_target_z(self, joint_id: int) -> np.array:
        point_to: List[np.array] = []
        for c in self.edges[joint_id]:
            point_to.append(self.joints_loc[c])
        
        if len(point_to) == 0:
            point_to = [self.joints_loc[self.edges_t[joint_id][0]]]
            return - np.mean(np.array(point_to), axis=0) + self.joints_loc[joint_id]
        return np.mean(np.array(point_to), axis=0) - self.joints_loc[joint_id]

    def align_to_target_z(self, joint_id: int, target_z: np.array):
            current_z = self.joints_basis[joint_id][2, :]            
            q = utils.from_2_vec_to_quat(v1=current_z, v2=target_z)                
            self.joints_basis[joint_id][0, :] = utils.qv_mult(v1=self.joints_basis[joint_id][0, :], q1=q)
            self.joints_basis[joint_id][1, :] = utils.qv_mult(v1=self.joints_basis[joint_id][1, :], q1=q)            
            self.joints_basis[joint_id][2, :] = target_z
            
    def set_joints_basis_alignemt(self, joints_angles: List[np.array]):
        for j in self.joints:            
            z = self.get_basis_target_z(joint_id=j.id)
            z = z/np.linalg.norm(z)            
            
            self.align_to_target_z(joint_id=j.id, target_z=z)
            utils.rotate_basis_by_angles(angles=joints_angles[j.id], start_basis=self.joints_basis[j.id])
            
    def refresh_visited_state(self):
        self.visited = [False]*len(self.joints)

    def __call__(self, id: int):
        return {
            "coords": self.joints_loc[id],
            "basis": self.joints_basis[id],           
            "id" : id 
        }

    def export_config(self) -> List[Dict[str, Any]]:
        conf = []
        for j in self.joints:
           conf + self.__call__(id=j.id)
        return conf    
    
    def joint_dfs_traversing(self, parent_id: int) -> List[int]:
        self.visited[parent_id] = True
        childs = []
        
        for j in self.edges[parent_id]: 
            if not self.visited[j]:                                                                                                                        
                childs += [j]+self.joint_dfs_traversing(parent_id=j)
        return childs    

    def process_command(self, joint_id: int, command: np.array) -> None:     
        childs_id = self.joint_dfs_traversing(parent_id=joint_id)
        # parallel processing of the displacement 
        for act in self.joints[joint_id].actuators:
            for c_i in childs_id:   # parallelize this loop                                                     
                act.apply(child_config=self.__call__(id=c_i),
                          parent_config=self.__call__(id=joint_id),                                             
                          command=command)        
            act.apply(child_config=self.__call__(id=joint_id),
                      parent_config=self.__call__(id=joint_id),                                             
                      command=command)       
        self.refresh_visited_state()
                                            

    def get_shadow(self) -> Skeleton:        
        return copy.deepcopy(self)

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
        self.visited: List[bool] = [False]*len(self.joints)     
        self.joints_basis: List[np.array] = [np.eye(3) for i in range(len(self.joints))]
        #self.point_to_childs()
        self.joints_basis: List[np.array] = [utils.rotate_basis_by_angles(angles=angles, start_basis=self.joints_basis[a_i])
                                             for a_i, angles in enumerate(joints_angles)]   

    def point_to_childs(self):
        for j in self.joints:
            point_to: List[np.array] = []
            for c in self.edges[j.id]:
                point_to.append(self.joints_loc[c])
            z = np.mean(np.array(point_to), axis=0) - self.joints_loc[j.id]
            z = z/np.linalg.norm(z)
            current_z = self.joints_basis[j.id][2, :]            
            rot_axis = np.cross(current_z, z)
            norm = np.linalg.norm(rot_axis)
            if norm > 0:
                rot_axis /= norm            
            theta = np.arctan(norm/current_z.dot(z)) 
            if theta != 0 and theta != np.nan:
                q = utils.axisangle_to_q(v=rot_axis, theta=theta)
                self.joints_basis[j.id][2, :] = z
                self.joints_basis[j.id][0, :] = utils.qv_mult(v1=self.joints_basis[j.id][0, :], q1=q)
                self.joints_basis[j.id][1, :] = utils.qv_mult(v1=self.joints_basis[j.id][1, :], q1=q)
            

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

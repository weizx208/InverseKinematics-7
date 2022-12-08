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
        self.joints_basis: List[np.array] = [utils.from_angles_2_basis(angles) for angles in joints_angles]
        self.edges: List[List[int]] = edges
        self.visited: List[bool] = [False]*len(self.joints)        

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

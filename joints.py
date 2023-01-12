import numpy as np
from typing import List, Dict, Any, Optional
from actuator import Actuator
from dataclasses import dataclass
import utils

@dataclass
class Joint:
    id: int
    actuators: List[Actuator]
    location: Optional[np.array] = None
    basis: Optional[np.array] = None

    def __post_init__(self) -> None:
        self.location = np.zeros(shape=(1, 3))
        self.basis = np.eye(3)

    @property
    def config(self) -> Dict[str, Any]:
        return {
            "coords": self.location,
            "basis": self.basis,           
            "id" : self.id 
        }
    
    def align_basis_to_target_z(self, target_z: np.array) -> None:
        """
            Align joint basis to target z direction. 
        """
        current_z = self.basis[2, :]            
        q = utils.from_2_vec_to_quat(v1=current_z, v2=target_z)                
        self.basis[0, :] = utils.qv_mult(v1=self.basis[0, :], q1=q)
        self.basis[1, :] = utils.qv_mult(v1=self.basis[1, :], q1=q)            
        self.basis[2, :] = target_z

    def rotate_basis_by_angles(self, angles: np.array) -> None:
        """
            Rotate joint basis based on angles in input.

            angles: np.array -> shape = (1, 3)  :: np.array([x_angle, y_angle, z_angle])
        """        
        utils.rotate_basis_by_angles(angles=angles, basis=self.basis)

    def apply_command(self, childs_configs: List[Dict[str, Any]], command: np.array) -> None:     
        """
            Apply rototraslation deltas. 
        """
        for act in self.actuators:
            if command[act.axis] != 0:
                for c_cfg in childs_configs:  # parallelize this loop                                 
                    # update child basis 
                    act.actuate(child_config=c_cfg,
                                parent_config=self.config,                                             
                                command=command)   
                # update joint_id basis    
                act.actuate(child_config=self.config,
                        parent_config=self.config,                                             
                        command=command)               
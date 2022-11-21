import math
import numpy as np 
from typing import List
import matplotlib.pyplot as plt


def plot_config(joints_coords: List[np.array], color: str = 'b') -> None:    
    prev_joint = None
    for j_index, j in enumerate(joints_coords):   
        if prev_joint is not None:
            line_xs = np.linspace(prev_joint[0], j[0], 100)    
            line_ys = np.linspace(prev_joint[1], j[1], 100)    
            plt.plot(line_xs, line_ys, color=color)
        prev_joint = joints_coords[j_index] # start from origin 


def apply_rotation(init_coord: np.array, tetha: float):
    return np.array([[math.cos(tetha), -math.sin(tetha)], [math.sin(tetha), math.cos(tetha)]])@init_coord


def apply_movement(init_coord: np.array, tx: float, ty: float) -> np.array:
    return (np.array([[1,0,tx], [0,1,ty], [0,0,1]])@np.array([init_coord[0], init_coord[1], 1]).T)[:-1]


def forward_kinematics(harms_lens: List[float], angles: List[float]) -> np.array:
    joints: List[np.array] = [np.array([0, 0])]
    for h_index, h in enumerate(harms_lens):
        joints.append(np.array([joints[h_index][0]+h, 0])) 

    for j_index in range(len(joints)):
        if j_index > 0:
            c_index = j_index
            while c_index < len(joints):
                if c_index > j_index:
                    joints[c_index] = apply_rotation(joints[c_index], angles[j_index]*math.pi/180)    
                else:
                    if j_index > 1:
                        joints[c_index] = apply_movement(joints[c_index], 
                                                tx=-(joints[c_index-1][0]-joints[0][0]),
                                                ty=-(joints[c_index-1][1]-joints[0][1]))    

                    joints[c_index] = apply_rotation(joints[c_index], angles[j_index]*math.pi/180)
                    
                    if j_index > 1:
                        joints[c_index] = apply_movement(joints[c_index], 
                                                        tx=joints[c_index-1][0]-joints[0][0],
                                                        ty=joints[c_index-1][1]-joints[0][1]) 
                c_index += 1
    return joints

def calc_error(p1: np.array, p2: np.array, rate: float):
    return math.sqrt(np.sum((p2-p1)**2))
    

def inverse_kinematics(harms_lens: List[float], angles: List[float], target: np.array, lr: float = 0.1):
    angle_dist = .5    
    count = 0
    max_count = 1000
    while count < max_count:
        for j in range(len(angles)):                                 
            joints = forward_kinematics(harms_lens=harms_lens, angles=angles)    
            error = calc_error(joints[-1], target, rate=angle_dist)

            angles_temp = angles
            angles_temp[j] += angle_dist

            temp_joints = forward_kinematics(harms_lens=harms_lens, angles=angles_temp)            
            new_error = calc_error(temp_joints[-1], target, rate=angle_dist)            

            angles[j] = angles[j] - lr*(new_error-error)/angle_dist

        count += 1
        
        # plot final config
        plot_config(joints_coords = joints, color='r')        
        plt.pause(.1)
        

if __name__ == "__main__":
    # starting config
    joints_angles = []
    joints = []
    harms_lens = np.array([10, 5])

    joints_angles.append(0)
    joints_angles.append(45)
    joints_angles.append(45)

    target = np.array([-10, -1])
    first_config = forward_kinematics(harms_lens=harms_lens, angles=joints_angles)            

    # plot initial configuration
    plt.figure()    
    plot_config(joints_coords=first_config)
    plt.plot(target[0], target[1], marker='*')
    

    inverse_kinematics(harms_lens=harms_lens, angles=joints_angles, target=target, lr=10)

    plt.show()
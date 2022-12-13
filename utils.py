import numpy as np 


def q_mult(q1: np.array, q2: np.array):
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)    


def q_conjugate(q):
    return q*np.array([1.0, -1.0, -1.0, -1.0])    


def qv_mult(q1: np.array, v1: np.array):
    q2 = np.append(0.0, v1)
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def axisangle_to_q(v: np.array, theta: float) -> np.array:
    v = v/np.linalg.norm(v)
    theta /= 2.0
    w = np.cos(theta)
    v *= np.sin(theta)    
    return np.append(w, v)


def rotate_basis_by_angles(angles: np.array, start_basis: np.array) -> np.array:
    # todo: avoid indexing and define a matrix mult strategy
    transformed_basis = start_basis
    for i in range(3):
        q = axisangle_to_q(v=transformed_basis[i, :], theta=angles[i])
        transformed_basis[0, :] = qv_mult(q1=q, v1=transformed_basis[0, :])
        transformed_basis[1, :] = qv_mult(q1=q, v1=transformed_basis[1, :])
        transformed_basis[2, :] = qv_mult(q1=q, v1=transformed_basis[2, :])        
    return transformed_basis
    
def from_2_vec_to_quat(v1: np.array, v2: np.array) -> np.array:    
    q = np.concatenate((np.array([np.linalg.norm(v2)*np.linalg.norm(v1) + v2.dot(v1)]), np.cross(v2, v1)))
    return q/np.linalg.norm(q)
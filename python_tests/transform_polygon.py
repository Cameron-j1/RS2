import numpy as np

def invert_transform(T):
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    return T_inv

# Example: load or define your transforms
T_base_to_ee = np.array([
    [-0.685, -0.729, -0.008,  0.225],
    [-0.729,  0.685,  0.013, -0.031],
    [-0.004,  0.015, -1.000,  0.355],
    [ 0.000,  0.000,  0.000,  1.000]
])

T_base_to_aruco = np.array([
    [1,  0,  0,  0.265],
    [ 0, 1,  0, -0.079],
    [ 0,  0,  -1,  0.0],
    [ 0,  0,  0,  1.0]
])

T_aruco_to_cam = np.array([
    [ 0.6789622 , -0.73380809,  0.02315235, -0.10793079],
    [ 0.73372105,  0.67931317,  0.01367649, -0.04036365],
    [-0.02576362,  0.00770154,  0.9996384 ,  0.37490213],
    [ 0.        ,  0.        ,  0.        ,  1.        ]
])

# Invert T_base_to_ee to get T_ee_to_base
T_ee_to_base = invert_transform(T_base_to_ee)

# Compute T_ee_to_cam
T_ee_to_cam = T_ee_to_base @ T_base_to_aruco @ T_aruco_to_cam

print("Transform from end effector to camera:")
print(T_ee_to_cam)

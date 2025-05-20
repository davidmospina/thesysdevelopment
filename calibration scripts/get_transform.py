import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(pose):
    T = np.eye(4)
    T[0:3, 3] = pose[0:3]
    rot = R.from_rotvec(pose[3:6])
    T[0:3, 0:3] = rot.as_matrix()
    return T

def matrix_to_pose(T):
    position = T[0:3, 3]
    rot = R.from_matrix(T[0:3, 0:3])
    rotvec = rot.as_rotvec()
    return np.concatenate((position, rotvec))

# Example: 180° rotation about Z (adjust if needed)
T_rel = np.eye(4)
T_rel[:3, :3] = R.from_euler('y', 180, degrees=True).as_matrix()

# Get actual poses from robots (mocked here)
pose_A = [0.404013,0.00280851,0.425405,-0.950408,1.58813,-1.50388]  # TCP pose en base A MASTER
pose_B =  [0.274776,-0.399499,0.426807,-0.932301,1.48158,-1.58466] # TCP pose en base B PARTNER

T_A_TCP = pose_to_matrix(pose_A)
T_B_TCP = pose_to_matrix(pose_B)

# Compute transformation between bases
T_A_B = T_A_TCP @ np.linalg.inv(T_rel) @ np.linalg.inv(T_B_TCP)

# Convert to pose vector
pose_A_B = matrix_to_pose(T_A_B)
np.set_printoptions(precision=7, suppress=True) #to get all decimals number
print(pose_A_B)

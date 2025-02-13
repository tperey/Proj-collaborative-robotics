import numpy as np

# Object Localization (Given pixel coordinates, localize into robot frame)

# /locobot/camera/camera_info
# height, width, d (distortion. k1, k2, t1, t2, k3), K

# Frame transforms
# look up on google: ros2 python lookup_transform
# this gives translation + rotation in quarternions, so change this to a rotation matrix format
# locobot/base_link, locobot/camera_depth_link
# -> extrinsics [R|t] (camera->world)

def quaternion_to_rotation_matrix(quaternion):
    """
    Converts a quaternion to a rotation matrix.

    Args:
        quaternion (np.array): A numpy array of shape (4,) representing the quaternion in the form [w, x, y, z].

    Returns:
        np.array: A 3x3 numpy array representing the rotation matrix.
    """
    w, x, y, z = quaternion
    
    # Compute the rotation matrix elements
    r00 = 1 - 2*y**2 - 2*z**2
    r01 = 2*x*y - 2*w*z
    r02 = 2*x*z + 2*w*y
    r10 = 2*x*y + 2*w*z
    r11 = 1 - 2*x**2 - 2*z**2
    r12 = 2*y*z - 2*w*x
    r20 = 2*x*z - 2*w*y
    r21 = 2*y*z + 2*w*x
    r22 = 1 - 2*x**2 - 2*y**2
    
    # Construct the rotation matrix
    rotation_matrix = np.array([[r00, r01, r02],
                                [r10, r11, r12],
                                [r20, r21, r22]])
    return rotation_matrix

# Assume that u, v, depth is based on undistorted image
def pixel_to_robot_coords(u, v, depth, K: np.ndarray, extrinsics: np.ndarray):
    cam_coord = np.ones(4)
    cam_coord[0] = (u - K[0,2]) * depth / K[0,0]
    cam_coord[1] = (v - K[1,2]) * depth / K[1,1]
    cam_coord[2] = depth

    return np.dot(extrinsics, cam_coord)

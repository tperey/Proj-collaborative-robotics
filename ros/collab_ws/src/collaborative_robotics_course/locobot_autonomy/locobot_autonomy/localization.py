#!/usr/bin/env python3
# Object Localization (Given pixel coordinates, localize into robot frame)

# /locobot/camera/camera_info
# height, width, d (distortion. k1, k2, t1, t2, k3), K

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import CameraInfo

class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')
        
        # create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_publisher = self.create_publisher(Point, "/project_goal", 10)

        # Get target point in image coordinates
        self.img_sub = self.create_subscription(Point, "/target_point", self.img_callback, 10)

        # camera_callback will store the camera matrix K into self.K
        # self.K will start as None; check that it is not None before calculating things
        self.camera_sub = self.create_subscription(
            CameraInfo,
            '/locobot/camera/camera_info',
            self.camera_callback,
            10)
        self.K = None

        self.get_logger().info('Localizer node has started')
    
    def img_callback(self, point):
        u = point.x
        v = point.y
        depth = point.z

        transform_mat = self.get_transformation()

        if self.K is not None:
            goal = pixel_to_robot_coords(u, v, depth, self.K, transform_mat)
            goal_point = Point()
            goal_point.x = goal[0]
            goal_point.y = goal[1]
            goal_point.z = goal[2]

            self.goal_publisher.publish(goal_point)
        else:
            self.get_logger().info(f'K has not yet been initialized')
    
    def camera_callback(self, camera_info):
        self.K = np.array(camera_info.k).reshape((3,3))

    def get_transformation(self):
        """
        Gets transformation data and calculates the transformation matrix
        """
        try:
            # Get transformation data from base_link to camera_frame
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'locobot/base_link', 'locobot/camera_depth_link', rclpy.time.Time())

            # 이동(translation) 정보 추출
            translation = np.array([trans.transform.translation.x,
                           trans.transform.translation.y,
                           trans.transform.translation.z]).reshape((3, 1))
            
            # Get rotation info (quaternions)
            quaternion = np.array([trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w])

            # Create transformation matrix
            rotation = quaternion_to_rotation_matrix(quaternion)
            transform_mat = np.concatenate((rotation, translation), axis=1)

            # Print transformation matrix
            self.get_logger().info(f'Transformation Matrix:\n{transform_mat}')

            return transform_mat

        except tf2_ros.LookupException as e:
            self.get_logger().error('Transform not available.')
            self.get_logger().error(f'{e}')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('Connectivity issue.')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('Extrapolation error.')

# Assume that u, v, depth is based on undistorted image
def pixel_to_robot_coords(u, v, depth, K: np.ndarray, extrinsics: np.ndarray):
    cam_coord = np.ones(4)
    cam_coord[0] = (u - K[0,2]) * depth / K[0,0]
    cam_coord[1] = (v - K[1,2]) * depth / K[1,1]
    cam_coord[2] = depth

    return np.dot(extrinsics, cam_coord)

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


def main():
    # initialize ROS 2
    rclpy.init()
    
    # Create node
    node = Localizer()
    rclpy.spin(node)
    
    # Destroy node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
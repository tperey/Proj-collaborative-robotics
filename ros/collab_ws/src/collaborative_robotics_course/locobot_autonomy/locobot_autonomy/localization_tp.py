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

IMAGE_WIDTH = 600.0 #***Trevor - May need to tune. This is what it was in sim.
IMAGE_HEIGHT = 480.0

class TPLocalizer(Node):
    def __init__(self):
        super().__init__('TPlocalizer')
        
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
            '/locobot/camera/depth/camera_info', #'/locobot/camera/camera_info',
            #***From Trevor - We are using Depth TransfMatrix, which seems right, so I think we want depth K
            # Its like same as normal so don't know that it matters.
            self.camera_callback,
            10)
        self.K = None

        self.get_logger().info('Localizer node has started')
    
    def img_callback(self, point):
        u = point.x
        v = point.y
        depth = point.z

        transform_mat = self.get_transformation()

        if (self.K is not None) and (transform_mat is not None):
            goal = self.pixel_to_robot_coords(u, v, depth, self.K, transform_mat)
            goal_point = Point()
            goal_point.x = goal[0]
            goal_point.y = goal[1]
            goal_point.z = goal[2]

            self.goal_publisher.publish(goal_point)
            self.get_logger().info(f'goal point = {goal_point}')
        else:
            self.get_logger().info(f'K or transform_mat has not yet been initialized')
    
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
            # ***From Trevor - From printing and looking at rotations (and ChatGPT) I think this is right
            
            # trans: TransformStamped = self.tf_buffer.lookup_transform(
            #     'locobot/base_link', 'camera_locobot_link', rclpy.time.Time())
                # ***From Trevor - I think this is what we want???

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
            self.get_logger().info(f'Camera->Base Tf Matrix:\n{transform_mat}')

            return transform_mat

        except tf2_ros.LookupException as e:
            self.get_logger().error('Transform not available.')
            self.get_logger().error(f'{e}')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('Connectivity issue.')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('Extrapolation error.')

    # Assume that u, v, depth is based on undistorted image
    def pixel_to_robot_coords(self, u, v, depth, K: np.ndarray, extrinsics: np.ndarray):

        # ***Trevor - A lot of findings!
        # Determined from simulation that cooridnates from image are
        # - x is positive right
        # - y is positive down
        # - origin is top LEFT of image
        # Target point (x,y) comes in FLIPPED. That is 

        self.get_logger().info(f'Original K = {K}')
        # ***Trevor - some more stuff
        # According to chat GPT, K[0,2] = cx, and K[1,2] = cy, i.e. the centers of the image
        # But, K prints out such that (cx, cy) = (0,0), which is NOT the center! (300, 240) is the center.
        # So, should change to that
        K[0,2] = IMAGE_WIDTH/2 # Assign half of image width to cx
        K[1,2] = IMAGE_HEIGHT/2 # Assign half of image height to cy
        # In simulation, this seemed to lead to much better goal points
        # I'm not complete confident in this tho so feel free to change

        self.get_logger().info(f'K = {K}')
        self.get_logger().info(f'pixel_coord = {u}, {v}, {depth}')
        cam_coord = np.ones(4)
        cam_coord[0] = (u - K[0,2]) * depth / K[0,0]
        cam_coord[1] = (v - K[1,2]) * depth / K[1,1]
        cam_coord[2] = depth
        self.get_logger().info(f'cam_coord = {cam_coord}')

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
    node = TPLocalizer()
    rclpy.spin(node)
    
    # Destroy node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Object Localization (Given pixel coordinates, localize into robot frame)

# /locobot/camera/camera_info
# height, width, d (distortion. k1, k2, t1, t2, k3), K

# Frame transforms
# look up on google: ros2 python lookup_transform
# this gives translation + rotation in quarternions, so change this to a rotation matrix format
# locobot/base_link, locobot/camera_depth_link
# -> extrinsics [R|t] (camera->world)

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from tf_transformations import quaternion_matrix
from sensor_msgs.msg import CameraInfo

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        # TF 버퍼와 리스너 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_publisher = self.create_publisher(Point, "/project_goal", 10)

        # Get target point in image coordinates
        self.img_sub = self.create_subscription(Point, "/target_point", self.img_callback, 10))

        # camera_callback will store the camera matrix K into self.K
        # self.K will start as None; check that it is not None before calculating things
        self.camera_sub = self.create_subscription(
            CameraInfo,
            '/locobot/camera/camera_info',
            self.camera_callback,
            10)
        self.K = None
    
    def img_callback(self, point):
        u = point.x
        v = point.y
        depth = point.z

        transform_mat = self.get_transformation()

        if self.K:
            goal = pixel_to_robot_coords(u, v, depth, self.K, transform_mat)
            goal_point = Point()
            goal_point.x = goal[0]
            goal_point.y = goal[1]
            goal_point.z = goal[2]

            self.goal_publisher.publish(goal_point)
        else:
            self.get_logger().info(f'K has not yet been initialized')
    
    def camera_callback(self, camera_info):
        self.K = camera_info.k

    def get_transformation(self):
        """
        TF 데이터를 가져와 변환 행렬을 계산하는 메서드
        """
        try:
            # base_link에서 camera_frame으로의 변환 데이터 조회
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'camera_frame', rclpy.time.Time())

            # 이동(translation) 정보 추출
            translation = [trans.transform.translation.x,
                           trans.transform.translation.y,
                           trans.transform.translation.z]
            
            # 회전(rotation) 정보 추출 (쿼터니언)
            rotation = [trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w]

            # Create transformation matrix
            transform_mat = quaternion_matrix(rotation)
            transform_mat[:3,3] = translation

            # 변환 행렬 출력
            self.get_logger().info(f'Transformation Matrix:\n{transform_mat}')

            return transform_mat

        except tf2_ros.LookupException:
            self.get_logger().error('Transform not available.')
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


def main():
    # ROS 2 초기화
    rclpy.init()
    
    # TFListener 노드 생성
    node = TFListener()
    rclpy.spin(node)
    
    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
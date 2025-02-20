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
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix, translation_matrix

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        # TF 버퍼와 리스너 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_transformation(self):
        """
        TF 데이터를 한 번만 가져와 변환 행렬을 계산하는 메서드
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

            # 이동 행렬과 회전 행렬 생성
            translation_mat = translation_matrix(translation)
            rotation_mat = quaternion_matrix(rotation)

            # 변환 행렬 계산 (이동 + 회전)
            transformation_matrix = np.dot(translation_mat, rotation_mat)

            # 변환 행렬 출력
            self.get_logger().info(f'Transformation Matrix:\n{transformation_matrix}')

            # 픽셀 좌표를 로봇 좌표계로 변환 (예시)
            K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])  # 카메라 내부 파라미터
            u, v, depth = 320, 240, 1.0  # 픽셀 좌표 및 깊이 값
            robot_coords = pixel_to_robot_coords(u, v, depth, K, transformation_matrix)
            self.get_logger().info(f'Robot Coordinates:\n{robot_coords}')

        except tf2_ros.LookupException:
            self.get_logger().error('Transform not available.')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('Connectivity issue.')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('Extrapolation error.')

def main():
    # ROS 2 초기화
    rclpy.init()
    
    # TFListener 노드 생성
    node = TFListener()
    
    # get_transformation 메서드 호출
    node.get_transformation()
    
    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


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

#!/usr/bin/env python3


import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from google.cloud import vision

import rclpy
from rclpy.node import Node
import sensor_msgs
from sensor_msgs.msg import Image

import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo

from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros

from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from verbal import SpeechTranscriber
from find_center import VisionObjectDetector
import google.generativeai as genai
from align_depth import align_depth

#json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'

DISTANCE_THRESHOLD = 0.5

class ScanApproachNode(Node):
    def __init__(self):
        super().__init__("scan_approach_node")

        # self.json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'
        # self.json_key_path = '/home/locobot/Downloads/united-potion-452200-b1-8bf065055d29.json'
        #self.desiredObject = self.speech.transcribe_audio(audio_content).lower()  #"Medicine"
        # self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/mobile_base/cmd_vel", 1)
        # self.target_publisher = self.create_publisher(Point, "/target_point", 10)
        
        """ VISION """
        self.json_key_path ="/home/ubuntu/Desktop/LabDocker/Proj-collaborative-robotics/ros/collab_ws/src/collaborative_robotics_course/locobot_autonomy/locobot_autonomy/united-potion-452200-b1-8bf065055d29.json"
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.json_key_path

        self.bridge = CvBridge()
        self.client = vision.ImageAnnotatorClient()

        self.speech = SpeechTranscriber()
        self.obj_detect = VisionObjectDetector()
        self.gemini = genai.GenerativeModel("gemini-1.5-flash")

        self.task = "retrieve"
        self.desiredObject = "block"
        self.destination = None

        """ PUBLISHERS """
        self.drive_state_publisher = self.create_publisher(String, "/drive_state", 10)
        self.obj_coord_publisher = self.create_publisher(Point, "/obj_coord", 10)
        self.gripper_state_publisher = self.create_publisher(String, "/gripper_state", 10)

        # Direct commands
        #self.base_twist_publisher = self.create_publisher(Twist, "/base_twist", 10) # For directly commanding base driver
        
        # Sim and/or testing only
        #self.test_timer = self.create_timer(1.0, self.test_callback)
        self.sim_arm_publisher = self.create_publisher(PoseStamped, "/arm_pose", 10) # Arbitrarily queued 10
        self.sim_base_publisher = self.create_publisher(Twist,"/locobot/diffdrive_controller/cmd_vel_unstamped", 1) #this is the topic we will publish to in order to move the base

        """ CREATE SUBSCRIBERS """
        rgb_camera = Subscriber(self, Image, "/locobot/camera/color/image_raw")
        depth_camera = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        camera_sub = ApproximateTimeSynchronizer([rgb_camera, depth_camera], 10, 1)
        camera_sub.registerCallback(self.ScanImage)

        self.create_subscription(
            CameraInfo,
            '/locobot/camera/camera_info',
            self.rgb_info_callback,
            10)
        self.create_subscription(
            CameraInfo,
            '/locobot/camera/depth/camera_info', #check topic name...
            self.depth_info_callback,
            10)
        
        self.create_subscription(String, "/gripper_success", self.gripper_callback, 10)

        self.get_logger().info('Subscribers created')

        """ Create TF buffer and listner"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        """ STATE MACHINE """
        self.state_var = "Init" # Initial state
        self.use_sim = False # sim for now

        self.get_logger().info('ScanApproachNode now running')

    def rgb_info_callback(self, camera_info):
        self.rgb_K = np.array(camera_info.k).reshape((3,3))
    
    def depth_info_callback(self, camera_info):
        self.depth_K = np.array(camera_info.k).reshape((3,3))
    
    def get_transformation(self):
        """
        Gets transformation data and calculates the transformation matrix
        """
        try:
            # Get transformation data from rgb_camera and depth_camera
            # Check frame names
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'locobot/camera_link', 'locobot/camera_depth_link', rclpy.time.Time())

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
    
    def gripper_callback(self, msg):
        if msg.data == "Success":
            if self.destination is not None:
                # Retrieve task.
                self.state_var = "RotateFind"
                self.desiredObject = self.destination
                self.destination = None
            else:
                self.state_var = "Init"
    
    def ScanImage(self, rgb_imgmsg, depth_imgmsg):

        self.get_logger().info('Camera callback triggered')

        """ STATE MACHINE """   
        if self.state_var == "Init":
            ### EXECUTE INITIALIZATION ###

            # Listen and get task
            # transcribed_audio = self.speech.record_audio("temp.wav")[0]
            
            # response = self.gemini.generate_content(
            #     "In the given voice transcript, identify the main action being required - retrieve or place. Then identify the main object."+\
            #         "Return only the action and object in lowercase and do not include any whitespaces, punctuation, or new lines other than a single whitespace between the action and object. "+\
            #         "Here is the voice transcript: " + transcribed_audio)
            # self.task, self.desiredObject = response.text.split()
            # if task == "place":
            #     response = self.gemini.generate_content("In the given voice transcript, where does the user want to have the object placed? "+\
            #                                             "Return only the destination in lowercase and do not include any whitespaces, punctuation, or new lines. "+\
            #                                                 "Here is the voice transcript: " + transcribed_audio)
            #     self.destination = response.text
            # else:
            #     #Not sure how to define return point when retrieving - "person" will probably be too many results.
            #     self.destination = None

            ### Gripper ###
            # Tell gripper to move out of camera view
            self.gripper_state_publisher.publish(String("wait"))

            ### Arm ###
            self.drive_state_publisher.publish(String("turn"))
            
            self.state_var = "RotateFind"
            
        if self.state_var == "RotateFind":
            """ GENERAL IMAGE PROCESSING - Get objects """
            cv_ColorImage = self.bridge.imgmsg_to_cv2(rgb_imgmsg, desired_encoding='passthrough')
            depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, desired_encoding='mono8')

            # convert to depth
            # May need to convert image to bytes first with
            success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            img_bytes = encoded_image.tobytes()

            ### ROTATION and ARM ###
            # Only change on transition

            ### OBJECT LOCALIZATION ###
            center = self.obj_detect.find_center(img_bytes, object.name.lower())

            if center is not None:
                target_point = Point()
                target_point.x = center[0]
                target_point.y = center[1]
                aligned_depth = align_depth(depth_image, self.depth_K, cv_ColorImage, self.rgb_K, self.get_transformation())    
                target_point.z = aligned_depth[center[0], center[1]]
                self.obj_coord_publisher(target_point)

                # State transition
                self.drive_state_publisher.publish(String("go"))

                # Don't change gripper

                self.state_var = "Drive2Obj"
            else:
                self.drive_state_publisher.publish(String("turn"))
        
        elif self.state_var == "Drive2Obj":
            ### DRIVE TOWARDS OBJECT ###
            # Only post once, on transition

            """ GENERAL IMAGE PROCESSING - Get objects """
            cv_ColorImage = self.bridge.imgmsg_to_cv2(rgb_imgmsg, desired_encoding='passthrough')
            depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, desired_encoding='mono8')

            # convert to depth
            # May need to convert image to bytes first with
            success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            img_bytes = encoded_image.tobytes()

            ### ROTATION and ARM ###
            # Only change on transition

            ### OBJECT LOCALIZATION ###
            center = self.obj_detect.find_center(img_bytes, object.name.lower())
            
            if center is not None:
                target_point = Point()
                target_point.x = center[0]
                target_point.y = center[1]
                aligned_depth = align_depth(depth_image, self.depth_K, cv_ColorImage, self.rgb_K, self.get_transformation())    
                target_point.z = aligned_depth[center[0], center[1]]
                self.obj_coord_publisher(target_point)

                # State transition
                self.drive_state_publisher.publish(String("go")) # Stop in btw for carefulness

                # Don't change gripper
                
                # If depth is close enough, switch to next state
                if target_point.z < DISTANCE_THRESHOLD:
                    self.state_var = "Grasp"

            else:
                # Can't see object anymore
                self.state_var = "RotateFind"

        if self.state_var == "Grasp":
            self.drive_state_publisher.publish(String("stop"))
            self.gripper_state_publisher.publish(String("grab"))

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

if __name__ == '__main__':
    rclpy.init()
    node = ScanApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
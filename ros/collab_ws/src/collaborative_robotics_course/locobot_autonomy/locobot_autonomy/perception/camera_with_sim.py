#!/usr/bin/env python3


import numpy as np
import cv2
from cv_bridge import CvBridge
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo

from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros

from verbal import SpeechTranscriber
from find_center import VisionObjectDetector
import google.generativeai as genai
from align_depth import align_depth

import time

#json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'

DISTANCE_THRESHOLD = 1000.0

def limit_vals(x, y, x_limit, y_limit):
    if x > x_limit:
        x = x_limit
    if x < 0:
        x = 0
    if y > y_limit:
        y = y_limit
    if y < 0:
        y = 0
    
    return x, y

class Sable_ScanApproachNode(Node):
    def __init__(self):
        super().__init__("scan_approach_node")

        # self.json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'
        # self.json_key_path = '/home/locobot/Downloads/united-potion-452200-b1-8bf065055d29.json'
        #self.desiredObject = self.speech.transcribe_audio(audio_content).lower()  #"Medicine"
        # self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/mobile_base/cmd_vel", 1)
        # self.target_publisher = self.create_publisher(Point, "/target_point", 10)
        
        """ SIMULATION """
        self.use_sim = False

        """ VISION """
        self.json_key_path ="src/collaborative_robotics_course/locobot_autonomy/locobot_autonomy/united-potion-452200-b1-8bf065055d29.json"
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.json_key_path

        self.bridge = CvBridge()

        self.speech = SpeechTranscriber()
        self.obj_detect = VisionObjectDetector()
        self.gemini = genai.GenerativeModel("gemini-1.5-flash")

        self.task = "retrieve"
        if self.use_sim:
            self.desiredObject = "suitcase" # CHANGE TO CHANGE DESIRED OBJECT
        else:
            self.desiredObject = "package" #"strawberry"#"apple" # CHANGE TO CHANGE DESIRED OBJECT
        self.destination = None

        """ PUBLISHERS """
        self.drive_state_publisher = self.create_publisher(String, "/drive_state", 10)
        self.obj_coord_publisher = self.create_publisher(Point, "/target_point", 10)
        self.gripper_state_publisher = self.create_publisher(String, "/gripper_state", 10)
        self.object_publisher = self.create_publisher(String, "/desired_object", 10)

        # Direct commands
        #self.base_twist_publisher = self.create_publisher(Twist, "/base_twist", 10) # For directly commanding base driver
        
        # Sim and/or testing only
        #self.test_timer = self.create_timer(1.0, self.test_callback)
        #self.sim_arm_publisher = self.create_publisher(PoseStamped, "/arm_pose", 10) # Arbitrarily queued 10
        #self.sim_base_publisher = self.create_publisher(Twist,"/locobot/diffdrive_controller/cmd_vel_unstamped", 1) #this is the topic we will publish to in order to move the base

        """ CREATE SUBSCRIBERS """
        # Synchronized rgb and depth - change based on sim
        if self.use_sim:
            rgb_camera = Subscriber(self, Image, "/locobot/camera/image_raw")
            depth_camera = Subscriber(self, Image, '/locobot/camera/depth/image_raw')

            self.get_logger().info('~SIMULATION~ Cameras')

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
        else:
            rgb_camera = Subscriber(self, Image, "/locobot/camera/camera/color/image_raw")
            depth_camera = Subscriber(self, Image, '/locobot/camera/camera/depth/image_rect_raw')

            self.create_subscription(
                CameraInfo,
                '/locobot/camera/camera/color/camera_info',
                self.rgb_info_callback,
                10)
            self.create_subscription(
                CameraInfo,
                '/locobot/camera/camera/depth/camera_info', #check topic name...
                self.depth_info_callback,
                10)
            self.rgb_K = None
            self.depth_K = None
        
        camera_sub = ApproximateTimeSynchronizer([rgb_camera, depth_camera], 10, 1)
        camera_sub.registerCallback(self.ScanImage)
        
        self.create_subscription(Bool, "/gripper_success", self.success_callback, 10)

        self.get_logger().info('Subscribers created')

        """ Create TF buffer and listner"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        """ STATE MACHINE """
        self.state_var = "Init" # Initial state

        self.get_logger().info('ScanApproachNode now running')

    def rgb_info_callback(self, camera_info):
        self.rgb_K = np.array(camera_info.k).reshape((3,3))
    
    def depth_info_callback(self, camera_info):
        self.depth_K = np.array(camera_info.k).reshape((3,3))
    
    def get_transformation(self):
        """
        Gets transformation data and calculates the transformation matrix
        """
        return np.concatenate([np.eye(3), np.zeros((3,1))], axis=1)
    
    def success_callback(self, msg):
        if msg.data:
            self.get_logger().info("GRASP SUCCEEDED!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            if self.destination is not None:
                # Retrieve task.
                self.state_var = "RotateFind"
                self.desiredObject = self.destination
                self.destination = None
            else:
                self.state_var = "Init"
    
    def image_processing(self, rgb_imgmsg, depth_imgmsg):
        ### GENERAL IMAGE PROCESSING - Get objects ###
        cv_ColorImage = self.bridge.imgmsg_to_cv2(rgb_imgmsg, desired_encoding='bgr8') # passthrough?
        depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, desired_encoding='passthrough') # mono8? bgr8?
        # convert to depth
        # May need to convert image to bytes first with
        success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
        img_bytes = encoded_image.tobytes()

        ### OBJECT LOCALIZATION ###
        center, obj_names = self.obj_detect.find_center(img_bytes, self.desiredObject)
        if self.desiredObject in obj_names:
            self.get_logger().info("FOUND OBJECT IN IMAGE_PROCESSING")
            self.get_logger().info(f"{center}")
        self.get_logger().info(f'{obj_names}')

        if center is not None:
            self.get_logger().info("!!! Desired obj found !!!")
            target_point = Point()

            # ***From Trevor - I think cv is in (y,x) format, esp for bgr8. So center ouptuts (y,x), and need to flip
            target_point.x = center[0]
            target_point.y = center[1]
            # ***Determined from testing that sometimes obj detected before center in frame
            if self.use_sim:
                target_point.x, target_point.y = limit_vals(target_point.x, target_point.y, 600.0, 480.0)
            else:
                target_point.x, target_point.y = limit_vals(target_point.x, target_point.y, 640.0, 480.0)
            ### DEPTH ALIGNMENT ###
            transform_mat = self.get_transformation()
            if (self.depth_K is not None) and (self.rgb_K is not None) and transform_mat is not None:
                aligned_depth = align_depth(depth_image, self.depth_K, cv_ColorImage, self.rgb_K, transform_mat)
                target_point.z = float(aligned_depth[int(target_point.y), int(target_point.x)]) # Need integer conversion for indexing, then float for Point publication
                return target_point
            else:
                self.get_logger().info("Missing K matrices :(((")
        return None

    def ScanImage(self, rgb_imgmsg, depth_imgmsg):
        #self.get_logger().info('Camera callback triggered')

        """ STATE MACHINE """   
        if self.state_var == "Init":
            ### EXECUTE INITIALIZATION ###

            # Listen and get task
            # self.get_logger().info("~~~SPEAK NOW!!!~~~")
            # transcribed_audio = self.speech.record_audio(duration = 5)[0]
            # self.get_logger().info(transcribed_audio)
            # self.get_logger().info("~ Recording over ~")
            
            # response = self.gemini.generate_content(
            #     "In the given voice transcript, what is the main task - retrieve, place, or pour?"+\
            #         "Return only the action in lowercase and do not include any whitespaces, punctuation, or new lines. "+\
            #         "Here is the voice transcript: " + transcribed_audio)
            # self.task = response.text.strip()
            # self.get_logger().info(f"Detected task {self.task}")

            # response = self.gemini.generate_content(
            #     "In the given voice transcript, what is the main object?"+\
            #         "Return only the object in lowercase and do not include any whitespaces, punctuation, or new lines. "+\
            #         "Here is the voice transcript: " + transcribed_audio)
            # self.desiredObject = response.text.strip()
            # self.get_logger().info(f"Detected object {self.desiredObject}")

            # if self.task == "place":
            #     response = self.gemini.generate_content("In the given voice transcript, where does the user want to have the object placed? "+\
            #                                             "Return only the destination in lowercase and do not include any whitespaces, punctuation, or new lines. "+\
            #                                                 "Here is the voice transcript: " + transcribed_audio)
            #     self.destination = response.text.strip()
            # elif self.task == "retrieve" or self.task == "pour":
            #     self.destination = "person"
            # else:
            #     #error
            #     return
            self.task = "pour"
            self.desiredObject = "package"
            self.destination = "person"

            ### Send out object
            self.object_publisher.publish(String(data=self.desiredObject))

            ### BASE - Ensure continuous rotation ###
            self.drive_state_publisher.publish(String(data="turn"))
            #self.get_logger().info(f'Rotate base')

            ### ARM - ensure out of camera view ###
            self.gripper_state_publisher.publish(String(data="wait"))
            self.get_logger().info(f'Moved gripper to wait in INIT')
            time.sleep(8)
            
            self.state_var = "RotateFind"
            
        if self.state_var == "RotateFind":
            ### BASE - Ensure continuous rotation ###
            self.drive_state_publisher.publish(String(data="turn"))
            #self.get_logger().info(f'Rotate base')

            ### ARM - ensure out of camera view ###
            # #***From Trevor - Only command once, on init. Doing it a ton is buggy. 
            # gripperstate_to_post = String()
            # gripperstate_to_post.data = "wait"
            # self.gripper_state_publisher.publish(gripperstate_to_post)
            # #self.get_logger().info(f'Moved gripper to wait')

            target_point = self.image_processing(rgb_imgmsg, depth_imgmsg)
            if target_point is not None:
                self.obj_coord_publisher.publish(target_point)

                ### STATE TRANSITION ###
                # Don't change gripper
                self.state_var = "Drive2Obj"
                #self.state_var = "Grasp" # Go to grasp to stop for testing

                ### LOGGING (esp for debugging) ###
                self.get_logger().info(f'!!! Desired obj {self.desiredObject} at {target_point.x}, {target_point.y}, {target_point.z}')
        
        elif self.state_var == "Drive2Obj":
            ### BASE - Now, drive towards object ###

            target_point = self.image_processing(rgb_imgmsg, depth_imgmsg)
            if target_point is not None:
                self.drive_state_publisher.publish(String(data="go"))
                self.get_logger().info(f'Go base {target_point.z}')
                self.obj_coord_publisher.publish(target_point)

                ### STATE TRANSITION ###
                # If depth is close enough, switch to next state
                if target_point.z < DISTANCE_THRESHOLD:
                    
                    self.state_var = "Grasp"

                    # Also immediately post a stop (for responsiveness)
                    self.drive_state_publisher.publish(String(data="stop"))
                    self.get_logger().info(f'Stopping...')

                    time.sleep(3) # Short pause to ensure stopped before sending

                    # DO change gripper. For first test, only post on transition (don't flood)
                    # Concerns with position changing, but target constantly updated, so probably ok?
                    #***From Trevor - Only command once, on change. Doing it a ton is buggy. 
                    if self.destination is not None:
                        self.gripper_state_publisher.publish(String(data="grab"))
                        self.get_logger().info(f'Told gripper to GRAB')
                    else:
                        self.gripper_state_publisher.publish(String(data="hand"))
                        self.get_logger().info(f'Told gripper to HAND')

                    #***From Trevor - Consider only posting grab on NEXT image detection, to ensure stopped and got most recent position?

                ### LOGGING (esp for debugging) ###
                #self.get_logger().info(f'!!! Desired obj at {target_point.x}, {target_point.y}, {target_point.z}')
            else:
                # Can't see object anymore
                #self.state_var = "RotateFind"
                pass

        elif self.state_var == "Grasp":

            ### BASE - Now, ensure stop ###
            #self.drive_state_publisher.publish(String(data="stop"))
            #self.get_logger().info(f'Stopped')

            ### ARM ###
            # #***From Trevor - Only command once, on init. Doing it a ton is buggy. 
            # gripperstate_to_post = String()
            # gripperstate_to_post.data = "grab"
            # self.gripper_state_publisher.publish(gripperstate_to_post)
            # #self.get_logger().info(f'Moved gripper to wait')

            target_point = self.image_processing(rgb_imgmsg, depth_imgmsg)
            if target_point is not None:
                self.obj_coord_publisher.publish(target_point)

                ### STATE TRANSITION ###
                # None for now
    
            
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
    node = Sable_ScanApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


""" DEBUGGING CODE PUT HERE FOR REF """
# Granularity, for debugging

                        # conver the instrinsics of the depth camera to the intrinsics of the rgb camera.
                        # K_old = self.depth_K
                        # self.get_logger().info(f'depth_K = {K_old}')
                        # K_new = self.rgb_K
                        # self.get_logger().info(f'rgb_K = {K_new}')
                        # aligned_depth = convert_intrinsics(depth_image, K_old, K_new, new_size=(cv_ColorImage.shape[1], cv_ColorImage.shape[0]))
                        # self.get_logger().info(f"Convert intrinsics dtype: {aligned_depth.dtype}, min: {aligned_depth.min()}, max: {aligned_depth.max()}")

                        # # warp the depth image to the rgb image with the transformation matrix from camera to camera.
                        # cam2cam_transform = self.get_transformation()
                        # self.get_logger().info(f'Transformation Matrix:\n{cam2cam_transform}')
                        # #aligned_depth = warp_image(aligned_depth, K_new, cam2cam_transform[:3, :3], cam2cam_transform[:3, 3])  
                        # """ PROBLEM IS WITH WARP_IMAGE FUNCTION """

                        # # Compute the homography matrix
                        # force_R = np.array([
                        #     [0, 0, 1],
                        #     [-1, 0, 0],
                        #     [0, -1, 0]
                        # ])
                        # self.get_logger().info(f"Forced R = \n{force_R}")
                        # force_t = np.array([0,0,0])
                        # self.get_logger().info(f'Forced t = \n{force_t}')
                        # H = compute_homography(K_new, force_R, force_t) #cam2cam_transform[:3, :3], cam2cam_transform[:3, 3])
                        # self.get_logger().info(f"Homography = \n{H}")

                        # # Warp the image using the homography
                        # height, width = aligned_depth.shape[:2]
                        # aligned_depth = cv2.warpPerspective(aligned_depth, H, (width, height))
                        # self.get_logger().info(f"Final aligned_depth dtype: {aligned_depth.dtype}, min: {aligned_depth.min()}, max: {aligned_depth.max()}")

                                                # self.get_logger().info(f'Color shape: {cv_ColorImage.shape}')
                        # self.get_logger().info(f'Original depth shape: {depth_image.shape}')
                        # self.get_logger().info(f'ALIGNED depth shape: {aligned_depth.shape}')
                        # self.get_logger().info(f"Depth dtype: {depth_image.dtype}, min: {depth_image.min()}, max: {depth_image.max()}")
                        # self.get_logger().info(f"ALIGNED depth dtype: {aligned_depth.dtype}, min: {aligned_depth.min()}, max: {aligned_depth.max()}")

                        # self.get_logger().info(f'Int target (x,y): {int(target_point.x)}, {int(target_point.y)}')
                        # self.get_logger().info(f'Expected z: {aligned_depth[int(target_point.x), int(target_point.y)]}')
                        # self.get_logger().info(f'Actual z after float: {target_point.z}')
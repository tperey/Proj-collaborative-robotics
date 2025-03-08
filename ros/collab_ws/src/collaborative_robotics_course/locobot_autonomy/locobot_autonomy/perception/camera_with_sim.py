#!/usr/bin/env python3


import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from google.cloud import vision

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo
from realsense2_camera_msgs.msg import Extrinsics

from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros

#from verbal import SpeechTranscriber
from find_center import VisionObjectDetector
#import google.generativeai as genai
from align_depth import align_depth

import time

#json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'

DISTANCE_THRESHOLD = 0.5

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
        self.client = vision.ImageAnnotatorClient()

        #self.speech = SpeechTranscriber()
        self.obj_detect = VisionObjectDetector()
        #self.gemini = genai.GenerativeModel("gemini-1.5-flash")

        self.task = "retrieve"
        if self.use_sim:
            self.desiredObject = "suitcase" # CHANGE TO CHANGE DESIRED OBJECT
        else:
            self.desiredObject = "banana" # CHANGE TO CHANGE DESIRED OBJECT
        self.destination = None

        """ PUBLISHERS """
        self.drive_state_publisher = self.create_publisher(String, "/drive_state", 10)
        self.obj_coord_publisher = self.create_publisher(Point, "/target_point", 10)
        self.gripper_state_publisher = self.create_publisher(String, "/gripper_state", 10)

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
            rgb_camera = Subscriber(self, Image, "/locobot/camera/color/image_raw")
            depth_camera = Subscriber(self, Image, '/locobot/camera/depth/image_rect_raw')

            self.create_subscription(
                CameraInfo,
                '/locobot/camera/color/camera_info',
                self.rgb_info_callback,
                10)
            self.create_subscription(
                CameraInfo,
                '/locobot/camera/depth/camera_info', #check topic name...
                self.depth_info_callback,
                10)
        
        camera_sub = ApproximateTimeSynchronizer([rgb_camera, depth_camera], 10, 1)
        camera_sub.registerCallback(self.ScanImage)
        
        self.create_subscription(String, "/gripper_success", self.gripper_callback, 10)

        self.get_logger().info('Subscribers created')

        if True:
            """ Create TF buffer and listner"""
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.create_subscription(
                Extrinsics,
                "/locobot/camera/extrinsics/depth_to_color", 
                self.extrinsics_callback, 10)
            self.extrinsics = None

        """ STATE MACHINE """
        self.state_var = "Init" # Initial state

        self.get_logger().info('ScanApproachNode now running')

    def rgb_info_callback(self, camera_info):
        self.rgb_K = np.array(camera_info.k).reshape((3,3))
    
    def depth_info_callback(self, camera_info):
        self.depth_K = np.array(camera_info.k).reshape((3,3))
    
    def extrinsics_callback(self, extrinsics):
        rotation = np.array(extrinsics.rotation).reshape((3,3))
        translation = np.array(extrinsics.translation).reshape((3,1))
        self.extrinsics = np.concatenate((rotation, translation), axis=1)
        self.get_logger().info(f'{self.extrinsics}')
    
    def get_transformation(self):
        """
        Gets transformation data and calculates the transformation matrix
        """
        if True:
            try:
                # Get transformation data from rgb_camera and depth_camera
                # Check frame names
                # trans: TransformStamped = self.tf_buffer.lookup_transform(
                #     'locobot/camera_link', 'locobot/camera_depth_link', rclpy.time.Time())
                # ***From Trevor*** = Looking at frame names, at least in sim, I think it should be
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    'camera_locobot_link', 'locobot/camera_depth_link', rclpy.time.Time())
                # Should be depth to rgb
                ### **********OTHER TEAM USED 4X4 IDENTITY FOR THIS********** ###

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
                #self.get_logger().info(f'Transformation Matrix:\n{transform_mat}')
                # *** Clutters output

                return transform_mat

            except tf2_ros.LookupException as e:
                self.get_logger().error('Transform not available.')
                self.get_logger().error(f'{e}')
            except tf2_ros.ConnectivityException:
                self.get_logger().error('Connectivity issue.')
            except tf2_ros.ExtrapolationException:
                self.get_logger().error('Extrapolation error.')
        else:
            return self.extrinsics
    
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

            ### BASE - Ensure continuous rotation ###
            drivestate_to_post = String()
            drivestate_to_post.data = "turn"
            self.drive_state_publisher.publish(drivestate_to_post)
            #self.get_logger().info(f'Rotate base')

            ### ARM - ensure out of camera view ###
            gripperstate_to_post = String()
            gripperstate_to_post.data = "wait"
            self.gripper_state_publisher.publish(gripperstate_to_post)
            self.get_logger().info(f'Moved gripper to wait in INIT')
            time.sleep(10)
            
            self.state_var = "RotateFind"
            
        if self.state_var == "RotateFind":
            ### BASE - Ensure continuous rotation ###
            drivestate_to_post = String()
            drivestate_to_post.data = "turn"
            self.drive_state_publisher.publish(drivestate_to_post)
            #self.get_logger().info(f'Rotate base')

            ### ARM - ensure out of camera view ###
            # #***From Trevor - Only command once, on init. Doing it a ton is buggy. 
            # gripperstate_to_post = String()
            # gripperstate_to_post.data = "wait"
            # self.gripper_state_publisher.publish(gripperstate_to_post)
            # #self.get_logger().info(f'Moved gripper to wait')

            ### GENERAL IMAGE PROCESSING - Get objects ###
            cv_ColorImage = self.bridge.imgmsg_to_cv2(rgb_imgmsg, desired_encoding='bgr8') # passthrough?
            depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, desired_encoding='passthrough') # mono8? bgr8?
            # convert to depth
            # May need to convert image to bytes first with
            success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            img_bytes = encoded_image.tobytes()

            ### OBJECT LOCALIZATION ###
            visionImage = vision.Image(content=img_bytes)
            response = self.client.object_localization(image=visionImage) # Send the image to the API for object localization
            objects = response.localized_object_annotations # Extract localized object annotations
            #self.get_logger().info(f'...Looking for {self.desiredObject.lower()}')
            
            for object in objects: # Only run if obj detected. Alos, need to check all objects for desired
                #self.get_logger().info(f'Detected object {object.name.lower()}')
                
                if object.name.lower() == self.desiredObject.lower(): # Only run if desired object. Use "name" to detect
                    center = self.obj_detect.find_center(img_bytes, object.name.lower())

                    if center is not None:
                        self.get_logger().info("!!! Desired obj found !!!")
                        target_point = Point()

                        # ***From Trevor - I think cv is in (y,x) format, esp for bgr8. So center ouptuts (y,x), and need to flip
                        target_point.x = center[0]
                        target_point.y = center[1]
                        # ***Determined from testing that sometimes obj detected before center in frame
                        if self.use_sim:
                            if target_point.x > 600.0:
                                target_point.x = 600.0
                            elif target_point.x < 0.0:
                                target_point.x = 0.0
                            
                            if target_point.y > 480.0:
                                target_point.y = 480.0
                            elif target_point.y < 0.0:
                                target_point.y = 0.0
                        else:
                            if target_point.x > 640.0:
                                target_point.x = 640.0
                            elif target_point.x < 0.0:
                                target_point.x = 0.0
                            
                            if target_point.y > 480.0:
                                target_point.y = 480.0
                            elif target_point.y < 0.0:
                                target_point.y = 0.0
                        ### DEPTH ALIGNMENT ###
                        transform_mat = self.get_transformation()
                        if (self.depth_K is not None) and (self.rgb_K is not None) and transform_mat is not None:
                            aligned_depth = align_depth(depth_image, self.depth_K, cv_ColorImage, self.rgb_K, transform_mat)
                            target_point.z = float(aligned_depth[int(target_point.y), int(target_point.x)]) # Need integer conversion for indexing, then float for Point publication
                            #***From Trevor - from testing, images are indexed [row, col] = [y, x]!
                            self.obj_coord_publisher.publish(target_point)

                            ### STATE TRANSITION ###
                            # Don't change gripper
                            self.state_var = "Drive2Obj"
                            #self.state_var = "Grasp" # Go to grasp to stop for testing

                            ### LOGGING (esp for debugging) ###
                            self.get_logger().info(f'!!! Desired obj at {target_point.x}, {target_point.y}, {target_point.z}')
                        else:
                            self.get_logger().info("Missing K matrices :(((")

                    else:
                        #self.drive_state_publisher.publish(String("turn")) #<-- Handled above

                        # Handled above
                        pass
        
        elif self.state_var == "Drive2Obj":
            ### BASE - Now, drive towards object ###
            drivestate_to_post = String()
            drivestate_to_post.data = "go"
            self.drive_state_publisher.publish(drivestate_to_post)
            self.get_logger().info(f'Go base')

            ### ARM - ensure out of camera view ###
            # #***From Trevor - Only command once, on init. Doing it a ton is buggy. 
            # gripperstate_to_post = String()
            # gripperstate_to_post.data = "wait"
            # self.gripper_state_publisher.publish(gripperstate_to_post)
            # #self.get_logger().info(f'Moved gripper to wait')

            ### GENERAL IMAGE PROCESSING - Get objects ###
            cv_ColorImage = self.bridge.imgmsg_to_cv2(rgb_imgmsg, desired_encoding='bgr8') # passthrough?
            depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, desired_encoding='passthrough') # mono8? bgr8?
            # convert to depth
            # May need to convert image to bytes first with
            success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            img_bytes = encoded_image.tobytes()

            ### OBJECT LOCALIZATION ###
            visionImage = vision.Image(content=img_bytes)
            response = self.client.object_localization(image=visionImage) # Send the image to the API for object localization
            objects = response.localized_object_annotations # Extract localized object annotations
            #self.get_logger().info(f'...Looking for {self.desiredObject.lower()}')
            
            for object in objects: # Only run if obj detected. Alos, need to check all objects for desired
                #self.get_logger().info(f'Detected object {object.name.lower()}')
                
                if object.name.lower() == self.desiredObject.lower(): # Only run if desired object. Use "name" to detect
                    center = self.obj_detect.find_center(img_bytes, object.name.lower())

                    if center is not None:
                        self.get_logger().info("!!! Desired obj found !!!")
                        target_point = Point()

                        # ***From Trevor - I think cv is in (y,x) format, esp for bgr8. So center ouptuts (y,x), and need to flip
                        target_point.x = center[0]
                        target_point.y = center[1]
                        # ***Determined from testing that sometimes obj detected before center in frame
                        if self.use_sim:
                            if target_point.x > 600.0:
                                target_point.x = 600.0
                            elif target_point.x < 0.0:
                                target_point.x = 0.0
                            
                            if target_point.y > 480.0:
                                target_point.y = 480.0
                            elif target_point.y < 0.0:
                                target_point.y = 0.0
                        else:
                            if target_point.x > 640.0:
                                target_point.x = 640.0
                            elif target_point.x < 0.0:
                                target_point.x = 0.0
                            
                            if target_point.y > 480.0:
                                target_point.y = 480.0
                            elif target_point.y < 0.0:
                                target_point.y = 0.0

                        ### DEPTH ALIGNMENT ###
                        if (self.depth_K is not None) and (self.rgb_K is not None):
                            aligned_depth = align_depth(depth_image, self.depth_K, cv_ColorImage, self.rgb_K, self.get_transformation())
                            target_point.z = float(aligned_depth[int(target_point.y), int(target_point.x)]) # Need integer conversion for indexing, then float for Point publication
                            #***From Trevor - from testing, images are indexed [row, col] = [y, x]!
                            self.obj_coord_publisher.publish(target_point)

                            ### STATE TRANSITION ###
                            # If depth is close enough, switch to next state
                            if target_point.z < DISTANCE_THRESHOLD:
                                
                                self.state_var = "Grasp"

                                # Also immediately post a stop (for responsiveness)
                                drivestate_to_post = String()
                                drivestate_to_post.data = "stop"
                                self.drive_state_publisher.publish(drivestate_to_post)
                                self.get_logger().info(f'Stopping...')

                                time.sleep(3) # Short pause to ensure stopped before sending

                                # DO change gripper. For first test, only post on transition (don't flood)
                                # Concerns with position changing, but target constantly updated, so probably ok?
                                #***From Trevor - Only command once, on change. Doing it a ton is buggy. 
                                gripperstate_to_post = String()
                                gripperstate_to_post.data = "grab"
                                self.gripper_state_publisher.publish(gripperstate_to_post)
                                self.get_logger().info(f'Told gripper to GRAB')

                                #***From Trevor - Consider only posting grab on NEXT image detection, to ensure stopped and got most recent position?

                            ### LOGGING (esp for debugging) ###
                            #self.get_logger().info(f'!!! Desired obj at {target_point.x}, {target_point.y}, {target_point.z}')
                        else:
                            self.get_logger().info("Missing K matrices :(((")
            else:
                # Can't see object anymore
                pass
                #self.state_var = "RotateFind"

        elif self.state_var == "Grasp":

            ### BASE - Now, ensure stop ###
            drivestate_to_post = String()
            drivestate_to_post.data = "stop"
            self.drive_state_publisher.publish(drivestate_to_post)
            #self.get_logger().info(f'Stopped')

            ### ARM ###
            # #***From Trevor - Only command once, on init. Doing it a ton is buggy. 
            # gripperstate_to_post = String()
            # gripperstate_to_post.data = "grab"
            # self.gripper_state_publisher.publish(gripperstate_to_post)
            # #self.get_logger().info(f'Moved gripper to wait')

            ### GENERAL IMAGE PROCESSING - Get objects ###
            cv_ColorImage = self.bridge.imgmsg_to_cv2(rgb_imgmsg, desired_encoding='bgr8') # passthrough?
            depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, desired_encoding='passthrough') # mono8? bgr8?
            # convert to depth
            # May need to convert image to bytes first with
            success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
            img_bytes = encoded_image.tobytes()

            ### OBJECT LOCALIZATION ###
            visionImage = vision.Image(content=img_bytes)
            response = self.client.object_localization(image=visionImage) # Send the image to the API for object localization
            objects = response.localized_object_annotations # Extract localized object annotations
            #self.get_logger().info(f'...Looking for {self.desiredObject.lower()}')
            
            for object in objects: # Only run if obj detected. Alos, need to check all objects for desired
                #self.get_logger().info(f'Detected object {object.name.lower()}')
                
                if object.name.lower() == self.desiredObject.lower(): # Only run if desired object. Use "name" to detect
                    center = self.obj_detect.find_center(img_bytes, object.name.lower())

                    if center is not None:
                        self.get_logger().info("!!! Desired obj found !!!")
                        target_point = Point()

                        # ***From Trevor - I think cv is in (y,x) format, esp for bgr8. So center ouptuts (y,x), and need to flip
                        target_point.x = center[0]
                        target_point.y = center[1]
                        # ***Determined from testing that sometimes obj detected before center in frame
                        if self.use_sim:
                            if target_point.x > 600.0:
                                target_point.x = 600.0
                            elif target_point.x < 0.0:
                                target_point.x = 0.0
                            
                            if target_point.y > 480.0:
                                target_point.y = 480.0
                            elif target_point.y < 0.0:
                                target_point.y = 0.0
                        else:
                            if target_point.x > 640.0:
                                target_point.x = 640.0
                            elif target_point.x < 0.0:
                                target_point.x = 0.0
                            
                            if target_point.y > 480.0:
                                target_point.y = 480.0
                            elif target_point.y < 0.0:
                                target_point.y = 0.0

                        ### DEPTH ALIGNMENT ###
                        if (self.depth_K is not None) and (self.rgb_K is not None):
                            aligned_depth = align_depth(depth_image, self.depth_K, cv_ColorImage, self.rgb_K, self.get_transformation())
                            target_point.z = float(aligned_depth[int(target_point.y), int(target_point.x)]) # Need integer conversion for indexing, then float for Point publication
                            #***From Trevor - from testing, images are indexed [row, col] = [y, x]!
                            self.obj_coord_publisher.publish(target_point)

                            ### STATE TRANSITION ###
                            # None for now
                        else:
                            self.get_logger().info("Missing K matrices :(((")
    
            
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
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
from geometry_msgs.msg import PoseStamped

from message_filters import ApproximateTimeSynchronizer, Subscriber

from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from verbal import SpeechTranscriber
from find_center import VisionObjectDetector
import google.generativeai as genai
#import align_depth

#json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'

class ScanApproachNode(Node):
    def __init__(self):
        super().__init__("scan_approach_node")

        # self.json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'
        # self.json_key_path = '/home/locobot/Downloads/united-potion-452200-b1-8bf065055d29.json'
        #self.desiredObject = self.speech.transcribe_audio(audio_content).lower()  #"Medicine"
        # self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/mobile_base/cmd_vel", 1)
        # self.target_publisher = self.create_publisher(Point, "/target_point", 10)

        # msg = Twist()
        # msg.angular.z = 0.5  # Set angular velocity (turn)
        # self.mobile_base_vel_publisher.publish(msg)
        
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

        self.get_logger().info('Subscribers created')

        """ STATE MACHINE """
        self.state_var = "Init" # Initial state
        self.use_sim = False # sim for now

        #self.init_timer = self.create_timer(1.0, self.initializePerception)

        self.get_logger().info('ScanApproachNode now running')

    def initializePerception(self):
        self.get_logger().info(f'~~~Running init')

        ### EXECUTE INITIALIZATION ###
        if self.use_sim:

            ### Base ### 
            # Directly command
            rotatemsg = Twist()
            rotatemsg.linear = Vector3(x=0.0, y=0.0, z=0.0)
            rotatemsg.angular = Vector3(x=0.0, y=0.0, z=5.0) # Just rotate

            #self.base_twist_publisher.publish(rotatemsg)
            self.sim_base_publisher.publish(rotatemsg)

            self.get_logger().info(f'Moved base in sim as {rotatemsg.linear}, {rotatemsg.angular}')

            ### Gripper ###
            desired_pose_msg = PoseStamped() # Define pose
            desired_pose_msg.pose.position.x = 0.1
            desired_pose_msg.pose.position.y = 0.2
            desired_pose_msg.pose.position.z = 0.1
            desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
            desired_pose_msg.pose.orientation.y = np.sqrt(2)/2
            desired_pose_msg.pose.orientation.z = 0.0
            desired_pose_msg.pose.orientation.w = np.sqrt(2)/2

            self.sim_arm_publisher.publish(desired_pose_msg) # Publish pose

            self.get_logger().info(f'Moved gripper in sim to {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}')

        else:
            ### Gripper ###
            # Tell gripper to move out of camera view
            gripperstate_to_post = String("wait")
            self.gripper_state_publisher.publish(gripperstate_to_post)

            ### Arm ###
            drivestate_to_post = String("turn")
            self.drive_state_publisher.publish(drivestate_to_post)
    
    # def test_callback(self):
    #     self.get_logger().info('TIMER TESTER callback triggered')
    #     # Directly command
    #     rotatemsg = Twist()
    #     rotatemsg.linear = Vector3(x=0.0, y=0.0, z=0.0)
    #     rotatemsg.angular = Vector3(x=0.0, y=0.0, z=10.0) # Just rotate

    #     #self.base_twist_publisher.publish(rotatemsg)
    #     self.sim_base_publisher.publish(rotatemsg)

    #     self.get_logger().info(f'Posted {rotatemsg.linear}, {rotatemsg.angular}')
    
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
            # self.destination = response.text

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
                # msg = Twist()
                # msg.linear.x = 0.5  # Set linear velocity (forward)
                # self.mobile_base_vel_publisher.publish(msg)

                target_point = Point()
                target_point.x = center[0]
                target_point.y = center[1]
                self.obj_coord_publisher(target_point)

                # NEED DEPTH CHANGES

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
                # msg = Twist()
                # msg.linear.x = 0.5  # Set linear velocity (forward)
                # self.mobile_base_vel_publisher.publish(msg)

                target_point = Point()
                target_point.x = center[0]
                target_point.y = center[1]
                self.obj_coord_publisher(target_point)

                # NEED DEPTH CHANGES

                # State transition
                self.drive_state_publisher.publish(String("go")) # Stop in btw for carefulness

                # Don't change gripper
                
                # If depth is close enough, switch to next state
            else:
                # Can't see object anymore
                self.state_var = "RotateFind"

        if self.state_var == "Grasp":
            self.drive_state_publisher.publish(String("stop"))
            self.gripper_state_publisher.publish(String("grab"))
            if self.destination is not None:
                # Retrieve task.
                self.state_var = "RotateFind"
                self.desiredObject = self.destination
                self.destination = None
            else:
                self.state_var = "Init"

if __name__ == '__main__':
    rclpy.init()
    node = ScanApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    # depth = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED)
    # rgb = cv2.imread("rgb.png")
    # depth_K = (360.01, 360.01, 243.87, 137.92)
    # rgb_K = (1297.67, 1298.63, 620.91, 238.28)
    # cam2cam_transform = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # # define images with created node
    # aligned_depth = align_depth(depth, depth_K, rgb, rgb_K, cam2cam_transform)

    # x,y = node.ScanImage

    # desired_depth = aligned_depth[x,y]
    # max_depth_thres = 3 # m
    # min_depth_thres = .5 # m
    # while desired_depth >= max_depth_thres or desired_depth <= min_depth_thres: 
    #     if desired_depth >= max_depth_thres:
    #         pass # approach the object
    #     else:
    #         pass # grab 
    # push depth, x, y to navigation



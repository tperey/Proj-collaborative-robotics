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

from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from verbal import SpeechTranscriber
from find_center import VisionObjectDetector
#import align_depth

#json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'

class ScanApproachNode(Node):
    def __init__(self):
        super().__init__("scan_approach_node")

        # self.json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'
        # self.json_key_path = '/home/locobot/Downloads/united-potion-452200-b1-8bf065055d29.json'
        self.json_key_path ="/home/ubuntu/Desktop/LabDocker/Proj-collaborative-robotics/ros/collab_ws/src/collaborative_robotics_course/locobot_autonomy/locobot_autonomy/united-potion-452200-b1-8bf065055d29.json"

        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.json_key_path

        self.bridge = CvBridge()
    
        self.client = vision.ImageAnnotatorClient()

        # self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/mobile_base/cmd_vel", 1)
        # self.target_publisher = self.create_publisher(Point, "/target_point", 10)

        # msg = Twist()
        # msg.angular.z = 0.5  # Set angular velocity (turn)
        # self.mobile_base_vel_publisher.publish(msg)

        self.speech = SpeechTranscriber()
        self.obj_detect = VisionObjectDetector()

        #self.desiredObject = self.speech.transcribe_audio(audio_content).lower()  #"Medicine"
        self.desiredObject = "block"

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
        self.camera_subscription = self.create_subscription(
            Image,
            "/locobot/camera/color/image_raw",
            self.ScanImage,
            qos_profile=qos_profile_sensor_data  # Best effort QoS profile for sensor data [usual would be queue size: 1]
            ) #this says: listen to the image_raw message, of type Image, and send that to the callback function specified
        self.camera_subscription  # prevent unused variable warning
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw', 
            self.ScanImage,
            qos_profile=qos_profile_sensor_data
            )
        self.depth_subscription

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
    
    def ScanImage(self,imageMessage):

        self.get_logger().info('Camera callback triggered')

        """ GENERAL IMAGE PROCESSING - Get objects """
        cv_ColorImage = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding='passthrough')
        depth_image = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')

        # convert to depth
        # May need to convert image to bytes first with
        success, encoded_image = cv2.imencode('.jpg', cv_ColorImage)
        content2 = encoded_image.tobytes()
        visionImage = vision.Image(content=content2)
        self.get_logger().info(str(type(visionImage)))
        # Send the image to the API for object localization
        response = self.client.object_localization(image=visionImage)
        # Extract localized object annotations
        objects = response.localized_object_annotations
        """ END GENERAL PROCESSING """

        """ STATE MACHINE """   
        if self.state_var == "Init":
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
            
            self.state_var = "RotateFind"
            
        if self.state_var == "RotateFind":
            ### ROTATION and ARM ###
            # Only change on transition

            ### OBJECT LOCALIZATION ###
            for object in objects:
                print("Detected object", object.name.lower())

                # If have desired object
                if object.name.lower() == self.desiredObject:

                    # Post its position
                    x_pixel, y_pixel = self.obj_detect.find_center(content2,object.name.lower())
                    # msg = Twist()
                    # msg.linear.x = 0.5  # Set linear velocity (forward)
                    # self.mobile_base_vel_publisher.publish(msg)

                    target_point = Point()
                    target_point.x = x_pixel
                    target_point.y = y_pixel
                    #self.target_publisher.publish(target_point)
                    self.obj_coord_publisher(target_point)
                    
                    #return x_pixel, y_pixel 

                    # NEED DEPTH CHANGES

                    # State transition
                    change_drive_to = String("go")
                    self.drive_state_publisher.publish(change_drive_to) # Stop in btw for carefullness

                    # Don't change gripper

                    self.state_var = "Drive2Obj"
        
        elif self.state_var == "Drive2Obj":
            ### DRIVE TOWARDS OBJECT ###
            # Only post once, on transition

            ### OBJECT LOCALIZATION ###
            for object in objects:
                print("Detected object", object.name.lower())

                # If have desired object
                if object.name.lower() == self.desiredObject:

                    # Post its position
                    x_pixel, y_pixel = self.obj_detect.find_center(content2,object.name.lower())
                    # msg = Twist()
                    # msg.linear.x = 0.5  # Set linear velocity (forward)
                    # self.mobile_base_vel_publisher.publish(msg)

                    target_point = Point()
                    target_point.x = x_pixel
                    target_point.y = y_pixel
                    #self.target_publisher.publish(target_point)
                    self.obj_coord_publisher(target_point)
                    
                    #return x_pixel, y_pixel 

                    # NEED DEPTH CHANGES

                    # State transition
                    #self.state_var = "Drive2Obj"
                else:
                    pass

                    # Do something if can't find object???
            


        #msg = Twist()
        #msg.angular.z = 0.5 # turn 

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



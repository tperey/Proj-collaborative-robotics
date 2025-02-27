#!/usr/bin/env python3
'''
Written by: Trevor Perey, Date: 1/29/2024

Adapted from Prof. Monroe Kennedy's Examplee

Node which talks to Arm Wrapper to grab object

read more about rospy publishers/subscribers here: https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
'''


import rclpy
from rclpy.node import Node

import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

#odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

import numpy as np

# POSITIONS
# PER TF LIST, standard T matrix from locobot/base_link to locobot/ee_gripper_link is
# R = Identity, p = [0.501, 0, 0.429]
# Pick something that makes sense with that
positions = np.array([[0.39, 0.0, 0.29],
                      [0.4, 0.0, 0.3],
                      [0.39, 0.0, 0.29],
                      [0.4, 0.0, 0.3]]) 

counter = 0

class ObjectGrabber(Node):
    """Class for simple object grabbing, assuming position is passed to it
    """
    def __init__(self):
        """
        No initialization params for now
        """
        super().__init__('test_arm_move')

        #Obtain or specify the goal pose (in sim, its between locobot/odom (world) and locobot/base_link (mobile base))
        # #Initialize to 0
        # target_pose = Pose()
        # target_pose.position.x = 0.0
        # target_pose.position.y = 0.0
        # #specify the desired pose to be the same orientation as the origin
        # target_pose.orientation.x = 0.0
        # target_pose.orientation.y = 0.0
        # target_pose.orientation.z = 0.0
        # target_pose.orientation.w = 1.0 # cos(theta/2)
        # self.target_pose = target_pose

        #Define the publishers
        self.arm_publisher = self.create_publisher(PoseStamped, "/arm_pose", 10) # Arbitrarily queued 10
        self.gripper_publisher = self.create_publisher(Bool, '/gripper', 10)
        self.get_logger().info('ObjectGrabber node has started')

        # BASIC TEST - use timer
        self.square_timer = self.create_timer(1.0, self.square_callback)

    def square_callback(self):
        global counter

        # Get desired pose
        """ IF LOOPING """
        # xd = positions[counter][0]
        # yd = positions[counter][1]
        # zd = positions[counter][2]

        """ IF ENTERING """
        xd = float(input("Enter x in m: "))
        yd = float(input("Enter y in m: "))
        zd = float(input("Enter z in m: "))

        # Publish
        desired_pose_msg = PoseStamped() # Define pose
        desired_pose_msg.pose.position.x = xd
        desired_pose_msg.pose.position.y = yd
        desired_pose_msg.pose.position.z = zd

        """ FOR STRAIGHT """
        # desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
        # desired_pose_msg.pose.orientation.y = 0.0
        # desired_pose_msg.pose.orientation.z = 0.0
        # desired_pose_msg.pose.orientation.w = 1.0

        """ FOR GRABBING FROM TOP (90º ROTATION ABOUT Y AXIS, SO ARM FACES DOWN)"""
        desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
        desired_pose_msg.pose.orientation.y = np.sqrt(2)/2
        desired_pose_msg.pose.orientation.z = 0.0
        desired_pose_msg.pose.orientation.w = np.sqrt(2)/2

        self.arm_publisher.publish(desired_pose_msg) # Publish pose

        # Update counter
        counter += 1
        if (counter > 3):
            counter = 0

        # Log
        self.get_logger().info(f"Published pose of: {xd}, {yd}, {zd}")

        """ GRIPPER DOESN'T WORK IN SIM """
        # Close gripper
        input("Press any key to CLOSE GRIPPER: ")
        gripper_msg = Bool()
        gripper_msg.data = False # CLOSE gripper
        self.gripper_publisher.publish(gripper_msg)
        self.get_logger().info(f"Closing gripper...")
    
        # WAIT to Reset timer
        input("Press any key to OPEN GRIPPER, then start timer for next command: ")
        gripper_msg.data = True # OPEN gripper
        self.gripper_publisher.publish(gripper_msg)
        self.get_logger().info(f"OPENING gripper...")

        self.get_logger().info(f"Running timer!")
        self.square_timer.reset()

def main(args=None):
    rclpy.init(args=args)

    #instantiate the class
    cls_obj = ObjectGrabber() #instantiate object of the class (to call and use the functions)

    rclpy.spin(cls_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cls_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


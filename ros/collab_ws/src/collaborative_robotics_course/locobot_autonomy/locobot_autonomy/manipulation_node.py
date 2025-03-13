#!/usr/bin/env python3
'''
Written by: Trevor Perey, Date: 2/26/2024

Adapted from Prof. Monroe Kennedy's Examplee

Node which talks to Arm Wrapper to grab object

read more about rospy publishers/subscribers here: https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
'''


# May not need several of these
import rclpy
from rclpy.node import Node

import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

#odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

import numpy as np

import time

# POSITIONS
# PER TF LIST, standard T matrix from locobot/base_link to locobot/ee_gripper_link is
# R = Identity, p = [0.501, 0, 0.429]
# Pick something that makes sense with that

OFFSET_Z = 0.15
Z_LIMIT = 0.2
OFFSET_X = 0.07
X_LIMIT = 0.0

class ManipulationNode(Node):
    """Class for simple object grabbing, assuming position is passed to it
    """
    def __init__(self):
        """
        No initialization params for now
        """
        super().__init__('ManipulationNode')

        """ CLASS VARIABLES """
        self.use_sim = False
        self.doPour = True
        self.topApproach = False
        #Obtain or specify the goal pose for end effector (ee)
        # Initialize desired object coords. Just the startup loc. 
        target_pose = Pose()
        target_pose.position.x = 0.5
        target_pose.position.y = 0.0
        target_pose.position.z = 0.4
        #specify the desired pose to be the same orientation as the origin
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0 # cos(theta/2)
        self.target_pose = target_pose

        self.gripper_state  = "wait" # Start out waiting

        """ SUBSCRIBERS """
        self.goal_coord_subscriber = self.create_subscription( # For storing desired object 
            Point,
            '/project_goal', # IN BASE_LINK FRAME
            self.goal_coord_callback, # Updates class var for desired loc coords
            10 # Just chosen randomly
        )

        self.gripper_state_subscriber = self.create_subscription( # For gripper state
            String,
            '/gripper_state',
            self.gripper_state_callback, # Drives the gripper
            10 # Just chosen randomly
        )
        self.object_subscriber = self.create_subscription(
            String,
            '/desired_object',
            self.object_callback,
            10
        )

        """ PUBLISHERS """
        self.arm_publisher = self.create_publisher(PoseStamped, "/arm_pose", 10) # Arbitrarily queued 10
        self.gripper_publisher = self.create_publisher(Bool, '/gripper', 10)
        self.sleep_publisher = self.create_publisher(Bool, "/go_home", 10)
        self.success_publisher = self.create_publisher(Bool, '/gripper_success', 10)
        
        # Acknowledge node start
        self.get_logger().info('ManipulationNode has started')
    
    def object_callback(self, msg):
        self.get_logger().info(f'ManipNode got {msg.data}')
        if msg.data == "strawberry":
            self.topApproach = True
            self.doPour = False
        elif msg.data == "package":
            self.topApproach = False
            self.doPour = True
        else:
            #error
            return
        self.get_logger().info(f'topApproach is {self.topApproach}, doPour is {self.doPour}')
    
    def goal_coord_callback(self, msg):

        """ Process Point message into target_pose """
        new_pose = Pose()
        #position
        new_pose.position.x = float(msg.x) # Ensure float
        new_pose.position.y = float(msg.y)
        new_pose.position.z = float(msg.z)
        #specify the desired pose to be facing DOWN
        new_pose.orientation.x = 0.0
        new_pose.orientation.y = np.sqrt(2)/2
        new_pose.orientation.z = 0.0
        new_pose.orientation.w = np.sqrt(2)/2 # cos(theta/2)

        """ Store """
        self.target_pose = new_pose

    def gripper_state_callback(self, msg):

        gripper_state = msg.data # Get string from msg
        gripper_state = gripper_state.lower() # Lowercase for consistency

        if gripper_state == "wait":
            # Move arm out of the way
            self.get_logger().info("Going home...")
            self.sleep_publisher.publish(Bool(data=True))
            self.success_publisher.publish(Bool(data=False))

        elif gripper_state == "grab": # Now, actually grab position
            self.get_logger().info("GRAB CALLBACK#################################")
                       
            if self.topApproach:
                """ INITIAL APPROACH - Convert stored target_pose into a msg to post """
                desired_pose_msg = PoseStamped() # Define pose
                #position
                desired_pose_msg.pose.position.x = max(float(self.target_pose.position.x - OFFSET_X), X_LIMIT)
                desired_pose_msg.pose.position.y = float(self.target_pose.position.y + 0.03)
                desired_pose_msg.pose.position.z = min(float(self.target_pose.position.z + OFFSET_Z), Z_LIMIT)
                #orientation
                desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
                desired_pose_msg.pose.orientation.y = np.sqrt(2)/2
                desired_pose_msg.pose.orientation.z = 0.0
                desired_pose_msg.pose.orientation.w = np.sqrt(2)/2
               
                """ Move arm ABOVE using publisher """
                self.arm_publisher.publish(desired_pose_msg) # Publish pose
                self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")

                time.sleep(2) # Pause to ensure got there

                """ open gripper using publisher"""
                if self.use_sim:
                    pass
                    # Gripper doesn't work in sim
                    self.get_logger().info(f"Got to position! Would do gripper, but we are in SIM")
                else:
                    self.get_logger().info(f"Found object! Opening gripper...")
                    gripper_msg = Bool()
                    gripper_msg.data = True # OPEN gripper
                    self.gripper_publisher.publish(gripper_msg)

                time.sleep(2) # Pause to ensure got there

                """ Lower arm onto object """
                desired_pose_msg.pose.position.z = self.target_pose.position.z+0.01
                self.arm_publisher.publish(desired_pose_msg) # Publish pose
                self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")

                time.sleep(4) # Pause to ensure got there

                """ Move arm into bottle """
                desired_pose_msg.pose.position.x = self.target_pose.position.x
                self.arm_publisher.publish(desired_pose_msg) # Publish pose
                self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")

                time.sleep(4) # Pause to ensure got there

            else:
                """ INITIAL APPROACH - Convert stored target_pose into a msg to post """
                desired_pose_msg = PoseStamped() # Define pose
                #position
                desired_pose_msg.pose.position.x = max(float(self.target_pose.position.x - 0.02), X_LIMIT)
                desired_pose_msg.pose.position.y = float(self.target_pose.position.y + 0.02)
                desired_pose_msg.pose.position.z = min(float(self.target_pose.position.z + 0.02), Z_LIMIT) # No z offset now
                # orientation - straight
                desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
                desired_pose_msg.pose.orientation.y = 0.0
                desired_pose_msg.pose.orientation.z = 0.0
                desired_pose_msg.pose.orientation.w = 1.0

                """ Move arm BEHIND using publisher """
                self.arm_publisher.publish(desired_pose_msg) # Publish pose
                self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")
            
                time.sleep(2) # Pause to ensure got there

                """ open gripper using publisher"""
                if self.use_sim:
                    pass
                    # Gripper doesn't work in sim
                    self.get_logger().info(f"Got to position! Would do gripper, but we are in SIM")
                else:
                    self.get_logger().info(f"Found object! Opening gripper...")
                    gripper_msg = Bool()
                    gripper_msg.data = True # OPEN gripper
                    self.gripper_publisher.publish(gripper_msg)

                time.sleep(4) # Pause to ensure got there

                """ Move arm into bottle """
                desired_pose_msg.pose.position.x = desired_pose_msg.pose.position.x + 0.07
                desired_pose_msg.pose.position.z = desired_pose_msg.pose.position.z - 0.03
                self.arm_publisher.publish(desired_pose_msg) # Publish pose
                self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")

                time.sleep(4) # Pause to ensure got there

            """ Close gripper using publisher"""
            if self.use_sim:
                pass
                # Gripper doesn't work in sim
                self.get_logger().info(f"Got to position! Would do gripper, but we are in SIM")
            else:              
                self.get_logger().info(f"Closing gripper...")
                gripper_msg = Bool()
                gripper_msg.data = False # CLOSE gripper
                self.gripper_publisher.publish(gripper_msg)
            
            time.sleep(4) # Pause to ensure got there

            """ Lift up """
            desired_pose_msg.pose.position.z = min(self.target_pose.position.z+OFFSET_Z, Z_LIMIT)
            self.arm_publisher.publish(desired_pose_msg) # Publish pose
            self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")

            time.sleep(4) # Pause to ensure got there
            
            self.success_publisher.publish(Bool(data=True))

            # Now go home
            self.get_logger().info("Going home...")
            self.sleep_publisher.publish(Bool(data=True))

            time.sleep(2) # Pause to ensure
        
        elif gripper_state == "hand":
            desired_pose_msg = PoseStamped() # Define pose
            #position
            desired_pose_msg.pose.position.x = float(self.target_pose.position.x)
            desired_pose_msg.pose.position.y = float(self.target_pose.position.y+0.05)
            desired_pose_msg.pose.position.z = min(float(self.target_pose.position.z)+0.10, Z_LIMIT)
            #orientation
            if not self.doPour:
                desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
                desired_pose_msg.pose.orientation.y = np.sqrt(2)/2
                desired_pose_msg.pose.orientation.z = 0.0
                desired_pose_msg.pose.orientation.w = np.sqrt(2)/2
            else:
                desired_pose_msg.pose.orientation.x = 0.0 # REQUIRES FLOATS
                desired_pose_msg.pose.orientation.y = 0.0
                desired_pose_msg.pose.orientation.z = 0.0
                desired_pose_msg.pose.orientation.w = 1.0
            
            """ Move arm ABOVE using publisher """
            self.arm_publisher.publish(desired_pose_msg) # Publish pose
            self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")
            time.sleep(4)

            """ POUR """
            if self.doPour:
                # Straighten out
                """ 60 degrees """
                # desired_pose_msg.pose.orientation.w = np.sqrt(3)*0.5
                # desired_pose_msg.pose.orientation.x = 0.5 # REQUIRES FLOATS
                # desired_pose_msg.pose.orientation.y = 0.0
                # desired_pose_msg.pose.orientation.z = 0.0
                """ 90 degrees"""
                desired_pose_msg.pose.orientation.w = np.sqrt(2)*0.5
                desired_pose_msg.pose.orientation.x = np.sqrt(2)*0.5 # REQUIRES FLOATS
                desired_pose_msg.pose.orientation.y = 0.0
                desired_pose_msg.pose.orientation.z = 0.0
                

                self.arm_publisher.publish(desired_pose_msg) # Publish pose
                self.get_logger().info(f"Published pose of: {desired_pose_msg.pose.position.x}, {desired_pose_msg.pose.position.y}, {desired_pose_msg.pose.position.z}")

                time.sleep(4) # Pause to ensure got there

            """ Go back home so arm doesn't crap out"""
            # First, drop item
            if self.use_sim:
                pass
                # Gripper doesn't work in sim
                self.get_logger().info(f"Got to position! Would do gripper, but we are in SIM")
            else:
                self.get_logger().info(f"Found object! Opening gripper...")
                gripper_msg = Bool()
                gripper_msg.data = True # OPEN gripper
                self.gripper_publisher.publish(gripper_msg)
            
            time.sleep(2) # Pause to ensure

            # Now go home
            self.get_logger().info("Going home...")
            self.sleep_publisher.publish(Bool(data=True))

            time.sleep(2) # Pause to ensure





def main(args=None):
    rclpy.init(args=args)

    #instantiate the class
    cls_obj = ManipulationNode() #instantiate object of the class (to call and use the functions)

    rclpy.spin(cls_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cls_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
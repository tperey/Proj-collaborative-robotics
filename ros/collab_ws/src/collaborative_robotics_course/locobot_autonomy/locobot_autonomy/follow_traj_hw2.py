#!/usr/bin/env python3
'''
Written by: Trevor Perey, Date: 1/29/2024

Adapted from Prof. Monroe Kennedy's Examplee

This script starts at the bottom "if __name__ == '__main__':" which is a function that calls "main():" function.
The main function then instanties a class object, which takes in a target pose, then listens to the topic 
of the robots odometry (pose), and then calculates the control action to take, then commands the velocity to the robot

This script shows how the robots can track a simple trajector

read more about rospy publishers/subscribers here: https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
'''


import rclpy
from rclpy.node import Node

import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

#odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

import numpy as np

KP_GAIN = 2.0 # Constant for changing control matrix
CIRCLE_RAD = 0.5 # Change for visualization. Final should be 0.5 m

class LocobotSimpleTraj(Node):
    """Class for simple trajectory locobot control

    This script demonstrates how to move follow a circular trajector using proportional Control
    The pose of the Locobot is defined by its (x,y) position 
    """
    def __init__(self,target_pose=None):
        """
        Input: Target_pose - ROS geometry_msgs.msg.Pose type
        Just initialize to 0
        """
        super().__init__('follow_traj_hw2')

        #Obtain or specify the goal pose (in sim, its between locobot/odom (world) and locobot/base_link (mobile base))
        #Initialize to 0
        if type(target_pose) == type(None):
            target_pose = Pose()
            target_pose.position.x = 0.0
            target_pose.position.y = 0.0
            #specify the desired pose to be the same orientation as the origin
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 1.0 # cos(theta/2)
            self.target_pose = target_pose
        elif type(target_pose) != type(Pose()):
            self.get_logger().info("Incorrect type for target pose, expects geometry_msgs Pose type") #send error msg if wrong type is send to go_to_pose
        else:
            self.target_pose = target_pose

        #Define the publishers
        self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/diffdrive_controller/cmd_vel_unstamped", 1) #this is the topic we will publish to in order to move the base
        # HW says to use the following, but test first
        #self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/diffdrive_controller/cmd_vel_unstamped", 1)
        self.get_logger().info('LocobotSimpleTraj Velo Publisher node has started')

        # use this if odometry message is reliable: 
        #using "/locobot/sim_ground_truth_pose" because "/odom" is from wheel commands in sim is unreliable
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/locobot/sim_ground_truth_pose",
            self.odom_mobile_base_callback,
            qos_profile=qos_profile_sensor_data  # Best effort QoS profile for sensor data [usual would be queue size: 1]
            ) #this says: listen to the odom message, of type odometry, and send that to the callback function specified
        self.odom_subscription  # prevent unused variable warning
        self.get_logger().info('LocobotSimpleTraj Pos Subscriber node has started')

        self.L = 0.1 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        self.t_init = self.get_clock().now() # Get initial time
        self.get_logger().info('LocobotSimpleTraj is now timing')

        # Timer to ensure only 20 seconds
        self.create_timer(20.0, self.shutdown_callback)

    def shutdown_callback(self):
        self.get_logger().info("Time limit of 20 s reached. Shutting down node.")
        rclpy.shutdown()

    def update_target_pos(self):

        t = self.get_clock().now() - self.t_init # Get current time
        t = t.nanoseconds/(1e9)

        #self.get_logger().info(f"Time = {t}")

        # Get desired position
        self.target_pose.position.x = CIRCLE_RAD*np.cos(t*(2*np.pi/10))
        self.target_pose.position.y = CIRCLE_RAD*np.sin(t*(2*np.pi/10))


    def odom_mobile_base_callback(self, data):

        # Step 0: Update target position based on time
        self.update_target_pos()
   
        # Step 1: Calculate the point P location (distance L on the x-axis), and publish the marker so it can be seen in Rviz
        #first determine the relative angle of the mobile base in the world xy-plane, this angle is needed to determine where to put the point P
        #the rotation will be about the world/body z-axis, so we will only need the qw, and qz quaternion components. We can then use knoweldge of the 
        #relationship between quaternions and rotation matricies to see how we must rotate the Lx vector into the world (odom) frame and add it to the base position
        #to obtain the point P (for more info on quaterions, see a primer at the bottom of this page: https://arm.stanford.edu/resources/armlab-references)
        # From existing example

        x_data = data.pose.pose.position.x
        y_data = data.pose.pose.position.y
        z_data = data.pose.pose.position.z
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z

        #frame rotation:
        R11 = qw**2 + qx**2 - qy**2 -qz**2
        R12 = 2*qx*qz + 2*qw*qz
        R21 = 2*qx*qz - 2*qw*qz
        R22 = qw**2 - qx**2 + qy**2 -qz**2

        point_P = Point()
        #NOTE: the following assumes that when at the origin, the baselink and odom/world frame are aligned, and the z-axis points up. If this is not true then there is not a simple rotation about the z-axis as shown below
        point_P.x = x_data + self.L*R11
        point_P.y = y_data + self.L*R21
        point_P.z = 0.1 #make it hover just above the ground (10cm)

        # No need to publish markers here


        # Step 2: Calculate the error between the target pose for position control (this will relate to the proportoinal gain matrix, the P in PID control)
        err_x = self.target_pose.position.x - point_P.x
        err_y = self.target_pose.position.y - point_P.y
        error_vect = np.matrix([[err_x],[err_y]]) #this is a column vector (2x1); equivalently, we could use the transpose operator (.T): np.matrix([err_x ,err_y]).T  

        Kp_mat = KP_GAIN*np.eye(2) #proportional gain matrix, diagonal with gain specified

        Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
        # This Angle is selected because its the frame rotation angle. Used in control law computation
        current_angle = np.arctan2(Rotation_mat[0,1],Rotation_mat[1,1]) #this is also the angle about the z-axis of the base

        '''
        We do not do perform derivative, integral, or rotational control here
        '''        

        # Step 3: now put it all together to calculate the control input (velocity) based on the position error and integrated error
        point_p_error_signal = Kp_mat*error_vect
        #The following relates the desired motion of the point P and the commanded forward and angular velocity of the mobile base [v,w]
        non_holonomic_mat = np.matrix([[np.cos(current_angle), -self.L*np.sin(current_angle)],[np.sin(current_angle),self.L*np.cos(current_angle)]])
        #Now perform inversion to find the forward velocity and angular velcoity of the mobile base.
        control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal #note: this matrix can always be inverted because the angle is L
   
        #Step 4. now let's turn this into the message type and publish it to the robot:
        control_msg = Twist()
        control_msg.linear.x = float(control_input.item(0)) #extract these elements then cast them in float type
        control_msg.angular.z = float(control_input.item(1))
        #now publish the control output:
        self.mobile_base_vel_publisher.publish(control_msg)

        #control_msg_plot = TwistStamped() # For plotting
        #self.get_logger().info(f"Current: ({point_P.x}, {point_P.y})")
        #self.get_logger().info(f"DESIRED: ({self.target_pose.position.x}, {self.target_pose.position.y})")
        self.get_logger().info(f"Des->Cur: {self.target_pose.position.x}, {self.target_pose.position.y}) -> ({point_P.x}, {point_P.y})")



def main(args=None):
    rclpy.init(args=args)

    #instantiate the class
    cls_obj = LocobotSimpleTraj() #instantiate object of the class (to call and use the functions)

    rclpy.spin(cls_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cls_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


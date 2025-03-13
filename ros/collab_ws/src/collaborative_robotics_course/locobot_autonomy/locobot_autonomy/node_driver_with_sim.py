#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import String
from interbotix_xs_modules.xs_robot.locobot import InterbotixLocobotXS

CRCT_TURN_SPEED = 0.5 #***From Trevor - idk units
ANGLE_CUTOFF = 10.0 #***From Trevor - degrees
TURN_TIME = 10.0 # Seconds for rotational correction 

FWD_SPEED = 0.7 #***From Trevor - m/s I guess?
FINDING_ROT_SPEED = -0.8 #***From Trevor - quite slow but ensures detectionresponsiveness

class Sable_Driver(Node):
    def __init__(self):
        super().__init__('driver_node')
         # Declare the parameter 'use_sim' with a default value of ***TRUE***
        self.declare_parameter('use_sim', True)
        
        # Get the parameter value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        
        # Log the value of 'use_sim'
        self.get_logger().info(f"use_sim parameter set to: {self.use_sim}")

        """ PUBLISHERS """
        self.twist_publisher = self.create_publisher(Twist,"/locobot/diffdrive_controller/cmd_vel_unstamped", 1) #this is the topic we will publish to in order to move the base in sim

        """ SUBSCRIBERS """
        self.drive_state_subscriber = self.create_subscription(
            String,  # Message type from std_msgs
            '/drive_state',  # Topic name
            self.drive_state_callback,  # Callback function
            10  # Queue size
        )

        self.goal_subscriber = self.create_subscription(
            Point,  # Message type from std_msgs
            '/project_goal',  # Topic name
            self.project_goal_callback,  # Callback function
            10  # Queue size
        )

        self.project_goal = None

        if not self.use_sim:
            self.locobot = InterbotixLocobotXS(
            robot_model='locobot_wx250s',
            robot_name='locobot',
            arm_model='mobile_wx250s'
            )


    def drive_state_callback(self, msg):
        """
        Drive state callback for controlling the LoCoBot base.
        """
        #self.get_logger().info(f"Received drive state message: {msg.data}")

        # Define velocity values based on the command
        if msg.data == "go":

            if self.project_goal is not None: # Guard to ensure there is a goal, prevents crashes
                vel = Twist()
                turn_time = TURN_TIME

                #compensate misalignment
                #x, y, z = self.project_goal # Syntax doesnt work
                x = self.project_goal.x
                y = self.project_goal.y
                z = self.project_goal.z

                base_misalignment = (180.0/np.pi)*np.arctan2(y,x) # Convert to degrees. Misalignment btw front of robot and 
                
                if base_misalignment >= ANGLE_CUTOFF: # Angle is POSITIVE. Meaning y is POSITIVE. Need CCW rot (positive turn)
                    vel = Twist()
                    vel.angular.z = CRCT_TURN_SPEED # Degrees per (turn_time) sec. So min of 2º per sec
                    vel.linear.x = 0.0
                    # self.get_logger().info(f'+++POSITIVE twist')
                elif base_misalignment <= -1*ANGLE_CUTOFF: # Angle is NEGATIVE. Meaning y is NEGATIVE. Need CW rot (negative turn)
                    vel.angular.z = -1*CRCT_TURN_SPEED # Degrees per (turn_time) sec. So min of 2º per sec
                    vel.linear.x = 0.0
                    # self.get_logger().info(f'---NEGATIVE twist')
                else:
                    vel.angular.z = 0.0 # Don't correct
                    vel.linear.x = FWD_SPEED
                    # self.get_logger().info(f'No twist')

                vel.linear.y = 0.0 # Others are static
                vel.linear.z = 0.0
                vel.angular.x = 0.0
                vel.angular.y = 0.0

                # Post
                if self.use_sim:
                    # In sim, Directly command
                    self.twist_publisher.publish(vel)
                    #self.get_logger().info(f'Moved base in sim as {vel.linear}, {vel.angular}')
                else:
                    self.locobot.base.command_velocity_for_duration(vel, turn_time) 
                    #Go straight
                    self.locobot.base.command_velocity_xyaw(0.1, 0)
                
        elif msg.data == "stop":
            if self.use_sim:
                # In sim, Directly command
                rotatemsg = Twist()
                rotatemsg.linear = Vector3(x=0.0, y=0.0, z=0.0)
                rotatemsg.angular = Vector3(x=0.0, y=0.0, z=0.0) # Stop

                self.twist_publisher.publish(rotatemsg)

                #self.get_logger().info(f'Moved base in sim as {rotatemsg.linear}, {rotatemsg.angular}')
            else:
                vel = Twist()
                self.locobot.base.command_velocity(vel)

        elif msg.data == "turn":
            if self.use_sim:
                # In sim, Directly command
                rotatemsg = Twist()
                rotatemsg.linear = Vector3(x=0.0, y=0.0, z=0.0)
                rotatemsg.angular = Vector3(x=0.0, y=0.0, z=FINDING_ROT_SPEED) # Just rotate

                self.twist_publisher.publish(rotatemsg)

                # self.get_logger().info(f'Moved base in sim as {rotatemsg.linear}, {rotatemsg.angular}')
            else:
                self.locobot.base.command_velocity_xyaw(0,0.2)

    def project_goal_callback(self,msg):
        self.get_logger().info(f"Received project goal: ({msg.x}, {msg.y}, {msg.z})")
        self.project_goal = msg





def main(args=None):
    rclpy.init(args=args)

    node = Sable_Driver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
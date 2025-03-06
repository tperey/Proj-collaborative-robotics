#!/usr/bin/env python3
'''
Written by: Trevor Perey, Date: 2/26/2024

Just publishes to topics to 

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
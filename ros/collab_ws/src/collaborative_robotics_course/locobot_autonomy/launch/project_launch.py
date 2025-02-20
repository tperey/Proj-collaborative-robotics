#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='locobot_autonomy',
            executable='navigation.py',
            name='navigation'
        ),
        # Node(
        #     package='locobot_autonomy',
        #     executable='perception.py',
        #     name='perception'
        # ),
        # Node(
        #     package='locobot_autonomy',
        #     executable='grasp.py',
        #     name='grasp'
        # ),
    ])

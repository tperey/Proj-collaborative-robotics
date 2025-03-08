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
            executable='localization_tp.py',
            name='TPlocalizer'
        ),
        Node(
            package='locobot_autonomy',
            executable='manipulation_node.py',
            name='ManipulationNode'
        ),
        Node(
            package='locobot_autonomy',
            executable='node_driver_with_sim.py',
            name='driver_node'
        ),
        Node(
            package='locobot_autonomy',
            executable='perception/camera_with_sim.py',
            name='scan_approach_node'
        ),
    ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='locobot_wx250s', description='Robot model'),
        DeclareLaunchArgument('hardware_type', default_value='actual', description='Hardware type'),
        DeclareLaunchArgument('use_lidar', default_value='false', description='Use lidar'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Use RViz'),
        DeclareLaunchArgument('slam_mode', default_value='mapping', description='SLAM mode'),
        DeclareLaunchArgument('rtabmap_args', default_value='-d', description='RTAB-Map arguments'),

        # Include the interbotix_xslocobot_nav launch file
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_nav'),
                'launch',
                'xslocobot_rtabmap.launch.py'
            ])
            ),
            launch_arguments={
                'robot_model': LaunchConfiguration('robot_model'),
                'hardware_type': LaunchConfiguration('hardware_type'),
                'use_lidar': LaunchConfiguration('use_lidar'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'slam_mode': LaunchConfiguration('slam_mode'),
                'rtabmap_args': LaunchConfiguration('rtabmap_args')
            }.items(),
        ),
    ])

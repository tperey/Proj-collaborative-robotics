#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    param_file = '/home/ubuntu/Desktop/collaborative/ros/collab_ws/move.yaml'
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='Whether to use simulation or not'),

        # Log the 'use_sim' parameter to confirm it
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_sim')),
            msg="use_sim parameter is set to True: Launching additional nodes"
        ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_sim')),
            msg="Launching pose_behavior_node with simulation..."
        ),
        
        # Launch the Arm Control node
        Node(
            package='locobot_wrapper',
            executable='arm_control_wrapper.py',
            name='arm_control_wrapper',
            output='screen',
            parameters=[{'use_sim': LaunchConfiguration('use_sim')}],
        ),

        # publishing the semantic robot description
        Node(
            package='locobot_wrapper',
            executable='publish_robot_description_semantic.py',
            name='publish_robot_description_semantic',
            output='screen',
            parameters=[param_file]
        ),

        # Only launch move_arm_action_server if use_sim is True
        Node(
            package='locobot_wrapper',
            executable='move_arm_action_server',
            name='move_arm_action_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),

        Node(
            package='locobot_wrapper',
            executable='move_gripper_action_server.py',
            name='move_gripper_action_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        )
    ])

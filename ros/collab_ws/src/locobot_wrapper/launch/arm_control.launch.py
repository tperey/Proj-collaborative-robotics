#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xslocobot_robot_description_launch_arguments,
)


def launch_setup(context, *args, **kwargs):
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    robot_description = {
        'robot_description': ParameterValue(robot_description_launch_arg, value_type=str)
    }

    move_arm_server_node = Node(
            package='locobot_wrapper',
            executable='move_arm_action_server',
            name='move_arm_action_server',
            output='screen',
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),

    return [move_arm_server_node]

def generate_launch_description():

    robot_description_launch_arg = LaunchConfiguration('robot_description')
    robot_description = {
        'robot_description': ParameterValue(robot_description_launch_arg, value_type=str)
    }

    import pdb; pdb.set_trace()

    locobots_nodes = [
        DeclareLaunchArgument('use_sim', default_value='true', description='Whether to use simulation or not'),
        # declare_interbotix_xslocobot_robot_description_launch_arguments(
        #     show_gripper_bar='true',
        #     show_gripper_fingers='true',
        #     hardware_type='actual',
        # ),
        # Log the 'use_sim' parameter to confirm it
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_sim')),
            msg="use_sim parameter is set to True: Launching additional nodes"
        ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_sim')),
            msg="Launching pose_behavior_node with simulation..."
        ),

        Node(
            package='locobot_wrapper',
            executable='move_arm_action_server',
            name='move_arm_action_server',
            output='screen',
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),
        
        # Launch the Arm Control node
        Node(
            package='locobot_wrapper',
            executable='arm_control_wrapper.py',
            name='arm_control_wrapper',
            output='screen',
            parameters=[{'use_sim': LaunchConfiguration('use_sim')}],
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
    ]

    X =declare_interbotix_xslocobot_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )

    locobots_nodes.extend(X)

    # Declare launch arguments
    return LaunchDescription(locobots_nodes)

# Copyright (c) AIDIN ROBOTICS Inc.
# SPDX-License-Identifier: Apache-2.0

"""URDF visualisation only, robot_state_publisher with joint_state_publisher_gui and rviz2.

The control stack is aidin_hand2_bringup/aidin_hand2.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('aidin_hand2_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'aidin_hand2.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

    use_left_hand = LaunchConfiguration('use_left_hand')
    use_right_hand = LaunchConfiguration('use_right_hand')
    use_gui = LaunchConfiguration('use_gui')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_left_hand:=', use_left_hand,
            ' use_right_hand:=', use_right_hand,
            ' use_mock:=true',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_left_hand', default_value='true'),
        DeclareLaunchArgument('use_right_hand', default_value='true'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Run joint_state_publisher_gui with its sliders.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])

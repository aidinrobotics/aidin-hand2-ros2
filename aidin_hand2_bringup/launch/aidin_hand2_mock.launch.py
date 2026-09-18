# Copyright (c) AIDIN ROBOTICS Inc.
# SPDX-License-Identifier: Apache-2.0

"""AIDIN Hand Gen2 on mock hardware, no CAN and no homing.

The path is ~/cmd -> joint_position_controller -> mock hardware -> /joint_states -> rviz.

  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py                        # both hands
  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_right_hand:=false  # left only
  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_left_hand:=false   # right only

The command topic is /<side>_joint_position_controller/cmd, a sensor_msgs/JointState whose name
entries select the joints.

aidin_hand2_controllers.launch.py is not included here, it also spawns the hand state and
diagnostics broadcasters, which controllers_mock.yaml leaves out.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Seconds the spawner waits for the controller_manager
_SPAWNER_TIMEOUT = "30"


def _spawn(controller_name, condition=None, param_file=None):
    args = [controller_name,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", _SPAWNER_TIMEOUT]
    if param_file is not None:
        args += ["--param-file", param_file]
    return Node(package="controller_manager", executable="spawner",
                arguments=args, output="screen", condition=condition)


def generate_launch_description():
    description_share = get_package_share_directory("aidin_hand2_description")
    bringup_share = get_package_share_directory("aidin_hand2_bringup")
    xacro_file = os.path.join(description_share, "urdf", "aidin_hand2.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers_mock.yaml")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")

    use_left_hand = LaunchConfiguration("use_left_hand")
    use_right_hand = LaunchConfiguration("use_right_hand")
    use_rviz = LaunchConfiguration("use_rviz")

    # The CAN and homing arguments are left out, the mock ignores them
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_left_hand:=", use_left_hand,
            " use_right_hand:=", use_right_hand,
            " use_mock:=true",
        ]),
        value_type=str,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_left_hand", default_value="true",
                              description="Start the left mock hand."),
        DeclareLaunchArgument("use_right_hand", default_value="true",
                              description="Start the right mock hand."),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Show the mock hand in rviz2."),

        control_node,
        robot_state_publisher,

        # One for the whole robot, spawned once
        _spawn("joint_state_broadcaster"),

        # One joint_position_controller per hand, nothing else
        _spawn("left_joint_position_controller", IfCondition(use_left_hand)),
        _spawn("right_joint_position_controller", IfCondition(use_right_hand)),

        Node(package="rviz2", executable="rviz2",
             arguments=["-d", rviz_config],
             condition=IfCondition(use_rviz), output="screen"),
    ])

# Copyright (c) AIDIN ROBOTICS Inc.
# SPDX-License-Identifier: Apache-2.0

"""rosbridge_websocket on port 9090, the WebSocket server the desktop GUI connects to.

Runs as its own process alongside the control stack, in either order.

  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py
  ros2 launch aidin_hand2_bringup gui_bridge.launch.py

In the GUI settings pick rosbridge with ws://localhost:9090. port:=<n> moves it, and the GUI URL
has to follow.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch", "rosbridge_websocket_launch.xml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="9090"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={"port": LaunchConfiguration("port")}.items(),
        ),
    ])
# Copyright (c) AIDIN ROBOTICS Inc.
# SPDX-License-Identifier: Apache-2.0

"""rosbridge_websocket on port 9090, which the GUI's web_bridge connects to.

Runs as its own process alongside the control stack, in either order.

  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py
  ros2 launch aidin_hand2_bringup gui_bridge.launch.py

In the GUI profile enter this PC's address and the port: localhost:9090 when the GUI runs on this
PC, this PC's IP (192.168.0.10:9090, say) when it runs on another PC on the network. port:=<n>
moves it, and the profile has to follow. address:=127.0.0.1 takes connections from this PC only;
the default takes them on every interface.

Clients may use only the names the GUI uses, below, and rosapi, which the GUI's profile check calls.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# The names the GUI's web_bridge subscribes to, publishes on and calls (web_bridge/src/ros_names.hpp
# in aidin-hand2-gui).
SIDES = ("left", "right")
COMMAND_CONTROLLERS = (
    "joint_position_controller", "joint_impedance_controller",
    "actuator_position_controller", "actuator_effort_controller",
)
HAND_SERVICES = ("run", "stop", "home", "reconnect", "get_parameters", "set_parameters")

STATE_TOPICS = [f"/{s}_hand_state_broadcaster/hand_state" for s in SIDES] + [
    f"/{s}_diagnostics_broadcaster/hand_diagnostics" for s in SIDES] + ["/rosout"]
COMMAND_TOPICS = [f"/{s}_{c}/cmd" for s in SIDES for c in COMMAND_CONTROLLERS]
SERVICES = [f"/{s}_hand_control/{v}" for s in SIDES for v in HAND_SERVICES] + [
    "/controller_manager/switch_controller"]


def glob_list(names):
    # rosbridge reads "['a', 'b']". The outer quotes keep the launch file's YAML from turning it
    # into a list, which the parameter would refuse.
    return '"[' + ", ".join(f"'{n}'" for n in names) + ']"'


def generate_launch_description():
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch", "rosbridge_websocket_launch.xml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="9090"),
        DeclareLaunchArgument(
            "address", default_value="",
            description="Address to take connections on; empty for every interface"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={
                "port": LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                # run blocks until the drives confirm, up to 4 s. On rosbridge's main thread that
                # would hold every other transfer, the hand's state included, for as long.
                "call_services_in_new_thread": "true",
                "topics_pub_glob": glob_list(COMMAND_TOPICS),
                # rosapi answers who subscribes to a topic only for topics this glob allows, and
                # the profile check asks it about the command topics.
                "topics_sub_glob": glob_list(STATE_TOPICS + COMMAND_TOPICS),
                "services_glob": glob_list(SERVICES),
            }.items(),
        ),
    ])

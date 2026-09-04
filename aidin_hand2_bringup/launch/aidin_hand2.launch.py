"""AIDIN Hand Gen2 control stack on real hardware.

Starts ros2_control_node and robot_state_publisher from the xacro description, then includes
aidin_hand2_controllers.launch.py to spawn the controllers and broadcasters.

Arguments come from config/hand_bringup.yaml, and the priority is
CLI (key:=value) > config YAML > the defaults below.

  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py
  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py config:=my.yaml
  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py use_left_hand:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Launch arguments and the defaults used when the config file has no such key
_ARG_DEFAULTS = {
    "use_left_hand": ("true", "Start the left hand."),
    "use_right_hand": ("true", "Start the right hand."),
    "auto_home": ("true", "Home right after startup, which moves the hand."),
    "left_hand_interface": ("can0", "CAN interface of the left hand."),
    "right_hand_interface": ("can1", "CAN interface of the right hand."),
    "left_hand_cpu_affinity": ("-1", "Core the left hand RT loop is pinned to, -1 = unset."),
    "right_hand_cpu_affinity": ("-1", "Core the right hand RT loop is pinned to, -1 = unset."),
    "left_hand_disabled_actuators": ("", "Left hand actuator indices to leave off, comma separated such as '0,1,2,3'. Empty leaves every actuator on."),
    "right_hand_disabled_actuators": ("", "Right hand actuator indices to leave off, comma separated. Empty leaves every actuator on."),
    "auto_reconnect": ("false", "Reconnect after a lost link, both hands."),
    "auto_reconnect_timeout_ms": ("0", "Timeout before giving up on auto reconnect [ms], 0 = no limit."),
    "auto_reconnect_home": ("false", "Home before run when auto reconnect recovers."),
}


def _load_config(path):
    """Read the config YAML as {argument: "string value"}, empty when there is no file.

    Values are stringified for xacro, bool as lowercase true or false.
    """
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    out = {}
    for key, value in raw.items():
        out[key] = "true" if value is True else "false" if value is False else str(value)
    return out


def _setup(context, *_):
    # The config path is itself an argument, resolved here in the OpaqueFunction
    config = _load_config(LaunchConfiguration("config").perform(context))

    # A config value becomes the argument default, and the CLI overrides that
    declared = [
        DeclareLaunchArgument(
            name, default_value=config.get(name, default), description=desc)
        for name, (default, desc) in _ARG_DEFAULTS.items()
    ]

    description_share = get_package_share_directory("aidin_hand2_description")
    bringup_share = get_package_share_directory("aidin_hand2_bringup")
    xacro_file = os.path.join(description_share, "urdf", "aidin_hand2.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")

    use_left_hand = LaunchConfiguration("use_left_hand")
    use_right_hand = LaunchConfiguration("use_right_hand")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_left_hand:=", use_left_hand,
            " use_right_hand:=", use_right_hand,
            " left_hand_interface:=", LaunchConfiguration("left_hand_interface"),
            " right_hand_interface:=", LaunchConfiguration("right_hand_interface"),
            " left_hand_cpu_affinity:=", LaunchConfiguration("left_hand_cpu_affinity"),
            " right_hand_cpu_affinity:=", LaunchConfiguration("right_hand_cpu_affinity"),
            " left_hand_disabled_actuators:=", LaunchConfiguration("left_hand_disabled_actuators"),
            " right_hand_disabled_actuators:=", LaunchConfiguration("right_hand_disabled_actuators"),
            " auto_home:=", LaunchConfiguration("auto_home"),
            " auto_reconnect:=", LaunchConfiguration("auto_reconnect"),
            " auto_reconnect_timeout_ms:=", LaunchConfiguration("auto_reconnect_timeout_ms"),
            " auto_reconnect_home:=", LaunchConfiguration("auto_reconnect_home"),
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

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "aidin_hand2_controllers.launch.py")),
        launch_arguments={
            "use_left_hand": use_left_hand,
            "use_right_hand": use_right_hand,
            "controller_manager": "/controller_manager",
        }.items(),
    )

    return [*declared, control_node, robot_state_publisher, controllers]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("aidin_hand2_bringup"), "config", "hand_bringup.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=default_config,
            description="Path to the argument config YAML, whose values override the defaults."),
        OpaqueFunction(function=_setup),
    ])

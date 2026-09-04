"""AIDIN Hand Gen2 on Isaac Sim, exchanging ROS 2 topics instead of CAN.

Out
  ~/command -> joint_position_controller -> bridge hardware
  -> <topic_prefix>/hand_command (sensor_msgs/JointState, 21 joints)
  -> Isaac Articulation Controller
In
  Isaac -> <topic_prefix>/joint_states -> bridge hardware -> /joint_states, hand_state,
  diagnostics
  Isaac tactile publisher -> <tactile_prefix>/<side>_<finger>_sensor
  (std_msgs/Float64MultiArray)

  ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py
  ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py topic_prefix:=/isaac_sim

The hand state and diagnostics broadcasters come up too, and the tactile values stay 0 while
Isaac publishes none.
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
    description_share = get_package_share_directory("aidin_hand2_description")
    bringup_share = get_package_share_directory("aidin_hand2_bringup")
    xacro_file = os.path.join(description_share, "urdf", "aidin_hand2.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers_isaac.yaml")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")

    use_rviz = LaunchConfiguration("use_rviz")

    # The CAN and homing arguments are left out, the bridge ignores them
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_left_hand:=true use_right_hand:=false",
            " use_isaac:=true",
            " isaac_topic_prefix:=", LaunchConfiguration("topic_prefix"),
            " isaac_joint_state_topic:=", LaunchConfiguration("joint_state_topic"),
            " isaac_joint_command_topic:=", LaunchConfiguration("joint_command_topic"),
            " isaac_tactile_prefix:=", LaunchConfiguration("tactile_prefix"),
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

    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    hand_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_hand_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    diagnostics_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_diagnostics_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    joint_position_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_joint_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("topic_prefix", default_value="/isaac",
                              description="The three topic names below hang off this prefix."),
        DeclareLaunchArgument("joint_state_topic", default_value="joint_states",
                              description="Isaac to hardware joint state topic "
                                          "(sensor_msgs/JointState). Matching is by name, so the "
                                          "rest of the robot can share it."),
        DeclareLaunchArgument("joint_command_topic", default_value="hand_command",
                              description="Hardware to Isaac joint command topic "
                                          "(sensor_msgs/JointState, 21 joints)."),
        DeclareLaunchArgument("tactile_prefix", default_value="tactile",
                              description="Tactile topic prefix. "
                                          "<prefix>/<side>_<finger>_sensor and "
                                          "<prefix>/<side>_palm_sensor are subscribed as "
                                          "std_msgs/Float64MultiArray."),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Show the hand in rviz2."),
        control_node,
        robot_state_publisher,
        joint_state_broadcaster,
        hand_state_broadcaster,
        diagnostics_broadcaster,
        joint_position_controller,
        rviz,
    ])

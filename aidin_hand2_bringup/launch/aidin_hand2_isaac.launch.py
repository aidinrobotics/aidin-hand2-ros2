"""AIDIN Hand Gen2 Isaac Sim 기동 — 실 CAN 없이 Isaac 과 ROS 2 토픽으로만 주고받는 스택.

데이터 흐름:
  (glove_teleop 또는 typed ~/command) → joint_position_controller
    → Isaac 브리지 hardware → <topic_prefix>/hand_command (sensor_msgs/JointState, 21 joint)
    → Isaac Articulation Controller
  Isaac → <topic_prefix>/joint_states → 브리지 hardware → /joint_states · hand_state · diagnostics
  Isaac 촉각 퍼블리셔(있다면) → <tactile_prefix>/<side>_<finger>_sensor (std_msgs/Float64MultiArray)

  ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py
  ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py topic_prefix:=/isaac_sim
  ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py use_glove:=true

mock(aidin_hand2_mock.launch.py)과 달리 촉각·diagnostics state 를 그대로 내보내므로
hand_state_broadcaster / diagnostics_broadcaster 도 함께 기동한다. Isaac 이 촉각을 발행하지
않으면 해당 값은 0 으로 남는다.
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
    examples_share = get_package_share_directory("aidin_hand2_examples")
    xacro_file = os.path.join(description_share, "urdf", "aidin_hand2.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers_isaac.yaml")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")
    glove_yaml = os.path.join(
        examples_share, "config", "glove_teleop", "glove_teleop_controller.yaml")

    use_glove = LaunchConfiguration("use_glove")
    use_rviz = LaunchConfiguration("use_rviz")

    # Isaac 브리지 hardware — use_isaac:=true. 실 CAN·homing 인자는 브리지가 무시하므로 넘기지 않는다.
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

    # 글러브 텔레오퍼 (옵션) — joint_position_controller 의 reference 를 claim 하는 chain 최상위.
    glove_teleop = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "left_glove_teleop_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", glove_yaml,
        ],
        condition=IfCondition(use_glove),
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
                              description="Isaac 브리지 토픽 prefix. 아래 세 인자를 비워 두면 "
                                          "<prefix>/joint_states · <prefix>/hand_command · "
                                          "<prefix>/tactile 을 쓴다."),
        DeclareLaunchArgument("joint_state_topic", default_value="",
                              description="Isaac -> hardware 조인트 상태 토픽 override "
                                          "(sensor_msgs/JointState). 이름 매칭이라 본체와 공유해도 된다."),
        DeclareLaunchArgument("joint_command_topic", default_value="",
                              description="hardware -> Isaac 조인트 명령 토픽 override "
                                          "(sensor_msgs/JointState, 21 joint)."),
        DeclareLaunchArgument("tactile_prefix", default_value="",
                              description="촉각 토픽 prefix override. "
                                          "<prefix>/<side>_<finger>_sensor 와 <prefix>/<side>_palm_sensor "
                                          "를 std_msgs/Float64MultiArray 로 구독한다."),
        DeclareLaunchArgument("use_glove", default_value="false",
                              description="MANUS 글러브 텔레오퍼 controller spawn (manus_data_publisher 필요)."),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="rviz2 로 손 시각화."),
        control_node,
        robot_state_publisher,
        joint_state_broadcaster,
        hand_state_broadcaster,
        diagnostics_broadcaster,
        joint_position_controller,
        glove_teleop,
        rviz,
    ])

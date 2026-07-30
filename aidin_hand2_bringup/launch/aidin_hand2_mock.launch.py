"""AIDIN Hand Gen2 mock 기동 — 실 CAN 없이 kinematics(clamp→IK→FK) 로 도는 검증/시각화 스택.

실 하드웨어·homing 없이 joint_position_controller 만으로 손을 움직여 rviz 로 확인한다.
데이터 흐름: (glove_teleop 또는 typed ~/command) → joint_position_controller
  → mock hardware(clamp→IK→FK) → /joint_states → rviz.

  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py                 # rviz + 왼손 mock
  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_glove:=true # + MANUS 글러브 텔레오퍼

글러브 텔레오퍼는 use_glove:=true 이고 manus_data_publisher 가 /manus_glove_0 발행 중일 때만.
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
    controllers_yaml = os.path.join(bringup_share, "config", "controllers_mock.yaml")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")
    glove_yaml = os.path.join(
        examples_share, "config", "glove_teleop", "glove_teleop_controller.yaml")

    use_glove = LaunchConfiguration("use_glove")
    use_rviz = LaunchConfiguration("use_rviz")

    # mock hardware — use_mock:=true. 실 CAN·homing 인자는 mock 이 무시하므로 넘기지 않는다.
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_left_hand:=true use_right_hand:=false",
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

    # mock 은 broadcaster/actuator/impedance 불필요 — joint_state_broadcaster + joint_position 만.
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
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
        DeclareLaunchArgument("use_glove", default_value="false",
                              description="MANUS 글러브 텔레오퍼 controller spawn (manus_data_publisher 필요)."),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="rviz2 로 mock 손 시각화."),
        control_node,
        robot_state_publisher,
        joint_state_broadcaster,
        joint_position_controller,
        glove_teleop,
        rviz,
    ])

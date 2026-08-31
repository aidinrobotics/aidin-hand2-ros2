"""AIDIN Hand Gen2 mock 기동 — 실 CAN 없이 kinematics(clamp→IK→FK) 로 도는 검증/시각화 스택.

실 하드웨어·homing 없이 joint_position_controller 만으로 손을 움직여 rviz 로 확인한다.
데이터 흐름: (glove_teleop 또는 typed ~/command) → joint_position_controller
  → mock hardware(clamp→IK→FK) → /joint_states → rviz.

손 선택 인자 이름은 aidin_hand2.launch.py 와 같은 독립 boolean 두 개다. 다만 기본값은 다르다 —
mock 은 검증용이라 양손을 다 올리고, 실 하드웨어 쪽은 연결된 손만 올린다.

  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py                        # 양손 (기본)
  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_right_hand:=false  # 왼손만
  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_left_hand:=false   # 오른손만
  ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_glove:=true        # + MANUS 글러브

두 손은 URDF 에서 y 로 +-0.1 m 벌려 둔다(aidin_hand2.urdf.xacro 의 *_hand_xyz 기본값) —
같은 자리에 렌더되면 rviz 에서 구분이 안 되기 때문이다.

지령 토픽은 손마다 /<side>_joint_position_controller/command
(aidin_hand2_msgs/JointPositionCommand, 16 값). joint_state_broadcaster 는 robot-wide
싱글톤이라 두 손을 올려도 하나만 spawn 한다.

aidin_hand2_controllers.launch.py 를 include 하지 않는 이유: 그쪽은 손마다
hand_state_broadcaster·diagnostics_broadcaster 도 spawn 하는데, mock 은 센서 상태를
모델링하지 않아 controllers_mock.yaml 이 그 둘을 빼기 때문이다. mock 이 필요한 것은
joint_state_broadcaster + joint_position_controller 뿐이다.

글러브 텔레오퍼는 use_glove:=true 이고 manus_data_publisher 가 /manus_glove_0 발행 중일 때만.
**glove_teleop_controller.yaml 은 왼손 인스턴스만 정의**하므로(scale/offset 이 실측
캘리브값이라 미러로 옮길 수 없다) use_glove:=true 는 왼손이 올라와 있을 때만 쓴다(기본값이면 올라와 있다).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# spawner 가 controller_manager 를 기다리는 시간 — 두 손이면 hardware 초기화가 그만큼 길어진다.
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
    examples_share = get_package_share_directory("aidin_hand2_examples")
    xacro_file = os.path.join(description_share, "urdf", "aidin_hand2.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers_mock.yaml")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")
    glove_yaml = os.path.join(
        examples_share, "config", "glove_teleop", "glove_teleop_controller.yaml")

    use_left_hand = LaunchConfiguration("use_left_hand")
    use_right_hand = LaunchConfiguration("use_right_hand")
    use_glove = LaunchConfiguration("use_glove")
    use_rviz = LaunchConfiguration("use_rviz")

    # mock hardware — use_mock:=true. 실 CAN·homing 인자는 mock 이 무시하므로 넘기지 않는다.
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
        # 기본값은 aidin_hand2.launch.py 와 동일 — 왼손만.
        # mock 기본은 양손 — 실 하드웨어(aidin_hand2.launch.py)와 달리 없는 손을 열 위험이 없다.
        DeclareLaunchArgument("use_left_hand", default_value="true",
                              description="왼손 mock 기동."),
        DeclareLaunchArgument("use_right_hand", default_value="true",
                              description="오른손 mock 기동."),
        DeclareLaunchArgument("use_glove", default_value="false",
                              description="MANUS 글러브 텔레오퍼 controller spawn "
                                          "(manus_data_publisher 필요, 왼손 전용)."),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="rviz2 로 mock 손 시각화."),

        control_node,
        robot_state_publisher,

        # rviz/TF 용 표준 joint_state_broadcaster — robot-wide 싱글톤이라 한 번만.
        _spawn("joint_state_broadcaster"),

        # mock 은 broadcaster/actuator/impedance 불필요 — 손마다 joint_position 하나씩.
        _spawn("left_joint_position_controller", IfCondition(use_left_hand)),
        _spawn("right_joint_position_controller", IfCondition(use_right_hand)),

        # 글러브 텔레오퍼 (옵션) — joint_position_controller 의 reference 를 claim 하는 chain 최상위.
        # 파라미터 파일이 왼손만 정의하므로 이름을 고정한다(위 docstring).
        _spawn("left_glove_teleop_controller", IfCondition(use_glove), glove_yaml),

        Node(package="rviz2", executable="rviz2",
             arguments=["-d", rviz_config],
             condition=IfCondition(use_rviz), output="screen"),
    ])

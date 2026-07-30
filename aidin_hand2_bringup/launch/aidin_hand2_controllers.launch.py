# 이미 떠 있는 controller_manager 에 컨트롤러/broadcaster 를 spawn 하는 재사용 launch.
# robot_description·ros2_control_node·robot_state_publisher 같은 robot-wide 싱글톤은 상위
# launch(aidin_hand2.launch.py)가 소유한다. 합성 시 부모가 이 launch 를 include 하고
# controller_manager 인자만 자기 CM 으로 지정한다.
#
# 기본 기동: broadcaster 3종 + joint_position_controller 는 active, actuator_position/
# actuator_effort/joint_impedance 는 --inactive(로드만) — command mode 충돌을 피한다. 다른
# command mode 를 쓰려면 사용자가 switch_controllers 로 교체한다.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# spawner 가 controller_manager 를 기다리는 시간 (합성 시 부모 CM 이 늦게 떠도 견디게 넉넉히).
_SPAWNER_TIMEOUT = "30"


def _spawn(controller_name, *, condition, inactive=False):
    args = [
        controller_name,
        "--controller-manager", LaunchConfiguration("controller_manager"),
        "--controller-manager-timeout", _SPAWNER_TIMEOUT,
    ]
    if inactive:
        args.append("--inactive")
    return Node(
        package="controller_manager", executable="spawner",
        arguments=args, output="screen", condition=condition,
    )


def _hand_actions(prefix, condition):
    # command mode 컨트롤러(joint_position active → actuator_position/effort·joint_impedance inactive)는
    # 같은 command interface 를 claim 해 서로 경쟁하므로 OnProcessExit 로 순차 spawn 한다. broadcaster 는
    # state interface 만 읽어 경쟁이 없어 독립 병렬. joint_position 종료 후 inactive 3종을 로드한다.
    joint_position = _spawn(f"{prefix}_joint_position_controller", condition=condition)
    inactive_chain = RegisterEventHandler(OnProcessExit(
        target_action=joint_position,
        on_exit=[
            _spawn(f"{prefix}_actuator_position_controller", condition=condition, inactive=True),
            _spawn(f"{prefix}_actuator_effort_controller", condition=condition, inactive=True),
            _spawn(f"{prefix}_joint_impedance_controller", condition=condition, inactive=True),
        ],
    ))
    return [
        _spawn(f"{prefix}_hand_state_broadcaster", condition=condition),
        _spawn(f"{prefix}_diagnostics_broadcaster", condition=condition),
        joint_position, inactive_chain,
    ]


def generate_launch_description():
    use_left = IfCondition(LaunchConfiguration("use_left_hand"))
    use_right = IfCondition(LaunchConfiguration("use_right_hand"))

    return LaunchDescription([
        DeclareLaunchArgument("use_left_hand", default_value="false"),
        DeclareLaunchArgument("use_right_hand", default_value="true"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),

        # rviz/TF 용 표준 joint_state_broadcaster — robot-wide 싱글톤이라 한 번만.
        _spawn("joint_state_broadcaster", condition=IfCondition("true")),

        *_hand_actions("left", use_left),
        *_hand_actions("right", use_right),
    ])

# Spawns the controllers and broadcasters on a controller_manager that is already up
# robot_description, ros2_control_node and robot_state_publisher belong to the parent launch,
# which includes this one and points the controller_manager argument at its own
#
# The broadcasters and joint_position_controller come up active, the other three command
# controllers are loaded --inactive, and switch_controllers picks another mode

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Seconds the spawner waits for the controller_manager
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
    # The command controllers claim the same command interfaces, so they spawn in sequence on
    # OnProcessExit
    # The broadcasters only read state interfaces and spawn in parallel
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

        # One for the whole robot, spawned once
        _spawn("joint_state_broadcaster", condition=IfCondition("true")),

        *_hand_actions("left", use_left),
        *_hand_actions("right", use_right),
    ])

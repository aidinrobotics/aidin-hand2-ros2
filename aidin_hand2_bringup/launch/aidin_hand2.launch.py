"""AIDIN Hand Gen2 standalone control stack — 실제 하드웨어(can) 기동.

robot_description(xacro) → ros2_control_node(controller_manager) + robot_state_publisher 를
띄우고, aidin_hand2_controllers.launch.py 를 include 해 컨트롤러/broadcaster 를 spawn 한다.

기동 인자(손별 CAN·미가동 actuator·affinity·auto_home·auto reconnect)는 config YAML 하나로 모아둔다:
config/hand_bringup.yaml. 우선순위는 CLI(key:=value) > config YAML > 하드코딩 기본값 순이다.

  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py                       # config/hand_bringup.yaml 로 기동
  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py config:=my.yaml       # 다른 config 파일
  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py use_left_hand:=true   # 특정 인자만 override
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

# config YAML 에 담기는 기동 인자와 하드코딩 기본값(파일에 키가 없거나 config 자체가 없을 때 쓴다).
# 키 = launch 인자명. description 은 --show-args 와 문서용.
_ARG_DEFAULTS = {
    "use_left_hand": ("true", "왼손 기동 (can0)."),
    "use_right_hand": ("true", "오른손 기동 (can1)."),
    "auto_home": ("true", "기동 직후 자동 homing (실물이 움직인다)."),
    "left_hand_interface": ("can0", "왼손 CAN interface."),
    "right_hand_interface": ("can1", "오른손 CAN interface."),
    "left_hand_cpu_affinity": ("-1", "왼손 SDK RT loop CPU 코어 pin (-1 = 미설정)."),
    "right_hand_cpu_affinity": ("-1", "오른손 SDK RT loop CPU 코어 pin (-1 = 미설정)."),
    "left_hand_disabled_actuators": ("", "왼손 미가동 actuator index (콤마 구분, 예 '0,1,2,3'). 빈 값 = 전부 가동."),
    "right_hand_disabled_actuators": ("", "오른손 미가동 actuator index (콤마 구분). 빈 값 = 전부 가동."),
    "auto_reconnect": ("false", "통신 두절 시 auto reconnect (양손 공통)."),
    "auto_reconnect_timeout_ms": ("0", "auto reconnect timeout [ms], 0 = No limit."),
    "auto_reconnect_home": ("false", "auto reconnect 시 homing 수행."),
}


def _load_config(path):
    """config YAML 을 {인자명: "문자열값"} 으로 읽는다. 없으면 빈 dict. 값은 xacro 로 넘기려고
    모두 문자열화한다(bool 은 소문자 true/false — xacro if 와 CLI 표기 일치)."""
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    out = {}
    for key, value in raw.items():
        out[key] = "true" if value is True else "false" if value is False else str(value)
    return out


def _setup(context, *_):
    # config 경로는 launch 인자라 이 시점(OpaqueFunction)에 해석해 파일을 읽는다.
    config = _load_config(LaunchConfiguration("config").perform(context))

    # 인자별 기본값 = config 값(있으면) else 하드코딩. CLI 로 준 값은 launch 가 이 default 를 덮으므로
    # 최종 우선순위는 CLI > config > 하드코딩이 된다.
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
            description="기동 인자 config YAML 경로. 이 파일 값이 각 인자 기본값을 덮는다(CLI 가 다시 우선)."),
        OpaqueFunction(function=_setup),
    ])

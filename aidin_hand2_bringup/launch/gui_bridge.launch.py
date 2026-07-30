"""AIDIN Hand Gen2 GUI 연결용 rosbridge 기동 — 데스크톱 GUI(roslib.js)가 붙는 WebSocket 서버.

GUI 는 브라우저/Electron 이라 ROS2 에 직접 못 붙고 rosbridge_server 를 경유한다. 이 launch 는
rosbridge_websocket 을 port 9090(GUI 기본값)으로 띄운다. 제어 스택(bringup 또는 mock)과 별개
프로세스이므로 순서 무관하게 따로 실행한다 — bridge 는 이미 떠 있는 topic/service 를 그대로 노출한다.

  ros2 launch aidin_hand2_bringup aidin_hand2.launch.py       # 제어 스택(실 CAN)
  ros2 launch aidin_hand2_bringup gui_bridge.launch.py        # + 이 bridge 를 다른 터미널에서

GUI Settings 에서 소스=rosbridge, URL=ws://localhost:9090 으로 두면 연결된다. 포트를 바꾸려면
port:=<n> 인자를 주고 GUI URL 도 함께 맞춘다.
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
# aidin_hand2_bringup

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

`aidin_hand2_bringup` package는 로봇 핸드와 mock을 실행하는 launch 파일과 controller 설정을 제공합니다.
이 문서에서 launch 인자·기본값·설정 파일과 기존 로봇에 추가하는 순서를 확인할 수 있습니다.
첫 실행은 [Bringup](../docs/ko/04_bringup.md)을 따라 하십시오. `{side}`는 `left` 또는 `right`입니다.
URDF 시각화는 [aidin_hand2_description](../aidin_hand2_description/README.ko.md#3-descriptionlaunchpy)에서 제공합니다.

## Contents

&nbsp;&nbsp;[**1. Overview**](#1-overview)<br>
&nbsp;&nbsp;[**2. aidin_hand2.launch.py**](#2-aidin_hand2launchpy)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Arguments](#21-arguments)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Config file](#22-config-file)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Nodes and controllers](#23-nodes-and-controllers)<br>
&nbsp;&nbsp;[**3. aidin_hand2_mock.launch.py**](#3-aidin_hand2_mocklaunchpy)<br>
&nbsp;&nbsp;[**4. aidin_hand2_controllers.launch.py**](#4-aidin_hand2_controllerslaunchpy)<br>
&nbsp;&nbsp;[**5. gui_bridge.launch.py**](#5-gui_bridgelaunchpy)<br>
&nbsp;&nbsp;[**6. Controller config**](#6-controller-config)<br>
&nbsp;&nbsp;[**7. Add to your robot**](#7-add-to-your-robot)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.1 Launch the robot and controllers](#71-launch-the-robot-and-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.2 Verify](#72-verify)

## 1. Overview

파일은 [aidin_hand2_bringup/launch/](launch)에 있습니다.
로봇 핸드 실행, mock 실행, controller 실행, GUI 연결에 각각 사용합니다.

| Launch | Package | Purpose |
|---|---|---|
| `aidin_hand2.launch.py` | `aidin_hand2_bringup` | 로봇 핸드. hardware component와 controller 전체 |
| `aidin_hand2_mock.launch.py` | `aidin_hand2_bringup` | mock. CAN·drive 없이 controller 확인 |
| `aidin_hand2_controllers.launch.py` | `aidin_hand2_bringup` | 이미 뜬 controller_manager에 controller만 spawn |
| `gui_bridge.launch.py` | `aidin_hand2_bringup` | GUI용 rosbridge WebSocket |

## 2. aidin_hand2.launch.py

`aidin_hand2.launch.py`는 로봇 핸드를 단독으로 실행하는 launch입니다. `aidin_hand2.urdf.xacro`로
`robot_description`을 만들고 `ros2_control_node`, `robot_state_publisher`, controller spawner를 올립니다.
RViz는 포함하지 않습니다.

다음은 homing 없이 실행하는 예입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py auto_home:=false
```

### 2.1 Arguments

`config`를 뺀 인자 12개의 기본값은 `hand_bringup.yaml`에서 전달됩니다. `auto_home` parameter와
이름이 `auto_reconnect`로 시작하는 인자 셋은 양손 공통이고,
`*_hand_interface`·`*_hand_cpu_affinity`·`*_hand_disabled_actuators` parameter는 손별입니다.

| Argument | Default | Description |
|---|---|---|
| `use_left_hand` | `true` | 왼손 hardware component를 만듭니다 |
| `use_right_hand` | `true` | 오른손 hardware component를 만듭니다 |
| `left_hand_interface` | `can0` | 왼손 CAN interface. `auto`는 side의 CAN ID로 채널을 탐색합니다 |
| `right_hand_interface` | `can1` | 오른손 CAN interface. `auto`는 side의 CAN ID로 채널을 탐색합니다 |
| `left_hand_cpu_affinity` | `-1` | 왼손 제어·통신 루프를 고정할 CPU 번호. `-1`은 고정 안 함 |
| `right_hand_cpu_affinity` | `-1` | 오른손 제어·통신 루프를 고정할 CPU 번호. `-1`은 고정 안 함 |
| `left_hand_disabled_actuators` | `""` | 왼손에서 사용하지 않을 actuator index. `"0,1,2,3"`처럼 쉼표로 구분 |
| `right_hand_disabled_actuators` | `""` | 오른손에서 사용하지 않을 actuator index. `"0,1,2,3"`처럼 쉼표로 구분 |
| `auto_home` | `true` | `~/run` service 성공 뒤 `homing_state` 값이 `Succeeded`가 아니면 homing을 한 번 시작합니다. AIDIN Hand Gen2가 움직입니다 |
| `auto_reconnect` | `false` | 통신 오류가 발생하면 SDK가 재연결을 반복 시도합니다 |
| `auto_reconnect_timeout_ms` | `0` | 자동 재연결 제한 시간 [ms]. `0`은 제한 없음 |
| `auto_reconnect_home` | `false` | 자동 재연결 뒤 homing을 수행합니다 |
| `config` | `<share>/config/hand_bringup.yaml` | 위 인자의 기본값을 담은 YAML 경로 |

`control_rate`와 `max_effort`는 xacro 매크로 인자이지만 이 launch가 선언하지 않으므로 `key:=value`로 바꿀 수
없습니다. 바꾸려면 [2. Add the robot hand to the URDF](../aidin_hand2_description/README.ko.md#2-add-the-robot-hand-to-the-urdf)의 매크로를 자기 URDF에서 호출하고, runtime의 effort
상한은 [6. Runtime settings](../aidin_hand2_hardware/README.ko.md#6-runtime-settings)의 hardware node parameter로 조정합니다.

이 launch는 인자를 `OpaqueFunction` 안에서 선언하므로 `ros2 launch --show-args`가 `config` 인자만 표시합니다.
전체 인자는 위 표와 `hand_bringup.yaml`을 보십시오.

### 2.2 Config file

값은 CLI 인자가 config YAML을 덮고, config YAML이 launch 소스의 기본값을 덮습니다.

```text
CLI key:=value  ▸  config YAML  ▸  launch 소스의 기본값
```

`hand_bringup.yaml`은 ROS parameter 파일이 아니라 launch 인자 이름과 1:1인 최상위 `key: value`입니다.
배포되는 파일의 값은 [2.1 Arguments](#21-arguments) 표의 기본값과 같습니다. 컴퓨터마다 다른 값(왼손과 오른손이 어느
interface에 있는지)은 `hand_bringup.yaml` 대신 자기 config 파일에 적어 `config` 인자로 넘깁니다.

다음은 자기 config 파일로 실행하는 예입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  config:=/absolute/path/my-hand.yaml auto_home:=false
```

### 2.3 Nodes and controllers

controller는 `aidin_hand2_controllers.launch.py`를 include해 올리고, `controllers.yaml`이 controller_manager의
parameter 파일입니다. launch가 올리는 node와 controller는 다음과 같습니다.

```text
xacro robot_description (aidin_hand2.urdf.xacro)
├── controller_manager/ros2_control_node   (/controller_manager, controllers.yaml)
├── robot_state_publisher
└── aidin_hand2_controllers.launch.py
    ├── joint_state_broadcaster                  active
    ├── <side>_hand_state_broadcaster            active
    ├── <side>_diagnostics_broadcaster           active
    ├── <side>_joint_position_controller         active
    ├── <side>_actuator_position_controller      inactive
    ├── <side>_actuator_effort_controller        inactive
    └── <side>_joint_impedance_controller        inactive
```

## 3. aidin_hand2_mock.launch.py

`aidin_hand2_mock.launch.py`는 mock backend로 양손을 실행합니다. CAN·homing 인자가 없고
`controllers_mock.yaml`을 씁니다.

| Argument | Default | Description |
|---|---|---|
| `use_left_hand` | `true` | 왼손 mock을 만듭니다 |
| `use_right_hand` | `true` | 오른손 mock을 만듭니다 |
| `use_rviz` | `true` | RViz를 실행합니다. config는 `aidin_hand2_description/rviz/view_robot.rviz` |

다음은 RViz 없이 실행하는 예입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_rviz:=false
```

active로 올라오는 controller는 `joint_state_broadcaster`와 손별 `{side}_joint_position_controller`뿐입니다.
mock은 `/joint_states` topic만 발행하므로 `HandStateBroadcaster`와
`DiagnosticsBroadcaster`를 실행하지 않습니다. 나머지 command controller 셋은 `controllers_mock.yaml`에
등록되어 있어 필요할 때 load합니다.

## 4. aidin_hand2_controllers.launch.py

`aidin_hand2_controllers.launch.py`는 이미 뜬 controller_manager에 [2.3 Nodes and controllers](#23-nodes-and-controllers)의
controller를 spawn합니다. 자기 launch로 `ros2_control_node`를 실행하고 controller 이름을 `left_`·`right_`
관례로 둔 경우에 씁니다. 사용하려면 선택한 쪽의 command controller 4개와 broadcaster 2개,
`joint_state_broadcaster`를 모두 YAML에 선언해야 합니다.
[7. Add to your robot](#7-add-to-your-robot)의 최소 구성에서는 필요한 controller만 개별 spawner로 실행합니다.

| Argument | Default | Description |
|---|---|---|
| `use_left_hand` | `true` | 왼손 controller를 spawn합니다 |
| `use_right_hand` | `true` | 오른손 controller를 spawn합니다 |
| `controller_manager` | `/controller_manager` | 대상 controller_manager node 이름 |

기본값은 다른 launch 파일과 같이 양손입니다. 다음은 왼손 controller만 기본 controller_manager에 올리는
예입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_controllers.launch.py \
  use_left_hand:=true use_right_hand:=false controller_manager:=/controller_manager
```

joint position controller와 broadcaster는 `active`, 나머지 command controller는 `inactive` 상태로
실행됩니다. spawner가 controller_manager를 기다리는 시간은 30 s입니다.

## 5. gui_bridge.launch.py

`gui_bridge.launch.py`는 desktop GUI가 접속하는 `rosbridge_websocket`을 올립니다. 인자는 `port`
하나이고 기본값은 `9090`입니다. GUI 설정에서 `ws://<robot host>:9090`을 지정합니다.

> [!WARNING]
> rosbridge는 인증과 TLS를 제공하지 않고 모든 network interface에 bind합니다. 신뢰할 수 있는 격리
> network에서만 쓰십시오.

다음은 기본 port로 올리는 예입니다.

```bash
ros2 launch aidin_hand2_bringup gui_bridge.launch.py port:=9090
```

## 6. Controller config

설정 파일은 [aidin_hand2_bringup/config/](config)에 있습니다.
controller_manager parameter 파일은 backend마다 하나입니다. 둘 모두 `update_rate: 500`이고 매크로의
`control_rate` 기본값과 같습니다.

| File | Backend | Controllers | Broadcaster rate | Hardware node block |
|---|---|---|---|---|
| `controllers.yaml` | 로봇 핸드 | 양손 각 command 4 + broadcaster 2, `joint_state_broadcaster` | `joint_states` 100 Hz, `hand_state` 100 Hz, `hand_diagnostics` 20 Hz | `left_hand_control` · `right_hand_control` |
| `controllers_mock.yaml` | mock | 양손 각 command 4, `joint_state_broadcaster` | 지정 없음. `joint_states` 500 Hz | 없음 |

`controllers.yaml`의 hardware node block은 [6. Runtime settings](../aidin_hand2_hardware/README.ko.md#6-runtime-settings)의 6개 parameter를 기본값으로
채워 둔 것입니다. 자기 로봇에 통합할 때는 `controllers.yaml`을 복사하지 말고 [7. Configuration](../aidin_hand2_controllers/README.ko.md#7-configuration)의
최소 구성에서 시작하십시오.

## 7. Add to your robot

기존 로봇의 `robot_state_publisher`와 `ros2_control_node`에 왼손을 추가하는 예입니다.
먼저 [Bringup](../docs/ko/04_bringup.md)으로 로봇 핸드의 단독 동작을 확인하십시오.
아래 경로는 사용자의 로봇 package에 있는 파일의 예시입니다.

| Order | Your file (example) | Change |
|---|---|---|
| 1 | `my_robot_description/urdf/my_robot.urdf.xacro` | [모델·제어 매크로](../aidin_hand2_description/README.ko.md#2-add-the-robot-hand-to-the-urdf)를 호출합니다. 첫 실행은 `auto_home="false"`로 설정합니다 |
| 2 | `my_robot_bringup/config/controllers.yaml` | [controller YAML 예제](../aidin_hand2_controllers/README.ko.md#7-configuration)의 controller와 broadcaster를 선언합니다 |
| 3 | `my_robot_bringup/launch/my_robot.launch.py` | 아래 절처럼 수정한 URDF와 YAML을 전달하고 실행합니다 |

effort 상한·filter를 시작할 때 적용하려면 같은 YAML에
[초기 설정 블록](../aidin_hand2_hardware/README.ko.md#64-initial-runtime-settings)을 추가합니다.

### 7.1 Launch the robot and controllers

로봇의 launch 파일을 엽니다. 위 예제에서는 `my_robot_bringup/launch/my_robot.launch.py`입니다.
위 표의 1·2단계에서 수정한 URDF와 YAML을 읽도록 지정합니다.
`ros2_control_node`에는 두 설정을 함께 전달하고, `robot_state_publisher`에도 같은 URDF를 전달합니다.

다음은 기존 launch에서 `ros2_control_node`를 구성하는 부분입니다. `robot_description`은 전체 로봇
xacro를 처리한 문자열이고, `controllers_yaml`은 앞에서 수정한 YAML의 절대 경로입니다.
이미 있는 node 선언을 수정하며 두 번째 `ros2_control_node`를 추가하지 않습니다.

```python
from launch_ros.actions import Node

control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[{"robot_description": robot_description}, controllers_yaml],
    output="screen",
)
```

기존 로봇 launch를 실행한 뒤 다른 터미널에서 환경을 적용하고 연결을 확인합니다.
SDK를 사용자 경로에 설치했다면 launch 터미널에는 [3. Build the wrapper](../docs/ko/03_installation.md#3-build-the-wrapper)의
`LD_LIBRARY_PATH` 설정도 적용하십시오.

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
ros2 control list_hardware_components
```

`left_hand_control`이 `active` 상태이면 선언한 controller를 실행합니다.
기존 로봇 launch가 이미 활성화한 controller는 다시 spawn하지 않습니다.

```bash
ros2 run controller_manager spawner left_joint_position_controller \
  left_hand_state_broadcaster left_diagnostics_broadcaster \
  --controller-manager /controller_manager
```

`joint_state_broadcaster`가 아직 실행되지 않은 경우에만 추가로 실행합니다.

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager
```

### 7.2 Verify

실행한 controller와 로봇 핸드의 상태를 확인합니다.

```bash
ros2 control list_hardware_components     # left_hand_control이 active
ros2 control list_controllers             # 로봇 핸드마다 command controller 하나가 active
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

`hand_diagnostics` topic의 `lifecycle` 값이 `Running`이면 hardware component가 동작 중입니다. `~/home`
service로 homing을 마쳐 `homing_state` 값이 `Succeeded`가 되면 command를 받을 준비가 된 상태입니다.
homing과 첫 command는 [2.4 Home](../docs/ko/04_bringup.md#24-home)부터 따라 확인하십시오.
controller 선택과 전환은 [Controllers](../aidin_hand2_controllers/README.ko.md)에, homing과 복구 service는
[Services](../aidin_hand2_hardware/README.ko.md)에, topic은 [Topics](../aidin_hand2_msgs/README.ko.md)에 있습니다.

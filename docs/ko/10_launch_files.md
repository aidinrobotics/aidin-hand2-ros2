# Launch files

`aidin_hand2_bringup` package의 launch 파일은 5개이고 `aidin_hand2_description` package에 시각화용 하나가
더 있습니다. 이 문서는 각 launch의 용도, 인자, 올리는 node와 controller, 그리고 값을 주는 config 파일을
설명합니다. launch로 로봇 핸드를 확인하는 절차는 [Bringup](04_bringup.md)에 있습니다. `{side}`는 `left` 또는
`right`입니다.

## Contents

&nbsp;&nbsp;[**1. Overview**](#1-overview)<br>
&nbsp;&nbsp;[**2. aidin_hand2.launch.py**](#2-aidin_hand2launchpy)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Arguments](#21-arguments)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Config file](#22-config-file)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Nodes and controllers](#23-nodes-and-controllers)<br>
&nbsp;&nbsp;[**3. aidin_hand2_mock.launch.py**](#3-aidin_hand2_mocklaunchpy)<br>
&nbsp;&nbsp;[**4. aidin_hand2_isaac.launch.py**](#4-aidin_hand2_isaaclaunchpy)<br>
&nbsp;&nbsp;[**5. aidin_hand2_controllers.launch.py**](#5-aidin_hand2_controllerslaunchpy)<br>
&nbsp;&nbsp;[**6. gui_bridge.launch.py**](#6-gui_bridgelaunchpy)<br>
&nbsp;&nbsp;[**7. description.launch.py**](#7-descriptionlaunchpy)<br>
&nbsp;&nbsp;[**8. Controller config**](#8-controller-config)

## 1. Overview

launch 파일은 backend마다 하나, controller만 올리는 것 하나, GUI bridge 하나, 시각화 하나입니다.

| Launch | Package | Purpose |
|---|---|---|
| `aidin_hand2.launch.py` | `aidin_hand2_bringup` | 로봇 핸드. hardware component와 controller 전체 |
| `aidin_hand2_mock.launch.py` | `aidin_hand2_bringup` | mock. CAN·drive 없이 controller 확인 |
| `aidin_hand2_isaac.launch.py` | `aidin_hand2_bringup` | Isaac Sim. ROS 2 topic으로 연결 |
| `aidin_hand2_controllers.launch.py` | `aidin_hand2_bringup` | 이미 뜬 controller_manager에 controller만 spawn |
| `gui_bridge.launch.py` | `aidin_hand2_bringup` | GUI용 rosbridge WebSocket |
| `description.launch.py` | `aidin_hand2_description` | URDF 시각화. hardware 없음 |

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
`auto_reconnect`로 시작하는 parameter 셋은 양손 공통이고,
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

`control_rate`와 `max_effort`는 xacro 인자이지만 이 launch가 선언하지 않으므로 `key:=value`로 바꿀 수
없습니다. 바꾸려면 [Integration](05_integration.md)의 매크로를 자기 URDF에서 호출하고, runtime의 effort
상한은 [Parameters](09_parameters.md) 2장의 hardware node parameter로 조정합니다.

이 launch는 인자를 `OpaqueFunction` 안에서 선언하므로 `ros2 launch --show-args`가 `config`만 표시합니다.
전체 인자는 위 표와 `hand_bringup.yaml`을 보십시오.

### 2.2 Config file

값은 CLI 인자가 config YAML을 덮고, config YAML이 launch 소스의 기본값을 덮습니다.

```text
CLI key:=value  ▸  config YAML  ▸  launch 소스의 기본값
```

`hand_bringup.yaml`은 ROS parameter 파일이 아니라 launch 인자 이름과 1:1인 최상위 `key: value`입니다.
배포되는 파일의 값은 [2.1](#21-arguments) 표의 기본값과 같습니다. 컴퓨터마다 다른 값(왼손과 오른손이 어느
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
mock에는 tactile·diagnostics·command echo state interface가 없어 `HandStateBroadcaster`와
`DiagnosticsBroadcaster`를 올리지 않습니다. 나머지 command controller 셋은 `controllers_mock.yaml`에
등록되어 있어 필요할 때 load합니다.

## 4. aidin_hand2_isaac.launch.py

`aidin_hand2_isaac.launch.py`는 isaac backend로 왼손을 실행합니다. `controllers_isaac.yaml`을 쓰고
topic 인자는 매크로의 `isaac_*` parameter로 전달됩니다.

| Argument | Default | Description |
|---|---|---|
| `topic_prefix` | `/isaac` | Isaac topic 접두어 |
| `joint_state_topic` | `joint_states` | Isaac → wrapper state topic. 접두어 뒤에 붙습니다 |
| `joint_command_topic` | `hand_command` | wrapper → Isaac command topic. 접두어 뒤에 붙습니다 |
| `tactile_prefix` | `tactile` | tactile topic 접두어. 접두어 뒤에 붙습니다 |
| `use_rviz` | `true` | RViz를 실행합니다 |

다음은 기본 topic 이름으로 실행하는 예입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py
```

`joint_state_broadcaster`, `left_hand_state_broadcaster`, `left_diagnostics_broadcaster`,
`left_joint_position_controller`가 active로 올라옵니다. topic의 message와 방향은 [Topics](08_topics.md)
5장에 있습니다.

## 5. aidin_hand2_controllers.launch.py

`aidin_hand2_controllers.launch.py`는 이미 뜬 controller_manager에 [2.3](#23-nodes-and-controllers)의
controller를 spawn합니다. 자기 launch로 `ros2_control_node`를 올리고 controller 이름을 `left_`·`right_`
관례로 둔 경우에 씁니다.

| Argument | Default | Description |
|---|---|---|
| `use_left_hand` | `false` | 왼손 controller를 spawn합니다 |
| `use_right_hand` | `true` | 오른손 controller를 spawn합니다 |
| `controller_manager` | `/controller_manager` | 대상 controller_manager node 이름 |

> [!IMPORTANT]
> `use_left_hand`·`use_right_hand` parameter의 기본값이 `aidin_hand2.launch.py`와 반대입니다. 두 인자를
> 항상 명시하십시오.

다음은 왼손 controller만 기본 controller_manager에 올리는 예입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_controllers.launch.py \
  use_left_hand:=true use_right_hand:=false controller_manager:=/controller_manager
```

active로 올릴 `{side}_joint_position_controller`가 `command_lock`을 claim하므로 spawner를 먼저 실행하고,
나머지 command controller 셋은 joint position controller의 spawner가 끝난 뒤 `--inactive`로 함께
실행됩니다. broadcaster는 state만
읽으므로 처음부터 병렬로 실행됩니다. spawner의 controller_manager 대기 시간은 30 s입니다.

## 6. gui_bridge.launch.py

`gui_bridge.launch.py`는 desktop GUI가 접속하는 `rosbridge_websocket`을 올립니다. 인자는 `port`
하나이고 기본값은 `9090`입니다. GUI 설정에서 `ws://<robot host>:9090`을 지정합니다.

> [!WARNING]
> rosbridge는 인증과 TLS를 제공하지 않고 모든 network interface에 bind합니다. 신뢰할 수 있는 격리
> network에서만 쓰십시오.

다음은 기본 port로 올리는 예입니다.

```bash
ros2 launch aidin_hand2_bringup gui_bridge.launch.py port:=9090
```

## 7. description.launch.py

`description.launch.py`는 hardware 없이 URDF만 RViz에 표시합니다. `use_mock:=true`로 description을 만들고
`robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`를 올립니다.

| Argument | Default | Description |
|---|---|---|
| `use_left_hand` | `true` | 왼손을 표시합니다 |
| `use_right_hand` | `false` | 오른손을 표시합니다 |
| `use_gui` | `true` | `joint_state_publisher_gui`의 slider로 joint를 움직입니다 |

다음은 양손을 표시하는 예입니다.

```bash
ros2 launch aidin_hand2_description description.launch.py use_right_hand:=true
```

## 8. Controller config

controller_manager parameter 파일은 backend마다 하나입니다. 셋 모두 `update_rate: 500`이고 매크로의
`control_rate` 기본값과 같습니다.

| File | Backend | Controllers | Broadcaster rate | Hardware node block |
|---|---|---|---|---|
| `controllers.yaml` | 로봇 핸드 | 양손 각 command 4 + broadcaster 2, `joint_state_broadcaster` | `joint_states` 100 Hz, `hand_state` 100 Hz, `hand_diagnostics` 20 Hz | `left_hand_control` · `right_hand_control` |
| `controllers_mock.yaml` | mock | 양손 각 command 4, `joint_state_broadcaster` | 지정 없음. `joint_states` 500 Hz | 없음 |
| `controllers_isaac.yaml` | isaac | 왼손 command 4 + broadcaster 2, `joint_state_broadcaster` | `joint_states` 100 Hz, `hand_state` 100 Hz, `hand_diagnostics` 20 Hz | 없음 |

`controllers.yaml`의 hardware node block은 [Parameters](09_parameters.md) 2장의 6개 parameter를 기본값으로
채워 둔 것입니다. 자기 로봇에 통합할 때는 `controllers.yaml`을 복사하지 말고 [Integration](05_integration.md) 3장의
최소 구성에서 시작하십시오.

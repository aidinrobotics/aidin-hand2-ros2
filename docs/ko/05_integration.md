# Integration

로봇 핸드를 자기 로봇(팔·이동로봇)의 URDF와 controller 구성에 넣는 절차입니다. `aidin_hand2_bringup`
package의 launch는 단독 확인용이므로 통합에서는 쓰지 않고, xacro 매크로를 자기 URDF에서 직접
호출합니다. 먼저 [Bringup](04_bringup.md)으로 하드웨어를 확인하십시오.

## Contents

&nbsp;&nbsp;[**1. Macro parameters**](#1-macro-parameters)<br>
&nbsp;&nbsp;[**2. Add the macros to the URDF**](#2-add-the-macros-to-the-urdf)<br>
&nbsp;&nbsp;[**3. Declare the controllers**](#3-declare-the-controllers)<br>
&nbsp;&nbsp;[**4. Verify**](#4-verify)

## 1. Macro parameters

`ros2_control` 매크로의 parameter는 넷이 필수이고 나머지는 기본값이 있습니다. 전체 목록과 기본값은
[Parameters](09_parameters.md) 1장에 있습니다.

| Parameter | Description |
|---|---|
| `name` | hardware component 이름. service node 이름과 parameter 블록 이름으로도 쓰입니다 |
| `prefix` | joint·actuator·interface 이름 접두어. `left_` 또는 `right_` |
| `hand_side` | `left` 또는 `right` |
| `can_interface` | CAN interface 이름. `auto`는 side의 CAN ID로 채널을 탐색합니다 |

통합할 때 기본값에서 결정하는 parameter는 넷입니다. backend는 `use_isaac` > `use_mock` > CAN 순서로
결정되고 기본값은 로봇 핸드이며, 셋의 차이는 [1.1 Backends](06_controllers.md#11-backends)에 있습니다.

| Parameter | Default | Description |
|---|---|---|
| `auto_home` | `true` | `~/run` service 성공 뒤 `homing_state` 값이 `Succeeded`가 아니면 homing을 한 번 시작할지 여부 |
| `control_rate` | `500` | 제어·통신 루프 주기 [Hz] |
| `disabled_actuators` | `''` | 사용하지 않을 actuator index. `"0,1,2,3"`처럼 쉼표로 구분 |
| `auto_reconnect` | `false` | 통신 오류가 발생하면 SDK가 재연결을 반복 시도할지 여부 |

> [!IMPORTANT]
> `control_rate` 값과 controller_manager의 `update_rate` 값을 같게 둡니다. 두 루프의 주기가 다르면
> command가 계단처럼 끊겨 전달되어 진동합니다. 기본값은 둘 다 `500`입니다.

> [!WARNING]
> `auto_home=true`이면 hardware component가 activate된 직후 AIDIN Hand Gen2가 hard stop까지
> 움직입니다. 첫 통합에서는 `false`로 두고, 주변을 확인한 뒤 `~/home` service를 호출하십시오.

## 2. Add the macros to the URDF

매크로는 둘입니다. geometry 매크로는 링크·joint·mesh를 넣고 side마다 파일이 다릅니다. `ros2_control`
매크로는 hardware component를 선언하고 side를 `hand_side` parameter로 받습니다.

| Macro | File | Required |
|---|---|---|
| `aidin_hand2_left` · `aidin_hand2_right` | `urdf/aidin_hand2_left.urdf.xacro` · `urdf/aidin_hand2_right.urdf.xacro` | `<origin>` block. `prefix`(기본 `left_`·`right_`)와 `parent`(기본 `world`)는 생략할 수 있습니다 |
| `aidin_hand2_ros2_control` | `ros2_control/aidin_hand2.ros2_control.xacro` | `name` `prefix` `hand_side` `can_interface` |

다음은 왼손을 `my_arm_tool0` 링크에 붙이는 예입니다.

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <xacro:include filename="$(find aidin_hand2_description)/urdf/aidin_hand2_left.urdf.xacro"/>
  <xacro:include filename="$(find aidin_hand2_description)/ros2_control/aidin_hand2.ros2_control.xacro"/>

  <xacro:aidin_hand2_left prefix="left_" parent="my_arm_tool0">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:aidin_hand2_left>

  <xacro:aidin_hand2_ros2_control
    name="left_hand_control" prefix="left_" hand_side="left"
    can_interface="can0" auto_home="false"/>

</robot>
```

- `parent`는 자기 URDF에 이미 있는 링크입니다.
- `prefix`는 두 매크로에 같은 값을 주고, joint·actuator·interface 이름 앞에 그대로 붙습니다.
  `aidin_hand2_controllers` package의 controller와 launch가 `left_`·`right_`를 가정하므로 두 접두어를
  권장합니다.
- `name`은 hardware component 이름이고, service node 이름(`/left_hand_control/run`)과
  `controllers.yaml`의 hardware node parameter 블록 이름으로도 쓰입니다.
- 양손이면 두 매크로 쌍을 `prefix`·`name`·`hand_side`·`can_interface` parameter만 바꿔 두 번
  호출합니다.

## 3. Declare the controllers

controller는 자기 `controllers.yaml`에 선언합니다. 로봇 핸드 하나에 필요한 최소 구성은 command controller
하나, wrapper의 broadcaster 둘, 표준 `joint_state_broadcaster` 하나입니다.

| Controller | Type | Role |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 로봇의 모든 joint 각도를 `/joint_states` topic으로 발행합니다. 로봇에 하나 |
| `left_joint_position_controller` | `aidin_hand2_controllers/JointPositionController` | active joint 16개의 목표 각도[rad]를 받습니다 |
| `left_hand_state_broadcaster` | `aidin_hand2_controllers/HandStateBroadcaster` | joint·actuator·tactile 관측과 적용된 command를 `~/hand_state` topic으로 발행합니다 |
| `left_diagnostics_broadcaster` | `aidin_hand2_controllers/DiagnosticsBroadcaster` | lifecycle, homing, 제어·통신 루프 통계, actuator fault를 `~/hand_diagnostics` topic으로 발행합니다 |

다른 command controller 셋(`JointImpedanceController`, `ActuatorPositionController`,
`ActuatorEffortController`)은 필요할 때 같은 방식으로 추가합니다. command controller는 한 순간 하나만
active일 수 있으며, 종류와 전환은 [Controllers](06_controllers.md)에 있습니다.

다음은 왼손의 최소 구성입니다.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500                    # 매크로의 control_rate와 같게

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    left_joint_position_controller:
      type: aidin_hand2_controllers/JointPositionController
    left_hand_state_broadcaster:
      type: aidin_hand2_controllers/HandStateBroadcaster
    left_diagnostics_broadcaster:
      type: aidin_hand2_controllers/DiagnosticsBroadcaster

joint_state_broadcaster:
  ros__parameters:
    update_rate: 100                    # 발행 주기. 없으면 500 Hz

left_joint_position_controller:
  ros__parameters:
    hand_side: left                     # prefix가 아니라 side 값

left_hand_state_broadcaster:
  ros__parameters:
    hand_side: left
    update_rate: 100

left_diagnostics_broadcaster:
  ros__parameters:
    hand_side: left
    update_rate: 20
```

hardware node parameter의 초깃값을 launch 때 주려면 `name`과 같은 이름의 블록을 둡니다. 생략하면 SDK
기본값이 적용됩니다. 전체 목록과 유효 범위는
[2. Hardware node parameters](09_parameters.md#2-hardware-node-parameters)에 있습니다.

```yaml
left_hand_control:
  ros__parameters:
    max_effort: [1000.0]                # 1개면 actuator 16개에 공통
    joint_position_controller:
      filter_enabled: true
      cutoff_freq: 10.0                 # Hz
      deadband: 0.000873                # rad
```

선언한 controller는 spawner로 올립니다. 이름을 `left_`·`right_` 관례로 두었다면
`aidin_hand2_controllers.launch.py`가 위 표의 넷과 나머지 command controller 셋(inactive)을 이미 뜬
controller_manager에 올립니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_controllers.launch.py \
  use_left_hand:=true use_right_hand:=false controller_manager:=/controller_manager
```

## 4. Verify

자기 launch로 로봇을 올린 뒤 확인합니다.

```bash
ros2 control list_hardware_components     # left_hand_control이 active
ros2 control list_controllers             # command controller가 하나만 active
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

`hand_diagnostics` topic의 `lifecycle` 값이 `Running`이면 hardware component가 동작 중입니다. `~/home`
service로 homing을 마쳐 `homing_state` 값이 `Succeeded`가 되면 command를 받을 준비가 된 상태입니다.
controller와 command 경로는 [Controllers](06_controllers.md)에, homing과 복구 service는
[Services](07_services.md)에, topic은 [Topics](08_topics.md)에 있습니다.

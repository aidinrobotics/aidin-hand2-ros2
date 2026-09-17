# aidin_hand2_controllers

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

원하는 제어 방식에 맞는 controller를 선택하고 목표값을 보내는 방법을 설명합니다. 처음 실행한다면
[Bringup](../docs/ko/04_bringup.md)에서 joint position command로 동작을 확인하십시오.
controller 설정은 [7. Configuration](#7-configuration), message 필드는 [Topics](../aidin_hand2_msgs/README.ko.md)에 있습니다.

`{side}`는 `left` 또는 `right`입니다. `~/cmd`의 `~`는 controller 이름을 나타냅니다.
왼손 joint position controller의 입력 topic은 `/left_joint_position_controller/cmd`입니다.

## Contents

&nbsp;&nbsp;[**1. Choose a controller**](#1-choose-a-controller)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Command controllers](#11-command-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Broadcasters](#12-broadcasters)<br>
&nbsp;&nbsp;[**2. Send a command**](#2-send-a-command)<br>
&nbsp;&nbsp;[**3. Switch controllers**](#3-switch-controllers)<br>
&nbsp;&nbsp;[**4. Command behavior**](#4-command-behavior)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 When commands are applied](#41-when-commands-are-applied)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Target values](#42-target-values)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.3 Command lifetime](#43-command-lifetime)<br>
&nbsp;&nbsp;[**5. Mock behavior**](#5-mock-behavior)<br>
&nbsp;&nbsp;[**6. Chaining**](#6-chaining)<br>
&nbsp;&nbsp;[**7. Configuration**](#7-configuration)

## 1. Choose a controller

command controller는 목표값을 받아 SDK에 전달하고, broadcaster는 관측값과 진단 정보를 topic으로
발행합니다. 로봇 핸드마다 command controller 하나를 활성화하며 broadcaster는 함께 사용할 수 있습니다.

### 1.1 Command controllers

일반적인 joint 각도 제어에는 `JointPositionController`를 사용합니다. 기본 launch도 이 controller를
활성화합니다. 각 controller의 `hand_side` parameter는 `left` 또는 `right`로 지정합니다.

| Controller | Input | Behavior |
|---|---|---|
| `aidin_hand2_controllers/JointPositionController` | active joint 16개의 목표 각도 [rad] | SDK가 목표 각도로 이동하도록 actuator 위치를 제어합니다 |
| `aidin_hand2_controllers/JointImpedanceController` | active joint 16개의 목표 각도 [rad] | SDK가 위치 오차와 gain으로 actuator effort를 계산합니다. 사용 보류 |
| `aidin_hand2_controllers/ActuatorPositionController` | actuator 16개의 목표 위치 [encoder count] | actuator 위치를 직접 지정합니다 |
| `aidin_hand2_controllers/ActuatorEffortController` | actuator 16개의 목표 effort [정격 전류의 0.1%] | actuator effort를 직접 지정합니다. 부호는 방향입니다 |

joint 계열 controller는 목표를 finger별 도달 범위 안으로 보정합니다. joint position의 filter와
joint impedance의 gain은 [6. Runtime settings](../aidin_hand2_hardware/README.ko.md#6-runtime-settings)에서
설정합니다.

> [!IMPORTANT]
> actuator 계열 controller에는 도달 범위 보정이 적용되지 않습니다. actuator position의 목표는
> `hand_state.actuator_position`에서 읽은 현재 값 근처에서 시작하십시오. effort의 단위와 상한은
> [2. Command message](../aidin_hand2_msgs/README.ko.md#2-command-message)에 있습니다.

> [!NOTE]
> joint impedance controller는 SDK에서 개발 중이므로 사용하지 마십시오. 이후 버전에서 동작이 바뀔 수 있습니다.

### 1.2 Broadcasters

상태는 다음 topic으로 확인합니다. 표준 `joint_state_broadcaster`는 전체 로봇에 하나만 두고,
나머지 broadcaster는 로봇 핸드마다 둡니다.

| Controller | Topic | Contents |
|---|---|---|
| `joint_state_broadcaster/JointStateBroadcaster` | `/joint_states` | passive joint를 포함한 joint 각도 |
| `aidin_hand2_controllers/HandStateBroadcaster` | `~/hand_state` | joint·actuator·촉각 관측값과 적용된 command |
| `aidin_hand2_controllers/DiagnosticsBroadcaster` | `~/hand_diagnostics` | lifecycle, homing, actuator fault, 제어·통신 루프 통계 |

wrapper의 broadcaster에는 `hand_side`를 지정합니다. 발행 주기는 `update_rate`로 조정하며 기본 launch는
`hand_state`를 100 Hz, `hand_diagnostics`를 20 Hz로 발행합니다. message 필드와 읽는 방법은
[4. Read state](../docs/ko/05_control_guide.md#4-read-state)에 있습니다.

## 2. Send a command

외부 node에서 사용할 때는 활성화된 controller의 `~/cmd` topic에 `sensor_msgs/JointState`를 보냅니다.
`name` 필드로 축을 지정하고 순서는 자유입니다. 이름과 controller가 읽는 필드는
[2. Command message](../aidin_hand2_msgs/README.ko.md#2-command-message)에 있습니다.

| Controller name | Field read |
|---|---|
| `{side}_joint_position_controller` | `position` [rad] |
| `{side}_joint_impedance_controller` | `position` [rad] |
| `{side}_actuator_position_controller` | `position` [encoder count] |
| `{side}_actuator_effort_controller` | `effort` [정격 전류의 0.1%] |

로봇 핸드에서는 controller가 `active`이고, `lifecycle`이 `Running`이며, `homing_state`가
`Succeeded`인지 확인한 뒤 전송합니다. mock에서는 homing 없이 보낼 수 있습니다.
처음 보낼 때와 controller를 바꾼 뒤에는 16개 이름을 모두 담아 보내십시오.

실제 topic 전송 명령은 [3. Send a command](../docs/ko/05_control_guide.md#3-send-a-command)에,
service 호출과 상태 확인은 같은 문서의 [Prepare the robot hand](../docs/ko/05_control_guide.md#2-prepare-the-robot-hand)에 있습니다.
상위 controller에서 연결하는 방법은 [6. Chaining](#6-chaining)에 있습니다.

## 3. Switch controllers

제어 방식을 바꾸려면 현재 command controller를 비활성화하고 대상 controller를 활성화합니다.
로봇 핸드마다 하나만 활성화할 수 있습니다. broadcaster는 그대로 둡니다.

현재 상태를 확인합니다.

```bash
ros2 control list_controllers
```

대상 controller가 목록에 없을 때만 `inactive`로 load합니다. 다음은 actuator position을 선택하는 예입니다.
기본 로봇 핸드 launch에서는 이미 load되어 있으므로 이 명령을 건너뜁니다.

```bash
ros2 control load_controller --set-state inactive left_actuator_position_controller
```

joint position이 `active`이고 actuator position이 `inactive`이면 두 전환을 함께 요청합니다.
`--strict`를 사용해 요청한 전환 중 일부만 성공한 상태를 성공으로 처리하지 않도록 합니다.

```bash
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_controller \
  --activate left_actuator_position_controller
```

다시 `ros2 control list_controllers`를 실행해 joint position이 `inactive`, actuator position이
`active`인지 확인합니다. 전환만으로 새 목표가 생기지는 않습니다. 새 controller에 목표를 보내기
전까지 SDK는 마지막 command를 유지합니다. 이전 mode가 effort였다면 effort도 계속 유지될 수 있습니다.

모든 command controller를 비활성화하면, 제어가 실행 중이고 homing을 마친 로봇 핸드는 `Idle`로 전환합니다.
`Idle`은 drive를 활성화한 채 effort `0`을 전송하므로 정지 service를 대신하지 않습니다.
토크를 제거하려면 [2. run and stop](../aidin_hand2_hardware/README.ko.md#2-run-and-stop)의 `~/stop`을 호출하십시오.

## 4. Command behavior

### 4.1 When commands are applied

로봇 핸드에 command를 적용하려면 다음 조건을 만족해야 합니다.

| Check | Required value |
|---|---|
| 대상 command controller | `active` |
| `hand_diagnostics.lifecycle` | `Running` |
| `hand_diagnostics.homing_state` | `Succeeded` |
| 입력 경로 | 일반 모드는 topic, chained mode는 상위 controller의 reference |

정지·복구·homing 중에 보낸 command는 나중 실행을 위해 대기하지 않습니다. 제어를 재개하고 homing이
완료된 뒤 필요한 목표를 다시 보내십시오. service 호출 순서는 [Services](../aidin_hand2_hardware/README.ko.md)에 있습니다.

publish 성공만으로 command 적용 여부를 알 수 없습니다. `hand_state.command_state`에서 입력 mode와
목표값, `selected_source`를 함께 확인합니다. `selected_source`가 `1`이면 SDK가 controller 출력을
선택했다는 뜻이며, 새 message의 수신 확인이나 실제 목표 도달을 보장하지는 않습니다.
관측값은 [3.1 HandState](../aidin_hand2_msgs/README.ko.md#31-handstate), command 표시는
[3.2 CommandState](../aidin_hand2_msgs/README.ko.md#32-commandstate)에 있습니다.

### 4.2 Target values

길이 16인 배열은 항상 필요합니다. 일부 축을 갱신하지 않을 때는 해당 위치에 NaN을 넣을 수 있습니다.

| Input | Result |
|---|---|
| 16개 모두 유한한 값 | 새 목표를 적용합니다 |
| 일부 NaN | 해당 축은 이전 목표값을 사용합니다 |
| 일부 NaN이고 해당 축의 이전 목표가 없음 | message 전체를 적용하지 않고 warning을 남깁니다 |
| 16개 모두 NaN | 새 목표를 적용하지 않고 마지막 command를 유지합니다 |
| Inf 포함 | warning을 남기고 해당 축을 NaN과 같이 처리합니다 |

joint 목표는 finger별 도달 범위로 보정됩니다. actuator position 목표가 `int32` 범위를 벗어나면
command가 거부되고 `hand_diagnostics.nan_command_count`가 증가합니다. effort 목표는 actuator별
`max_effort` 범위로 제한됩니다. 이 처리는 충돌 검사나 속도 제한을 제공하지 않습니다.

### 4.3 Command lifetime

새 목표를 한 번 적용하면 다음 목표를 받을 때까지 SDK가 마지막 command를 유지합니다.
같은 목표를 유지하기 위해 계속 publish할 필요는 없습니다.

> [!IMPORTANT]
> wrapper와 SDK에는 command의 수명 감시가 없습니다. publisher가 종료되거나 topic이 끊겨도
> 자동으로 정지하지 않습니다. 정지가 필요하면 `~/stop` service를 호출하십시오.

## 5. Mock behavior

mock에서는 로봇 핸드와 같은 command topic으로 입력을 보내고 `/joint_states`로 결과를 확인합니다.
CAN 연결과 homing은 필요하지 않습니다.

| Feature | Mock behavior |
|---|---|
| joint position·joint impedance | 도달 범위 보정과 kinematics 계산 결과를 즉시 반영합니다 |
| actuator position | kinematics로 계산한 joint 각도를 반영합니다 |
| actuator effort | 자세를 바꾸지 않습니다 |
| 관측 topic | `/joint_states`를 제공합니다. `hand_state`·`hand_diagnostics`는 없습니다 |
| service | run·stop·home·reconnect를 제공하지 않습니다 |

mock은 controller 연결과 목표값 전달을 확인하는 용도입니다. 로봇 핸드의 운동·힘 응답을 재현하지 않습니다.
실행 절차는 [1. Mock](../docs/ko/04_bringup.md#1-mock)에 있습니다.

## 6. Chaining

상위 controller를 작성하면 command controller의 reference interface에 목표값을 전달할 수 있습니다.
사용자 node가 상위 controller의 입력 topic에 목표값을 보내는 경로도 가능합니다.
입력 topic과 message 타입은 상위 controller가 정의합니다. 제공된 skeleton에는 목표 입력 subscriber가
없으므로 이 경로를 사용하려면 직접 구현해야 합니다.
chained mode에서는 command controller가 `~/cmd` topic 대신 상위 controller의 입력을 사용합니다.

연결할 reference 이름과 구현·실행 예제는
[Chainable controller examples](../aidin_hand2_examples/EXAMPLE.md)에 있습니다.
mode를 바꿀 때는 상위 controller를 먼저 비활성화하고 하위 command controller를 전환한 뒤
대상 mode의 상위 controller를 활성화합니다.

## 7. Configuration

로봇의 controller 설정 파일(예: `my_robot_bringup/config/controllers.yaml`)을 엽니다. 아래 예제는 joint position 제어와
상태·진단 확인에 필요한 controller를 선언합니다. 기존 로봇에 `joint_state_broadcaster`가 있다면
기존 선언을 사용하고 중복으로 추가하지 않습니다.

| Controller | Type | Role |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 로봇의 모든 joint 각도를 `/joint_states` topic으로 발행합니다. 로봇에 하나 |
| `left_joint_position_controller` | `aidin_hand2_controllers/JointPositionController` | active joint 16개의 목표 각도[rad]를 받습니다 |
| `left_hand_state_broadcaster` | `aidin_hand2_controllers/HandStateBroadcaster` | joint·actuator·tactile 관측과 적용된 command를 `~/hand_state` topic으로 발행합니다 |
| `left_diagnostics_broadcaster` | `aidin_hand2_controllers/DiagnosticsBroadcaster` | lifecycle, homing, 제어·통신 루프 통계, actuator fault를 `~/hand_diagnostics` topic으로 발행합니다 |

다른 command controller는 필요할 때 추가합니다. 로봇 핸드마다 command controller 하나만
활성화할 수 있습니다. 종류와 지원 상태는 [1. Choose a controller](#1-choose-a-controller),
전환 방법은 [3. Switch controllers](#3-switch-controllers)에서 확인하십시오.

기존 `controllers.yaml`에 다음 선언과 설정을 합칩니다.

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

controller parameter는 `controllers.yaml`의 controller 이름 블록에 둡니다.

| Controller | Parameter | Type | Default | Description |
|---|---|---|---|---|
| command controller 4종 | `hand_side` | `string` | 필수 | `left` 또는 `right`. 제어하거나 관측할 로봇 핸드를 선택합니다 |
| `HandStateBroadcaster` | `hand_side` | `string` | 필수 | `left` 또는 `right`. 제어하거나 관측할 로봇 핸드를 선택합니다 |
| `HandStateBroadcaster` | `update_rate` | `int` | controller_manager 주기 | 발행 주기 [Hz]. `controllers.yaml`은 `100` |
| `DiagnosticsBroadcaster` | `hand_side` | `string` | 필수 | `left` 또는 `right`. 제어하거나 관측할 로봇 핸드를 선택합니다 |
| `DiagnosticsBroadcaster` | `update_rate` | `int` | controller_manager 주기 | 발행 주기 [Hz]. `controllers.yaml`은 `20` |
| `joint_state_broadcaster` | `update_rate` | `int` | controller_manager 주기 | 발행 주기 [Hz]. `controllers.yaml`은 `100` |

`hand_side` parameter는 `prefix`가 아니라 side 값입니다. `left_`를 주면 controller가 configure에서
실패합니다.

broadcaster의 `update_rate` parameter를 지정하지 않으면 controller_manager 주기(500 Hz)로 발행됩니다.
배포된 `controllers.yaml`은 `hand_state`를 100 Hz, `hand_diagnostics`를 20 Hz로 설정합니다.

이 YAML을 launch에 전달하고 controller를 실행하는 순서는
[7. Add to your robot](../aidin_hand2_bringup/README.ko.md#7-add-to-your-robot)에 있습니다.

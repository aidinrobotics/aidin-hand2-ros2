# Controllers

wrapper의 controller는 `ros2_control` controller 6개이고, command controller 4개와 broadcaster 2개입니다.
이 문서는 controller가 붙는 hardware component와 hardware component의 lifecycle, controller가 claim하는
interface, command가 로봇 핸드에 전달되는 조건을 설명합니다. topic, service, parameter는 각각 [Topics](08_topics.md),
[Services](07_services.md), [Parameters](09_parameters.md)에 있습니다.

`{side}`는 `left` 또는 `right`이고, `{joint}`·`{actuator}`는 [2. Joint and actuator order](#2-joint-and-actuator-order)의
이름입니다.

## Contents

&nbsp;&nbsp;[**1. Hardware component**](#1-hardware-component)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Backends](#11-backends)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Lifecycle](#12-lifecycle)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Command and state interfaces](#13-command-and-state-interfaces)<br>
&nbsp;&nbsp;[**2. Joint and actuator order**](#2-joint-and-actuator-order)<br>
&nbsp;&nbsp;[**3. Controllers**](#3-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 Command controllers](#31-command-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 Broadcasters](#32-broadcasters)<br>
&nbsp;&nbsp;[**4. Command path**](#4-command-path)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Topic](#41-topic)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Reference interface](#42-reference-interface)<br>
&nbsp;&nbsp;[**5. Mode switching**](#5-mode-switching)<br>
&nbsp;&nbsp;[**6. Command validity**](#6-command-validity)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.1 NaN and complete set](#61-nan-and-complete-set)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.2 Suppressed cycles](#62-suppressed-cycles)

## 1. Hardware component

hardware component는 `aidin_hand2_hardware` package의 `SystemInterface` plugin이고, URDF의 `ros2_control`
매크로 호출 하나가 component 하나입니다. 이름은 매크로의 `name` parameter(`left_hand_control`)이고, 로봇
핸드 하나의 SDK `Hand`를 소유합니다. controller_manager의 루프가 매 cycle `read()`와 `write()`를 호출하고,
SDK의 제어·통신 루프는 component 안에서 따로 돕니다.

두 루프의 관계는 다음과 같습니다.

```text
controller_manager 루프 (update_rate 500 Hz)
  read() → controller update → write()
                │ hardware command interface
                ▼
SDK 제어·통신 루프 (control_rate 500 Hz, SCHED_FIFO 90)
  CAN RX → FK → controller → IK → CAN TX
```

### 1.1 Backends

backend는 셋이고 매크로의 `use_mock`·`use_isaac` parameter로 고릅니다. command interface는 셋이 같아
controller가 그대로 붙습니다.

| Plugin | Selected by | Talks to | State interfaces | Services |
|---|---|---|---|---|
| `aidin_hand2_hardware/AidinHand2SystemInterface` | 기본값 | CAN-FD (SDK) | 352 | 4 |
| `aidin_hand2_hardware/AidinHand2MockSystemInterface` | `use_mock=true` | 없음. kinematics만 계산 | 69 | 없음 |
| `aidin_hand2_hardware/AidinHand2IsaacSystemInterface` | `use_isaac=true` | Isaac Sim (ROS 2 topic) | 352 | 없음 |

mock은 joint position·joint impedance command를 clamp → IK → FK로, actuator position command를 FK로
곧바로 state에 반영하고, actuator velocity·current는 항상 `0`입니다. Idle과 effort command는 자세를 바꾸지
않습니다. tactile, diagnostics, command echo, timestamp state interface가 없으므로 `HandStateBroadcaster`와
`DiagnosticsBroadcaster`를 붙일 수 없고, homing과 service도 없습니다.

isaac은 Isaac Sim이 발행하는 joint state를 읽어 actuator 값을 IK로 채우고, command를 joint 21개의 목표로
발행합니다. homing이 없어 `homing_state` 값은 항상 `Succeeded`이고, actuator fault는 항상 없습니다.
topic은 [Topics](08_topics.md) 5장에 있습니다.

### 1.2 Lifecycle

lifecycle은 SDK `Hand`의 상태이고 다음 state 5개 중 하나의 값을 가지며, `hand_diagnostics` topic의
`lifecycle` 필드로 확인합니다.

| State | Description | Command |
|---|---|---|
| `Disconnected` | CAN socket이 열리지 않은 상태. configure 전과 cleanup 후 | 전송 안 함 |
| `Connected` | CAN socket이 열려 state를 수신하지만 제어 명령은 송신하지 않는 상태 | 전송 안 함 |
| `Running` | actuator enable이 확인되어 제어 명령을 송신하는 상태 | `homing_state` 값이 `Succeeded`이면 전송 |
| `Stopped` | actuator quick stop이 확인된 무토크 상태 | 전송 안 함 |
| `Faulted` | 통신 오류나 제어·통신 루프 예외로 통신과 제어가 종료된 상태. `~/reconnect` service로만 벗어남 | 전송 안 함 |

`ros2_control` hardware component의 lifecycle callback이 SDK 함수를 호출합니다. launch는 configure와
activate를 연속으로 진행하므로 정상 bringup에서는 `Running`까지 한 번에 갑니다.

| Callback | SDK | Lifecycle after |
|---|---|---|
| `on_configure` | `create()` + `connect()`. 첫 state 수신을 300 ms까지 대기 | `Connected` |
| `on_activate` | `run()`. actuator enable을 4000 ms까지 대기 | `Running` |
| `on_deactivate` | `stop()`. quick stop을 500 ms까지 대기 | `Stopped` |
| `on_cleanup` | `disconnect()` + `destroy()` | `Disconnected` |

component의 state는 controller_manager 명령으로 직접 전이할 수도 있습니다. 전이 전에 command controller를
deactivate하십시오.

```bash
ros2 control set_hardware_component_state left_hand_control inactive
ros2 control set_hardware_component_state left_hand_control active
```

component를 active로 둔 채 제어만 켜고 끄는 수단은 [Services](07_services.md)의 `~/run`·`~/stop`
service이고, `Faulted`에서 벗어나는 수단은 `~/reconnect` service입니다.

### 1.3 Command and state interfaces

command interface는 손마다 65개입니다. claim 전용 lock 하나와 mode별 port 4개이고, port 하나는 16개
interface입니다.

| Command interface | Count | Unit |
|---|---|---|
| `{side}_hand_control/command_lock` | 1 | 값 없음. mode 상호 배제를 위한 claim 전용 |
| `{side}_joint_position_command/target_position_rad.{joint}` | 16 | rad |
| `{side}_joint_impedance_command/target_position_rad.{joint}` | 16 | rad |
| `{side}_actuator_position_command/target_position_cnt.{actuator}` | 16 | encoder count |
| `{side}_actuator_effort_command/target_effort_pct.{actuator}` | 16 | 정격 전류의 0.1% |

state interface는 로봇 핸드와 isaac에서 352개, mock에서 69개입니다.

| State interface group | Count | Mock |
|---|---|---|
| `{side}_{actuator}/position_cnt` · `velocity_rpm` · `current_ma` | 48 | 있음. velocity·current는 `0` |
| `{side}_{joint}/position` | 21 | 있음 |
| `{side}_{finger}_sensor/tactile_1..17` · `{side}_palm_sensor/palm1_upper_1..20` · `palm1_lower_1..20` · `palm2_1..18` | 143 | 없음 |
| `{side}_diagnostics/...` | 39 | 없음 |
| `{side}_commanded/...` | 99 | 없음 |
| `{side}_timestamp/sec` · `nanosec` | 2 | 없음 |

이름 하나하나는 [Interface matrix](11_interface_matrix.md)에 있습니다.

## 2. Joint and actuator order

배열 16개의 순서는 thumb 4개 뒤에 index, middle, ring, baby가 3개씩 이어지는 SDK 순서입니다. 이름 앞에는
`{side}_`가 붙습니다.

active joint 16개는 command의 대상이고, joint 21개에는 finger마다 passive `joint4`가 더해집니다.

```text
thumb_joint0, thumb_joint1, thumb_joint2, thumb_joint3,
index_joint1, index_joint2, index_joint3,
middle_joint1, middle_joint2, middle_joint3,
ring_joint1, ring_joint2, ring_joint3,
baby_joint1, baby_joint2, baby_joint3
```

actuator 16개는 thumb만 `actuator0`이 있습니다. thumb의 CMC가 3축이기 때문입니다. actuator 16개의 이름은
다음과 같습니다.

```text
thumb_actuator0, thumb_actuator1, thumb_actuator2, thumb_actuator3,
index_actuator1, index_actuator2, index_actuator3,
middle_actuator1, middle_actuator2, middle_actuator3,
ring_actuator1, ring_actuator2, ring_actuator3,
baby_actuator1, baby_actuator2, baby_actuator3
```

같은 index의 joint와 actuator는 서로 대응하지 않습니다. 두 공간의 변환은 SDK 문서의
[C++ guide](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/07_cpp_usage_guide.md#6-kinematics)
6장에 있습니다.

## 3. Controllers

controller는 command controller 4개와 broadcaster 2개입니다. command controller는 입력을 SDK command로
바꾸고, broadcaster는 state interface를 topic으로 발행합니다.

### 3.1 Command controllers

command controller는 topic 또는 상위 controller의 reference를 SDK command 하나로 바꾸는 adapter입니다. 새
제어 알고리즘을 담지 않습니다. 넷 모두 `ChainableControllerInterface`이고, `command_lock`과 자기 mode의
port 16개를 claim하며, state interface는 claim하지 않습니다. parameter는 `hand_side` 하나입니다.

| Controller | Claims | Input | Unit | SDK command |
|---|---|---|---|---|
| `aidin_hand2_controllers/JointPositionController` | `command_lock` + `joint_position_command` 16 | active joint 16개의 목표 각도 | rad | `JointPositionCommand` |
| `aidin_hand2_controllers/JointImpedanceController` | `command_lock` + `joint_impedance_command` 16 | active joint 16개의 평형 각도 | rad | `JointImpedanceCommand` |
| `aidin_hand2_controllers/ActuatorPositionController` | `command_lock` + `actuator_position_command` 16 | actuator 16개의 목표 위치 | encoder count | `ActuatorPositionCommand` |
| `aidin_hand2_controllers/ActuatorEffortController` | `command_lock` + `actuator_effort_command` 16 | actuator 16개의 목표 effort. 부호가 방향 | 정격 전류의 0.1% | `ActuatorEffortCommand` |

joint 계열 둘은 SDK가 목표를 finger별 도달 범위로 투영한 뒤 kinematics로 actuator 값을 만듭니다.
`JointPositionController`는 SDK의 joint position controller를 거쳐 actuator position을,
`JointImpedanceController`는 joint impedance controller를 거쳐 actuator effort를 전송합니다. 두 SDK
controller의 filter와 gain은 hardware node의 parameter이고 [Parameters](09_parameters.md) 2장에 있습니다.

> [!IMPORTANT]
> actuator 계열 둘은 kinematics를 거치지 않으므로 도달 범위 투영이 적용되지 않습니다. 계측과 검증에
> 쓰고, 목표는 `hand_state` topic의 `actuator_position` 필드에서 읽은 현재 값 근처에서 시작하십시오.

> [!NOTE]
> joint impedance controller는 SDK에서 아직 개발 중이므로 사용하지 마십시오. 동작하지만 이후 버전에서
> 거동이 바뀔 수 있습니다.

### 3.2 Broadcasters

broadcaster는 state interface를 읽어 topic으로 발행합니다. command interface는 claim하지 않으므로 command
controller와 함께 active일 수 있습니다.

| Controller | Claims | Topic |
|---|---|---|
| `aidin_hand2_controllers/HandStateBroadcaster` | state 313개. joint·actuator·tactile·command echo·timestamp | `~/hand_state` |
| `aidin_hand2_controllers/DiagnosticsBroadcaster` | state 39개. `{side}_diagnostics/*` | `~/hand_diagnostics` |
| `joint_state_broadcaster/JointStateBroadcaster` | 모든 `{joint}/position` | `/joint_states` |

parameter는 `hand_side`와 발행 주기 `update_rate`입니다. 표준 `joint_state_broadcaster`는 양손의 joint를
한 topic에 담으므로 로봇에 하나만 둡니다. 세 topic의 내용은 [Topics](08_topics.md) 3장에 있습니다.

## 4. Command path

command가 hardware command interface에 도달하는 경로는 둘입니다. chained mode가 아니면 topic이고, 상위
controller가 있으면 reference interface입니다. controller는 두 경로 중 하나만 받습니다.

두 경로는 다음과 같습니다.

```text
~/command topic  ──┐
                   ├─→ command controller ─→ hardware command port + command_lock ─→ SDK set_command()
reference interface┘
```

### 4.1 Topic

chained mode가 아닌 command controller는 자기 namespace의 `~/command` topic 하나를 구독합니다. message
하나가 한 cycle의 목표 16개 전부이고, 일부만 갱신하는 message는 없습니다. controller는 message를 받은
cycle에만 port에 값을 쓰고 다음 cycle에 port를 NaN으로 되돌립니다. 그래서 message 하나가 SDK
`set_command()` 한 번이 되고, 다음 message까지는 SDK가 마지막 command를 유지합니다.

message 필드와 단위는 [Topics](08_topics.md) 2장에 있습니다. 다음은 왼손 joint position command를 한 번
전송하는 예입니다.

```bash
ros2 topic pub --once \
  /left_joint_position_controller/command \
  aidin_hand2_msgs/msg/JointPositionCommand \
  "{target_position_rad: [
      0.0, 0.0, 0.0, 0.0,
      0.10, 0.20, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0]}"
```

### 4.2 Reference interface

command controller 넷은 chained mode에서 reference interface 16개를 export하고 topic 구독을 끊습니다.
상위 controller가 reference를 claim하면 topic 왕복 없이 같은 cycle 안에 command가 전달됩니다.

| Command controller | Exported reference |
|---|---|
| `{side}_joint_position_controller` | `{side}_joint_position_controller/{side}_{joint}/position` ×16 |
| `{side}_joint_impedance_controller` | `{side}_joint_impedance_controller/{side}_{joint}/position` ×16 |
| `{side}_actuator_position_controller` | `{side}_actuator_position_controller/{side}_{actuator}/position_cnt` ×16 |
| `{side}_actuator_effort_controller` | `{side}_actuator_effort_controller/{side}_{actuator}/effort_pct` ×16 |

상위 controller는 `command_interface_configuration()`에서 위 이름 16개를 claim하고, 한 update 안에서 16개
모두에 유한한 값을 씁니다. 상위 controller의 skeleton 4종과 실행 방법은
[`aidin_hand2_examples/EXAMPLE.md`](../../aidin_hand2_examples/EXAMPLE.md)에 있습니다.

## 5. Mode switching

command controller는 한 순간 하나만 active일 수 있습니다. hardware component가 mode 전환에서 받아들이는
claim 집합이 둘뿐이기 때문입니다.

- 빈 집합. active인 command controller가 없고, [6.2](#62-suppressed-cycles)의 조건을 모두 통과한
  cycle마다 SDK에 `Idle`이 전송됩니다.
- `command_lock`과 한 mode의 port 16개 전부. claim한 mode의 command가 전송됩니다.

일부만 claim하거나, 두 mode를 함께 claim하거나, lock 없이 port만 claim하면 전환이 거부되고
`rejected mode switch: command interfaces must be one complete mode port plus command_lock` log가
남습니다. `--strict` 없이 전환하면 일부만 성공한 상태가 성공으로 보고될 수 있으므로 항상 `--strict`로
deactivate와 activate를 한 transaction에 넣습니다.

다음은 joint position에서 joint impedance로 전환하는 예입니다. 대상 controller가 load되어 있지 않으면
먼저 `inactive`로 load합니다.

```bash
ros2 control load_controller --set-state inactive left_joint_impedance_controller
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_controller \
  --activate left_joint_impedance_controller
```

전환이 끝나면 hardware component가 새 mode의 port를 NaN으로 비우고, controller도 activate에서 reference를
NaN으로 채웁니다. 첫 message가 오기 전까지 command가 전송되지 않으므로 로봇 핸드는 SDK가 유지하는 마지막
command의 자세에 머무릅니다.

`Idle`은 drive를 활성 상태로 둔 채 effort `0`을 전송하는 command입니다. joint를 외력으로 움직이면 저항이
있으므로, 토크를 완전히 제거하려면 `~/stop` service를 호출하십시오.

## 6. Command validity

command validity는 hardware command interface의 NaN 처리와 전송이 억제되는 cycle 두 가지입니다.

### 6.1 NaN and complete set

hardware command interface의 NaN은 값이 아니라 이번 cycle에 command가 없다는 뜻입니다. controller가 쓴
port 16개를 hardware component가 `write()`에서 다음 규칙으로 처리합니다.

| Port values | Hardware behavior |
|---|---|
| 16개 모두 NaN | `set_command()`를 호출하지 않습니다. SDK가 직전 command를 유지합니다 |
| 일부 NaN | NaN인 축을 직전 command의 값으로 채워 전송합니다. 상위가 점유하지 않은 축입니다 |
| 일부 NaN이고 직전 command도 없음 | 전송하지 않고 warning `command has axes that were never commanded — skipped. Send a complete command once.`를 5 s에 한 번 남깁니다 |
| 16개 모두 유한 | 그대로 전송합니다 |

controller는 message나 reference의 Inf를 NaN으로 바꾸고 `<Mode> reference has an Inf value — ignored`
warning을 5 s에 한 번 남깁니다. 그래서 첫 command는 16개 모두 유한한 값이어야 하고, 이후에는 상위가
점유한 축만 써도 됩니다.

SDK의 검사는 hardware component의 NaN 처리 뒤에 있습니다. joint 목표는 도달 범위로 투영되고,
`ActuatorPositionCommand`의 목표가 `int32` 범위를 벗어나면 SDK가 command를 거부해 `hand_diagnostics`
topic의 `nan_command_count` 값이 늘어납니다. effort 목표는 ±`max_effort`로 제한됩니다.

### 6.2 Suppressed cycles

hardware component는 다음 조건에서 `write()`가 아무것도 전송하지 않고 성공을 반환합니다.

| Condition | Until |
|---|---|
| activate 전, `~/stop` service 뒤, `~/reconnect` service 뒤 | activate 또는 `~/run` service 성공 |
| `lifecycle` 값이 `Running`이 아님 | `Running` 복귀 |
| homing 진행 중, 또는 `homing_state` 값이 `Succeeded`가 아님 | `homing_state` 값이 `Succeeded` |

전송을 건너뛴 cycle에서도 topic publish와 controller update는 성공하므로 publisher 쪽에서는 알 수 없습니다.
command가 로봇 핸드에 닿았는지는 `hand_state` topic의 `command_state.selected_source` 값으로 확인합니다.
값은 [Topics](08_topics.md) 3.3절에 있습니다.

> [!IMPORTANT]
> wrapper와 SDK 어디에도 command의 수명 감시가 없습니다. publisher가 사라져도 SDK는 마지막 command를
> 계속 유지하고, topic이 끊겨도 정지하지 않습니다. 정지 조건은 상위 application이 판단해 `~/stop`
> service를 호출해야 합니다.

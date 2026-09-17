# Topics

wrapper가 구독하는 topic은 command controller의 `~/command` 4개이고, 발행하는 topic은 `/joint_states`,
`~/hand_state`, `~/hand_diagnostics` 3개입니다. 이 문서는 각 topic의 message, 단위, 발행 주기, 유효성
조건을 설명합니다. command topic과 state topic의 QoS는 모두 `SystemDefaultsQoS`이고, Isaac Sim topic은
5장에 있습니다.

`{side}`는 `left` 또는 `right`이고, 배열 16개와 21개의 순서는 [Controllers](06_controllers.md) 2장에
있습니다.

## Contents

&nbsp;&nbsp;[**1. Overview**](#1-overview)<br>
&nbsp;&nbsp;[**2. Command topics**](#2-command-topics)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 JointPositionCommand](#21-jointpositioncommand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 JointImpedanceCommand](#22-jointimpedancecommand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 ActuatorPositionCommand](#23-actuatorpositioncommand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.4 ActuatorEffortCommand](#24-actuatoreffortcommand)<br>
&nbsp;&nbsp;[**3. State topics**](#3-state-topics)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 joint_states](#31-joint_states)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 HandState](#32-handstate)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.3 CommandState](#33-commandstate)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.4 HandDiagnostics](#34-handdiagnostics)<br>
&nbsp;&nbsp;[**4. Monitoring**](#4-monitoring)<br>
&nbsp;&nbsp;[**5. Isaac Sim topics**](#5-isaac-sim-topics)

## 1. Overview

topic 이름은 controller 이름 아래에 있습니다. 발행 주기는 `aidin_hand2_bringup` package의
`controllers.yaml`이 지정한 값이고, 자기 `controllers.yaml`에서 바꿀 수 있습니다.

| Topic | Type | Direction | Rate | Backend |
|---|---|---|---|---|
| `/{side}_joint_position_controller/command` | `aidin_hand2_msgs/JointPositionCommand` | 구독 | — | real · mock · isaac |
| `/{side}_joint_impedance_controller/command` | `aidin_hand2_msgs/JointImpedanceCommand` | 구독 | — | real · mock · isaac |
| `/{side}_actuator_position_controller/command` | `aidin_hand2_msgs/ActuatorPositionCommand` | 구독 | — | real · mock · isaac |
| `/{side}_actuator_effort_controller/command` | `aidin_hand2_msgs/ActuatorEffortCommand` | 구독 | — | real · mock · isaac |
| `/joint_states` | `sensor_msgs/JointState` | 발행 | 100 Hz. mock은 `controllers_mock.yaml`이 지정하지 않아 500 Hz | real · mock · isaac |
| `/{side}_hand_state_broadcaster/hand_state` | `aidin_hand2_msgs/HandState` | 발행 | 100 Hz | real · isaac |
| `/{side}_diagnostics_broadcaster/hand_diagnostics` | `aidin_hand2_msgs/HandDiagnostics` | 발행 | 20 Hz | real · isaac |

command topic은 command controller가 active이고 chained mode가 아닐 때만 구독됩니다. mock에는
`hand_state`와 `hand_diagnostics` topic이 없습니다. broadcaster가 읽을 state interface가 없기
때문입니다.

## 2. Command topics

command topic의 message는 길이 16의 `float64` 배열 하나입니다. message 하나가 한 cycle의 목표 전부이고,
controller는 message를 받은 cycle에만 command를 전달합니다. 배열의 NaN은 해당 축을 이번 cycle에 지정하지
않았다는 뜻이며, 첫 message는 16개 모두 유한해야 합니다. 규칙은 [Controllers](06_controllers.md) 6장에
있습니다.

### 2.1 JointPositionCommand

`JointPositionCommand`는 active joint 16개의 목표 각도입니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_position_rad` | `float64[16]` | rad | active joint 16개의 목표 각도 |

SDK가 목표를 finger별 도달 범위로 투영한 뒤 joint position controller를 거쳐 actuator position으로
변환합니다. controller의 filter는 hardware node의 parameter이고 [Parameters](09_parameters.md) 2.2절에
있습니다.

다음은 왼손 index finger의 joint1을 0.10 rad, joint2를 0.20 rad로 보내는 예입니다.

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

### 2.2 JointImpedanceCommand

`JointImpedanceCommand`는 active joint 16개의 평형 각도입니다. 필드는 `JointPositionCommand`와 같고
SDK가 actuator position이 아니라 actuator effort를 전송합니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_position_rad` | `float64[16]` | rad | active joint 16개의 평형 각도 |

`stiffness`·`damping` gain은 message가 아니라 hardware node의 parameter입니다. joint impedance controller는
SDK에서 아직 개발 중이며 사용 보류는 [3.1 Command controllers](06_controllers.md#31-command-controllers)에
있습니다. 다음은 같은 목표를 joint impedance로 보내는 예입니다.

```bash
ros2 topic pub --once \
  /left_joint_impedance_controller/command \
  aidin_hand2_msgs/msg/JointImpedanceCommand \
  "{target_position_rad: [
      0.0, 0.0, 0.0, 0.0,
      0.10, 0.20, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0]}"
```

### 2.3 ActuatorPositionCommand

`ActuatorPositionCommand`는 actuator 16개의 목표 위치입니다. kinematics를 거치지 않아 도달 범위 투영이
없습니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_position_cnt` | `float64[16]` | encoder count | actuator 16개의 목표 위치. `int32` 범위 |

목표는 `hand_state` topic의 `actuator_position` 필드에서 읽은 현재 값에 작은 차이를 더해 만듭니다. 다음
예의 `<cnt_0>`부터 `<cnt_15>`는 읽어 온 현재 값입니다.

```bash
ros2 topic pub --once \
  /left_actuator_position_controller/command \
  aidin_hand2_msgs/msg/ActuatorPositionCommand \
  "{target_position_cnt: [
      <cnt_0>, <cnt_1>, <cnt_2>, <cnt_3>,
      <cnt_4>, <cnt_5>, <cnt_6>,
      <cnt_7>, <cnt_8>, <cnt_9>,
      <cnt_10>, <cnt_11>, <cnt_12>,
      <cnt_13>, <cnt_14>, <cnt_15>]}"
```

### 2.4 ActuatorEffortCommand

`ActuatorEffortCommand`는 actuator 16개의 목표 effort입니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_effort_pct` | `float64[16]` | 정격 전류의 0.1% | actuator 16개의 목표 effort. 부호가 방향이고 `1000`이 100% |

SDK가 절댓값을 actuator별 `max_effort` 값으로 제한한 뒤 전송합니다. 다음은 30%를 보내는 예입니다.

```bash
ros2 topic pub --once \
  /left_actuator_effort_controller/command \
  aidin_hand2_msgs/msg/ActuatorEffortCommand \
  "{target_effort_pct: [
      300.0, 300.0, 300.0, 300.0,
      300.0, 300.0, 300.0,
      300.0, 300.0, 300.0,
      300.0, 300.0, 300.0,
      300.0, 300.0, 300.0]}"
```

## 3. State topics

state topic 셋은 broadcaster가 hardware component의 state interface를 읽어 발행합니다. 셋은 각각 최신
값을 읽으므로 같은 시각에 발행된 message라도 cycle이 하나 다를 수 있습니다.

### 3.1 joint_states

`/joint_states` topic은 표준 `joint_state_broadcaster`가 발행하는 `sensor_msgs/JointState`입니다. `name`
필드에 로봇의 모든 joint, `position` 필드에 각도[rad]가 있고, 로봇 핸드의 joint는 손마다 21개입니다.
wrapper는 joint의 velocity·effort state interface를 export하지 않습니다.

발행되는 message를 한 번 읽습니다.

```bash
ros2 topic echo /joint_states --once
```

### 3.2 HandState

`HandState`는 로봇 핸드 하나의 관측을 한 message에 모은 것이며 `HandStateBroadcaster`가 발행합니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `header.stamp` | `builtin_interfaces/Time` | — | SDK가 state를 관측한 시각(`CLOCK_REALTIME`). 발행 시각이 아닙니다 |
| `hand_side` | `string` | — | `left` 또는 `right` |
| `joint_position` | `float64[21]` | rad | forward kinematics 결과. passive `joint4` 포함 |
| `actuator_position` | `float64[16]` | encoder count | drive가 보고한 위치 |
| `actuator_velocity` | `float64[16]` | rpm | drive가 보고한 속도 |
| `actuator_current` | `float64[16]` | mA | drive가 보고한 전류 |
| `tactile_thumb` · `tactile_index` · `tactile_middle` · `tactile_ring` · `tactile_baby` | `float64[17]` | 원시 count | finger별 taxel |
| `tactile_palm1_upper` · `tactile_palm1_lower` | `float64[20]` | 원시 count | palm1 taxel |
| `tactile_palm2` | `float64[18]` | 원시 count | palm2 taxel |
| `command_state` | `CommandState` | — | 같은 cycle에 적용된 command. [3.3](#33-commandstate) |

`header.stamp` 필드는 SDK가 아직 state를 채우지 않아 `0`일 때만 broadcaster의 update 시각으로
대체됩니다.

tactile 값은 센서가 전송한 원시 count라 단위도 정규화도 없습니다. 접촉 판정 임계값은 SDK가 기준을
제시하지 않으므로, 접촉이 없는 상태를 baseline으로 두고 차이를 보십시오.

발행되는 message를 한 번 읽습니다.

```bash
ros2 topic echo /left_hand_state_broadcaster/hand_state --once
```

### 3.3 CommandState

`CommandState`는 한 cycle의 command 처리 과정을 담은 `HandState`의 `command_state` 필드입니다. 세 enum
필드가 어느 nested 필드가 유효한지 결정하고, 유효하지 않은 필드는 NaN일 수 있습니다.

| Field | Type | Description |
|---|---|---|
| `controller_input_mode` | `uint8` | 적용 중인 command의 종류. `0` idle, `1` joint position, `2` joint impedance, `3` actuator position, `4` actuator effort |
| `joint_position_input` · `joint_impedance_input` · `actuator_position_input` · `actuator_effort_input` | command message | `controller_input_mode`가 나타내는 하나만 유효. joint 둘은 같은 값 |
| `controller_output_type` | `uint8` | SDK 변환 결과의 종류. `0` none, `1` actuator position, `2` actuator effort |
| `target_position_cnt` · `target_effort_pct` | `float64[16]` | `controller_output_type`이 나타내는 하나만 유효 |
| `selected_source` | `uint8` | 이번 cycle에 실제로 전송된 출처. `0` none, `1` controller, `2` quick stop, `3` homing |
| `max_effort_pct` | `float64[16]` | 변환에 적용된 actuator별 effort 상한 |

`controller_output_type` 값이 있어도 drive가 변환 결과를 적용했다는 뜻은 아닙니다. `selected_source` 값이
`1`인지 함께 확인하십시오. joint 목표의 echo는 SDK가 도달 범위로 투영한 뒤의 값입니다.

> [!WARNING]
> NaN을 `0`으로 바꾸어 기록하면 "해당 없음"과 실제 목표 `0`을 구분할 수 없습니다. 기록할 때는 세 enum
> 필드를 validity mask로 함께 남기십시오.

`command_state` 필드만 읽습니다.

```bash
ros2 topic echo /left_hand_state_broadcaster/hand_state --once --field command_state
```

### 3.4 HandDiagnostics

`HandDiagnostics`는 SDK와 제어·통신 루프의 상태를 담으며 `DiagnosticsBroadcaster`가 발행합니다.
`header.stamp` 필드는 발행 시각입니다.

| Field | Type | Normal | Description |
|---|---|---|---|
| `hand_side` | `string` | — | `left` 또는 `right` |
| `lifecycle` | `string` | `Running` | `Disconnected` `Connected` `Running` `Stopped` `Faulted` 중 하나. 뜻은 [1.2 Lifecycle](06_controllers.md#12-lifecycle) |
| `homing_state` | `string` | `Succeeded` | `NotRun` `InProgress` `Succeeded` `Failed` 중 하나 |
| `nan_command_count` | `uint64` | 늘지 않음 | 값이 유효하지 않아 SDK가 거부한 command의 누적 개수 |
| `control_cycles` | `uint64` | 계속 증가 | 제어·통신 루프의 누적 cycle |
| `deadline_misses` | `uint64` | 늘지 않음 | 계산 시간이 목표 주기를 넘긴 cycle의 누적 개수 |
| `last_period_ms` | `float64` | 2.0 근처 | cycle 시작 사이의 간격 |
| `last_compute_ms` | `float64` | — | 마지막 cycle의 계산 시간 |
| `actuator_enabled` | `bool[16]` | 모두 `true` | actuator enable 여부. `disabled_actuators`에 지정한 것은 `false` |
| `actuator_fault_name` | `string[16]` | 모두 빈 문자열 | SDK `ActuatorFault` 이름. 비어 있으면 정상 |

`deadline_misses` 값은 순간 값보다 일정 구간의 증가율로 봅니다. 증가율은 `Δdeadline_misses /
Δcontrol_cycles`이고 임계값은 CPU·payload에 따라 달라 SDK가 기준을 제시하지 않습니다.

`actuator_fault_name` 값별 점검 항목은 SDK 문서의
[Error messages](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/15_error_messages.md#11-actuator-fault)에
있습니다. fault가 있는 actuator는 SDK가 command에서 mask하고 fault reset을 반복 시도하며, wrapper는 스스로
정지하지 않습니다.

발행되는 message를 한 번 읽습니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

## 4. Monitoring

감시는 topic의 값이 실제로 갱신되는지를 보고 로봇 핸드의 동작 여부를 판단하는 일입니다. topic이 계속
발행된다는 사실은 로봇 핸드가 동작 중이라는 뜻이 아닙니다. broadcaster는 state interface의
최신 값을 매 update 발행하므로, SDK가 `Faulted`로 정지해도 마지막 값이 같은 주기로 반복됩니다. 동작 여부는
값의 변화로 판단합니다.

| Signal | Alive when |
|---|---|
| `hand_diagnostics.control_cycles` | 두 message 사이에 증가 |
| `hand_state.header.stamp` | 두 message 사이에 증가 |
| `hand_diagnostics.lifecycle` | `Running` |
| `hand_diagnostics.homing_state` | `Succeeded` |

`control_cycles`와 `header.stamp` 필드는 `Faulted`가 아닌 동안 frame 수신 여부와 무관하게 매 cycle
갱신되므로 통신 생존의 근거는 아닙니다. 통신이 끊기면 약 100 ms 뒤 `lifecycle` 값이 `Faulted`로 바뀌고
전이 시점부터 둘이 함께 멈춥니다.

> [!IMPORTANT]
> `HandDiagnostics`에는 종합 판정 필드가 없습니다. 준비 여부는 구독자가 `lifecycle`·`homing_state`·
> `actuator_fault_name` 필드와 `control_cycles` 값의 증가를 합쳐 판단해야 합니다. 마지막 예외 문구와
> 재연결 시도 횟수도 message에 없습니다.

## 5. Isaac Sim topics

isaac backend는 CAN 대신 다음 topic으로 Isaac Sim과 주고받습니다. 이름은 매크로의 `isaac_*` parameter로
바꿀 수 있고 기본값은 아래와 같습니다.

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/isaac/joint_states` | `sensor_msgs/JointState` | Isaac → wrapper | joint 이름으로 대응합니다. `{side}_` 접두어가 붙은 이름만 읽고 나머지는 무시합니다 |
| `/isaac/hand_command` | `sensor_msgs/JointState` | wrapper → Isaac | joint 21개의 목표 각도. joint position·joint impedance·actuator position mode에서 목표 16개가 모두 유한한 cycle에만 발행합니다 |
| `/isaac/tactile/{side}_{finger}_sensor` | `std_msgs/Float64MultiArray` | Isaac → wrapper | finger별 taxel 17개. `{finger}`는 `thumb` `index` `middle` `ring` `baby` |
| `/isaac/tactile/{side}_palm_sensor` | `std_msgs/Float64MultiArray` | Isaac → wrapper | palm taxel 58개 |

구독은 `SensorDataQoS`, 발행은 depth 1입니다. Isaac에서 오는 joint state가 `isaac_state_timeout` 값(기본
0.1 s)보다 오래 끊기면 `lifecycle` 값이 `Disconnected`가 되고 `deadline_misses` 값이 늘어납니다.

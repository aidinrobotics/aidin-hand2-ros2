# aidin_hand2_msgs

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

`aidin_hand2_msgs`는 command와 상태 message 타입을 정의합니다.
이 문서에서 필드·타입·단위와 배열 순서를 확인할 수 있습니다. message 정의 파일은 [msg/](msg)에 있습니다.
목표 전송과 상태 관측 명령은 [Control guide](../docs/ko/05_control_guide.md)에 있습니다.

## Contents

&nbsp;&nbsp;[**1. Message types**](#1-message-types)<br>
&nbsp;&nbsp;[**2. Command messages**](#2-command-messages)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 JointPositionCommand](#21-jointpositioncommand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 JointImpedanceCommand](#22-jointimpedancecommand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 ActuatorPositionCommand](#23-actuatorpositioncommand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.4 ActuatorEffortCommand](#24-actuatoreffortcommand)<br>
&nbsp;&nbsp;[**3. State messages**](#3-state-messages)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 HandState](#31-handstate)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 CommandState](#32-commandstate)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.3 HandDiagnostics](#33-handdiagnostics)<br>
&nbsp;&nbsp;[**4. Joint and actuator order**](#4-joint-and-actuator-order)

## 1. Message types

| Type | Purpose |
|---|---|
| [JointPositionCommand](msg/JointPositionCommand.msg) | joint 목표 각도 |
| [JointImpedanceCommand](msg/JointImpedanceCommand.msg) | joint impedance 목표 각도. 사용 보류 |
| [ActuatorPositionCommand](msg/ActuatorPositionCommand.msg) | actuator 목표 위치 |
| [ActuatorEffortCommand](msg/ActuatorEffortCommand.msg) | actuator 목표 effort |
| [HandState](msg/HandState.msg) | joint·actuator·tactile 관측과 적용된 command |
| [CommandState](msg/CommandState.msg) | `HandState.command_state`에 포함된 command 정보 |
| [HandDiagnostics](msg/HandDiagnostics.msg) | lifecycle·homing 상태, 주기 통계와 actuator fault |

## 2. Command messages

command message는 길이 16의 `float64` 배열 하나입니다. 처음 보낼 때는 목표값 16개를 모두
유한한 값으로 지정합니다. 이후 일부 축을 유지하려면 해당 위치에 NaN을 넣을 수 있습니다.
값 처리 규칙은 [4.2 Target values](../aidin_hand2_controllers/README.ko.md#42-target-values)에 있습니다.

### 2.1 JointPositionCommand

`JointPositionCommand`는 active joint 16개의 목표 각도입니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_position_rad` | `float64[16]` | rad | active joint 16개의 목표 각도 |

SDK가 목표를 finger별 도달 범위로 투영한 뒤 joint position controller를 거쳐 actuator position으로
변환합니다. controller의 filter는 hardware node의 parameter이고 [6.2 Joint position controller](../aidin_hand2_hardware/README.ko.md#62-joint-position-controller)에
있습니다.

### 2.2 JointImpedanceCommand

`JointImpedanceCommand`는 active joint 16개의 목표 각도입니다. 필드는 `JointPositionCommand`와 같고
SDK가 actuator position이 아니라 actuator effort를 전송합니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_position_rad` | `float64[16]` | rad | active joint 16개의 목표 각도 |

`stiffness`·`damping`은 message에 넣지 않고 hardware node parameter로 설정합니다.

> [!NOTE]
> joint impedance controller는 SDK에서 개발 중이므로 사용하지 마십시오. 이 절은 message 형식만 설명합니다.

### 2.3 ActuatorPositionCommand

`ActuatorPositionCommand`는 actuator 16개의 목표 위치입니다. kinematics를 거치지 않아 도달 범위 투영이
없습니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_position_cnt` | `float64[16]` | encoder count | actuator 16개의 목표 위치. `int32` 범위 |

### 2.4 ActuatorEffortCommand

`ActuatorEffortCommand`는 actuator 16개의 목표 effort입니다.

| Field | Type | Unit | Description |
|---|---|---|---|
| `target_effort_pct` | `float64[16]` | 정격 전류의 0.1% | actuator 16개의 목표 effort. 부호가 방향이고 `1000`이 100% |

SDK가 절댓값을 actuator별 `max_effort` 값으로 제한한 뒤 전송합니다.

## 3. State messages

### 3.1 HandState

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
| `command_state` | `CommandState` | — | 같은 cycle에 적용된 command. [3.2 CommandState](#32-commandstate) |

`header.stamp` 필드는 SDK가 아직 state를 채우지 않아 `0`일 때만 broadcaster의 update 시각으로
대체됩니다.

tactile 값은 센서가 전송한 원시 count라 단위도 정규화도 없습니다. 접촉 판정 임계값은 SDK가 기준을
제시하지 않으므로, 접촉이 없는 상태를 baseline으로 두고 차이를 보십시오.

### 3.2 CommandState

`CommandState`는 한 cycle의 command 처리 과정을 담은 `HandState`의 `command_state` 필드입니다. 세 enum
필드가 어느 nested 필드가 유효한지 결정하고, 유효하지 않은 필드는 NaN일 수 있습니다.

| Field | Type | Description |
|---|---|---|
| `controller_input_mode` | `uint8` | 적용 중인 command의 종류. `0` idle, `1` joint position, `2` joint impedance, `3` actuator position, `4` actuator effort |
| `joint_position_input` · `joint_impedance_input` · `actuator_position_input` · `actuator_effort_input` | command message | `controller_input_mode`가 나타내는 하나만 유효. joint 둘은 같은 값 |
| `controller_output_type` | `uint8` | SDK 변환 결과의 종류. `0` none, `1` actuator position, `2` actuator effort |
| `target_position_cnt` · `target_effort_pct` | `float64[16]` | `controller_output_type`이 나타내는 하나만 유효 |
| `selected_source` | `uint8` | 이번 cycle에 전송 대상으로 선택된 출처. `0` none, `1` controller, `2` quick stop, `3` homing |
| `max_effort_pct` | `float64[16]` | 변환에 적용된 actuator별 effort 상한 |

`selected_source`가 `1`이면 SDK가 controller 출력을 선택한 상태입니다. 마지막 command를 유지하는
동안에도 같은 값이므로 새 message의 수신 확인으로 사용하지 않습니다. 목표 도달 여부는
`joint_position` 또는 `actuator_position` 관측값으로 확인합니다.
joint 목표의 echo는 SDK가 도달 범위로 보정한 뒤의 값입니다.

> [!IMPORTANT]
> NaN을 `0`으로 바꾸어 기록하면 "해당 없음"과 실제 목표 `0`을 구분할 수 없습니다. 기록할 때는 세 enum
> 필드를 validity mask로 함께 남기십시오.

### 3.3 HandDiagnostics

`HandDiagnostics`는 SDK와 제어·통신 루프의 상태를 담으며 `DiagnosticsBroadcaster`가 발행합니다.
`header.stamp` 필드는 발행 시각입니다.
`Running` 열은 homing 완료 후 제어 중일 때의 참고값이며, 시간 값은 기본 500 Hz 설정 기준입니다.

| Field | Type | Running | Description |
|---|---|---|---|
| `hand_side` | `string` | — | `left` 또는 `right` |
| `lifecycle` | `string` | `Running` | `Disconnected` `Connected` `Running` `Stopped` `Faulted` 중 하나. 뜻은 [1.1 Lifecycle](../aidin_hand2_hardware/README.ko.md#11-lifecycle) |
| `homing_state` | `string` | `Succeeded` | `NotRun` `InProgress` `Succeeded` `Failed` 중 하나 |
| `nan_command_count` | `uint64` | 늘지 않음 | 값이 유효하지 않아 SDK가 거부한 command의 누적 개수 |
| `control_cycles` | `uint64` | 계속 증가 | 제어·통신 루프의 누적 cycle |
| `deadline_misses` | `uint64` | 늘지 않음 | 계산 시간이 목표 주기를 넘긴 cycle의 누적 개수 |
| `last_period_ms` | `float64` | 2.0 근처 | cycle 시작 사이의 간격 |
| `last_compute_ms` | `float64` | — | 마지막 cycle의 계산 시간 |
| `actuator_enabled` | `bool[16]` | 모두 `true` | actuator enable 여부. `disabled_actuators`에 지정한 것은 `false` |
| `actuator_fault_name` | `string[16]` | 모두 빈 문자열 | SDK `ActuatorFault` 이름. 비어 있으면 정상 |

`actuator_fault_name` 값별 점검 항목은 SDK 문서의
[Error messages](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/15_error_messages.md#11-actuator-fault)에
있습니다. fault가 있는 actuator는 SDK가 command에서 mask하고 fault reset을 반복 시도하며, wrapper는 스스로
정지하지 않습니다.


## 4. Joint and actuator order

command와 actuator 관측 배열의 길이는 16입니다. thumb 4개 뒤에 index, middle, ring, baby가
3개씩 이어집니다. 다음 표의 index는 배열 위치이며 이름 앞에는 `left_` 또는 `right_`가 붙습니다.

| Array index | Active joint | Actuator |
|---|---|---|
| 0~3 | `thumb_joint0`~`thumb_joint3` | `thumb_actuator0`~`thumb_actuator3` |
| 4~6 | `index_joint1`~`index_joint3` | `index_actuator1`~`index_actuator3` |
| 7~9 | `middle_joint1`~`middle_joint3` | `middle_actuator1`~`middle_actuator3` |
| 10~12 | `ring_joint1`~`ring_joint3` | `ring_actuator1`~`ring_actuator3` |
| 13~15 | `baby_joint1`~`baby_joint3` | `baby_actuator1`~`baby_actuator3` |

같은 index의 joint와 actuator가 물리적으로 일대일 대응하는 것은 아닙니다.
SDK가 kinematics로 두 공간 사이의 값을 변환합니다.

`HandState.joint_position`은 passive `joint4`를 포함하므로 길이가 21입니다.
순서는 다음과 같습니다. passive joint는 command 배열에 넣지 않습니다.

```text
thumb_joint0, thumb_joint1, thumb_joint2, thumb_joint3, thumb_joint4,
index_joint1, index_joint2, index_joint3, index_joint4,
middle_joint1, middle_joint2, middle_joint3, middle_joint4,
ring_joint1, ring_joint2, ring_joint3, ring_joint4,
baby_joint1, baby_joint2, baby_joint3, baby_joint4
```

`/joint_states`는 전체 로봇의 joint를 담을 수 있으므로 위 고정 순서를 적용하지 않고 `name`과
`position`의 같은 index를 대응시켜 읽습니다.

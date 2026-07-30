# Interface reference

이 문서는 AIDIN Hand Gen2 ROS 2 wrapper의 public 계약입니다. `{side}`는 `left` 또는
`right`입니다. 16개 배열 순서는 thumb 4개 뒤에 index, middle, ring, baby 각 3개가
이어지는 SDK 순서로 고정됩니다.

## 1. 명령 계층

```text
상위 controller 또는 JointTrajectoryController
  → basic controller가 export한 reference interface
  → 기존 4개 basic controller(command-port adapter)
  → mode별 complete hardware command port
  → real/mock SystemInterface
  → SDK ControllerCommand
```

`JointPositionController`, `JointImpedanceController`,
`ActuatorPositionController`, `ActuatorEffortController`가 기존 basic controller입니다.
새로운 알고리즘 controller를 하나 더 넣은 것이 아니라, 이 네 controller가 topic/reference를
완전한 SDK typed command로 바꾸는 hardware 직전 adapter 역할을 합니다.

공통 `command_ports.hpp` 또는 별도 interface package는 없습니다. 각 basic controller
source 최상단이 해당 controller의 hardware command·state·reference·topic 계약을 정의하며,
real/mock hardware는 같은 고정 이름을 각각 검증합니다.

## 2. Hardware plugin과 parameter

| Plugin | 역할 |
|---|---|
| `aidin_hand2_hardware/AidinHand2SystemInterface` | 실제 SDK·CAN hardware |
| `aidin_hand2_hardware/AidinHand2MockSystemInterface` | 동일 command port를 쓰는 kinematics mock |

실제 plugin lifecycle:

| Callback | SDK operation |
|---|---|
| `on_configure` | `create()` + `connect()` + initial max effort |
| `on_activate` | `run()` + 현재 상태 기반 command seed |
| `read` | State·diagnostics snapshot, optional auto-home trigger |
| `write` | 활성 port를 완전한 SDK typed command로 변환 |
| `on_deactivate` | blocking `stop()` |
| `on_cleanup` | `disconnect()` + `destroy()` |

| Hardware parameter | Type | Xacro default | 의미 |
|---|---|---:|---|
| `hand_side` | string | required | `left` 또는 `right` |
| `can_interface` | string | side별 | `can0`, `can1`, `auto` |
| `auto_home` | bool text | `true` | 활성화 뒤 non-blocking homing trigger |
| `max_effort` | double | 1000 | Rated current % |
| `control_rate` | int | 500 | SDK loop Hz |
| `rt_cpu_affinity` | int | -1 | SDK RT thread CPU |
| `disabled_actuators` | CSV | empty | 제외할 actuator index |
| `auto_reconnect` | bool text | `false` | SDK automatic reconnect |
| `auto_reconnect_timeout_ms` | int | 0 | 0은 무제한 |
| `auto_reconnect_home` | bool text | `false` | 복구 후 homing |

정상 launch는 [Launch reference](02_launch_reference.md)의 YAML 값으로 일부 default를 덮습니다.

## 3. Standalone typed command topic

Chained 상태가 아닌 basic controller는 자기 namespace의 `~/command` 하나를 구독합니다.
고정 배열은 모두 16개이며, 한 message가 한 cycle에 필요한 target과 부속값 전체를 가집니다.
Partial update는 없습니다.

| Controller | Topic | Message와 field |
|---|---|---|
| `left_joint_position_controller` | `/left_joint_position_controller/command` | `JointPositionCommand`: `target_position_rad[16]`, `speed_rad_s` |
| `left_joint_impedance_controller` | `/left_joint_impedance_controller/command` | `JointImpedanceCommand`: `target_position_rad[16]`, `stiffness[16]`, `damping[16]` |
| `left_actuator_position_controller` | `/left_actuator_position_controller/command` | `ActuatorPositionCommand`: `target_position_cnt[16]` |
| `left_actuator_effort_controller` | `/left_actuator_effort_controller/command` | `ActuatorEffortCommand`: `target_effort_pct[16]` |
| Hardware system | `/left_hand_control/set_max_effort` | `std_msgs/Float64`: 모든 actuator 공통 상한 |

모든 subscription은 `SystemDefaultsQoS`입니다. Basic controller는 NaN/Inf, 음수 speed,
음수 gain을 거부합니다. SDK는 actuator position의 int32 범위를 검증하고 joint target을
workspace 안으로 자동 clamp합니다.

## 4. Chainable reference interface

아래 suffix 앞에는 export한 basic controller 이름이 붙습니다. 예를 들어 첫 joint-position
resource 전체 이름은
`left_joint_position_controller/left_thumb_joint0/position`입니다.

| Basic controller | Exported reference |
|---|---|
| JointPosition | `{side}_{active_joint}/position` ×16 + `{side}_joint_position/speed_rad_s` ×1 |
| JointImpedance | `{side}_{active_joint}/position` ×16 + `{side}_{actuator}/stiffness` ×16 + `{side}_{actuator}/damping` ×16 |
| ActuatorPosition | `{side}_{actuator}/position_cnt` ×16 |
| ActuatorEffort | `{side}_{actuator}/effort_pct` ×16 |

상위 controller가 reference 중 하나라도 만들면 해당 mode의 reference 전체를 같은 update에서
유한한 값으로 써야 합니다. Joint speed와 impedance gain은 0 이상이어야 합니다. Chained
mode에서는 basic controller의 standalone topic이 reference source가 아닙니다.

네 상위 controller skeleton과 controller별 YAML은
[aidin_hand2_examples/EXAMPLE.md](../../aidin_hand2_examples/EXAMPLE.md)에 있습니다. 네
skeleton 모두 전체 `HandState`를 realtime buffer에서 멤버로 복사하고, 기본 상태에서는
아무 reference 값도 만들지 않습니다.

## 5. Hardware command port

Real과 mock은 손 하나당 같은 98개 command interface를 export합니다.

| Port | 수 | Resource |
|---|---:|---|
| Claim-only lock | 1 | `{side}_hand_control/command_lock` |
| JointPosition | 17 | `{side}_joint_position_command/target_position_rad.<joint>` ×16 + `/speed_rad_s` |
| JointImpedance | 48 | `{side}_joint_impedance_command/target_position_rad.<joint>` ×16 + `/stiffness.<actuator>` ×16 + `/damping.<actuator>` ×16 |
| ActuatorPosition | 16 | `{side}_actuator_position_command/target_position_cnt.<actuator>` ×16 |
| ActuatorEffort | 16 | `{side}_actuator_effort_command/target_effort_pct.<actuator>` ×16 |

Basic controller 하나가 `command_lock`과 자기 mode port 전체를 claim합니다.
`command_lock`의 수치값은 사용하지 않으며 mode 상호 배제만 담당합니다. Hardware mode switch는
빈 claim set(Idle) 또는 네 complete set 중 정확히 하나만 허용합니다. Partial·mixed set은
real과 mock 모두 거부합니다.

## 6. Joint·actuator 순서

Active joint 16개:

```text
thumb_joint0, thumb_joint1, thumb_joint2, thumb_joint3,
index_joint1, index_joint2, index_joint3,
middle_joint1, middle_joint2, middle_joint3,
ring_joint1, ring_joint2, ring_joint3,
baby_joint1, baby_joint2, baby_joint3
```

전체 joint state 21개에는 thumb와 네 long finger의 passive `joint4`가 추가됩니다.

Actuator 16개:

```text
thumb_actuator0, thumb_actuator1, thumb_actuator2, thumb_actuator3,
index_actuator1, index_actuator2, index_actuator3,
middle_actuator1, middle_actuator2, middle_actuator3,
ring_actuator1, ring_actuator2, ring_actuator3,
baby_actuator1, baby_actuator2, baby_actuator3
```

각 이름 앞에는 `left_` 또는 `right_`가 붙습니다.

## 7. State interface와 단위

- Joint broadcaster와 JTC가 읽는 joint 21개만 표준 `{joint}/position`을 사용하며 단위는 rad입니다.
- Custom actuator state는 `{actuator}/position_cnt`, `/velocity_rpm`, `/current_ma`입니다.
- Custom physical command/reference 이름에는 `rad`, `rad_s`, `cnt`, `pct`, `rpm`, `ma` 등
  단위 suffix가 있습니다.
- Real과 mock은 actuator physical state 48개, joint position state 21개를 공통으로
  export합니다.
- 실제 hardware만 tactile 143개, diagnostics 39개, command state 132개와 timestamp
  2개를 추가합니다. 실제 hardware state interface 총수는 385개입니다.

현재 xacro에는 physical·diagnostics 251개가 정적으로 선언되고 command state 132개와
timestamp 2개는 실제 plugin이 동적으로 export합니다. Runtime schema는
`ros2 control list_hardware_interfaces`로 확인하십시오.

`HandStateBroadcaster`는 diagnostics 39개를 제외한 state 346개를 claim합니다.
`DiagnosticsBroadcaster`는 diagnostics 39개를 별도로 claim합니다.

## 8. State·diagnostics topic

| Topic | Type | 제공 YAML rate | 주요 내용 |
|---|---|---:|---|
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | 표준 joint position 21개 |
| `/left_hand_state_broadcaster/hand_state` | `aidin_hand2_msgs/HandState` | 100 Hz | Joint·actuator·tactile·nested `CommandState` |
| `/left_diagnostics_broadcaster/hand_diagnostics` | `aidin_hand2_msgs/HandDiagnostics` | 20 Hz | Lifecycle, homed, RT 통계, actuator health |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 20 Hz/hand | 표준 diagnostics |

`HandState.header.stamp`는 SDK RX 관측 시각이며 초기값이 0일 때만 broadcaster update time을
사용합니다.

`CommandState`는 다음을 분리합니다.

- `controller_input_mode`와 해당 mode의 완전한 typed input
- `controller_output_type`과 `target_position_cnt[16]` 또는 `target_effort_pct[16]`
- `selected_source`: `NONE`, `CONTROLLER`, `QUICK_STOP`, `HOMING`
- `max_effort_pct[16]`

Controller output은 SDK 변환 결과이지 drive 수신·적용 확인이 아닙니다.
`transmit_succeeded` field는 없습니다.

## 9. Runtime service

실제 hardware만 다음 `std_srvs/srv/Trigger` service를 제공합니다.

| Service | 성공 message | 계약 |
|---|---|---|
| `/left_hand_control/run` | `running` | Drive enable 또는 stop 뒤 재개 |
| `/left_hand_control/stop` | `stopped` | Blocking quick stop |
| `/left_hand_control/home` | `homing started — poll diagnostics 'homed'` | `start_homing()` non-blocking trigger |
| `/left_hand_control/reconnect` | `reconnected — call ~/run to resume control` | 통신만 복구, 별도 `run` 필요 |

Service node는 hardware component와 별도 single-thread executor를 사용합니다. `home` 성공은
시작 접수만 뜻하며 완료는 `HandDiagnostics.homed`로 확인합니다.

## 10. Mock 범위

Mock은 real과 동일한 98개 command port, 같은 exact mode switch, actuator physical state
48개와 joint position state 21개를 제공합니다. 네 basic controller를 모두 load할 수
있지만 default launch는 `joint_state_broadcaster`와
`left_joint_position_controller`만 spawn합니다.

Mock은 CAN, drive, homing, tactile, hardware diagnostics, command echo와 runtime service를
제공하지 않습니다. 따라서 default mock에서는 `HandStateBroadcaster`와
`DiagnosticsBroadcaster`를 spawn하지 않습니다.

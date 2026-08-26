# Interface reference

이 문서는 AIDIN Hand Gen2 ROS 2 wrapper의 public 계약입니다. `{side}`는 `left` 또는
`right`입니다. 16개 배열 순서는 thumb 4개 뒤에 index, middle, ring, baby 각 3개가
이어지는 SDK 순서로 고정됩니다.

## 1. 명령 계층

```text
상위 controller (chaining) 또는 ~/command topic
  → command controller
  → SystemInterface (real·mock)
  → SDK
```

`JointPositionController`, `JointImpedanceController`, `ActuatorPositionController`,
`ActuatorEffortController` 네 개가 command controller입니다. 새 알고리즘 계층이 아니라 topic
또는 reference를 완전한 SDK typed command로 바꾸는 adapter입니다.

## 2. Hardware plugin과 parameter

| Plugin | 역할 |
|---|---|
| `aidin_hand2_hardware/AidinHand2SystemInterface` | 실제 SDK·CAN hardware |
| `aidin_hand2_hardware/AidinHand2MockSystemInterface` | 동일 command port를 쓰는 kinematics mock |

Lifecycle callback과 SDK operation의 대응은 [운영과 복구](06_operations.md)에 있습니다.

Hardware parameter 전체와 기본값은 [ros2_control 설정](03_setup.md)의 매크로 계약에 있습니다.
정상 launch는 [Bringup 예제](05_bringup_example.md)의 YAML 값으로 일부를 덮습니다.

## 3. Standalone typed command topic

Chained 상태가 아닌 basic controller는 자기 namespace의 `~/command` 하나를 구독합니다.
고정 배열은 모두 16개이며, 한 message가 한 cycle에 필요한 target과 부속값 전체를 가집니다.
Partial update는 없습니다.

| Controller | Topic | Message와 field |
|---|---|---|
| `left_joint_position_controller` | `/left_joint_position_controller/command` | `JointPositionCommand`: `target_position_rad[16]` |
| `left_joint_impedance_controller` | `/left_joint_impedance_controller/command` | `JointImpedanceCommand`: `target_position_rad[16]` |
| `left_actuator_position_controller` | `/left_actuator_position_controller/command` | `ActuatorPositionCommand`: `target_position_cnt[16]` |
| `left_actuator_effort_controller` | `/left_actuator_effort_controller/command` | `ActuatorEffortCommand`: `target_effort_pct[16]` |

모든 subscription은 `SystemDefaultsQoS`입니다. Basic controller는 NaN/Inf를 거부하고,
SDK는 actuator position의 int32 범위를 검증하고 joint target을 workspace 안으로 자동
clamp합니다. Effort 상한과 controller tuning(filter·impedance gain)은 command가 아니라
hardware node parameter입니다 — 아래 [Tuning parameter](#tuning-parameter) 참조.

네 basic controller는 각각 `~/command` 하나를 받습니다. 한 message가 완전한 한-cycle 입력이며
partial update는 허용하지 않습니다.

### Joint position

Default active controller입니다.

```bash
ros2 topic pub --once \
  /left_joint_position_controller/command \
  aidin_hand2_msgs/msg/JointPositionCommand \
  "{target_position_rad: [
      0.0, 0.0, 0.0, 0.0,
      0.10, 0.20, 0.0,
      0.0, 0.20, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0]}"
```

Target 단위는 rad입니다. 목표 filter 는 hardware node parameter
(`joint_position_controller.cutoff_freq`·`deadband`)가 정하고 command에는 들어가지 않습니다.
SDK `set_command()`가 joint target을 reachable workspace 안으로 자동 clamp합니다.

### Joint impedance

먼저 현재 controller와 원자적으로 전환합니다.

```bash
ros2 control load_controller \
  --set-state inactive left_joint_impedance_controller
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_controller \
  --activate left_joint_impedance_controller
```

Target과 gain을 하나의 message로 보냅니다.

```bash
ros2 topic pub --once \
  /left_joint_impedance_controller/command \
  aidin_hand2_msgs/msg/JointImpedanceCommand \
  "{target_position_rad: [
      0.0, 0.0, 0.0, 0.0,
      0.10, 0.20, 0.0,
      0.0, 0.20, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0]}"
```

Gain은 command가 아니라 `joint_impedance_controller.stiffness`·`damping` parameter입니다
(actuator encoder-space PD gain, 유한·0 이상). Joint target은 SDK에서 joint-position과 같은
방식으로 자동 clamp됩니다.

### Actuator position

Raw actuator mode입니다. 먼저 실물 `HandState.actuator_position`에서 현재 count 16개를 읽고
그 근처의 작은 차이로 시작하십시오. 0 count를 일반 예제로 복사하지 마십시오.

```bash
ros2 control load_controller \
  --set-state inactive left_actuator_position_controller
ros2 control switch_controllers --strict \
  --deactivate left_joint_impedance_controller \
  --activate left_actuator_position_controller

ros2 topic pub --once \
  /left_actuator_position_controller/command \
  aidin_hand2_msgs/msg/ActuatorPositionCommand \
  "{target_position_cnt: [
      CURRENT_CNT_0, CURRENT_CNT_1, CURRENT_CNT_2, CURRENT_CNT_3,
      CURRENT_CNT_4, CURRENT_CNT_5, CURRENT_CNT_6, CURRENT_CNT_7,
      CURRENT_CNT_8, CURRENT_CNT_9, CURRENT_CNT_10, CURRENT_CNT_11,
      CURRENT_CNT_12, CURRENT_CNT_13, CURRENT_CNT_14, CURRENT_CNT_15]}"
```

단위는 absolute encoder count입니다. 모든 값은 finite이며 int32 범위 안이어야 합니다.

### Actuator effort

```bash
ros2 control load_controller \
  --set-state inactive left_actuator_effort_controller
ros2 control switch_controllers --strict \
  --deactivate left_actuator_position_controller \
  --activate left_actuator_effort_controller

ros2 topic pub --once \
  /left_actuator_effort_controller/command \
  aidin_hand2_msgs/msg/ActuatorEffortCommand \
  "{target_effort_pct: [
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0]}"
```

단위는 rated current percent이고 부호가 방향입니다. SDK가 per-actuator max effort로
절댓값을 제한합니다.

## 4. Chainable reference interface

아래 suffix 앞에는 export한 basic controller 이름이 붙습니다. 예를 들어 첫 joint-position
resource 전체 이름은
`left_joint_position_controller/left_thumb_joint0/position`입니다.

| Basic controller | Exported reference |
|---|---|
| JointPosition | `{side}_{active_joint}/position` ×16 |
| JointImpedance | `{side}_{active_joint}/position` ×16 |
| ActuatorPosition | `{side}_{actuator}/position_cnt` ×16 |
| ActuatorEffort | `{side}_{actuator}/effort_pct` ×16 |

상위 controller가 reference 중 하나라도 만들면 해당 mode의 reference 전체를 같은 update에서
유한한 값으로 써야 합니다. Chained mode에서는 basic controller의 standalone topic이
reference source가 아닙니다.

네 상위 controller skeleton과 controller별 YAML은
[aidin_hand2_examples/EXAMPLE.md](../../aidin_hand2_examples/EXAMPLE.md)에 있습니다. 네
skeleton 모두 전체 `HandState`를 realtime buffer에서 멤버로 복사하고, 기본 상태에서는
아무 reference 값도 만들지 않습니다.

## 5. Mode 전환

Command controller는 한 순간 하나만 active여야 합니다. Hardware가 mode별로 서로 다른 command
interface를 claim하기 때문입니다. 이전 controller deactivate와 새 controller activate를 같은
strict transaction으로 수행합니다.

```bash
ros2 control switch_controllers --strict \
  --deactivate left_actuator_effort_controller \
  --activate left_joint_position_controller
```

Partial claim, mixed claim, 두 mode 동시 활성화는 real과 mock 모두 거부합니다. 전환 후
controller state를 확인하고 안전한 complete command를 보냅니다.

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

Runtime schema는 `ros2 control list_hardware_interfaces`로 확인하십시오.

## 8. State·diagnostics topic

| Topic | Type | 제공 YAML rate | 주요 내용 |
|---|---|---:|---|
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | 표준 joint position 21개 |
| `/left_hand_state_broadcaster/hand_state` | `aidin_hand2_msgs/HandState` | 100 Hz | Joint·actuator·tactile·nested `CommandState` |
| `/left_diagnostics_broadcaster/hand_diagnostics` | `aidin_hand2_msgs/HandDiagnostics` | 20 Hz | Lifecycle, homing_state, RT 통계, actuator health |
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

### CommandState 읽기

실물 `HandState`에는 SDK same-cycle command conversion record가 들어 있습니다.

```bash
ros2 topic echo \
  /left_hand_state_broadcaster/hand_state \
  --once --field command_state
```

- `controller_input_mode`가 유효한 nested input을 정합니다.
- `controller_output_type`이 `target_position_cnt` 또는 `target_effort_pct`의 유효성을 정합니다.
- `selected_source`가 `CONTROLLER`, `QUICK_STOP`, `HOMING`, `NONE` 중 실제 source를 나타냅니다.
- `max_effort_pct`는 conversion에 사용한 actuator별 상한입니다.

Joint input echo는 SDK 자동 workspace clamp 뒤 값입니다. Controller output은 변환 결과이지
drive가 실제 수신·적용했다는 확인은 아닙니다. `transmit_succeeded` field는 없습니다.

## 9. Runtime service

실제 hardware만 다음 `std_srvs/srv/Trigger` service를 제공합니다.

| Service | 성공 message | 계약 |
|---|---|---|
| `/left_hand_control/run` | `running` | Drive enable 또는 stop 뒤 재개 |
| `/left_hand_control/stop` | `stopped` | Blocking quick stop |
| `/left_hand_control/home` | `homing started — poll diagnostics 'homing_state'` | `start_homing()` non-blocking trigger |
| `/left_hand_control/reconnect` | `reconnected — call ~/run to resume control` | 통신만 복구, 별도 `run` 필요 |

Service node는 hardware component와 별도 single-thread executor를 사용합니다. `home` 성공은
시작 접수만 뜻하며 완료는 `HandDiagnostics.homing_state`로 확인합니다.

### 호출 예시

실제 hardware:

```bash
ros2 service call /left_hand_control/run std_srvs/srv/Trigger "{}"
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger "{}"
ros2 service call /left_hand_control/home std_srvs/srv/Trigger "{}"
ros2 service call /left_hand_control/reconnect std_srvs/srv/Trigger "{}"
```

- `stop`은 blocking quick stop입니다.
- `home`은 non-blocking trigger입니다. 완료는 diagnostics의 `homing_state=Succeeded`로 확인합니다.
- `reconnect`는 통신만 복구합니다. 성공 뒤 `run`을 별도로 호출합니다.

### Tuning parameter

Effort 상한과 controller tuning은 hardware component가 자체 node로 노출하는 parameter입니다
(node 이름 = xacro의 `<ros2_control name>`). 초기값은 `controllers.yaml`에 두고, 런타임에는
`ros2 param set`으로 바꿉니다 — 다음 cycle부터 적용됩니다.

| Parameter | 타입 | 기본값 | 뜻 |
|---|---|---|---|
| `max_effort` | `double[]` | `1000.0` ×16 | rated current %(1000 = 100%). SDK가 `[0, 2000]`으로 clamp |
| `joint_position_controller.filter_enabled` | `bool` | `true` | `false`면 filter 없이 target 즉시 반영 |
| `joint_position_controller.cutoff_freq` | `double` | `60.0` | 저역통과 cutoff [Hz]. 상위 발행 rate의 절반 이하 |
| `joint_position_controller.deadband` | `double` | `0.000873` | Trailing deadband [rad] (0.05°) |
| `joint_impedance_controller.stiffness` | `double[]` | thumb `0.02`, long finger a1·a2 `0.01`, a3 `0.02` | 강성 K (actuator별) |
| `joint_impedance_controller.damping` | `double[]` | `0.00001` ×16 | 감쇠 D (actuator별) |

배열은 actuator 16개를 순서대로 채우고, 전 actuator를 같은 값으로 둘 때만 길이 1로 줄일 수
있습니다. 그 외 길이거나 non-finite·음수면 `ros2 param set`이 실패하고 값이 반영되지 않습니다.

아래는 여섯 parameter를 모두 기본값으로 다시 쓰는 예입니다.

```bash
ros2 param list /left_hand_control

ros2 param set /left_hand_control max_effort \
  "[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0,
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]"

ros2 param set /left_hand_control joint_position_controller.filter_enabled true
ros2 param set /left_hand_control joint_position_controller.cutoff_freq 60.0
ros2 param set /left_hand_control joint_position_controller.deadband 0.000873

ros2 param set /left_hand_control joint_impedance_controller.stiffness \
  "[0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.02, 0.01,
    0.01, 0.02, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02]"
ros2 param set /left_hand_control joint_impedance_controller.damping \
  "[0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
    0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]"
```

값에 소수점을 붙여야 `double_array`로 파싱됩니다 — `[300]`은 integer array라 거부됩니다.

## 10. Mock 범위

Mock은 real과 동일한 98개 command port, 같은 exact mode switch, actuator physical state
48개와 joint position state 21개를 제공합니다. 네 basic controller를 모두 load할 수
있지만 default launch는 `joint_state_broadcaster`와
`left_joint_position_controller`만 spawn합니다.

Mock은 CAN, drive, homing, tactile, hardware diagnostics, command echo와 runtime service를
제공하지 않습니다. 따라서 default mock에서는 `HandStateBroadcaster`와
`DiagnosticsBroadcaster`를 spawn하지 않습니다.

이름이 같아도 값이 다르게 동작하는 state가 있습니다.

| 항목 | Mock 동작 |
|---|---|
| `velocity_rpm`·`current_ma` | 항상 0 (actuator state 48개 중 32개) |
| Joint position | clamp → IK → FK. filter 없이 target 즉시 반영 |
| Actuator position | count를 정수로 반올림한 뒤 FK |
| Actuator effort | `max_effort`로 clamp. pose는 변하지 않음 |
| `read()` | no-op. state는 `write()`에서만 갱신 — command controller가 없으면 `/joint_states`가 고정값 |
| Max effort | 정적 xacro `max_effort`로 clamp만. tuning parameter 없음 |

## 11. NaN과 validity

`CommandState`에서 현재 mode/type이 선택하지 않은 nested input·output field는 NaN일 수
있습니다. `controller_input_mode`, `controller_output_type`, `selected_source`를 validity
discriminator로 사용하십시오.

> [!WARNING]
> NaN을 0으로 바꾸면 "해당 없음"과 실제 target 0을 구분할 수 없습니다. Strict JSON, database,
> ML pipeline은 nullable 또는 validity mask를 쓰십시오.

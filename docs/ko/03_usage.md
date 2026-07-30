# 사용법

아래 예시는 왼손 default 이름을 사용합니다. 오른손은 controller·topic의 `left`를
`right`로 바꿉니다.

> [!WARNING]
> Command는 실제 hand를 움직입니다. 실물에서는 `Running`, `homed=true`, empty fault와
> 낮은 max effort를 확인한 뒤 실행하십시오. Application command timeout은 없습니다.

## 1. Runtime 확인

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

실물에서는 상태와 진단도 확인합니다.

```bash
ros2 topic echo \
  /left_hand_state_broadcaster/hand_state --once
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics --once
```

Default mock은 `joint_state_broadcaster`와
`left_joint_position_controller`만 active입니다.

## 2. Lifecycle service와 max effort

실제 hardware:

```bash
ros2 service call /left_hand_control/run std_srvs/srv/Trigger "{}"
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger "{}"
ros2 service call /left_hand_control/home std_srvs/srv/Trigger "{}"
ros2 service call /left_hand_control/reconnect std_srvs/srv/Trigger "{}"
```

- `stop`은 blocking quick stop입니다.
- `home`은 non-blocking trigger입니다. 완료는 diagnostics의 `homed=true`로 확인합니다.
- `reconnect`는 통신만 복구합니다. 성공 뒤 `run`을 별도로 호출합니다.

모든 actuator 공통 effort 상한:

```bash
ros2 topic pub --once \
  /left_hand_control/set_max_effort \
  std_msgs/msg/Float64 \
  "{data: 300.0}"
```

단위는 rated current percent이며 SDK가 `[0, 2000]`으로 clamp합니다.

## 3. Standalone typed command

네 basic controller는 각각 `~/command` 하나를 받습니다. 한 message가 target과 speed/gain을
포함한 완전한 한-cycle 입력이며 partial update는 허용하지 않습니다.

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
      0.0, 0.0, 0.0],
    speed_rad_s: 0.25}"
```

Target 단위는 rad, speed는 rad/s입니다. `speed_rad_s: 0`은 정지가 아니라 slew 제한 없는
즉시 추종입니다. SDK `set_command()`가 joint target을 reachable workspace 안으로 자동
clamp합니다.

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
      0.0, 0.0, 0.0],
    stiffness: [
      0.02, 0.02, 0.02, 0.02,
      0.01, 0.01, 0.02,
      0.01, 0.01, 0.02,
      0.01, 0.01, 0.02,
      0.01, 0.01, 0.02],
    damping: [
      0.00001, 0.00001, 0.00001, 0.00001,
      0.00001, 0.00001, 0.00001,
      0.00001, 0.00001, 0.00001,
      0.00001, 0.00001, 0.00001,
      0.00001, 0.00001, 0.00001]}"
```

Stiffness와 damping은 actuator encoder-space PD gain이며 모두 유한하고 0 이상이어야 합니다.
Joint target은 SDK에서 joint-position과 같은 방식으로 자동 clamp됩니다.

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

## 4. Mode 전환 규칙

Basic command controller는 한 순간 하나만 active여야 합니다. 이전 controller deactivate와
새 controller activate를 같은 strict transaction으로 수행합니다.

```bash
ros2 control switch_controllers --strict \
  --deactivate left_actuator_effort_controller \
  --activate left_joint_position_controller
```

Hardware는 `command_lock`과 mode별 complete port를 검증하므로 partial claim, mixed claim,
두 mode 동시 활성화를 거부합니다. 전환 후 controller state를 확인하고 안전한 complete
command를 보냅니다.

## 5. Chaining

하위 basic controller를 먼저 활성화하고 상위 controller를 나중에 활성화합니다. 종료 순서는
반대입니다. 상위가 하위 reference를 claim하면 basic controller의 standalone topic 대신
chained reference가 입력이 됩니다.

네 basic controller별 빈 상위 skeleton, reference 전체 이름, YAML과 수정 지점은
[Chainable controller examples](../../aidin_hand2_examples/EXAMPLE.md)에 있습니다.

Skeleton은 모두 다음 규칙을 따릅니다.

- 전체 `HandState`를 realtime buffer에서 `hand_state_`에 복사합니다.
- 기본 상태에서는 reference를 만들지 않아 하위의 activation seed를 유지합니다.
- 하나라도 출력하기 시작하면 해당 mode의 reference 전체를 한 update에서 유한한 값으로
  채워야 합니다.
- 자신도 같은 shape의 reference를 export하므로 위에 한 단계를 더 연결할 수 있습니다.

## 6. CommandState 읽기

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

## 7. Stop과 재개

Command controller만 deactivate해도 hardware와 SDK `Running`은 유지될 수 있습니다. Drive를
실제로 stop하려면 service 또는 hardware lifecycle을 사용합니다.

```bash
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_controller
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger "{}"
```

재개할 때는 `run`, `homed` 상태 확인, controller 활성화, 새 complete command 승인 순서로
진행합니다.

```bash
ros2 service call /left_hand_control/run std_srvs/srv/Trigger "{}"
ros2 control switch_controllers --strict \
  --activate left_joint_position_controller
```

## 8. Example package 구성

`aidin_hand2_examples`는 controller 성격별로 나뉘어 있습니다.

| 성격 | source | config |
|---|---|---|
| 상위 controller skeleton 4종 | `src/upper_controllers/` | `config/upper_controllers/` |
| 글러브 텔레오퍼 controller | `src/glove_teleop/` | `config/glove_teleop/` |

Optional MANUS integration은 `manus_ros2_msgs`와 별도 data publisher가 있을 때만 build·동작합니다.
캘리브 스크립트는 `ros2 run aidin_hand2_examples glove_calibrate`입니다.

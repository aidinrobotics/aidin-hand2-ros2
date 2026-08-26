# Chainable controller examples

이 파일은 기존 basic controller 위에 사용자 controller를 연결하는 기준 예제입니다.
basic controller의 class/plugin 이름은 그대로 유지되며, hardware command port로 가기 직전의
adapter 역할을 합니다.

```text
사용자 알고리즘 또는 JTC
  → basic controller reference
  → 기존 basic controller
  → complete hardware command port + command_lock
  → real/mock hardware
```

## 제공 파일

성격이 같은 템플릿이라 source는 `src/upper_controllers/` 한 곳에 모으고, 파라미터는 controller별로
나눠 둡니다.

| 상위 skeleton (plugin class) | source | config | 연결할 basic controller |
|---|---|---|---|
| `aidin_hand2_examples/JointPositionUpperController` | `src/upper_controllers/joint_position_upper_controller.cpp` | `config/upper_controllers/joint_position_upper.yaml` | `JointPositionController` |
| `aidin_hand2_examples/JointImpedanceUpperController` | `src/upper_controllers/joint_impedance_upper_controller.cpp` | `config/upper_controllers/joint_impedance_upper.yaml` | `JointImpedanceController` |
| `aidin_hand2_examples/ActuatorPositionUpperController` | `src/upper_controllers/actuator_position_upper_controller.cpp` | `config/upper_controllers/actuator_position_upper.yaml` | `ActuatorPositionController` |
| `aidin_hand2_examples/ActuatorEffortUpperController` | `src/upper_controllers/actuator_effort_upper_controller.cpp` | `config/upper_controllers/actuator_effort_upper.yaml` | `ActuatorEffortController` |

네 파일은 독립적으로 복사해 수정할 수 있는 템플릿이며 claim/export하는 reference shape만
각 basic controller에 맞게 다릅니다. `command_lock` 때문에 서로 다른 mode의 basic controller 둘을
동시에 active로 만들 수 없으므로, config 네 개 중 하나만 선택해 spawn합니다.

## HandState 입력

네 템플릿 모두 `hand_state_topic`을 구독하고 realtime buffer를 거쳐 최신
`aidin_hand2_msgs::msg::HandState` 전체를 멤버 `hand_state_`에 복사합니다. 따라서 다음 데이터가
모두 한 변수에 보존됩니다.

- header stamp와 hand side
- joint position 21개
- actuator position/velocity/current 각 16개
- finger/palm tactile 전체
- nested `CommandState`

`has_hand_state_`가 true일 때만 유효한 상태가 도착한 것입니다. 이 복사는 chained mode에서도
항상 실행되도록 `update_and_write_commands()` 맨 앞에 있습니다. 기본 topic은
`/<side>_hand_state_broadcaster/hand_state`이며 YAML에서 바꿀 수 있습니다.

## 의도적으로 아무 값도 만들지 않는 동작

skeleton은 알고리즘 예제가 아니라 안전한 구조 템플릿입니다.

- `on_activate()`는 자기 export reference를 NaN으로 초기화합니다.
- `update_reference_from_subscribers()`는 명령 입력을 만들지 않습니다.
- `update_and_write_commands()`는 HandState 전체를 저장한 뒤 reference 전체가 유한할 때만
  한 묶음으로 하위 controller에 전달합니다. 일부만 유한한 입력은 오류로 거부합니다.
- 아무 입력도 없으면 하위 basic controller가 NaN(= 이번 cycle 명령 없음)을 그대로 내보내고
  hardware는 SDK로 아무것도 보내지 않습니다. 손은 직전 명령 자세를 유지합니다.

즉 파일을 그대로 활성화해도 새 target을 만들지 않습니다. 실제 상위 controller를 만들 때
각 파일의 `TODO(user algorithm)` 위치에서 `hand_state_`를 읽고, 한 update에서 대응
reference 전체를 유한한 값으로 갱신하십시오.

## Reference shape

`target_controller`가 `left_joint_position_controller`일 때 전체 resource 이름 예시는
`left_joint_position_controller/left_thumb_joint0/position`입니다.

| mode | suffix |
|---|---|
| JointPosition | `left_<active_joint>/position` ×16 |
| JointImpedance | `left_<active_joint>/position` ×16 |
| ActuatorPosition | `left_<actuator>/position_cnt` ×16 |
| ActuatorEffort | `left_<actuator>/effort_pct` ×16 |

오른손은 `hand_side: right`와 오른손 basic controller 이름을 사용합니다.

## Skeleton 실행

mock을 먼저 띄우면 joint position basic controller가 active입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_rviz:=false
```

다른 terminal:

```bash
source install/setup.bash
EXAMPLE_SHARE="$(ros2 pkg prefix aidin_hand2_examples)/share/aidin_hand2_examples"

ros2 run controller_manager spawner left_joint_position_upper \
  --inactive \
  --controller-manager /controller_manager \
  --controller-type aidin_hand2_examples/JointPositionUpperController \
  --param-file "$EXAMPLE_SHARE/config/upper_controllers/joint_position_upper.yaml"

ros2 control switch_controllers --strict \
  --activate left_joint_position_upper

ros2 control list_controllers
```

`left_joint_position_upper`가 하위 reference를 claim하면
`left_joint_position_controller`는 chained mode가 됩니다. skeleton이 값을 만들지 않으므로
손은 하위 activation seed를 유지합니다.

다른 mode skeleton을 시험하려면 기존 상위를 먼저 내리고, 기존 basic controller와 새 basic
controller를 원자 전환한 뒤, 새 상위를 마지막에 올립니다. `command_lock` 때문에 서로 다른 mode의
basic controller 둘을 동시에 active로 만들 수 없습니다.

내릴 때는 항상 상위부터 내립니다.

```bash
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_upper
ros2 control unload_controller left_joint_position_upper
```

## 구현 체크리스트

- `command_interface_configuration()`에서 lower reference 이름을 정확히 claim합니다.
- 자신도 같은 shape의 reference를 export하면 한 단계 더 위로 chain할 수 있습니다.
- subscriber callback은 realtime buffer에 완전한 command만 기록합니다.
- `HandState` subscriber callback은 계산하지 않고 realtime buffer에 최신 message만 기록합니다.
- update에서 NaN/Inf와 음수 speed/gain을 거부합니다.
- activate 시 reference를 NaN으로 두고 목표를 만들지 않습니다.
- mode 전환은 basic controller 단위로 하고 hardware command interface를 직접 부분 claim하지 않습니다.
- real과 mock에서 동일한 controller/config를 사용합니다.

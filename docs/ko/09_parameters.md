# Parameters

wrapper의 parameter는 세 층에 있습니다. URDF의 `ros2_control` 매크로 parameter는 hardware component를
만들 때 한 번 정해지고, hardware node의 parameter는 runtime에 `ros2 param set`으로 바꾸며, controller
parameter는 `controllers.yaml`에 둡니다. 이 문서는 세 층의 이름, 타입, 기본값, 유효 범위를 설명합니다.

`{side}`는 `left` 또는 `right`이고 배열 16개의 순서는 [Controllers](06_controllers.md) 2장에 있습니다.

## Contents

&nbsp;&nbsp;[**1. Macro parameters**](#1-macro-parameters)<br>
&nbsp;&nbsp;[**2. Hardware node parameters**](#2-hardware-node-parameters)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Max effort](#21-max-effort)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Joint position controller](#22-joint-position-controller)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Joint impedance controller](#23-joint-impedance-controller)<br>
&nbsp;&nbsp;[**3. Controller parameters**](#3-controller-parameters)

## 1. Macro parameters

매크로 parameter는 `aidin_hand2_ros2_control` 매크로가 URDF의 `<hardware>` 요소에 `<param>`으로 내보내는
값이고, hardware component가 `on_init`에서 한 번 읽습니다. 바꾸려면 URDF를 다시 만들어 launch합니다.
`name`부터 `can_interface`까지 넷이 필수이고 나머지는 기본값이 있습니다.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `name` | string | 필수 | hardware component 이름. service node 이름과 2장의 parameter 블록 이름으로도 쓰입니다 |
| `prefix` | string | 필수 | joint·actuator·interface 이름 접두어. `left_` 또는 `right_` |
| `hand_side` | string | 필수 | `left` 또는 `right`. CAN ID와 kinematics를 결정합니다 |
| `can_interface` | string | 필수 | CAN interface 이름. `auto`는 side의 CAN ID로 채널을 탐색합니다 |
| `auto_home` | bool | `true` | `~/run` service 성공 뒤 `homing_state` 값이 `Succeeded`가 아니면 homing을 한 번 시작합니다 |
| `max_effort` | double | `1000` | effort 상한의 초깃값. 단위는 정격 전류의 0.1%이고 `1000`이 100%. runtime 값은 [2.1](#21-max-effort) |
| `control_rate` | int | `500` | 제어·통신 루프 주기 [Hz]. controller_manager의 `update_rate` 값과 같게 둡니다 |
| `rt_cpu_affinity` | int | `-1` | 제어·통신 루프 thread를 고정할 CPU 번호. `-1`은 고정 안 함 |
| `disabled_actuators` | string | `''` | 사용하지 않을 actuator index. `"0,1,2,3"`처럼 쉼표로 구분하고 공백은 무시합니다 |
| `auto_reconnect` | bool | `false` | 통신 오류가 발생하면 SDK가 재연결을 반복 시도합니다 |
| `auto_reconnect_timeout_ms` | int | `0` | 자동 재연결 제한 시간 [ms]. `0`은 제한 없음 |
| `auto_reconnect_home` | bool | `false` | 자동 재연결 뒤 `Running`으로 복귀하기 전에 homing을 수행합니다 |
| `use_mock` | bool | `false` | CAN 대신 kinematics mock backend를 씁니다 |
| `use_isaac` | bool | `false` | CAN 대신 Isaac Sim backend를 씁니다. `use_mock`보다 우선합니다 |
| `isaac_topic_prefix` | string | `/isaac` | Isaac topic 접두어 |
| `isaac_joint_state_topic` | string | `joint_states` | Isaac → wrapper state topic. 접두어 뒤에 붙습니다 |
| `isaac_joint_command_topic` | string | `hand_command` | wrapper → Isaac command topic. 접두어 뒤에 붙습니다 |
| `isaac_tactile_prefix` | string | `tactile` | tactile topic 접두어. 접두어 뒤에 붙습니다 |
| `isaac_state_timeout` | double | `0.1` | 이 시간[s]보다 오래 state가 끊기면 `lifecycle` 값이 `Disconnected`. `0` 이하는 검사 안 함 |

`control_rate`부터 `auto_reconnect_home`까지는 SDK `HandConfig`의 같은 이름 필드에 그대로 전달됩니다.
`auto_home` parameter는 wrapper가 SDK 대신 처리하고([Services](07_services.md) 3장), `max_effort`
parameter는 2장 hardware node parameter의 초깃값이 되어 `set_max_effort()`로 SDK에 전달됩니다.
`hand_side` parameter가 `left`·`right`가 아니거나 `can_interface` parameter가 비어 있으면 `on_init`이
실패합니다.

`disabled_actuators` parameter에 지정한 actuator는 enable하지 않고, fault로 보고하지 않으며, homing에서도
제외합니다. 물리적으로 없는 actuator나 고장으로 당장 쓰지 않을 actuator를 뺄 때 쓰며, 되도록 finger
단위로 지정합니다. index와 actuator 이름의 대응은 다음과 같습니다.

```text
0..3    thumb_actuator0..3
4..6    index_actuator1..3
7..9    middle_actuator1..3
10..12  ring_actuator1..3
13..15  baby_actuator1..3
```

## 2. Hardware node parameters

hardware node parameter는 hardware component가 자기 node(`/{side}_hand_control`)에 선언하는 runtime 설정
6개입니다. command가 drive 전송값으로 바뀌는 방식을 결정하고, 값을 바꾸면 다음 cycle부터 적용됩니다.
lifecycle과 무관하게 언제든 바꿀 수 있고 `~/reconnect` service 뒤에도 유지됩니다.

| Parameter | Type | Default | Valid |
|---|---|---|---|
| `max_effort` | `double[]` | `[1000.0]` | 길이 1 또는 16. 유한하고 `0` 이상 |
| `joint_position_controller.filter_enabled` | `bool` | `true` | `true` 또는 `false` |
| `joint_position_controller.cutoff_freq` | `double` | `10.0` | 유한하고 `0` 이상 |
| `joint_position_controller.deadband` | `double` | `0.000873` | 유한하고 `0` 이상 |
| `joint_impedance_controller.stiffness` | `double[]` | thumb `0.02`, 나머지 finger는 `0.01`·`0.01`·`0.02` | 길이 1 또는 16. 유한하고 `0` 이상 |
| `joint_impedance_controller.damping` | `double[]` | `1e-5` × 16 | 길이 1 또는 16. 유한하고 `0` 이상 |

배열 parameter는 길이 1이면 actuator 16개에 같은 값을, 길이 16이면 actuator마다 다른 값을 적용합니다. 다른
길이거나 값이 유효하지 않으면 `ros2 param set`이 실패하고 어느 값도 바뀌지 않습니다.

초깃값은 `controllers.yaml`에 `name`과 같은 이름의 블록으로 줍니다. 블록이 없으면 위 기본값이
적용됩니다. 다음은 여섯 parameter를 기본값으로 적은 블록입니다.

```yaml
left_hand_control:
  ros__parameters:
    max_effort: [1000.0]
    joint_position_controller:
      filter_enabled: true
      cutoff_freq: 10.0
      deadband: 0.000873
    joint_impedance_controller:
      stiffness: [0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02]
      damping: [1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5]
```

선언된 parameter와 현재 값은 다음으로 읽습니다.

```bash
ros2 param list /left_hand_control
ros2 param get /left_hand_control max_effort
```

### 2.1 Max effort

`max_effort` parameter는 actuator로 전송되는 effort의 상한입니다. 단위는 정격 전류의 0.1%이고 정격 전류는
모든 모터가 400 mA이므로 `1000`(100%)이 400 mA입니다. SDK가 `[0, 2000]` 밖의 값을 `[0, 2000]`으로
제한합니다.

`ros2 param set`으로 쓸 때는 값에 소수점을 붙여야 `double` 배열로 해석됩니다. `[1000]`은 integer 배열이라
거부됩니다. 다음은 actuator 전체에 같은 값을 주는 예입니다.

```bash
ros2 param set /left_hand_control max_effort "[800.0]"
```

다음은 actuator마다 다르게 주는 예이고 값은 예시입니다.

```bash
ros2 param set /left_hand_control max_effort \
  "[1000.0, 1000.0, 1000.0, 1000.0,
     800.0,  800.0,  800.0,
     800.0,  800.0,  800.0,
     800.0,  800.0,  800.0,
     800.0,  800.0,  800.0]"
```

적용 중인 값은 `hand_state` topic의 `command_state.max_effort_pct` 필드로 확인합니다.

### 2.2 Joint position controller

`joint_position_controller` parameter 셋은 SDK의 joint position controller 설정입니다. `JointPositionCommand`의
목표는 매 cycle 다음 세 단계를 거쳐 actuator position이 됩니다.

1. 직전에 통과한 목표에서 `deadband` 값 이내로 움직인 입력은 무시합니다.
2. 3차 low-pass filter가 적용됩니다. `cutoff_freq` 값보다 빠른 변화가 줄어듭니다.
3. inverse kinematics로 actuator position이 됩니다.

| Parameter | Description |
|---|---|
| `filter_enabled` | `false`면 1·2단계를 모두 생략하고 목표가 즉시 반영됩니다 |
| `cutoff_freq` | 차단 주파수 [Hz]. `0`이면 2단계만 생략 |
| `deadband` | 무시할 변화의 크기 [rad]. 기본값 0.000873 rad는 0.05°. `0`이면 1단계만 생략 |

`cutoff_freq` 값은 목표가 실제로 갱신되는 주기를 기준으로 결정합니다. `~/command` topic을 발행하는 주기가
기준이고, 발행 주기의 절반보다 높이면 발행 사이의 계단이 그대로 전송됩니다. 낮은 값에서 시작해 올려 가며
조정하십시오. 낮추면 지연이 커지고 높이면 filter 효과가 줄어듭니다. 어느 값에서도 overshoot은 발생하지
않습니다.

`deadband` parameter가 필요한 이유는 감속비가 크기 때문입니다. joint에서 미세한 변동이라도 actuator 축에서는
큰 반전이 되어 backlash 구간을 반복해서 통과합니다. 입력에 섞인 변동의 크기를 확인하며 조정하십시오.

다음은 차단 주파수를 20 Hz로 바꾸는 예입니다.

```bash
ros2 param set /left_hand_control joint_position_controller.cutoff_freq 20.0
```

### 2.3 Joint impedance controller

`joint_impedance_controller` parameter 둘은 SDK의 joint impedance controller gain입니다.
`JointImpedanceCommand`의 목표를 inverse kinematics로 actuator position으로 바꾼 뒤, encoder 공간에서
다음 식으로 effort를 산출합니다.

```text
effort = stiffness × position_error − damping × velocity
```

두 gain은 joint가 아니라 actuator 공간의 값이고 actuator 16개에 대응합니다.

| Parameter | Description |
|---|---|
| `stiffness` | 위치 오차에 곱하는 gain |
| `damping` | 속도에 곱하는 gain |

다음은 actuator 16개에 같은 stiffness를 주는 예입니다.

```bash
ros2 param set /left_hand_control joint_impedance_controller.stiffness "[0.02]"
```

joint impedance controller의 사용 보류는 [3.1 Command controllers](06_controllers.md#31-command-controllers)에
있습니다.

## 3. Controller parameters

controller parameter는 `controllers.yaml`의 controller 이름 블록에 둡니다.

| Controller | Parameter | Type | Default | Description |
|---|---|---|---|---|
| command controller 4종 | `hand_side` | `string` | 필수 | `left` 또는 `right`. interface 이름의 접두어를 결정합니다 |
| `HandStateBroadcaster` | `hand_side` | `string` | 필수 | `left` 또는 `right`. interface 이름의 접두어를 결정합니다 |
| `HandStateBroadcaster` | `update_rate` | `int` | controller_manager 주기 | 발행 주기 [Hz]. `controllers.yaml`은 `100` |
| `DiagnosticsBroadcaster` | `hand_side` | `string` | 필수 | `left` 또는 `right`. interface 이름의 접두어를 결정합니다 |
| `DiagnosticsBroadcaster` | `update_rate` | `int` | controller_manager 주기 | 발행 주기 [Hz]. `controllers.yaml`은 `20` |
| `joint_state_broadcaster` | `update_rate` | `int` | controller_manager 주기 | 발행 주기 [Hz]. `controllers.yaml`은 `100` |

`hand_side` parameter는 `prefix`가 아니라 side 값입니다. `left_`를 주면 controller가 configure에서
실패합니다.

broadcaster의 `update_rate` parameter를 지정하지 않으면 controller_manager 주기(500 Hz)로 발행됩니다.
GUI와 RViz는 사람이 보는 용도이고, 500 Hz로 양손의 `HandState`를 발행하면 rosbridge가 포화되므로
`controllers.yaml`은 100 Hz와 20 Hz로 낮춰 둡니다.

`aidin_hand2_examples` package의 상위 controller skeleton은 `hand_side` 외에 `target_controller`와
`hand_state_topic` parameter를 받습니다. `target_controller` parameter는 아래에 둘 command controller의
이름이고, `hand_state_topic` parameter는 구독할 `HandState` topic이며 비우면
`/{side}_hand_state_broadcaster/hand_state`입니다.

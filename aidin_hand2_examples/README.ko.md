# aidin_hand2_examples

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

`aidin_hand2_examples`는 command controller 위에 사용자 제어 알고리즘을 연결하는
chainable 상위 controller skeleton 4개를 제공합니다. 일반 node에서 목표값을 보낼 때는
[aidin_hand2_controllers](../aidin_hand2_controllers/README.ko.md)의 command topic을 사용하면 됩니다.

skeleton은 입력받은 목표값을 하위 controller의 reference로 전달하며 자체 목표를 생성하지 않습니다.
mock에서도 실행할 수 있습니다. 먼저 [Bringup](../docs/ko/04_bringup.md#1-mock)으로 mock 동작을 확인하십시오.

## Contents

&nbsp;&nbsp;[**1. Files**](#1-files)<br>
&nbsp;&nbsp;[**2. Configuration**](#2-configuration)<br>
&nbsp;&nbsp;[**3. Run and extend**](#3-run-and-extend)

## 1. Files

소스와 설정은 이 저장소의 `aidin_hand2_examples` package 안에 있습니다.
사용할 하위 controller와 같은 제어 방식의 skeleton을 선택합니다.

| Mode | Source | Config |
|---|---|---|
| Joint position | [joint_position_upper_controller.cpp](src/upper_controllers/joint_position_upper_controller.cpp) | [joint_position_upper.yaml](config/upper_controllers/joint_position_upper.yaml) |
| Joint impedance | [joint_impedance_upper_controller.cpp](src/upper_controllers/joint_impedance_upper_controller.cpp) | [joint_impedance_upper.yaml](config/upper_controllers/joint_impedance_upper.yaml) |
| Actuator position | [actuator_position_upper_controller.cpp](src/upper_controllers/actuator_position_upper_controller.cpp) | [actuator_position_upper.yaml](config/upper_controllers/actuator_position_upper.yaml) |
| Actuator effort | [actuator_effort_upper_controller.cpp](src/upper_controllers/actuator_effort_upper_controller.cpp) | [actuator_effort_upper.yaml](config/upper_controllers/actuator_effort_upper.yaml) |

> [!NOTE]
> joint impedance 제어는 SDK에서 개발 중이므로 로봇 핸드에서 사용하지 마십시오.

## 2. Configuration

선택한 YAML에서 상위 controller가 연결할 하위 controller와 상태 topic을 지정합니다.
아래는 제공된 왼손 joint position 설정입니다.

```yaml
controller_manager:
  ros__parameters:
    left_joint_position_upper:
      type: aidin_hand2_examples/JointPositionUpperController

left_joint_position_upper:
  ros__parameters:
    hand_side: left
    target_controller: left_joint_position_controller
    hand_state_topic: /left_hand_state_broadcaster/hand_state
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `hand_side` | `string` | 필수 | `left` 또는 `right` |
| `target_controller` | `string` | 필수 | 연결할 하위 command controller 이름 |
| `hand_state_topic` | `string` | 빈 문자열 | 비우면 `/{side}_hand_state_broadcaster/hand_state`를 구독합니다 |

mock에는 `HandState` 발행이 없습니다. mock에서 알고리즘을 시험하려면 관측값 없이도 동작하도록
구현하거나 별도의 상태 입력을 제공해야 합니다.

## 3. Run and extend

빌드된 skeleton을 실행하고 해제하는 명령은
[Running a skeleton](EXAMPLE.md#running-a-skeleton)에 있습니다.
상위 controller가 연결되면 하위 controller는 chained mode로 바뀌고 command topic 입력을 받지 않습니다.
목표 생성 코드를 추가하지 않은 skeleton을 활성화해도 새로운 목표값은 만들어지지 않습니다.

알고리즘을 구현할 때는 선택한 소스의 `Write the algorithm here` 위치를 수정합니다.
입력으로 쓸 상태와 출력할 reference 이름·단위는
[HandState input](EXAMPLE.md#handstate-input)과
[Reference shape](EXAMPLE.md#reference-shape)에서 확인하십시오.
제어 방식을 바꾸거나 종료할 때는 상위 controller를 먼저 비활성화합니다.

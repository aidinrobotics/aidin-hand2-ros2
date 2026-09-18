# aidin_hand2_examples

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

`aidin_hand2_examples`는 command controller 위에 사용자 제어 알고리즘을 연결하는
chainable 상위 controller skeleton 4개를 제공합니다. 일반 node에서 목표값을 보낼 때는
[aidin_hand2_controllers](../aidin_hand2_controllers/README.ko.md)의 command topic을 사용하면 됩니다.

skeleton은 자기 `~/cmd` topic에 `sensor_msgs/JointState`를 받아 0을 곱한 값을 하위 controller의 reference로
전달합니다. 알고리즘 자리를 표시하는 틀이고 mock에서도 실행할 수 있습니다. 먼저 [Bringup](../docs/ko/04_bringup.md#1-mock)으로 mock 동작을 확인하십시오.

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

선택한 YAML에서 상위 controller가 연결할 하위 controller와 tactile을 읽을지를 지정합니다.
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
    read_tactile: false
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `hand_side` | `string` | 필수 | `left` 또는 `right` |
| `target_controller` | `string` | 필수 | 연결할 하위 command controller 이름 |
| `read_tactile` | `bool` | `false` | `true`면 tactile state interface 143개도 claim합니다 |

skeleton은 joint 각도, actuator 위치·속도·전류를 state interface로 매 cycle 읽어 멤버 배열에 담습니다.
mock에는 tactile state interface가 없으므로 mock에서는 `read_tactile`을 `false`로 두고, 로봇 핸드에서
tactile을 쓰려면 `true`로 바꿉니다. 없는 interface를 claim하면 activate가 실패합니다.

## 3. Run and extend

빌드된 skeleton을 실행하고 해제하는 명령은
[Running a skeleton](EXAMPLE.md#running-a-skeleton)에 있습니다.
상위 controller가 연결되면 하위 controller는 chained mode로 바뀌고 자기 command topic 입력을 받지 않으며,
command는 skeleton의 `~/cmd` topic으로 보냅니다. 고치지 않은 skeleton은 입력에 0을 곱하므로 어떤 command를
보내도 0 자세가 됩니다.

알고리즘을 구현할 때는 선택한 소스의 `WRITE` 블록을 수정합니다.
입력으로 쓸 상태와 출력할 reference 이름·단위는
[State input](EXAMPLE.md#state-input)과
[Reference shape](EXAMPLE.md#reference-shape)에서 확인하십시오.
제어 방식을 바꾸거나 종료할 때는 상위 controller를 먼저 비활성화합니다.

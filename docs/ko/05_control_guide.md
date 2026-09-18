# Control guide

실행 중인 로봇 핸드의 상태를 확인하고, homing을 수행한 뒤 목표값을 보내는 방법을 설명합니다.
처음 실행한다면 [Bringup](04_bringup.md)으로 먼저 launch하십시오.
아래 예제는 왼손과 기본 node 이름을 사용합니다. `{side}`는 `left` 또는 `right`입니다.
launch는 계속 실행하고, 명령은 ROS와 workspace 환경을 적용한 다른 터미널에서 실행합니다.

## Contents

&nbsp;&nbsp;[**1. Lifecycle**](#1-lifecycle)<br>
&nbsp;&nbsp;[**2. Prepare the robot hand**](#2-prepare-the-robot-hand)<br>
&nbsp;&nbsp;[**3. Send a command**](#3-send-a-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 Joint position](#31-joint-position)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 Actuator position](#32-actuator-position)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.3 Actuator effort](#33-actuator-effort)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.4 Upper controller input](#34-upper-controller-input)<br>
&nbsp;&nbsp;[**4. Read state**](#4-read-state)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Topics](#41-topics)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Observe the result](#42-observe-the-result)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.3 Monitoring](#43-monitoring)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.4 QoS](#44-qos)<br>
&nbsp;&nbsp;[**5. Stop and recover**](#5-stop-and-recover)

## 1. Lifecycle

lifecycle은 로봇 핸드가 연결되어 있는지, 제어 중인지, 정지했는지 나타내는 상태입니다.
`hand_diagnostics.lifecycle`은 SDK가 보고하는 통신·제어 상태입니다.
`ros2 control`이 보여 주는 hardware component의 `active`와는 별개입니다.
component가 `active`여도 service로 정지했거나 통신 오류가 발생했다면 command를 적용하지 않습니다.

| State | Meaning | Next action |
|---|---|---|
| `Disconnected` | 연결되지 않은 상태 | launch의 연결 결과를 확인합니다 |
| `Connected` | 상태를 수신하지만 제어는 시작하지 않은 상태 | `~/run`을 호출합니다 |
| `Running` | actuator가 enable되어 제어 중인 상태 | homing 완료 후 command를 보냅니다 |
| `Stopped` | quick stop이 확인된 무토크 상태 | 제어를 재개하려면 `~/run`을 호출합니다 |
| `Faulted` | 통신 오류나 제어·통신 루프 예외로 제어가 종료된 상태 | 원인을 확인하고 [4. reconnect](../../aidin_hand2_hardware/README.ko.md#4-reconnect)로 복구합니다 |

기본 launch는 연결과 제어 시작을 함께 수행하므로 정상 실행 후에는 `Running`입니다.
command 적용에는 `homing_state`가 `Succeeded`라는 조건도 필요합니다.

`ros2 control list_controllers`의 `active`는 controller가 실행 중이라는 뜻입니다.
로봇 핸드의 제어 가능 여부는 `hand_diagnostics`를 함께 읽어 판단합니다.
`homing_state`는 원점 설정 상태로 lifecycle과 별도로 확인하며,
`NotRun` → `InProgress` → `Succeeded` 순서로 진행합니다. 실패하면 `Failed`입니다.
mock은 이 lifecycle·homing 상태와 service를 제공하지 않으므로 3장으로 진행합니다.

## 2. Prepare the robot hand

controller와 로봇 핸드의 현재 상태를 확인합니다.

```bash
ros2 control list_controllers
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

기본 launch에서는 `left_joint_position_controller`가 `active`이고 `lifecycle`은 `Running`입니다.
`Faulted`이면 [5. Stop and recover](#5-stop-and-recover)의 복구 절차로 진행하십시오.

> [!WARNING]
> homing은 finger를 hard stop까지 움직입니다. 주변을 비우고 완료까지 접촉하지 마십시오.
> 움직임이 막히면 잘못된 위치가 원점으로 설정될 수 있습니다. `auto_home=true`이면 `run` 뒤에도
> homing이 시작될 수 있으므로 제어를 재개하기 전에 주변을 확인하십시오.

`Connected` 또는 `Stopped`이면 아래 service로 제어를 시작합니다.
`auto_home=true`이고 homing이 완료되지 않았다면 이 호출 뒤 자동으로 homing이 시작됩니다.

```bash
ros2 service call /left_hand_control/run std_srvs/srv/Trigger
```

service는 `std_srvs/srv/Trigger` 타입이며 요청 인자가 없습니다. `success=True`인지 확인한 뒤
다음 단계로 진행하십시오. 실패하면 응답의 `message`를 확인합니다.

`auto_home=false`이고 `homing_state`가 `Succeeded`가 아니면 직접 homing을 시작합니다.
이미 `InProgress`이면 다시 호출하지 말고 완료를 기다립니다.

```bash
ros2 service call /left_hand_control/home std_srvs/srv/Trigger
```

`success=True`는 homing 시작을 뜻합니다. 완료될 때까지 상태를 관측합니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --field homing_state
```

`Succeeded`가 출력되면 `Ctrl-C`로 관측을 끝냅니다. `Failed`이면 명령을 보내지 말고
`actuator_fault_name`과 주변 상태를 확인합니다. 자세한 조건은
[home](../../aidin_hand2_hardware/README.ko.md#3-home)에 있습니다.

## 3. Send a command

직접 topic을 사용할 때는 대상 controller가 `active`이고 chained mode가 아니어야 합니다.
로봇 핸드는 `lifecycle=Running`, `homing_state=Succeeded`도 확인합니다. mock에는 homing이 없습니다.
명령은 `sensor_msgs/JointState`이고 `name` 필드로 축을 지정합니다. 최초 입력과 controller 전환 후 첫
입력은 16개 이름을 모두 담아야 합니다. 이름과 읽는 필드는
[Command message](../../aidin_hand2_msgs/README.ko.md#2-command-message)에 있습니다.

기본 launch는 joint position controller를 활성화합니다. 다른 제어 방식의 예제를 실행하기 전에는
[Switch controllers](../../aidin_hand2_controllers/README.ko.md#3-switch-controllers)로 해당 controller를
활성화하십시오. 한 손에는 command controller 하나만 활성화합니다.

### 3.1 Joint position

다음은 SDK 예제 `09_joint_position.cpp`의 grasp 자세를 왼손에 보내는 예입니다.

```bash
ros2 topic pub --once \
  /left_joint_position_controller/cmd \
  sensor_msgs/msg/JointState \
  "{name: [left_thumb_joint0, left_thumb_joint1, left_thumb_joint2, left_thumb_joint3,
           left_index_joint1, left_index_joint2, left_index_joint3,
           left_middle_joint1, left_middle_joint2, left_middle_joint3,
           left_ring_joint1, left_ring_joint2, left_ring_joint3,
           left_baby_joint1, left_baby_joint2, left_baby_joint3],
    position: [0.20, 0.35, 0.10, 0.25,
               0.08, 0.45, 0.30,
               0.04, 0.55, 0.40,
               -0.04, 0.65, 0.50,
               -0.08, 0.75, 0.60]}"
```

첫 message 뒤에는 바꿀 축만 보낼 수 있습니다. 다음은 index finger의 joint2만 0.90 rad로 바꾸는 예입니다.

```bash
ros2 topic pub --once \
  /left_joint_position_controller/cmd \
  sensor_msgs/msg/JointState \
  "{name: [left_index_joint2], position: [0.90]}"
```

### 3.2 Actuator position

목표는 `hand_state` topic의 `actuator_position` 필드에서 읽은 현재 값에 작은 차이를 더해 만듭니다. 다음
예의 `<cnt_0>`부터 `<cnt_15>`는 읽어 온 현재 값입니다.

```bash
ros2 topic pub --once \
  /left_actuator_position_controller/cmd \
  sensor_msgs/msg/JointState \
  "{name: [left_thumb_actuator0, left_thumb_actuator1, left_thumb_actuator2, left_thumb_actuator3,
           left_index_actuator1, left_index_actuator2, left_index_actuator3,
           left_middle_actuator1, left_middle_actuator2, left_middle_actuator3,
           left_ring_actuator1, left_ring_actuator2, left_ring_actuator3,
           left_baby_actuator1, left_baby_actuator2, left_baby_actuator3],
    position: [<cnt_0>, <cnt_1>, <cnt_2>, <cnt_3>,
               <cnt_4>, <cnt_5>, <cnt_6>,
               <cnt_7>, <cnt_8>, <cnt_9>,
               <cnt_10>, <cnt_11>, <cnt_12>,
               <cnt_13>, <cnt_14>, <cnt_15>]}"
```

### 3.3 Actuator effort

actuator effort는 위치 제한 없이 토크를 계속 가할 수 있습니다. 물체와의 접촉·기구 제한을 확인하고
정지 수단을 준비한 상태에서 사용하십시오. publisher를 종료해도 마지막 effort는 유지됩니다.

SDK가 절댓값을 actuator별 `max_effort` 값으로 제한한 뒤 전송합니다. 다음은 30%를 보내는 예입니다.

```bash
ros2 topic pub --once \
  /left_actuator_effort_controller/cmd \
  sensor_msgs/msg/JointState \
  "{name: [left_thumb_actuator0, left_thumb_actuator1, left_thumb_actuator2, left_thumb_actuator3,
           left_index_actuator1, left_index_actuator2, left_index_actuator3,
           left_middle_actuator1, left_middle_actuator2, left_middle_actuator3,
           left_ring_actuator1, left_ring_actuator2, left_ring_actuator3,
           left_baby_actuator1, left_baby_actuator2, left_baby_actuator3],
    effort: [300.0, 300.0, 300.0, 300.0,
             300.0, 300.0, 300.0,
             300.0, 300.0, 300.0,
             300.0, 300.0, 300.0,
             300.0, 300.0, 300.0]}"
```

joint impedance 제어는 SDK에서 개발 중이므로 사용하지 마십시오.

### 3.4 Upper controller input

사용자 node는 상위 controller가 제공하는 topic에 목표값을 보낼 수도 있습니다.
상위 controller가 이를 처리해 하위 command controller의 reference interface에 전달합니다.
이때 하위 controller는 chained mode이므로 자신의 `~/cmd` topic 입력을 받지 않습니다.

상위 controller의 topic 이름과 message 타입은 해당 controller의 구현에 따릅니다.
제공된 [상위 controller skeleton](../../aidin_hand2_examples/README.ko.md)은 자기 `~/cmd` topic에
`sensor_msgs/JointState`를 받고, 알고리즘 자리에서 입력에 0을 곱해 전달합니다. 그 자리를 사용자 알고리즘으로
바꾸면 됩니다.

## 4. Read state

### 4.1 Topics


topic 이름은 controller 이름 아래에 있습니다. 발행 주기는 `aidin_hand2_bringup` package의
`controllers.yaml`이 지정한 값이고, 자기 `controllers.yaml`에서 바꿀 수 있습니다.

| Topic | Type | Direction | Rate | Backend |
|---|---|---|---|---|
| `/{side}_joint_position_controller/cmd` | `sensor_msgs/JointState` | 구독 | — | real · mock |
| `/{side}_joint_impedance_controller/cmd` | `sensor_msgs/JointState` | 구독 | — | real · mock |
| `/{side}_actuator_position_controller/cmd` | `sensor_msgs/JointState` | 구독 | — | real · mock |
| `/{side}_actuator_effort_controller/cmd` | `sensor_msgs/JointState` | 구독 | — | real · mock |
| `/joint_states` | `sensor_msgs/JointState` | 발행 | 100 Hz. mock은 `controllers_mock.yaml`이 지정하지 않아 500 Hz | real · mock |
| `/{side}_hand_state_broadcaster/hand_state` | `aidin_hand2_msgs/HandState` | 발행 | 100 Hz | real |
| `/{side}_diagnostics_broadcaster/hand_diagnostics` | `aidin_hand2_msgs/HandDiagnostics` | 발행 | 20 Hz | real |

command를 적용하려면 대상 controller가 `active`이고 chained mode가 아니어야 합니다.
mock에서는 `/joint_states`로 결과를 확인합니다. `hand_state`와 `hand_diagnostics`는 제공하지 않습니다.

### 4.2 Observe the result

서로 다른 상태 topic은 같은 시점의 측정이라고 가정하지 마십시오.
mock에서는 `/joint_states`, 로봇 핸드에서는 다음 세 topic으로 관측합니다.


`/joint_states` topic은 표준 `joint_state_broadcaster`가 발행하는 `sensor_msgs/JointState`입니다. `name`
필드에 로봇의 모든 joint, `position` 필드에 각도[rad]가 있고, 로봇 핸드의 joint는 손마다 21개입니다.
로봇 핸드의 joint velocity·effort 측정값은 제공하지 않습니다.
배열 위치는 `name`으로 확인하십시오. 다른 로봇의 joint가 함께 포함될 수 있습니다.

발행되는 message를 한 번 읽습니다.

```bash
ros2 topic echo /joint_states --once
```

joint·actuator·tactile 관측을 한 번 읽습니다. 목표 도달 여부는 `joint_position` 또는
`actuator_position`을 보십시오.

```bash
ros2 topic echo /left_hand_state_broadcaster/hand_state --once
```

적용 중인 command와 effort 상한을 읽습니다. `command_state`는 새 message 수신 확인 응답이 아니며,
마지막 command를 유지하는 동안에도 같은 값일 수 있습니다.

```bash
ros2 topic echo /left_hand_state_broadcaster/hand_state --once --field command_state
```

lifecycle·homing 상태와 actuator fault를 읽습니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

필드의 타입·단위는 [aidin_hand2_msgs](../../aidin_hand2_msgs/README.ko.md)에 있습니다.

### 4.3 Monitoring

topic이 계속 발행되어도 로봇 핸드의 상태가 갱신되고 있다는 뜻은 아닙니다.
SDK가 `Faulted`로 정지하면 broadcaster가 마지막 관측값을 같은 주기로 반복할 수 있습니다.
각 필드가 나타내는 정보를 구별해서 확인합니다.

| Signal | Meaning |
|---|---|
| `hand_diagnostics.control_cycles` 증가 | 제어·통신 루프가 진행 중입니다 |
| `hand_state.header.stamp` 증가 | SDK가 관측 시각을 갱신하고 있습니다 |
| `hand_diagnostics.lifecycle` | 통신·제어 상태입니다. command를 적용할 때는 `Running`이어야 합니다 |
| `hand_diagnostics.homing_state` | homing 상태입니다. command를 적용할 때는 `Succeeded`여야 합니다 |

`control_cycles`와 `header.stamp` 필드는 `Faulted`가 아닌 동안 frame 수신 여부와 무관하게 매 cycle
갱신되므로 통신 생존의 근거는 아닙니다. 통신이 끊기면 약 100 ms 뒤 `lifecycle` 값이 `Faulted`로 바뀌고
전이 시점부터 둘이 함께 멈춥니다.

> [!IMPORTANT]
> `HandDiagnostics`에는 종합 판정 필드가 없습니다. 준비 여부는 구독자가 `lifecycle`·`homing_state`·
> `actuator_fault_name` 필드와 `control_cycles` 값의 증가를 합쳐 판단해야 합니다. 마지막 예외 문구와
> 재연결 시도 횟수도 message에 없습니다.

### 4.4 QoS

현재 command controller 4종의 `~/cmd` 구독과 `HandStateBroadcaster`·`DiagnosticsBroadcaster`의
발행은 `rclcpp::SystemDefaultsQoS()`를 사용합니다. 상위 controller skeleton의 `HandState` 구독도
같습니다. 이 설정은 history·depth·reliability 등을 RMW 기본값에 맡기므로 실행 환경에서 실제 값을 확인합니다.
`/joint_states`는 별도로 설치된 `joint_state_broadcaster`의 설정을 확인하십시오.

```bash
ros2 topic info /left_joint_position_controller/cmd --verbose
ros2 topic info /left_hand_state_broadcaster/hand_state --verbose
```

publisher와 subscriber의 QoS가 호환되어야 message가 전달됩니다. 예를 들어 `best_effort` publisher는
`reliable` subscriber와 연결되지 않습니다. 정책과 호환 조건은
[ROS 2 Humble QoS 문서](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)에 있습니다.

QoS는 전달 정책이며, 제어 주기의 실행 시간과 jitter를 보장하지 않습니다.
실시간 실행의 조건은 [ROS 2 real-time programming](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Demos/Real-Time-Programming.rst)을 참고하십시오.
현재 wrapper에는 command timeout에 따른 자동 정지가 없습니다. QoS의 deadline·lifespan을 설정하더라도
이미 적용한 마지막 목표를 자동으로 해제하지 않습니다.

## 5. Stop and recover

로봇 핸드의 토크를 제거하려면 `stop` service를 호출합니다. topic publisher를 종료하거나
controller만 비활성화하는 것으로 무토크 상태를 보장하지 않습니다.

```bash
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once --field lifecycle
```

응답의 `success=True`, `message='stopped'`와 `lifecycle=Stopped`를 확인합니다.
quick stop 확인에 실패하면 actuator가 마지막 command를 유지할 수 있으므로 전원을 차단하십시오.
재개할 때는 `run`을 호출하고, homing 상태를 확인한 뒤 필요한 목표를 다시 보냅니다.
종료할 때는 양손 모두 정지를 확인한 뒤 launch 터미널에서 `Ctrl-C`를 누릅니다.
mock은 [Stop the mock](04_bringup.md#14-stop-the-mock)을 따릅니다.

`Faulted`에서는 전원·배선·CAN 오류 원인을 제거한 다음 재연결합니다.

```bash
ros2 service call /left_hand_control/reconnect std_srvs/srv/Trigger
```

수동 재연결이 성공하면 `Connected`로 돌아갑니다. 2장처럼 `run` → homing 완료 확인을 거친 뒤
command를 다시 보냅니다. 재연결은 homing 상태를 초기화합니다.
실패 조건과 자동 재연결은 [reconnect](../../aidin_hand2_hardware/README.ko.md#4-reconnect)에 있습니다.

# Bringup

`aidin_hand2_bringup` package의 launch로 로봇 핸드를 단독으로 실행해 설치와 하드웨어를 확인하는
절차입니다. 단독 launch는 확인용이고, 자기 로봇에 붙이는 절차는 [Integration](05_integration.md)에
있습니다. 먼저 [Installation](03_installation.md)을 마치십시오.

이 문서의 명령은 왼손이 `can0`에 연결된 경우이고 `{side}`는 `left` 또는 `right`입니다. 새 terminal을 열
때마다 다음 두 줄을 먼저 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
```

## Contents

&nbsp;&nbsp;[**1. Mock**](#1-mock)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Launch](#11-launch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Check the interfaces](#12-check-the-interfaces)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Send a command](#13-send-a-command)<br>
&nbsp;&nbsp;[**2. Robot hand**](#2-robot-hand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Check the host](#21-check-the-host)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Launch](#22-launch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Check the state](#23-check-the-state)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.4 Home](#24-home)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.5 Send a command](#25-send-a-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.6 Stop](#26-stop)<br>
&nbsp;&nbsp;[**3. Both hands**](#3-both-hands)<br>
&nbsp;&nbsp;[**4. Isaac Sim**](#4-isaac-sim)

## 1. Mock

mock은 CAN, drive, homing 없이 controller와 command interface 계약을 확인하는 backend입니다. 로봇
핸드와 같은 command interface 65개를 export하고 command를 kinematics로 곧바로 state에 반영합니다.
계약과 차이는 [1.1 Backends](06_controllers.md#11-backends)에 있습니다.

### 1.1 Launch

양손 mock과 RViz를 실행합니다. 기본값이 양손과 RViz이고 인자는 [Launch files](10_launch_files.md)
3장에 있습니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
```

왼손과 오른손은 URDF에서 y축으로 ±0.1 m 벌려 두므로 RViz에서 겹치지 않습니다. active controller는
`joint_state_broadcaster`와 손별 `{side}_joint_position_controller`입니다.

### 1.2 Check the interfaces

다른 terminal에서 hardware component, controller, interface를 확인합니다.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

출력에서 다음을 확인합니다.

- hardware component `left_hand_control`과 `right_hand_control`이 `active`입니다.
- `joint_state_broadcaster`, `left_joint_position_controller`, `right_joint_position_controller`가
  `active`입니다.
- command interface가 손마다 65개입니다. `{side}_hand_control/command_lock` 1개와 mode별 16개씩
  4묶음이고, `left_hand_control/command_lock`과 `left_joint_position_command/target_position_rad.*`
  16개만 `[claimed]`입니다. 계약은 [1.3 Command and state interfaces](06_controllers.md#13-command-and-state-interfaces)에
  있습니다.
- state interface가 손마다 69개입니다.

### 1.3 Send a command

`JointPositionCommand` 하나에 active joint 16개의 목표 각도[rad]를 모두 담아 전송합니다. 다음은
왼손 index finger의 joint1을 0.10 rad, joint2를 0.20 rad로 보내는 예입니다.

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

RViz에서 왼손 index finger가 굽고, `/joint_states` topic에 반영됩니다. 반영된 값을 읽습니다.

```bash
ros2 topic echo /joint_states --once --field position
```

`left_index_joint1`과 `left_index_joint2`의 값이 0.10과 0.20 근처입니다. mock의 값은 clamp → IK →
FK를 거친 결과이므로 입력과 소수점 아래에서 다를 수 있습니다.

## 2. Robot hand

로봇 핸드는 `aidin_hand2.launch.py`로 실행합니다. 이 launch는 hardware component를 configure하고
activate까지 진행하며, activate에서 SDK가 actuator를 enable합니다.

### 2.1 Check the host

[Real-time kernel setup](01_real_time_kernel_setup.md)의 결과를 확인합니다.

```bash
cat /sys/kernel/realtime        # 1
ulimit -r                       # 99
ulimit -l                       # unlimited
```

[CAN-FD setup](02_can_fd_setup.md)의 결과를 확인합니다.

```bash
ip -details link show can0 | grep -E '<FD>|bitrate'   # <FD>가 있고 bitrate 두 줄이 나와야 합니다
candump -n 5 can0                                     # 0x2xx → 왼손, 0x1xx → 오른손
```

`candump`가 5개 frame을 출력하고 ID가 `0x2xx`이면 `can0`에 왼손이 연결되어 있고 전원이 켜진
상태입니다. frame이 없으면 전원과 배선을 확인하십시오.

### 2.2 Launch

기본 config `hand_bringup.yaml`은 양손을 `can0`·`can1`에 `auto_home=true`로 실행하므로, 첫 실행에서는
왼손만 homing 없이 실행하도록 인자로 덮습니다.

> [!WARNING]
> launch가 hardware component를 activate하면서 actuator가 enable되어 토크가 걸립니다.
> `auto_home:=false`이면 command는 전송되지 않지만, `auto_home:=true`이면 activate 직후 AIDIN Hand
> Gen2가 hard stop까지 움직입니다. 주변을 비운 뒤 실행하십시오.

왼손만 `can0`에서 homing 없이 실행합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  use_right_hand:=false left_hand_interface:=can0 auto_home:=false
```

인자 전체와 config 파일의 우선순위는 [Launch files](10_launch_files.md) 2장에 있습니다.

### 2.3 Check the state

다른 terminal에서 확인합니다.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

출력에서 다음을 확인합니다.

- hardware component `left_hand_control`이 `active`입니다.
- `joint_state_broadcaster`, `left_hand_state_broadcaster`, `left_diagnostics_broadcaster`,
  `left_joint_position_controller`가 `active`이고, `left_actuator_position_controller`,
  `left_actuator_effort_controller`, `left_joint_impedance_controller`가 `inactive`입니다.
- `hand_diagnostics` topic의 `lifecycle` 값이 `Running`, `homing_state` 값이 `NotRun`,
  `actuator_fault_name` 필드 16개가 모두 빈 문자열입니다. `lifecycle` 값의 뜻은
  [1.2 Lifecycle](06_controllers.md#12-lifecycle)에 있습니다.

`control_cycles` 값은 제어·통신 루프의 누적 cycle이므로 두 번 읽으면 증가해야 합니다.

### 2.4 Home

homing은 각 finger를 hard stop까지 밀어 hard stop 지점을 원점으로 삼는 절차입니다. `homing_state`
값이 `Succeeded`가 되기 전에는 wrapper가 command를 전송하지 않습니다.

> [!WARNING]
> finger가 완전히 펴지지 못하게 막혀 있으면 막힌 지점이 hard stop으로 인식되어 원점이 어긋난 채
> 성공으로 보고됩니다. homing 전에 주변을 비우고 완료까지 접촉하지 마십시오.

`~/home` service를 호출합니다.

```bash
ros2 service call /left_hand_control/home std_srvs/srv/Trigger
```

응답은 다음과 같습니다.

```text
response:
std_srvs.srv.Trigger_Response(success=True, message="homing started — poll diagnostics 'homing_state'")
```

이 응답은 시작 접수이고 완료가 아닙니다. `homing_state` 값을 계속 읽습니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --field homing_state
```

`InProgress`가 `Succeeded`로 바뀌면 완료입니다. 제한 시간과 실패 처리는 [3. home](07_services.md#3-home)에
있습니다.

### 2.5 Send a command

[1.3](#13-send-a-command)과 같은 command를 로봇 핸드에 전송합니다. 목표 단위는 rad입니다.

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

왼손 index finger가 굽습니다. 적용된 command는 `hand_state` topic의 `command_state` 필드로 확인합니다.

```bash
ros2 topic echo /left_hand_state_broadcaster/hand_state --once --field command_state
```

`controller_input_mode` 값이 `1`(joint position), `selected_source` 값이 `1`(controller)이고
`joint_position_input.target_position_rad` 필드에 전송한 값이 있습니다. 값의 뜻은
[3.3 CommandState](08_topics.md#33-commandstate)에, 전송값에 적용되는 effort 상한은
[2.1 Max effort](09_parameters.md#21-max-effort)에 있습니다.

> [!IMPORTANT]
> SDK는 joint 목표를 finger별 도달 범위로 투영한 뒤 적용합니다. self-collision, 주변 물체, 속도는
> 검사하지 않으므로 충돌·속도·힘 제한은 상위 application이 따로 걸어야 합니다. 범위 모델은 SDK
> 문서의 [Workspace limits](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/14_workspace_limits.md)에
> 있습니다.

### 2.6 Stop

command controller를 먼저 deactivate합니다.

```bash
ros2 control switch_controllers --strict --deactivate left_joint_position_controller
```

`~/stop` service로 actuator를 quick stop합니다. 응답이 `stopped`이면 `lifecycle` 값이 `Stopped`이고
actuator는 무토크입니다.

```bash
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger
```

launch terminal에서 `Ctrl-C`로 종료합니다. 종료 조건과 하지 말아야 할 정지 방법은
[5. Shutdown](07_services.md#5-shutdown)에 있습니다.

## 3. Both hands

양손은 interface 둘을 지정해 실행합니다. 어느 interface에 왼손과 오른손 중 어느 쪽이 연결되어
있는지는 [2.1](#21-check-the-host)의 `candump`로 interface마다 확인합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  left_hand_interface:=can0 right_hand_interface:=can1 auto_home:=false
```

`left_hand_control`과 `right_hand_control` 두 hardware component가 생기고 service도 각각의
namespace에 있습니다. 2장의 확인·homing·command·정지는 오른손에 대해 `right_`로 반복합니다.

adapter를 다시 꽂거나 재부팅하면 `can0`·`can1`과 왼손·오른손의 대응이 바뀔 수 있습니다. interface
이름을 `auto`로 지정하면 SDK가 side의 CAN ID로 채널을 탐색합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  left_hand_interface:=auto right_hand_interface:=auto auto_home:=false
```

## 4. Isaac Sim

isaac backend는 CAN 대신 ROS 2 topic으로 Isaac Sim과 state와 command를 주고받는 hardware plugin입니다.
로봇 핸드가 없고 Isaac Sim을 쓴다면 1장을 마친 뒤 2장과 3장을 건너뛰고 4장을 수행하십시오. backend의 성격은
[1.1 Backends](06_controllers.md#11-backends)에, topic 넷의 message와 방향은 [Topics](08_topics.md)
5장에 있습니다.

왼손 하나를 실행합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_isaac.launch.py
```

`ros2 control list_controllers`로 active controller를 확인합니다. 목록은 [Launch files](10_launch_files.md)
4장에 있습니다. Isaac Sim 쪽에서는 `/isaac/joint_states` topic을 발행하고 `/isaac/hand_command` topic을
구독해야 합니다. topic 이름을 바꾸는 인자는
[Launch files](10_launch_files.md) 4장에 있습니다.

state가 들어오면 `hand_diagnostics` topic의 `lifecycle` 값이 `Running`이 되고, [1.3](#13-send-a-command)의
command를 그대로 전송할 수 있습니다.

이어서 [Integration](05_integration.md)에서 로봇 핸드를 자기 로봇의 URDF와 controller 구성에
넣으십시오.

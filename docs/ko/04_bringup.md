# Bringup

설치한 wrapper로 mock 또는 로봇 핸드를 실행하고, 첫 command와 종료까지 확인합니다.
먼저 [Installation](03_installation.md)을 마치십시오. 기존 로봇에 연결할 때는 단독 동작을 확인한 뒤
[7. Add to your robot](../../aidin_hand2_bringup/README.ko.md#7-add-to-your-robot)으로 진행합니다.

예제는 왼손을 사용하며 로봇 핸드의 CAN interface는 `can0`입니다. `{side}`는 `left` 또는 `right`입니다.
새 터미널을 열 때마다 ROS 2와 workspace 환경을 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
```

SDK를 `~/.local`에 설치했다면 실행 터미널마다 다음 경로도 설정합니다.
다른 사용자 경로에 설치했다면 `$HOME/.local`을 바꾸십시오.

```bash
export LD_LIBRARY_PATH="$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

## Contents

&nbsp;&nbsp;[**1. Mock**](#1-mock)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Launch](#11-launch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Check the controllers](#12-check-the-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Send a command](#13-send-a-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 Stop the mock](#14-stop-the-mock)<br>
&nbsp;&nbsp;[**2. Robot hand**](#2-robot-hand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Check the host](#21-check-the-host)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Launch](#22-launch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Check the state](#23-check-the-state)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.4 Home](#24-home)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.5 Send a command](#25-send-a-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.6 Stop](#26-stop)<br>
&nbsp;&nbsp;[**3. Both hands**](#3-both-hands)

## 1. Mock

mock으로 로봇 핸드 없이 목표값 전달과 RViz 표시를 확인합니다. RT kernel·CAN 설정과 homing은
필요하지 않습니다. 로봇 핸드와의 차이는 [5. Mock behavior](../../aidin_hand2_controllers/README.ko.md#5-mock-behavior)에 있습니다.

### 1.1 Launch

양손 mock과 RViz를 실행합니다. 인자는 [3. aidin_hand2_mock.launch.py](../../aidin_hand2_bringup/README.ko.md#3-aidin_hand2_mocklaunchpy)에 있습니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
```

왼손과 오른손은 URDF에서 y축으로 ±0.1 m 벌려 두므로 RViz에서 겹치지 않습니다. active controller는
`joint_state_broadcaster`와 손별 `{side}_joint_position_controller`입니다.

### 1.2 Check the controllers

다른 터미널에서 hardware component와 controller 상태를 확인합니다.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
```

출력에서 다음을 확인합니다.

- hardware component `left_hand_control`과 `right_hand_control`이 `active`입니다.
- `joint_state_broadcaster`, `left_joint_position_controller`, `right_joint_position_controller`가
  `active`입니다.

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
ros2 topic echo /joint_states --once
```

`name` 배열에서 `left_index_joint1`과 `left_index_joint2`를 찾습니다. 같은 위치의 `position` 값이
각각 0.10과 0.20 근처인지 확인합니다. 도달 범위 보정과 kinematics 계산을 거치므로
입력과 소수점 아래에서 다를 수 있습니다.

### 1.4 Stop the mock

mock을 실행한 터미널에서 `Ctrl-C`를 누르고 종료될 때까지 기다립니다. 로봇 핸드 launch와 mock은
같은 node·controller 이름을 사용하므로 다음 절을 실행하기 전에 mock을 종료하십시오.

## 2. Robot hand

[Real-time kernel setup](01_real_time_kernel_setup.md)과 [CAN-FD setup](02_can_fd_setup.md)을 마친 뒤
진행합니다. `aidin_hand2.launch.py`를 실행하면 로봇 핸드에 연결하고 actuator를 enable합니다.

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
> launch를 실행하면 actuator가 enable되어 토크가 걸립니다.
> `auto_home:=false`는 자동 homing만 끄는 설정입니다. `auto_home:=true`이면 AIDIN Hand Gen2가
> 곧바로 hard stop까지 움직입니다. 주변을 비운 뒤 실행하십시오.

왼손만 `can0`에서 homing 없이 실행합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  use_right_hand:=false left_hand_interface:=can0 auto_home:=false
```

인자 전체와 config 파일의 우선순위는 [2. aidin_hand2.launch.py](../../aidin_hand2_bringup/README.ko.md#2-aidin_hand2launchpy)에 있습니다.

### 2.3 Check the state

다른 터미널에서 확인합니다.

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
  [1.1 Lifecycle](../../aidin_hand2_hardware/README.ko.md#11-lifecycle)에 있습니다.

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

`Succeeded`가 출력되면 완료입니다. 관측 중에는 `InProgress`를 볼 수 있습니다.
이 echo 명령을 실행한 터미널에서 `Ctrl-C`로 관측을 끝냅니다.
제한 시간과 실패 처리는 [3. home](../../aidin_hand2_hardware/README.ko.md#3-home)에 있습니다.

### 2.5 Send a command

> [!IMPORTANT]
> SDK는 joint 목표를 finger별 도달 범위로 투영한 뒤 적용합니다. self-collision, 주변 물체, 속도는
> 검사하지 않으므로 충돌·속도·힘 제한은 상위 application이 따로 걸어야 합니다. 범위 모델은 SDK
> 문서의 [Workspace limits](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/14_workspace_limits.md)에
> 있습니다.

[1.3 Send a command](#13-send-a-command)과 같은 command를 로봇 핸드에 전송합니다. 목표 단위는 rad입니다.

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

`controller_input_mode`가 `1`(joint position), `selected_source`가 `1`(controller)이고
`joint_position_input.target_position_rad`에 도달 범위 보정 후의 목표값이 있습니다.
실제 joint 각도는 `joint_position`에서 확인합니다. 값의 뜻은
[3.2 CommandState](../../aidin_hand2_msgs/README.ko.md#32-commandstate)에, 전송값에 적용되는 effort 상한은
[6.1 Max effort](../../aidin_hand2_hardware/README.ko.md#61-max-effort)에 있습니다.

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

launch 터미널에서 `Ctrl-C`로 종료합니다. 종료 조건과 하지 말아야 할 정지 방법은
[5. Shutdown](../../aidin_hand2_hardware/README.ko.md#5-shutdown)에 있습니다.

## 3. Both hands

기존 단독 launch를 종료한 뒤 양손을 실행합니다. interface 둘을 지정합니다. 어느 interface에 왼손과 오른손 중 어느 쪽이 연결되어
있는지는 [2.1 Check the host](#21-check-the-host)의 `candump`로 interface마다 확인합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  left_hand_interface:=can0 right_hand_interface:=can1 auto_home:=false
```

`left_hand_control`과 `right_hand_control` 두 hardware component가 생기고 service도 각각의
namespace에 있습니다. 2장의 확인·homing·command·정지는 오른손에 대해 `right_`로 반복합니다.

adapter를 다시 꽂거나 재부팅하면 `can0`·`can1`과 왼손·오른손의 대응이 바뀔 수 있습니다. interface
이름을 `auto`로 지정하면 SDK가 side의 CAN ID로 채널을 탐색합니다.
다음은 위 명령 대신 사용할 수 있는 실행 방법입니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  left_hand_interface:=auto right_hand_interface:=auto auto_home:=false
```

단독 실행은 여기까지입니다. 기존 로봇에 로봇 핸드를 추가할 때는
[7. Add to your robot](../../aidin_hand2_bringup/README.ko.md#7-add-to-your-robot)으로 진행하십시오.

반복적인 제어 작업과 종류별 명령 예제는 [Control guide](05_control_guide.md)를 참고하십시오.

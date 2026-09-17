# Troubleshooting

문제는 증상이 나타나는 층 순서로 분리합니다. 층은 build, launch, controller, hardware component, 통신의
다섯이고, 장은 층 하나이고 절은 증상 하나입니다. 각 절은 증상, 원인, 조치 순서입니다.

## Contents

&nbsp;&nbsp;[**1. Collect the basics**](#1-collect-the-basics)<br>
&nbsp;&nbsp;[**2. Build**](#2-build)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 ament_cmake not found](#21-ament_cmake-not-found)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 aidin_hand2 not found](#22-aidin_hand2-not-found)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 rosdep fails](#23-rosdep-fails)<br>
&nbsp;&nbsp;[**3. Launch**](#3-launch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 libaidin_hand2.so not found](#31-libaidin_hand2so-not-found)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 Package or launch file not found](#32-package-or-launch-file-not-found)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.3 robot_description deprecation warning](#33-robot_description-deprecation-warning)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.4 Hardware component fails to configure](#34-hardware-component-fails-to-configure)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.5 The hands are swapped between interfaces](#35-the-hands-are-swapped-between-interfaces)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.6 The robot hand moves right after launch](#36-the-robot-hand-moves-right-after-launch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.7 GUI cannot connect to rosbridge](#37-gui-cannot-connect-to-rosbridge)<br>
&nbsp;&nbsp;[**4. Controllers**](#4-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Mode switch is rejected](#41-mode-switch-is-rejected)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 A command is published but nothing moves](#42-a-command-is-published-but-nothing-moves)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.3 Part of a command is ignored](#43-part-of-a-command-is-ignored)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.4 A controller fails to configure](#44-a-controller-fails-to-configure)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.5 Joints move too fast](#45-joints-move-too-fast)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.6 Broadcasters missing on mock](#46-broadcasters-missing-on-mock)<br>
&nbsp;&nbsp;[**5. Hardware component**](#5-hardware-component)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.1 Service does not exist](#51-service-does-not-exist)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2 home succeeds but homing_state is not Succeeded](#52-home-succeeds-but-homing_state-is-not-succeeded)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.3 Topics keep publishing but the robot hand has stopped](#53-topics-keep-publishing-but-the-robot-hand-has-stopped)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.4 reconnect fails](#54-reconnect-fails)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.5 stop fails](#55-stop-fails)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.6 Real-time warning](#56-real-time-warning)<br>
&nbsp;&nbsp;[**6. Communication**](#6-communication)<br>
&nbsp;&nbsp;[**7. Support bundle**](#7-support-bundle)

## 1. Collect the basics

어느 층의 문제든 먼저 기본 정보를 수집합니다. 문제가 난 시각과 실행한 launch 명령도 함께 남깁니다.

> [!WARNING]
> AIDIN Hand Gen2가 의도하지 않게 움직이면 log 수집보다 전원 차단을 먼저 하십시오. `Ctrl-C`와 네트워크
> 단절은 정지를 보장하지 않습니다.

수집 명령은 다음과 같습니다.

```bash
printenv ROS_DISTRO
ros2 doctor --report
ros2 pkg prefix aidin_hand2_bringup
ros2 control list_hardware_components
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
ip -details -statistics link show can0
```

SDK의 log는 hardware component가 ROS logger `aidin_hand2`로 중계하므로 `/rosout` topic에서 봅니다. SDK는
예외를 던지기 전에 `[left][exception] <ErrorCode>: <message>` 형태로 남깁니다. 앞의 `[left]`·`[right]`는
어느 로봇 핸드인지를 나타내며, 로봇 핸드에 매이지 않은 예외에는 붙지 않습니다.

```bash
ros2 topic echo /rosout
```

## 2. Build

build 층의 증상은 colcon이 CMake에 넘기는 환경과 SDK install 위치에서 나옵니다.

### 2.1 ament_cmake not found

```text
Could not find a package configuration file provided by "ament_cmake"
```

ROS 2 환경을 source하지 않은 shell에서 `colcon build`를 실행한 경우입니다. `AMENT_PREFIX_PATH`가 비어
있어 colcon이 CMake에 넘길 경로가 없습니다. 같은 shell에서 source한 뒤 다시 build합니다. 새 terminal마다
필요합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 2.2 aidin_hand2 not found

```text
Could not find a package configuration file provided by "aidin_hand2"
```

SDK가 install되지 않았거나, 사용자 prefix에 install했는데 `CMAKE_PREFIX_PATH`에 install prefix가 없는
경우입니다. 어느 prefix에 어느 version이 있는지 확인합니다.

```bash
find /usr/local ~/.local -name aidin_hand2Config.cmake -print 2>/dev/null
printf '%s\n' "$CMAKE_PREFIX_PATH" | tr ':' '\n'
```

- 출력이 없으면 [Installation](03_installation.md) 3장대로 SDK를 install합니다.
- `~/.local` 아래에만 있으면 build하는 shell에서 `CMAKE_PREFIX_PATH`에 `$HOME/.local`을 넣습니다.
  SDK build tree(`cpp/build`)를 prefix로 넣지 마십시오.
- 두 곳 이상 나오면 prefix가 혼재한 상태입니다. `find_package`가 어느 쪽을 찾을지 정해지지 않으므로
  하나만 남기고 지웁니다. 지우는 절차는 SDK 문서의
  [SDK build & install](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/06_sdk_build_and_install.md#5-uninstall)
  5장에 있습니다.

### 2.3 rosdep fails

```text
ERROR: your rosdep installation has not been initialized yet. Please run: sudo rosdep init
```

rosdep이 초기화되지 않은 경우입니다. 초기화한 뒤 다시 실행합니다.

```bash
sudo rosdep init
rosdep update
```

`Cannot locate rosdep definition for [<name>]`은 `<name>`이 rosdep key도 ROS package도 아니라는
뜻입니다. `--skip-keys "<name>"`으로 제외합니다.

## 3. Launch

launch 층의 증상은 overlay, plugin load, hardware component configure에서 나옵니다.

### 3.1 libaidin_hand2.so not found

```text
libaidin_hand2.so.0.5: cannot open shared object file: No such file or directory
```

build는 통과했는데 launch에서 hardware plugin을 load하지 못하는 경우입니다. `/usr/local`에 install한 뒤
`sudo ldconfig`를 실행하지 않아 loader 캐시에 라이브러리가 없습니다.

```bash
sudo ldconfig
ldconfig -p | grep aidin_hand2          # libaidin_hand2.so.0.5가 나와야 합니다
```

wrapper가 실제로 어느 파일에 링크되었는지는 `ldd`로 확인합니다.

```bash
ldd ~/your_ws/install/aidin_hand2_hardware/lib/libaidin_hand2_hardware.so | grep aidin_hand2
```

### 3.2 Package or launch file not found

```text
Package 'aidin_hand2_bringup' not found
```

overlay를 source하지 않은 shell입니다. 새 terminal에서 두 setup 파일을 source합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
ros2 pkg prefix aidin_hand2_bringup
```

다른 workspace의 overlay가 같은 package 이름을 가리는 경우도 있습니다. `AMENT_PREFIX_PATH`의 순서를
확인합니다.

```bash
printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n'
```

### 3.3 robot_description deprecation warning

```text
[Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated.
```

launch가 `robot_description`을 `ros2_control_node`의 parameter로 직접 전달하기 때문에 Humble이 내는
warning입니다. hardware component와 controller가 이어서 configure·activate되면 실패 원인이 아니므로
무시합니다.

### 3.4 Hardware component fails to configure

hardware component가 `unconfigured`에 머물고 `/rosout` topic에 SDK 예외가 남는 경우입니다. `ErrorCode`로
점검 대상을 나눕니다.

| ErrorCode | Check |
|---|---|
| `InvalidArgument` | 매크로 parameter. `hand_side`, `disabled_actuators`의 index 범위, `control_rate` |
| `InterfaceUnavailable` | CAN interface 이름, `ip link`의 `UP` 여부, 권한, 다른 프로세스의 점유 |
| `CommunicationLost` | 300 ms 안에 첫 state frame이 오지 않음. 전원, 배선, `candump` |
| `HardwareFault` | drive fault. `actuator_fault_name` 필드 |

parameter 자체가 잘못되면 SDK보다 먼저 `on_init`이 실패하고 다음 중 하나가 FATAL로 남습니다.

```text
missing hardware parameter: can_interface
hand_side must be 'left' or 'right', got '<value>'
auto_home must be True/False, got '<value>'
invalid numeric hardware parameter: <text>
```

isaac backend의 `on_init` 문구는 다음 셋입니다.

```text
hand_side must be 'left' or 'right' (got '<value>')
state_timeout must be a number (got '<value>')
max_effort must be a number (got '<value>')
```

CAN 층은 [6. Communication](#6-communication)으로 확인하고, 문구별 조치는 SDK 문서의
[Error messages](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/15_error_messages.md)에
있습니다.

### 3.5 The hands are swapped between interfaces

기본 config는 `left_hand_interface=can0`, `right_hand_interface=can1`로 고정되어 있고 왼손과 오른손이 어느
interface에 있는지 검사하지 않습니다. 물리적으로 반대로 꽂혀 있거나 USB adapter의 열거 순서가 부팅마다
바뀌면 왼손과 오른손이 뒤바뀌거나 configure가 `CommunicationLost`로 실패합니다.

interface마다 `candump`로 ID를 확인합니다. `0x2xx`는 왼손, `0x1xx`는 오른손입니다.

```bash
candump -n 5 can0
candump -n 5 can1
```

확인한 대응을 인자로 주거나, `auto`로 SDK가 side의 CAN ID로 채널을 찾게 합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  left_hand_interface:=auto right_hand_interface:=auto auto_home:=false
```

### 3.6 The robot hand moves right after launch

기본 config의 `auto_home=true` 때문입니다. hardware component가 activate된 직후 homing이 시작되어 finger가
hard stop까지 움직입니다. 다음 실행부터 인자로 끕니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py auto_home:=false auto_reconnect_home:=false
```

### 3.7 GUI cannot connect to rosbridge

rosbridge를 올립니다.

```bash
ros2 launch aidin_hand2_bringup gui_bridge.launch.py
```

다른 terminal에서 node와 port를 확인합니다.

```bash
ros2 node list | grep rosbridge
ss -ltn | grep 9090
```

다른 컴퓨터의 browser에서 `localhost`는 robot host가 아니라 browser가 실행되는 컴퓨터입니다. robot
host의 IP를 쓰되, port를 신뢰할 수 없는 network에 열지 마십시오.

## 4. Controllers

controller 층의 증상은 mode 전환, command topic, controller parameter에서 나옵니다.

### 4.1 Mode switch is rejected

```text
rejected mode switch: command interfaces must be one complete mode port plus command_lock
```

hardware component는 `command_lock`과 한 mode의 port 16개 전부를 함께 claim하는 전환만 받아들입니다. 두
command controller가 동시에 active가 되려 했거나, 이전 controller를 deactivate하지 않은 경우입니다.

```bash
ros2 control list_controllers
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_controller \
  --activate left_joint_impedance_controller
```

새 controller가 `unconfigured`나 `finalized`이면 spawner log를 확인합니다. 전환 규칙은
[Controllers](06_controllers.md) 5장에 있습니다.

### 4.2 A command is published but nothing moves

publish는 성공하는데 로봇 핸드가 움직이지 않는 경우입니다. 다음을 순서대로 확인합니다.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
ros2 topic echo /left_hand_state_broadcaster/hand_state --once --field command_state
```

| Check | Expected |
|---|---|
| hardware component | `active` |
| `hand_diagnostics.lifecycle` | `Running` |
| `hand_diagnostics.homing_state` | `Succeeded` |
| 대상 command controller | `active`이고 chained mode가 아님 |
| topic 이름과 message 타입 | controller 이름 아래의 `~/command` topic과 controller별 message |
| `command_state.selected_source` | `1` |

`lifecycle` 값이 `Stopped`이면 `~/run` service를, `Faulted`이면 `~/reconnect` service를, `homing_state` 값이
`Succeeded`가 아니면 `~/home` service를 호출합니다. `selected_source` 값이 `3`이면 homing 중, `2`면 quick
stop입니다. wrapper는 `lifecycle` 값이 `Running`이 아니거나 `homing_state` 값이 `Succeeded`가 아니면
command를 error 없이 건너뜁니다. 조건은 [Controllers](06_controllers.md) 6.2절에 있습니다.

### 4.3 Part of a command is ignored

```text
command has axes that were never commanded — skipped. Send a complete command once.
```

message의 일부 축이 NaN인데 NaN인 축에 직전 command가 없는 경우입니다. 첫 command는 16개 모두 유한해야
하고, 이후에는 NaN인 축이 직전 값으로 채워집니다. Inf는 controller가 NaN으로 바꾸며 다음 warning을
남깁니다.

```text
JointPosition reference has an Inf value — ignored
```

값이 유한한데도 `hand_diagnostics` topic의 `nan_command_count` 값이 늘면 SDK가 거부한 것입니다.
`ActuatorPositionCommand`의 목표가 `int32` 범위를 벗어난 경우가 해당합니다. 거부가 시작되면 SDK가
`/rosout` topic에 다음 warning을 한 번 남깁니다.

```text
set_command: command contains an invalid value — holding previous command; watch nan_command_count
```

### 4.4 A controller fails to configure

```text
hand_side must be 'left' or 'right', got 'left_'
```

controller의 `hand_side` parameter에 `prefix`를 준 경우입니다. `controllers.yaml`에서 `hand_side: left`로
고칩니다. 상위 skeleton은 `target_controller` parameter가 비어 있어도 실패합니다.

### 4.5 Joints move too fast

joint position command의 filter는 hardware node의 parameter가 결정합니다. 기본 `cutoff_freq` 값 10 Hz가
발행 주기의 절반보다 높으면 발행 사이의 계단이 그대로 전송됩니다.

```bash
ros2 param get /left_hand_control joint_position_controller.cutoff_freq
ros2 param set /left_hand_control joint_position_controller.cutoff_freq 5.0
```

`filter_enabled` 값이 `false`면 filter 없이 목표가 즉시 반영됩니다. parameter는
[Parameters](09_parameters.md) 2.2절에 있습니다.

### 4.6 Broadcasters missing on mock

mock에서 `HandStateBroadcaster`와 `DiagnosticsBroadcaster`가 없는 것은 정상입니다. mock은 tactile,
diagnostics, command echo state interface를 export하지 않으므로 두 broadcaster가 claim할 대상이 없습니다.
mock이 올리는 controller는 [Launch files](10_launch_files.md) 3장에 있습니다.

## 5. Hardware component

hardware component 층의 증상은 service와 SDK lifecycle에서 나옵니다.

### 5.1 Service does not exist

```text
waiting for service to become available...
```

`/left_hand_control/home` service가 없는 원인은 다음 중 하나입니다.

- mock이나 isaac backend입니다. service는 로봇 핸드 backend에만 있습니다.
- hardware component가 configure되지 않았습니다. service node는 `on_configure`에서 시작됩니다.
- 자기 URDF에서 매크로의 `name` parameter를 바꿨습니다. service는 `/<name>/home`입니다.
- `ROS_DOMAIN_ID`가 다릅니다.

다음 셋으로 원인을 좁힙니다.

```bash
ros2 service list | grep -E '/(run|stop|home|reconnect)$'
ros2 control list_hardware_components
printenv ROS_DOMAIN_ID
```

### 5.2 home succeeds but homing_state is not Succeeded

`~/home` service의 성공은 시작 접수이고 완료가 아닙니다. 완료는 `homing_state` 값이 `Succeeded`로 바뀌는
것으로 확인하며, 제한 시간과 완료 조건은 [3. home](07_services.md#3-home)에 있습니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --field homing_state
```

- `InProgress`가 계속되면 finger가 막혀 hard stop에 닿지 못하는 상태입니다.
- `Failed`면 `actuator_fault_name` 필드에서 fault가 있는 actuator를 확인하고 원인을 제거한 뒤 `~/home`
  service를 다시 호출합니다.
- `NotRun`으로 돌아갔다면 직전에 `~/reconnect` service를 호출한 경우입니다. `~/run` service 뒤 `~/home`
  service를 다시 호출합니다.

### 5.3 Topics keep publishing but the robot hand has stopped

broadcaster는 state interface의 마지막 값을 같은 주기로 반복 발행하므로 topic이 계속 와도 SDK가 정지한
상태일 수 있습니다. 값의 변화로 판단합니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --field lifecycle
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --field control_cycles
```

`lifecycle` 값이 `Faulted`이면 통신 오류나 제어·통신 루프 예외로 정지한 것이고, `control_cycles` 값이
`Faulted`로 전이한 시점부터 멈춥니다. 복구 절차는 [Services](07_services.md) 4장에 있습니다. `auto_reconnect=true`이면 복구
중에도 마지막 state가 반복 발행됩니다.

### 5.4 reconnect fails

```text
success: False
```

`~/reconnect` service는 `lifecycle` 값이 `Faulted`일 때만 성공합니다. 통신이 살아 있는 상태에서 연결을
다시 수립하려면 hardware component를 inactive → active로 전이합니다.

```bash
ros2 control set_hardware_component_state left_hand_control inactive
ros2 control set_hardware_component_state left_hand_control active
```

`Faulted`인데도 실패하면 300 ms 안에 첫 state frame이 오지 않은 것입니다. 전원과 CAN을
[6. Communication](#6-communication)으로 확인한 뒤 다시 호출합니다.

### 5.5 stop fails

`~/stop` service가 500 ms 안에 actuator quick stop을 확인하지 못하면 `success` 필드가 `false`이고
`lifecycle` 값은 `Running`으로 남습니다. actuator가 마지막 command를 유지하고 있을 수 있습니다.

> [!WARNING]
> `~/stop` service를 반복 호출하는 대신 로봇 핸드의 전원을 차단하십시오. CAN이 끊긴 상태라면 quick stop
> frame이 drive에 닿지 않습니다.

### 5.6 Real-time warning

```text
SCHED_FIFO not applied (need privileges) — continuing without realtime scheduling
```

SDK가 제어·통신 루프를 `SCHED_FIFO` priority 90으로 올리지 못한 경우입니다. 동작은 계속되지만
`deadline_misses` 값이 늘어납니다. [Real-time kernel setup](01_real_time_kernel_setup.md) 2장의 `realtime`
group과 limit을 확인합니다.

```bash
id -nG          # realtime 포함
ulimit -r       # 99
ulimit -l       # unlimited
```

`rt_cpu_affinity` parameter에 없는 CPU 번호를 주면 SDK가 경고 없이 요청을 버립니다. `realtime scheduling
applied` log의 `CPU affinity` 값과 `running on CPU` 값이 다르면 적용되지 않은 것입니다. `nproc`과
`lscpu --extended`로 번호를 확인하고, 확실하지 않으면 `-1`로 둡니다.

## 6. Communication

CAN 층의 문제는 wrapper와 무관하게 [1.4 Verify the link](02_can_fd_setup.md#14-verify-the-link)의 절차로
확인합니다.

```bash
ip -details -statistics link show can0
candump -n 5 can0
```

`state`가 `ERROR-ACTIVE`이고 `berr-counter`가 오르지 않으며, 로봇 핸드 전원이 켜져 있을 때 `0x2xx`(왼손)
또는 `0x1xx`(오른손) frame이 500 Hz로 들어오면 정상입니다. `BUS-OFF`·`ERROR-PASSIVE`이거나 frame이 없으면
배선, termination, bitrate, 전원을 점검합니다.

## 7. Support bundle

지원을 요청할 때 다음을 함께 보냅니다.

- wrapper와 SDK의 git commit, `ros2 pkg list | grep aidin_hand2_`의 version
- Ubuntu, ROS 2 distribution, `uname -a`
- launch 명령과 config YAML
- `ros2 doctor --report`
- `ros2 control` list 3종의 출력
- `hand_state`와 `hand_diagnostics` topic의 sample 또는 bag
- `/rosout` topic과 `journalctl -b`
- `ip -details -statistics link show can0`과 `candump` 기록
- 재현 순서와 취한 안전 조치

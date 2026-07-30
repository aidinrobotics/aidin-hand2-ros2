# 시작하기

이 문서는 Ubuntu 22.04·ROS 2 Humble 환경에서 SDK와 wrapper를 build하고 mock을 먼저 통과한 뒤 실제 hand를 의도적으로 homing하는 절차입니다.

## 1. 사전 조건

- Ubuntu 22.04
- ROS 2 Humble desktop 또는 필요한 base package
- `ros-dev-tools`, `rosdep`, `colcon`
- AIDIN Hand Gen2 SDK source
- AIDIN Hand Gen2 ROS 2 wrapper source
- 실물 사용 시 CAN-FD adapter와 안전한 작업 공간

ROS 2 Humble의 Ubuntu binary package는 Ubuntu 22.04 Jammy를 대상으로 합니다. ROS 2가 아직 없다면 [공식 Humble Ubuntu 설치 절차](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로 설치하십시오.

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

결과가 `humble`인지 확인합니다.

## 2. Workspace layout

권장 layout:

```text
~/your_ws/
├── src/
│   └── aidin-hand2-ros2/
├── build/
├── install/
└── log/
```

SDK는 이 workspace에 포함되지 않습니다. Plain CMake package라 colcon workspace 밖에서 따로 build·install하고, wrapper는 설치된 결과를 `find_package(aidin_hand2)`로 찾습니다. Build tree는 SDK 문서의 표준 위치인 `cpp/build`, install prefix는 system prefix `/usr/local`입니다. SDK repository를 편의상 `src/` 아래에 두더라도 root의 `COLCON_IGNORE` 때문에 colcon은 무시합니다.

## 3. Dependency 설치

SDK dependency:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libeigen3-dev \
  libspdlog-dev \
  can-utils \
  ros-dev-tools \
  ros-humble-ros2controlcli
```

ROS package dependency를 설치합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install \
  --from-paths src/aidin-hand2-ros2 \
  --ignore-src \
  --recursive \
  --rosdistro humble \
  --skip-keys "aidin_hand2 manus_ros2_msgs" \
  -y
```

`aidin_hand2`는 rosdep key가 아니라 아래에서 설치하는 CMake package입니다. `manus_ros2_msgs`는 optional glove integration이지만 `package.xml`에는 mandatory dependency로 선언돼 있어 일반 환경에서는 skip합니다.

`ros-humble-ros2controlcli`는 이 문서의 `ros2 control ...` 진단·전환 명령에 필요합니다. 현재 wrapper package metadata가 CLI 자체를 runtime dependency로 선언하지 않으므로 명시적으로 설치합니다.

## 4. SDK build와 설치

SDK repository root에서 build하고 install합니다. Wrapper는 web bridge를 쓰지 않으므로 꺼서 build 시간을 줄입니다.

```bash
cd <aidin-hand2-sdk clone 경로>

cmake -S cpp -B cpp/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DAIDIN_HAND2_BUILD_WEB_BRIDGE=OFF

cmake --build cpp/build -j"$(nproc)"
ctest --test-dir cpp/build --output-on-failure
sudo cmake --install cpp/build
```

Package config를 확인합니다.

```bash
test -f /usr/local/lib/cmake/aidin_hand2/aidin_hand2Config.cmake
```

`/usr/local`은 CMake 기본 탐색 경로이므로 `CMAKE_PREFIX_PATH` 설정이 필요 없습니다.

Sudo를 쓰지 않으려면 사용자 prefix에 install하고 그 경로를 새 terminal마다 `CMAKE_PREFIX_PATH`에 넣습니다. 자동화할 때는 workspace-specific setup script에 넣고 global shell profile에 hard-code하지 않는 편이 version 관리에 안전합니다.

```bash
cmake --install cpp/build --prefix "$HOME/.local"
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

## 5. Wrapper build

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Overlay를 적용합니다.

```bash
source install/setup.bash
ros2 pkg list | grep '^aidin_hand2_'
```

다음 6개 package가 보여야 합니다.

```text
aidin_hand2_bringup
aidin_hand2_controllers
aidin_hand2_description
aidin_hand2_examples
aidin_hand2_hardware
aidin_hand2_msgs
```

> [!NOTE]
> 이 repository에는 현재 automated ROS 2 test가 없습니다. `colcon test`가 통과하더라도 wrapper behavior를 검증하는 test case가 있다는 뜻은 아닙니다. 아래 mock smoke test를 별도로 수행하십시오.

## 6. Mock smoke test

Mock은 CAN, drive와 homing 없이 real과 같은 98개 command port와 exact mode switch를
검증합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
```

기본값은 왼손 mock 하나, RViz on, controller manager 500 Hz,
`joint_state_broadcaster`와 `left_joint_position_controller` active입니다. 나머지 세 basic
controller는 설정에 등록되어 필요할 때 load할 수 있습니다.

다른 terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash

ros2 control list_hardware_components
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

손 하나당 hardware command interface가 정확히 98개인지 확인하고 complete typed command를
보냅니다.

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
    speed_rad_s: 0.5}"
```

Mock도 mode 진입 때 position mode를 현재 pose로, effort mode를 0%로 seed합니다. Real과 같은
네 complete command port를 지원하지만 tactile, hardware diagnostics, command echo,
homing·reconnect service는 제공하지 않습니다. Default launch는 필요한 state가 없으므로
`HandStateBroadcaster`와 `DiagnosticsBroadcaster`를 spawn하지 않습니다.

## 7. 실물 host 준비

실물 실행 전에 SDK의 [Ubuntu 22.04 host setup](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/02_host_setup.md)을 완료합니다. 해당 문서가 다음 항목의 단일 기준입니다.

- Ubuntu Pro PREEMPT_RT 설치와 재부팅 검증
- realtime group, `rtprio 99`, memory lock
- CAN-FD 1 Mbit/s / 5 Mbit/s
- `restart-ms 100`
- udev·systemd boot 자동 설정
- Scheduling latency 측정
- Optional CPU latency tuning

여기서는 결과만 확인합니다.

```bash
cat /sys/kernel/realtime
ulimit -r
ulimit -l
ip -details -statistics link show can0
timeout 3 candump -L can0
```

`/sys/kernel/realtime=1`, RT priority 90 이상, CAN-FD와 지속 frame을 확인합니다.

## 8. Config 검토

배포된 default config를 확인합니다.

```bash
ros2 pkg prefix --share aidin_hand2_bringup
sed -n '1,220p' \
  "$(ros2 pkg prefix --share aidin_hand2_bringup)/config/hand_bringup.yaml"
```

Default YAML은 `auto_home=true`, `auto_reconnect=true`, timeout 0, reconnect home true입니다. Commissioning에서는 CLI로 안전하게 덮습니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false \
  auto_reconnect:=false \
  left_hand_interface:=can0 \
  left_hand_cpu_affinity:=-1
```

CPU 4가 실제로 있고 격리·배치가 검증된 host에서는 default affinity를 사용할 수 있습니다. 작은 host에서 존재하지 않는 CPU 번호를 그대로 사용하지 마십시오.

## 9. 첫 실제 bringup

> [!WARNING]
> 다음 launch는 `auto_home=false`여도 `on_activate → SDK run()`으로 drive를 enable합니다. Joint command는 homing 전까지 wrapper가 보내지 않지만 hardware safety 상태와 주변을 먼저 확인해야 합니다.

Terminal A:

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash

ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false \
  auto_reconnect:=false \
  left_hand_interface:=can0 \
  left_hand_cpu_affinity:=-1
```

Terminal B:

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash

ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics --once
```

기대 상태:

- Hardware component `left_hand_control` active
- `joint_state_broadcaster` active
- `left_hand_state_broadcaster` active
- `left_diagnostics_broadcaster` active
- `left_joint_position_controller` active
- 나머지 command controller 3개 inactive
- Diagnostics lifecycle `Running`
- Diagnostics homed `false`

## 10. Homing

Workspace를 비우고 service를 호출합니다.

```bash
ros2 service call \
  /left_hand_control/home \
  std_srvs/srv/Trigger
```

응답 예:

```text
success: true
message: homing started — poll diagnostics 'homed'
```

이는 완료 응답이 아닙니다. Polling합니다.

```bash
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics
```

`homed: true`와 empty actuator fault를 확인합니다.

## 11. 첫 command

먼저 max effort를 낮추고 target과 speed를 하나의 typed command로 보냅니다.

```bash
ros2 topic pub --once \
  /left_hand_control/set_max_effort \
  std_msgs/msg/Float64 \
  "{data: 300.0}"

ros2 topic pub --once \
  /left_joint_position_controller/command \
  aidin_hand2_msgs/msg/JointPositionCommand \
  "{target_position_rad: [
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.10, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0],
    speed_rad_s: 0.25}"
```

단위는 rad와 rad/s입니다. Speed 0은 정지가 아니라 즉시 추종입니다. SDK
`set_command()`가 joint target을 workspace 안으로 자동 clamp하지만, collision·trajectory
limit와 주변 환경 안전은 상위 application의 책임입니다.

## 12. 정상 종료

먼저 command controller를 deactivate합니다.

```bash
ros2 control switch_controllers \
  --deactivate left_joint_position_controller \
  --strict
```

Hardware stop:

```bash
ros2 service call \
  /left_hand_control/stop \
  std_srvs/srv/Trigger
```

Launch process에는 `SIGINT`를 보내 lifecycle cleanup이 실행되게 합니다. `SIGKILL`을 정지 방법으로 사용하지 마십시오.


다음으로 [Launch reference](02_launch_reference.md)와 [운영과 복구](06_operations.md)를 읽고 production config를 작성하십시오.
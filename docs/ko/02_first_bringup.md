# 첫 bringup

`aidin_hand2_bringup`은 손을 단독 실행하는 예제입니다. 하드웨어가 정상인지 확인하는
용도이며 통합 경로가 아닙니다. 자기 로봇에 붙이려면
[ros2_control 설정](03_setup.md)을 보십시오.

Build와 설치는 [설치](01_installation.md)에서 먼저 마칩니다.

## 1. Mock smoke test

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

## 2. 실물 host 준비

실물 실행 전에 SDK의 [real-time kernel setup](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/04_real_time_kernel_setup.md)과 [CAN-FD setup](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/05_can_fd_setup.md)을 완료합니다. 해당 문서가 다음 항목의 단일 기준입니다.

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

## 3. Config 검토

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

Default는 `-1`(미설정)입니다. RT thread를 특정 코어에 pin하려면 격리·배치를 검증한 뒤 코어 번호를 지정하고, 존재하지 않는 CPU 번호를 쓰지 마십시오.

## 4. 첫 실제 bringup

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
- Diagnostics `homing_state != Succeeded`

## 5. Homing

Workspace를 비우고 service를 호출합니다.

```bash
ros2 service call \
  /left_hand_control/home \
  std_srvs/srv/Trigger
```

응답 예:

```text
success: true
message: homing started — poll diagnostics 'homing_state'
```

이는 완료 응답이 아닙니다. Polling합니다.

```bash
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics
```

`homing_state: Succeeded`와 empty actuator fault를 확인합니다.

## 6. 첫 command

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

## 7. 정상 종료

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


다음으로 [Bringup 예제](05_bringup_example.md)와 [운영과 복구](06_operations.md)를 읽고 production config를 작성하십시오.

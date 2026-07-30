# 문제 해결

문제를 다음 경계 순서로 분리합니다.

```text
Build → package discovery → launch/xacro → controller_manager
     → hardware plugin → SDK lifecycle → SocketCAN → physical hand
```

손이 안전하지 않게 움직이면 log 수집보다 E-stop·전원 차단 절차를 먼저 수행합니다.

## 1. 기본 수집

```bash
printenv ROS_DISTRO
ros2 doctor --report
ros2 pkg prefix aidin_hand2_bringup
ros2 control list_hardware_components
ros2 control list_controllers
ros2 control list_hardware_interfaces
ip -details -statistics link show can0
```

Launch command, 최종 config YAML, `/rosout`, hand diagnostics와 발생 시각을 보존합니다.

## 2. `find_package(aidin_hand2)` 실패

대표 증상:

```text
Could not find a package configuration file provided by "aidin_hand2"
```

확인:

```bash
find /usr/local ~/.local \
  -name aidin_hand2Config.cmake -print 2>/dev/null
printf '%s\n' "$CMAKE_PREFIX_PATH" | tr ':' '\n'
```

SDK를 install한 뒤 wrapper를 build합니다.

```bash
cd <aidin-hand2-sdk clone 경로>
sudo cmake --install cpp/build

cd ~/your_ws
colcon build --symlink-install
```

사용자 prefix(`--prefix "$HOME/.local"`)에 install했다면 그 경로를 `CMAKE_PREFIX_PATH`에 export한 shell에서 build해야 합니다. `/usr/local`은 CMake 기본 탐색 경로라 export가 필요 없습니다. SDK build tree(`cpp/build`) 자체를 prefix로 넣지 말고 install prefix를 사용하십시오.

## 3. `rosdep`이 `aidin_hand2` 또는 `manus_ros2_msgs`에서 실패

SDK는 rosdep package가 아니며 MANUS integration은 CMake에서 optional입니다.

```bash
rosdep install \
  --from-paths src/aidin-hand2-ros2 \
  --ignore-src \
  --recursive \
  --rosdistro humble \
  --skip-keys "aidin_hand2 manus_ros2_msgs" \
  -y
```

Glove를 실제로 쓸 때만 호환되는 `manus_ros2_msgs` package를 workspace에 추가합니다.

## 4. Package 또는 launch를 찾지 못함

새 terminal에서 overlay source를 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
ros2 pkg prefix aidin_hand2_bringup
```

다른 workspace overlay가 같은 package 이름을 가리는지 확인합니다.

```bash
printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n'
```

### `robot_description` deprecation warning

실제·mock launch 모두 Humble에서 `Passing the robot description parameter directly ... is deprecated` warning을 출력할 수 있습니다. 현재 launch가 `robot_description` parameter를 직접 전달하기 때문이며 controller와 hardware가 이어서 정상 configure·activate되면 즉시 실패 원인은 아닙니다. Migration 상태는 [알려진 제한](07_known_limitations.md#15-controller-manager-integration-debt)을 참조하십시오.

## 5. Mock에서 expected controller가 없음

Default mock에서 active인 controller는 두 개입니다.

```text
joint_state_broadcaster
left_joint_position_controller
```

다른 세 basic controller도 `controllers_mock.yaml`에 등록되어 필요할 때 load할 수 있습니다.
Mock에는 `HandStateBroadcaster`와 `DiagnosticsBroadcaster`가 없습니다.
[Mock 범위](04_interfaces.md#10-mock-범위)를 참조하십시오.

RViz가 없으면:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py \
  use_rviz:=true
```

Display server와 RViz dependency를 확인합니다. Headless smoke test는 `use_rviz:=false`를 사용합니다.

Basic controller command는 mock과 실물 모두 partial update를 받지 않습니다. 각 controller의
typed `~/command`에 16개 target과 필요한 speed/gain 전체를 보내십시오.

## 6. Hardware component configure 실패

SDK log가 ROS logger `aidin_hand2`로 중계됩니다.

```bash
ros2 topic echo /rosout
```

원인 분류:

| SDK ErrorCode | 조사 |
|---|---|
| `InvalidArgument` | Xacro parameter, disabled index, rate |
| `InterfaceUnavailable` | CAN 이름·up·권한·점유 |
| `CommunicationLost` | Frame 수신·전원·배선 |
| `HardwareFault` | Drive fault와 기구 |

CAN layer:

```bash
ip -brief link
ip -details -statistics link show can0
timeout 3 candump -L can0
```

전체 host 절차는 [SDK troubleshooting](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/09_troubleshooting.md)을 따르십시오.

## 7. Default인데 `can0`가 아니라 `auto`를 사용함

정상 top-level launch는 default `hand_bringup.yaml`을 읽고 `left_hand_interface=auto`를 적용합니다. `can0`은 source fallback입니다.

명시적으로 고정:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  left_hand_interface:=can0 \
  auto_home:=false
```

Config 우선순위는 [Launch reference](02_launch_reference.md#3-config-우선순위)를 참조하십시오.

## 8. Launch 직후 손이 움직임

Default YAML의 `auto_home=true` 때문입니다. `Ctrl-C`나 network disconnect만으로 안전 정지를 보장하지 않습니다. External stop을 사용하고 다음 bringup에서:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false \
  auto_reconnect_home:=false
```

운영 config에서 자동 움직임을 명시적으로 review합니다.

## 9. Affinity warning 또는 RT warning

Default YAML은 left CPU 4, right CPU 6입니다.

```bash
nproc
lscpu --extended
```

CPU가 없거나 검증되지 않았으면:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false \
  left_hand_cpu_affinity:=-1
```

PREEMPT_RT와 permission은 [SDK host setup](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/02_host_setup.md)을 완료합니다. SDK 실제 priority는 90입니다.

## 10. `/left_hand_control/home` service가 없음

가능한 원인:

- Mock launch를 사용 중
- Hardware component configure 실패
- Custom xacro에서 system name 변경
- Service node가 cleanup됨
- 다른 ROS domain

```bash
ros2 service list | grep -E '/(run|stop|home|reconnect)$'
ros2 control list_hardware_components
printenv ROS_DOMAIN_ID
```

Default 실제 hardware system name에서만 `/left_hand_control/...`가 됩니다.

## 11. Home service success인데 `homed=false`

Service는 non-blocking trigger입니다.

```bash
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics
```

Lifecycle, actuator fault, `control_cycles`와 SDK log를 확인합니다. Reconnect 직후에는 homed가 false로 reset됩니다.

## 12. Topic publish는 성공하지만 움직이지 않음

순서대로 확인합니다.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics --once
ros2 topic echo \
  /left_hand_state_broadcaster/hand_state --once
```

필수 조건:

- Hardware component active
- SDK lifecycle `Running`
- `homed=true`
- 원하는 command controller active
- Joint·actuator name 정확
- Message field 정확
- Auto reconnect 또는 homing 진행 중 아님

Wrapper는 homing 중 또는 homed false일 때 command를 error 없이 skip합니다.

## 13. Joint가 너무 빠르게 움직임

Speed 0은 정지가 아니라 무제한입니다.

Speed는 별도 topic이 아니라 `JointPositionCommand.speed_rad_s`에 target 16개와 함께
들어갑니다. [Joint position command 예제](03_usage.md#joint-position)처럼 complete message를
다시 보내십시오. Target 단위는 rad, speed는 rad/s입니다.

## 14. Out-of-range target이 clamp되지 않음

SDK `set_command()`는 joint position·impedance target을 자동 workspace clamp합니다.
`HandState.command_state`의 해당 controller input이 clamp 결과인지 확인하십시오.

Clamp는 self-collision, 외부 장애물과 trajectory 속도 정책을 대신하지 않습니다. 기대한
workspace 경계와 다르면 SDK [Workspace Clamp](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/05_workspace_clamp.md)
문서와 사용 중인 SDK build를 확인하십시오.

## 15. Controller switch 실패

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Command controller는 하나만 active여야 합니다.

```bash
ros2 control switch_controllers \
  --deactivate left_joint_position_controller \
  --activate left_joint_impedance_controller \
  --strict
```

새 controller가 `unconfigured` 또는 `finalized`이면 launch spawner log를 확인합니다. Mock에서도
다른 세 basic controller를 load할 수 있으며 real과 같은 `command_lock`/complete-set 규칙을
적용합니다.

## 16. Impedance gain이 반영되지 않음

Message array가 각각 정확히 16개인지 확인합니다.

```bash
ros2 topic info \
  /left_joint_impedance_controller/command --verbose
```

`JointImpedanceCommand` 하나에 target, stiffness, damping 각 16개를 모두 넣어야 합니다.
값은 finite이고 gain은 0 이상이어야 합니다. Controller가 active인지, chained mode에서
upstream이 reference를 덮는지도 확인합니다.

## 17. Actuator command 일부가 반영되지 않음

Actuator controller도 partial name/value mapping을 하지 않습니다.

- Position: `ActuatorPositionCommand.target_position_cnt[16]`
- Effort: `ActuatorEffortCommand.target_effort_pct[16]`

16개 전체를 SDK actuator 순서로 보내십시오. NaN/Inf는 거부되고 actuator position은 SDK에서
int32 범위를 검증합니다.

## 18. Topic은 계속 오는데 hand가 멈춤

Auto reconnect 중 stale snapshot일 수 있습니다.

다음을 시간에 따라 비교합니다.

```bash
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics
```

- Lifecycle
- `control_cycles`
- HandState header stamp
- SocketCAN RX counter

Broadcaster 수신 여부가 아니라 underlying 값 변화로 health를 판정합니다. [운영과 복구](06_operations.md#5-health-monitoring)를 참조하십시오.

## 19. `/diagnostics`가 OK인데 motion-ready가 아님

Standard level은 homed false, stale state와 deadline miss rate를 반영하지 않습니다. Custom supervisor가 별도 gate를 적용해야 합니다.

```bash
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics --once
```

Lifecycle, homed, cycle, timing과 actuator array를 직접 확인합니다.

## 20. `description.launch.py`로 mock이 되지 않음

현재 description launch가 잘못된 `use_mock_hardware` argument를 전달합니다. Mock control에는 다음을 사용합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
```

상세는 [알려진 제한](07_known_limitations.md#9-descriptionlaunchpy-mock-argument-typo)에 있습니다.

## 21. Rosbridge GUI 연결 실패

```bash
ros2 launch aidin_hand2_bringup gui_bridge.launch.py
ros2 node list | grep rosbridge
ss -ltn | grep 9090
```

Remote browser에서 `localhost`는 robot host가 아니라 browser host입니다. Robot IP를 사용하되 port를 신뢰할 수 없는 network에 공개하지 마십시오.

## 22. 지원 요청 bundle

- Wrapper와 SDK Git commit
- Package version 목록
- Ubuntu, ROS distribution, kernel
- Launch command와 config YAML
- `ros2 doctor --report`
- Hardware·controller·interface list
- HandState·HandDiagnostics sample과 time series
- `/rosout`, `/diagnostics`, process journal
- SocketCAN statistics와 CAN capture
- 재현 순서와 안전 조치

Root `LICENSE` 부재, dynamic state schema와 test 부재는 runtime troubleshooting으로 해결할
수 없습니다. [알려진 제한](07_known_limitations.md)의 acceptance status를 함께 제출하십시오.

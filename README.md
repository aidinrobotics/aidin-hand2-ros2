# AIDIN Hand Gen2 ROS 2

AIDIN Hand Gen2 C++ SDK를 ROS 2 Humble과 `ros2_control`에 연결하는 thin wrapper입니다. CAN-FD protocol, drive state machine, kinematics와 500 Hz hand control loop는 SDK가 소유하며 이 repository는 hardware plugin, controller, message, URDF와 launch를 제공합니다.

> [!CAUTION]
> 기본 launch는 왼손 한 개를 활성화하고 `auto_home=true`로 시작합니다. Launch 직후 실제 hand가 움직입니다. 처음에는 반드시 `auto_home:=false`로 실행하고 작업 공간·E-stop·diagnostics를 확인한 뒤 `/left_hand_control/home`을 호출하십시오.

## 지원 범위

| 항목 | 대상 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Control framework | `ros2_control` |
| SDK | `aidin_hand2` C++ SDK 0.1.x |
| Hardware transport | Linux SocketCAN, CAN-FD 1 Mbit/s / 5 Mbit/s |

## Package

| Package | 역할 |
|---|---|
| `aidin_hand2_hardware` | 실제·mock `SystemInterface`, SDK lifecycle mapping |
| `aidin_hand2_controllers` | 4개 command controller, 2개 broadcaster |
| `aidin_hand2_msgs` | 4개 typed command, `CommandState`, `HandState`, `HandDiagnostics` |
| `aidin_hand2_description` | URDF, xacro, mesh, ros2_control description |
| `aidin_hand2_bringup` | 실제·mock launch와 controller config |
| `aidin_hand2_examples` | 4개 chainable 상위 controller skeleton, optional MANUS glove teleop controller |

## 명령 경로와 chaining

새 controller 계층을 추가하지 않습니다. 기존 네 basic controller가 ROS topic 또는 상위
controller reference를 SDK의 완전한 typed command로 바꾸는 command-port adapter입니다.

```text
상위 controller
  → basic controller가 export한 reference interface
  → 기존 basic controller
  → mode별 complete hardware command port + claim-only command_lock
  → real/mock SystemInterface
  → SDK ControllerCommand
```

Standalone에서는 각 controller의 `~/command`에 한 cycle의 target과 부속값을 모두 담아
보냅니다. Joint position은 target 16개와 speed, joint impedance는 target 16개와
stiffness·damping 각 16개가 한 message입니다. Partial update는 허용하지 않습니다.

상위 controller용 reference 계약과 기본값을 만들지 않는 네 skeleton은
[Chainable controller examples](aidin_hand2_examples/EXAMPLE.md)에 있습니다. 예제는 controller
성격별로 나뉘어 있고(`src/upper_controllers/`, `src/glove_teleop/`), 네 skeleton 모두
전체 `HandState`를 realtime buffer에서 멤버로 복사한 뒤, 대응 basic controller로 보낼
완전한 reference 묶음만 출력하도록 구성돼 있습니다.

## 빠른 시작

SDK를 먼저 build하고 install합니다. Build tree와 install prefix는 SDK 문서의 표준 절차와
같습니다 — build는 SDK repository의 `cpp/build`, install은 system prefix `/usr/local`입니다.
Web bridge는 wrapper가 쓰지 않으므로 꺼서 build 시간을 줄입니다.

```bash
cd <aidin-hand2-sdk clone 경로>

cmake -S cpp -B cpp/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DAIDIN_HAND2_BUILD_WEB_BRIDGE=OFF
cmake --build cpp/build -j"$(nproc)"
sudo cmake --install cpp/build
```

Sudo를 쓰지 않으려면 사용자 prefix에 install하고 그 경로를 `CMAKE_PREFIX_PATH`에 넣습니다.

```bash
cmake --install cpp/build --prefix "$HOME/.local"
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

ROS 2 environment를 적용하고 workspace root에서 wrapper를 build합니다. `/usr/local`에
install했다면 CMake 기본 탐색 경로이므로 `CMAKE_PREFIX_PATH` 설정이 필요 없습니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

실물보다 먼저 mock을 실행합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Mock에서는 `joint_state_broadcaster`와 `left_joint_position_controller`만 active입니다. 실제 hand state·diagnostics broadcaster와 runtime service는 없습니다.

실물 host의 PREEMPT_RT와 boot-time CAN-FD 설정은 SDK의 [Ubuntu 22.04 host setup](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/02_host_setup.md)을 먼저 완료하십시오.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false
```

다른 terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics --once
```

Lifecycle이 `Running`이고 workspace가 안전할 때 homing을 trigger합니다.

```bash
ros2 service call \
  /left_hand_control/home \
  std_srvs/srv/Trigger
```

Service 응답은 시작 접수만 의미합니다. Completion은 `hand_diagnostics.homed: true`로 확인합니다.

## 문서 지도

| 문서 | 범위 |
|---|---|
| [시작하기](docs/ko/01_getting_started.md) | SDK·wrapper build, mock, 실제 hand 첫 실행 |
| [Launch reference](docs/ko/02_launch_reference.md) | Config 우선순위, launch argument와 실제 기본값 |
| [사용법](docs/ko/03_usage.md) | Command publish, controller mode 전환, service |
| [Interface reference](docs/ko/04_interfaces.md) | Topic·service·message·ros2_control interface 계약 |
| [Interface matrix](docs/ko/05_interface_matrix.md) | Command·state·reference interface 전체 이름 목록(생략 없음) |
| [Chainable 예제](aidin_hand2_examples/EXAMPLE.md) | 네 basic controller별 상위 controller skeleton |
| [운영과 복구](docs/ko/06_operations.md) | Lifecycle, auto reconnect, RT, monitoring, shutdown |
| [알려진 제한](docs/ko/07_known_limitations.md) | Mock 범위, xacro state schema, watchdog·license 제한 |
| [문제 해결](docs/ko/08_troubleshooting.md) | Build, launch, controller, stale state, CAN 분리 진단 |

## Lifecycle mapping

| ros2_control callback | SDK operation | 의미 |
|---|---|---|
| `on_configure` | `create()` + `connect()` | Resource 생성, 첫 CAN frame 확인, listen-only |
| `on_activate` | `run()` | Drive enable, 이후 read cycle에서 optional homing |
| `on_deactivate` | `stop()` | Blocking quick stop |
| `on_cleanup` | `disconnect()` + `destroy()` | 통신·resource 해제 |

Wrapper는 SDK config의 blocking `auto_home`을 강제로 끕니다. ROS `auto_home`은 `start_homing()`을 한 번 trigger하므로 controller manager executor를 block하지 않습니다.

## 기본 runtime

Normal default YAML 기준:

- Left: enabled, interface `auto`, SDK RT affinity CPU 4
- Right: disabled, interface `auto`, SDK RT affinity CPU 6
- `auto_home=true`
- `auto_reconnect=true`
- Reconnect timeout 0, 즉 무제한
- `auto_reconnect_home=true`
- Controller manager 500 Hz
- Joint position controller active
- Actuator position·effort와 joint impedance controller loaded inactive
- Hand state 100 Hz, diagnostics 20 Hz, joint state 100 Hz

> [!WARNING]
> Auto reconnect 중 wrapper는 ros2_control component를 살려 두기 위해 SDK read/write exception을 `OK`로 처리하고 마지막 state를 계속 publish할 수 있습니다. Header stamp와 `control_cycles`가 증가하는지 확인하지 않고 topic 수신만으로 health를 판정하지 마십시오.

## 책임 경계

ROS 2 문서는 wrapper가 추가하는 topic, service, controller와 launch behavior만 설명합니다. 다음 항목은 SDK 문서가 기준입니다.

- CAN-FD bit timing과 host service
- SDK lifecycle와 ErrorCode
- Command latch, max effort, automatic clamp의 현재 상태
- Kinematics, actuator·joint·tactile layout
- Communication loss와 drive hold 위험
- Logging과 diagnostics 원본 계약

관련 문서:

- [SDK C++ guide](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/03_cpp_guide.md)
- [SDK safety](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/07_safety.md)
- [SDK observability](https://github.com/JJhyeongg/aidin-hand2-sdk/blob/main/docs/ko/06_observability.md)

## License 상태

현재 repository root에 `LICENSE` file이 없고 6개 package 중 5개의 `package.xml` license가 `TODO`입니다. `aidin_hand2_description`만 `Apache-2.0`으로 선언돼 있습니다. Repository 전체를 Apache-2.0으로 추정하지 말고 재배포 전에 권리 조건을 확인하십시오.

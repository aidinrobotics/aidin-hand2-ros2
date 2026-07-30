# Launch reference

이 문서는 제공 launch file, config 우선순위와 실제 default를 정리합니다. Command와 service는 [사용법](03_usage.md), runtime recovery는 [운영과 복구](06_operations.md)를 참조하십시오.

## 1. 제공 launch

| Launch file | 목적 |
|---|---|
| `aidin_hand2.launch.py` | 실제 CAN hardware와 controller 전체 bringup |
| `aidin_hand2_controllers.launch.py` | 이미 존재하는 controller manager에 controller spawn |
| `aidin_hand2_mock.launch.py` | 왼손 mock + joint position + optional RViz·glove |
| `gui_bridge.launch.py` | GUI를 위한 rosbridge WebSocket |
| `description.launch.py` | Robot description과 RViz |

## 2. 실제 hardware launch

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py
```

구성:

```text
xacro robot_description
├── ros2_control_node (/controller_manager)
├── robot_state_publisher
└── controller spawner
    ├── joint_state_broadcaster
    ├── <side>_hand_state_broadcaster
    ├── <side>_diagnostics_broadcaster
    ├── <side>_joint_position_controller       active
    ├── <side>_actuator_position_controller    inactive
    ├── <side>_actuator_effort_controller      inactive
    └── <side>_joint_impedance_controller      inactive
```

Command controller 4종은 같은 hardware의 서로 다른 mode를 claim하므로 한 순간 하나만 active여야 합니다.

## 3. Config 우선순위

최종 launch argument는 다음 우선순위로 결정됩니다.

```text
CLI key:=value
        ▼
config YAML의 같은 key
        ▼
launch source의 hard-coded fallback
```

Default config path:

```text
<aidin_hand2_bringup share>/config/hand_bringup.yaml
```

다른 file:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  config:=/absolute/path/production-hand.yaml
```

한 값만 임시 override:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  config:=/absolute/path/production-hand.yaml \
  auto_home:=false
```

Config YAML은 ROS parameter file이 아닙니다. 최상위 `key: value`를 launch argument 이름과 1:1로 적습니다.

## 4. Normal default

Repository를 인자 없이 실행하면 제공 `hand_bringup.yaml`이 hard-coded fallback을 덮습니다.

| Argument | Default YAML | 의미 |
|---|---:|---|
| `use_left_hand` | `true` | 왼손 system 생성 |
| `use_right_hand` | `false` | 오른손 system 생성 안 함 |
| `left_hand_interface` | `auto` | Side ID로 CAN interface scan |
| `right_hand_interface` | `auto` | Side ID로 CAN interface scan |
| `left_hand_disabled_actuators` | `""` | 전 actuator 사용 |
| `right_hand_disabled_actuators` | `""` | 전 actuator 사용 |
| `left_hand_cpu_affinity` | `4` | Left SDK RT thread CPU 4 |
| `right_hand_cpu_affinity` | `6` | Right SDK RT thread CPU 6 |
| `auto_home` | `true` | Activation 후 non-blocking homing |
| `auto_reconnect` | `true` | 통신 두절 SDK 자동 복구 |
| `auto_reconnect_timeout_ms` | `0` | 무제한 재시도 |
| `auto_reconnect_home` | `true` | 복구 후 homing |

> [!WARNING]
> Normal default는 launch 직후 hand를 움직이고, 통신이 돌아오면 자동으로 homing·재개할 수 있는 운용 지향 설정입니다. Commissioning default로 사용하지 마십시오.

## 5. Hard-coded fallback

Config file이 없거나 key가 빠졌을 때 source fallback은 다음과 다릅니다.

| Argument | Fallback |
|---|---:|
| Left·Right enable | `true` / `false` |
| Left·Right interface | `can0` / `can1` |
| Left·Right affinity | `-1` / `-1` |
| `auto_home` | `true` |
| `auto_reconnect` | `false` |
| `auto_reconnect_timeout_ms` | `0` |
| `auto_reconnect_home` | `false` |

문서나 운영 절차에서 “default”를 말할 때는 정상 launch가 읽는 default YAML을 기준으로 하십시오.

## 6. 권장 config profile

### Commissioning

```yaml
use_left_hand: true
use_right_hand: false

left_hand_interface: can0
right_hand_interface: can1

left_hand_disabled_actuators: ""
right_hand_disabled_actuators: ""

left_hand_cpu_affinity: -1
right_hand_cpu_affinity: -1

auto_home: false
auto_reconnect: false
auto_reconnect_timeout_ms: 5000
auto_reconnect_home: false
```

### Production 예시

다음 값은 예시일 뿐 risk assessment와 CPU topology에 맞게 결정합니다.

```yaml
use_left_hand: true
use_right_hand: true

left_hand_interface: can0
right_hand_interface: can1

left_hand_disabled_actuators: ""
right_hand_disabled_actuators: ""

left_hand_cpu_affinity: 4
right_hand_cpu_affinity: 6

auto_home: false
auto_reconnect: true
auto_reconnect_timeout_ms: 5000
auto_reconnect_home: false
```

Operator 승인 없이 자동 homing을 허용할 수 있을 때만 `auto_home` 또는 `auto_reconnect_home`을 켭니다. Timeout 0은 무제한이므로 상위 supervisor 없이 사용하지 않는 편이 안전합니다.

## 7. 양손

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  use_left_hand:=true \
  use_right_hand:=true \
  left_hand_interface:=can0 \
  right_hand_interface:=can1 \
  auto_home:=false
```

양손 구성 조건:

- Side별 CAN mapping이 물리적으로 검증돼 있습니다.
- SDK RT affinity가 서로 다르고 실제 CPU입니다.
- Controller manager thread는 SDK RT thread와 다른 CPU를 사용합니다.
- 두 hand의 auto-home을 동시에 수행해도 전원과 기구가 안전합니다.
- Global `/diagnostics`에서 `hardware_id` 또는 custom topic의 `hand_side`로 구분합니다.

`interface=auto`는 편리하지만 cable swap을 운영 configuration change로 감지해야 하는 system에서는 명시적 mapping을 권장합니다.

## 8. Disabled actuator

Comma-separated index:

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false \
  left_hand_disabled_actuators:="0,1,2,3"
```

YAML:

```yaml
left_hand_disabled_actuators: "0,1,2,3"
```

Index map:

```text
0..3   thumb actuator0..3
4..6   index actuator1..3
7..9   middle actuator1..3
10..12 ring actuator1..3
13..15 baby actuator1..3
```

Disabled actuator는 enable하지 않고 fault·homing 대상에서도 제외됩니다. 고장을 숨기는 runtime switch로 사용하지 마십시오.

## 9. `auto_home`의 wrapper 동작

Hardware plugin은 SDK `HandConfig::auto_home`을 항상 `false`로 둡니다. Wrapper argument `auto_home`은 다음처럼 동작합니다.

```text
on_activate → SDK run()
             ↓
다음 read/write cycle에서 homed=false 확인
             ↓
SDK start_homing() 1회 trigger
             ↓
homing 중·homed=false 동안 command write 억제
```

이 방식은 SDK blocking `home()`이 controller manager를 멈추지 않게 합니다. Homing completion은 topic에서 `homed=true`로 확인합니다.

## 10. `auto_reconnect`의 wrapper 동작

Auto reconnect가 켜져 있으면 hardware plugin은 SDK가 재연결하는 동안 read/write error를 ROS 2 `ERROR`로 올리지 않고 `OK`를 반환합니다. 그렇지 않으면 ros2_control이 component를 deactivate하여 SDK 복구 loop가 끝날 수 있기 때문입니다.

결과:

- Controller manager와 broadcaster가 active로 남을 수 있습니다.
- 마지막 state snapshot이 반복 발행될 수 있습니다.
- SDK lifecycle이 `Running`이 아니면 command write를 skip합니다.
- 복구 후 `homed`와 auto-home latch에 따라 motion이 재개됩니다.

Topic이 계속 온다는 이유로 healthy라고 판단하지 말고 stamp, cycle과 lifecycle를 확인하십시오.

## 11. `control_rate`와 `max_effort`

`control_rate`와 initial `max_effort`은 xacro argument에는 있지만 top-level `aidin_hand2.launch.py`의 declared argument가 아닙니다.

Default:

- `control_rate=500`
- `max_effort=1000`

다음 명령은 top-level launch override로 지원되지 않습니다.

```text
control_rate:=1000
max_effort:=300
```

변경하려면 custom robot description/xacro 또는 launch source 구성이 필요합니다. Runtime max effort만 바꾸려면 지원 topic을 사용합니다.

```bash
ros2 topic pub --once \
  /left_hand_control/set_max_effort \
  std_msgs/msg/Float64 \
  "{data: 300.0}"
```

Controller manager `update_rate`와 SDK `control_rate`를 바꿀 때는 둘을 함께 설계하고 검증하십시오.

## 12. Controller config

`controllers.yaml`의 실제 default:

| Parameter | 값 |
|---|---:|
| Controller manager update | 500 Hz |
| Joint state broadcaster | 100 Hz |
| Hand state broadcaster | 100 Hz |
| Diagnostics broadcaster | 20 Hz |
| Joint position speed | 0 rad/s, 즉 무제한 |

SDK code는 `SCHED_FIFO` priority 90을 요청합니다.

ROS 2 Humble Controller Manager는 update loop의 realtime priority를 기본 50으로 구성합니다. `lock_memory`, `cpu_affinity`, `thread_priority`를 조정할 때는 설치한 ros2_control version의 [Controller Manager user documentation](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)을 기준으로 하고 SDK thread priority 90·CPU와 충돌하지 않게 합니다.

## 13. Mock launch

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py \
  use_rviz:=true \
  use_glove:=false
```

| Argument | 기본값 |
|---|---:|
| `use_rviz` | `true` |
| `use_glove` | `false` |

Mock은 항상 왼손 하나이며 실제 launch의 config YAML을 사용하지 않습니다.

`use_glove=true`에는 `manus_ros2_msgs`, glove controller plugin과 별도 MANUS publisher가 필요합니다. Dependency가 없으면 controller target 자체가 build되지 않습니다.

## 14. GUI bridge

```bash
ros2 launch aidin_hand2_bringup gui_bridge.launch.py
```

Default rosbridge WebSocket port는 9090입니다. Rosbridge는 control network에 새로운 remote command surface를 만드므로 authentication·TLS와 network restriction을 별도 설계하십시오.

## 15. Argument 확인

Mock launch의 argument는 다음처럼 확인합니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py --show-args
```

현재 실제 hardware launch는 `OpaqueFunction` 안에서 hand별 argument를 동적으로 declare하므로 `--show-args` 출력에는 `config` 하나만 나타납니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py --show-args
```

전체 key는 설치된 `config/hand_bringup.yaml`과 이 문서의 표를 기준으로 확인합니다. Config file과 CLI override를 함께 기록해야 재현 가능한 incident 분석이 가능합니다.

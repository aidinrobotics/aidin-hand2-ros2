# Bringup 예제

`aidin_hand2_bringup`은 손을 단독 실행하는 예제입니다. 하드웨어 확인용이며 통합 경로가
아닙니다 — 자기 로봇에 붙일 때는 [ros2_control 설정](03_setup.md)을 보십시오.

Launch마다 받는 인자가 다릅니다.

| Launch | 용도 |
|---|---|
| `aidin_hand2.launch.py` | 실물 — hardware + controller 전체 |
| `aidin_hand2_mock.launch.py` | mock — CAN·drive 없이 |
| `aidin_hand2_isaac.launch.py` | Isaac Sim — ROS 2 토픽 브리지 (촉각·diagnostics 포함) |
| `aidin_hand2_controllers.launch.py` | 이미 뜬 controller_manager에 controller만 |
| `gui_bridge.launch.py` | GUI용 rosbridge WebSocket |

## 1. 실물 launch

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py auto_home:=false
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

### 인자

아래 기본값은 배포되는 `hand_bringup.yaml`의 값입니다. 이 파일이 launch가 선언하는 인자
12개를 모두 정의하므로, 정상 실행에서 적용되는 값은 전부 여기서 옵니다. 값을 덮는 순서는
아래 [Config 파일](#config-파일)에 있습니다.

| Argument | 기본값 | 의미 |
|---|---|---|
| `use_left_hand` | `true` | 왼손 system 생성 |
| `use_right_hand` | `true` | 오른손 system 생성 |
| `left_hand_interface` | `can0` | CAN interface. `auto`로 주면 side ID로 채널 탐색 |
| `right_hand_interface` | `can1` | 위와 같음 |
| `left_hand_cpu_affinity` | `-1` | SDK RT thread CPU pin. `-1` = 미설정 |
| `right_hand_cpu_affinity` | `-1` | 위와 같음 |
| `left_hand_disabled_actuators` | `""` | 미가동 actuator index (예: `"0,1,2,3"`) |
| `right_hand_disabled_actuators` | `""` | 위와 같음 |
| `auto_home` | `true` | 기동 직후 자동 homing — **실물이 움직인다** |
| `auto_reconnect` | `false` | 통신 두절 시 SDK 자동 재수립 |
| `auto_reconnect_timeout_ms` | `0` | 재수립 포기 상한 [ms]. `0` = 무제한 |
| `auto_reconnect_home` | `false` | 재수립 복귀 후 homing |

`auto_home`과 `auto_reconnect` 3종은 **양손 공통**이고, interface·affinity·disabled는 손별입니다.

`control_rate`(500)와 `max_effort`(1000)는 xacro 인자이지만 이 launch가 선언하지 않으므로
`key:=value`로 바꿀 수 없습니다. 바꾸려면 자기 xacro를 쓰고([ros2_control 설정](03_setup.md)),
runtime max effort는 hardware node parameter로 조정합니다.

```bash
ros2 param set /left_hand_control max_effort \
  "[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0,
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]"
```

이 launch는 인자를 `OpaqueFunction` 안에서 선언하므로 `--show-args`가 `config`만 표시할 수
있습니다. 전체 key는 위 표와 설치된 `hand_bringup.yaml`을 함께 보십시오.

### Config 파일

CLI가 YAML을 덮고, YAML이 fallback을 덮습니다.

```text
CLI key:=value  ▸  config YAML  ▸  launch source fallback
```

기본 파일은 `<aidin_hand2_bringup share>/config/hand_bringup.yaml`입니다. ROS parameter
파일이 아니라 최상위 `key: value`를 launch 인자 이름과 1:1로 적습니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  config:=/absolute/path/production-hand.yaml \
  auto_home:=false
```

### Commissioning

처음 켤 때는 기본값에서 아래만 바꿉니다. `auto_reconnect`·`auto_reconnect_home`은
기본값이 이미 `false`라 따로 덮을 것이 없습니다.

| Argument | 기본값 | Commissioning |
|---|---|---|
| `use_right_hand` | `true` | `false` — 한 손씩 확인 |
| `auto_home` | `true` | `false` — 작업 공간 확인 후 `~/home` 직접 호출 |

> [!WARNING]
> Operator 승인 없이 자동 homing을 허용할 수 있을 때만 `auto_home`·`auto_reconnect_home`을
> 켭니다. `auto_reconnect_timeout_ms: 0`(무제한)은 상위 supervisor 없이 쓰지 않는 편이
> 안전합니다.

`auto_home`·`auto_reconnect`의 wrapper 동작은 [운영과 복구](06_operations.md)에 있습니다.

### 양손

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  use_left_hand:=true use_right_hand:=true \
  left_hand_interface:=can0 right_hand_interface:=can1 \
  auto_home:=false
```

독립 `ros2_control` system 두 개(`left_hand_control`·`right_hand_control`)가 생기고 service
namespace도 각각입니다. 조건:

- Side별 CAN mapping이 물리적으로 검증돼 있습니다.
- SDK RT affinity가 서로 다르고 실제 존재하는 CPU입니다.
- Controller manager thread는 SDK RT thread와 다른 CPU를 씁니다.
- 두 손의 auto-home을 동시에 수행해도 전원과 기구가 안전합니다.

`auto`는 편리하지만 cable swap을 운영 configuration change로 감지해야 하는 system에서는
명시적 mapping을 권장합니다.

### 미가동 actuator

여기 적힌 actuator는 enable하지 않고, fault로 보고하지 않으며, homing에서도 뺍니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2.launch.py \
  auto_home:=false left_hand_disabled_actuators:="0,1,2,3"
```

```text
0..3   thumb_actuator0..3
4..6   index_actuator1..3
7..9   middle_actuator1..3
10..12 ring_actuator1..3
13..15 baby_actuator1..3
```

## 2. Mock launch

CAN·drive·homing 없이 같은 98개 command port와 mode switch를 검증합니다. Mock의 동작 차이는
[Interface reference](04_interfaces.md)에 있습니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
```

| Argument | 기본값 | 의미 |
|---|---|---|
| `use_rviz` | `false` | RViz 실행 |

왼손 mock 하나에 `joint_state_broadcaster`와 `left_joint_position_controller`만 active입니다.

## 3. Controller만 띄우기

이미 다른 launch가 controller_manager를 올린 경우에 씁니다.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_controllers.launch.py \
  use_left_hand:=true controller_manager:=/my_controller_manager
```

| Argument | 기본값 | 의미 |
|---|---|---|
| `use_left_hand` | `false` | 왼손 controller spawn |
| `use_right_hand` | `true` | 오른손 controller spawn |
| `controller_manager` | `/controller_manager` | 대상 controller_manager 이름 |

> [!NOTE]
> 이 launch의 `use_left_hand`·`use_right_hand` 기본값은 실물 launch와 반대입니다. 명시해서
> 쓰십시오.

## 4. GUI bridge

```bash
ros2 launch aidin_hand2_bringup gui_bridge.launch.py
```

> [!WARNING]
> rosbridge는 인증·TLS를 제공하지 않고 모든 network interface에 bind합니다. 신뢰할 수 있는
> 격리 network에서만 쓰십시오.

## 5. Controller config

`controllers.yaml`의 실제 값입니다. 모두 표준 ROS 파라미터이므로 자기 YAML에서 바꿀 수 있습니다.

| Parameter | 값 |
|---|---:|
| Controller manager `update_rate` | 500 Hz |
| Joint state broadcaster | 100 Hz |
| Hand state broadcaster | 100 Hz |
| Diagnostics broadcaster | 20 Hz |
| Joint position `speed_rad_s` | 0 rad/s (무제한, 정지 아님) |

Broadcaster rate를 낮춘 이유는 GUI·rviz가 사람 눈용이고, 500 Hz로 양손 메시지를 발행하면
rosbridge가 포화되기 때문입니다.

Controller manager `update_rate`와 SDK `control_rate`는 함께 설계하고 검증하십시오. 두 값이
다르면 명령이 계단처럼 끊겨 진동합니다.

# ros2_control 설정

손을 자기 로봇(팔·이동로봇)에 붙이는 방법입니다. 이 문서가 wrapper의 통합 계약입니다.

`aidin_hand2_bringup`의 launch는 손 단독 시험용 예제이므로, 통합할 때는 그것을 쓰지 않고 아래
xacro 매크로를 자기 URDF에 직접 호출합니다. 예제 launch의 argument는
[bringup 예제](05_bringup_example.md)에 있습니다.

## 1. URDF에 손 추가

`ros2_control` system을 선언하는 매크로 하나가 계약입니다.

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- 손 링크·조인트 (mesh 포함) — 왼손은 _left, 오른손은 _right -->
  <xacro:include filename="$(find aidin_hand2_description)/urdf/aidin_hand2_left.urdf.xacro"/>
  <!-- ros2_control system -->
  <xacro:include filename="$(find aidin_hand2_description)/ros2_control/aidin_hand2.ros2_control.xacro"/>

  <xacro:aidin_hand2_left prefix="left_" parent="my_arm_tool0">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:aidin_hand2_left>

  <xacro:aidin_hand2_ros2_control
    name="left_hand" prefix="left_" hand_side="left"
    can_interface="can0" auto_home="false"/>

</robot>
```

기하 매크로는 side별로 나뉩니다 — `aidin_hand2_left`(`aidin_hand2_left.urdf.xacro`)와
`aidin_hand2_right`(`aidin_hand2_right.urdf.xacro`). `ros2_control` 매크로는 하나이고
`hand_side`로 side를 받습니다.

`parent`는 이미 존재하는 링크여야 하고, `prefix`는 interface·controller 이름에 그대로 붙으므로
양손이면 서로 달라야 합니다.

## 2. 매크로 파라미터

`can_interface`까지가 필수이고 나머지는 SDK 기본값을 따릅니다. 값은 SDK `HandConfig`와 1:1입니다.
Backend 는 `use_isaac` > `use_mock` > 실 CAN 순으로 정해지고, `isaac_*` 인자는 `use_isaac` 일 때만
쓰입니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `name` | — | `ros2_control` system 이름. controller가 claim할 때 쓰입니다 |
| `prefix` | — | joint·interface 이름 접두어 (예: `left_`) |
| `hand_side` | — | `left` 또는 `right`. CAN ID와 actuator 배선을 결정합니다 |
| `can_interface` | — | CAN interface 이름. `auto`는 side ID로 채널을 탐색합니다 |
| `auto_home` | `true` | activation 후 non-blocking homing을 1회 trigger |
| `max_effort` | `1000` | rated current 대비 % (0~2000). `1000` = 100% |
| `control_rate` | `500` | SDK RT loop 주기 [Hz] |
| `rt_cpu_affinity` | `-1` | SDK RT thread CPU pin. `-1` = 미설정 |
| `disabled_actuators` | `''` | 미가동 actuator index (예: `"0,1,2,3"`). 빈 값 = 전부 가동 |
| `auto_reconnect` | `false` | 통신 두절 시 SDK가 재수립을 반복 시도 |
| `auto_reconnect_timeout_ms` | `0` | 재수립 포기 상한 [ms]. `0` = 무제한 |
| `auto_reconnect_home` | `false` | 재수립 복귀 후 run 전에 homing |
| `use_mock` | `false` | 실 CAN 대신 kinematics mock. interface 이름은 동일 |
| `use_isaac` | `false` | 실 CAN 대신 Isaac Sim 토픽 브리지. `use_mock` 보다 우선하고, state·command 계약은 real 과 동일 |
| `isaac_topic_prefix` | `/isaac` | Isaac 토픽 prefix. 아래 세 인자가 비면 `<prefix>/joint_states` · `<prefix>/hand_command` · `<prefix>/tactile` |
| `isaac_joint_state_topic` | `''` | Isaac → hardware 상태 토픽 override (`sensor_msgs/JointState`, 이름 매칭) |
| `isaac_joint_command_topic` | `''` | hardware → Isaac 명령 토픽 override (`sensor_msgs/JointState`, joint 21) |
| `isaac_tactile_prefix` | `''` | 촉각 토픽 prefix override. `<prefix>/<side>_<finger>_sensor` · `<prefix>/<side>_palm_sensor` (`std_msgs/Float64MultiArray`) |
| `isaac_state_timeout` | `0.1` | Isaac 상태가 이 시간[s]을 넘겨 끊기면 lifecycle 이 `Disconnected`. `0` 이하면 검사 안 함 |

> [!CAUTION]
> `auto_home=true`는 activation 직후 손을 움직입니다. 첫 통합에서는 `false`로 두고 작업 공간을
> 확인한 뒤 `~/home` service를 직접 호출하십시오.

`control_rate`를 바꾸면 controller manager `update_rate`도 함께 맞춥니다. 두 값이 다르면 명령이
계단처럼 끊겨 진동합니다.

## 3. Controller 연결

Controller는 자기 `controllers.yaml`에 선언하고 spawner로 올립니다. Hardware가 mode별로
서로 다른 command interface를 claim하므로 **command controller는 한 순간 하나만 active**여야
합니다.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500          # 매크로의 control_rate 와 동일하게

    left_joint_position_controller:
      type: aidin_hand2_controllers/JointPositionController
    left_hand_state_broadcaster:
      type: aidin_hand2_controllers/HandStateBroadcaster
    left_diagnostics_broadcaster:
      type: aidin_hand2_controllers/DiagnosticsBroadcaster

left_joint_position_controller:
  ros__parameters:
    hand_side: left           # prefix 가 아니라 side 값
```

| Controller | 명령 |
|---|---|
| `JointPositionController` | active joint 16개 위치 [rad] |
| `JointImpedanceController` | active joint 16개 평형 자세 [rad] (gain 은 hardware node parameter) |
| `HandStateBroadcaster` | joint·actuator·tactile 관측을 `~/hand_state`로 발행 |
| `DiagnosticsBroadcaster` | SDK diagnostics·actuator fault를 `/diagnostics`로 발행 |

<!-- INTERNAL-BEGIN: actuator controller (개발용 — 외부 배포 시 이 블록 삭제) -->
> [!NOTE]
> **(내부 개발용)** 아래 두 controller는 actuator 공간에 직접 명령을 씁니다. Kinematics를 우회하므로
> 워크스페이스 clamp가 적용되지 않습니다. 계측·검증에만 사용하십시오.
>
> | Controller | 명령 |
> |---|---|
> | `ActuatorPositionController` | actuator 16개 위치 [encoder count] |
> | `ActuatorEffortController` | actuator 16개 effort [rated current %] |
<!-- INTERNAL-END -->

## 4. 명령 보내는 두 방법

### Topic (standalone)

각 controller의 `~/command`에 한 cycle 분을 **전부** 담아 보냅니다. Partial update는 받지
않습니다 — joint position이면 target 16개가 한 message입니다.

```bash
ros2 topic pub --once \
  /left_joint_position_controller/command \
  aidin_hand2_msgs/msg/JointPositionCommand \
  "{target_position_rad: [0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

### Reference interface (chainable)

네 command controller 모두 `ChainableControllerInterface`이므로 상위 controller가 reference
interface에 직접 씁니다. Topic 왕복이 없어 한 cycle 안에 명령이 전달됩니다.

```
상위 controller
  → <controller>/target_position_rad.<joint>  (reference interface)
  → 기존 command controller
  → hardware command port
  → SDK
```

상위 controller는 `command_interface_configuration()`에서 대상 controller의 reference
interface 이름을 claim합니다. Skeleton 4종이 [예제](../../aidin_hand2_examples/EXAMPLE.md)에
있습니다.

Interface 전체 이름 규칙은 [interface 계약](04_interfaces.md)에 있습니다.

## 5. 확인

```bash
ros2 control list_hardware_components     # <name> 이 active 인지
ros2 control list_controllers             # command controller 가 하나만 active 인지
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --once
```

`hand_diagnostics`의 `lifecycle`이 `Running`이고 `homing_state: Succeeded`면 명령을 받을 준비가 된
상태입니다. 그 밖의 값은 [운영과 복구](06_operations.md)를 참조하십시오.
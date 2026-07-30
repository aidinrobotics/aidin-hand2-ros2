# 알려진 제한

이 문서는 현재 source에 남아 있는 제한만 기록합니다. 이미 해결된 97-port mode 계약,
mock의 첫 partial command, joint workspace clamp 비활성 문제는 현재 제한이 아닙니다.

## 1. 지원·검증 범위

공개 target은 Ubuntu 22.04와 ROS 2 Humble입니다.

- 다른 ROS distribution용 CI 결과가 없습니다.
- Repository에 GitHub Actions 등 CI workflow가 없습니다.
- ROS 2 package test가 정의돼 있지 않습니다.
- 실제 hardware acceptance test는 자동화돼 있지 않습니다.

현재 mock에서는 네 basic controller의 load/configure, 98개 command port, exact mode switch와
chain activation을 검증할 수 있습니다. 이것이 CAN, drive, tactile, homing, fault와
reconnect 검증을 대신하지는 않습니다.

## 2. Mock과 실물의 차이

| 기능 | 실제 | Mock |
|---|---|---|
| Command port·mode switch | 98개, 네 mode | 동일 |
| Actuator physical state | 48개 | 동일 이름 48개 |
| Joint position state | 21개 | 동일 |
| Joint position·impedance | SDK + hardware | clamp → IK → FK |
| Actuator position | Hardware CSP | count → FK |
| Actuator effort | Hardware CST | effort clamp만, 동역학 없음 |
| CAN·drive·homing·reconnect | 있음 | 없음 |
| Tactile·hardware diagnostics | 있음 | 없음 |
| Command echo·timestamp | 있음 | 없음 |
| Runtime service | 있음 | 없음 |

Default mock launch는 `joint_state_broadcaster`와
`left_joint_position_controller`만 spawn합니다. 다른 세 basic controller는 config에
등록되어 load할 수 있지만, `HandStateBroadcaster`와 `DiagnosticsBroadcaster`는 mock에
필요한 state block이 없어 spawn하지 않습니다.

## 3. Xacro와 dynamic state interface

Ros2 control xacro는 실제 physical·diagnostics state interface 251개를 정적으로 선언합니다.
실제 plugin은 typed command state 132개와 timestamp 2개를 동적으로 추가해 총 385개를
export합니다.

영향:

- URDF만 분석하는 tool은 동적 134개를 보지 못합니다.
- Recorder schema와 interface 검증은 runtime output을 기준으로 해야 합니다.
- `HandStateBroadcaster`가 claim하는 346개와 실제 export 순서의 regression test가 필요합니다.

```bash
ros2 control list_hardware_interfaces
```

## 4. Broadcaster layout coupling

`HandStateBroadcaster`는 346개 interface의 순서를 compile-time offset으로 해석합니다. 이름
목록과 offset이 같은 source에 있어 현재는 일치하지만 hardware state block을 추가·재배열할
때 compile만으로 잘못된 field mapping을 검출할 수 없습니다.

`DiagnosticsBroadcaster`도 39개 diagnostics naming·ordering 계약에 의존합니다. Production
fork에서는 runtime name snapshot과 message mapping test를 추가하십시오.

## 5. Default launch의 자동 motion

제공 `hand_bringup.yaml`은 다음 운용 지향값을 사용합니다.

- `auto_home=true`
- `auto_reconnect=true`
- `auto_reconnect_timeout_ms=0`, 즉 무제한
- `auto_reconnect_home=true`

Launch 직후 또는 통신 복귀 뒤 homing motion이 시작될 수 있습니다. Commissioning에서는
명시적으로 끄고 operator 승인 뒤 `/home`을 호출하십시오.

## 6. Auto reconnect 중 stale state

`auto_reconnect=true`이면 wrapper는 SDK 복구 loop를 살리기 위해 read/write exception을
ros2_control `ERROR`로 올리지 않고 `OK`로 처리합니다.

- Controller와 broadcaster가 active로 보일 수 있습니다.
- 마지막 snapshot이 계속 publish될 수 있습니다.
- SDK lifecycle가 `Running`이 아니면 command write는 skip됩니다.
- Timeout 0이면 복구 시도가 무기한 계속될 수 있습니다.

Topic 수신 여부만 보지 말고 `HandState.header.stamp`, `control_cycles`, lifecycle와
SocketCAN counter의 변화를 함께 확인하십시오.

## 7. Command timeout 없음

ROS basic controller와 SDK에는 application command age watchdog이 없습니다. Publisher가
사라져도 마지막 reference·command가 유지될 수 있습니다. DDS graph disconnect나 topic
silence가 자동 `stop()`을 일으키지 않습니다.

상위 controller/supervisor에 monotonic command-age watchdog과 독립적인 stop 정책을
두십시오.

## 8. Diagnostics의 범위

`HandDiagnostics`에는 last exception text, reconnect attempt count, state freshness boolean,
command age와 composite operation-ready field가 없습니다. 표준 `/diagnostics` level도
`homed=false`, stale timestamp와 deadline miss 증가만으로 WARN이 되지 않습니다.

Supervisor가 lifecycle, homed, actuator fault, stamp와 control cycle을 합성해야 합니다.

## 9. `description.launch.py` mock argument typo

`aidin_hand2_description/launch/description.launch.py`는 xacro에
`use_mock_hardware:=true`를 전달하지만 실제 argument는 `use_mock`입니다. Description-only
RViz에서는 즉시 드러나지 않을 수 있습니다. Mock control은 다음 launch를 사용하십시오.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py
```

## 10. Top-level `control_rate`·initial `max_effort`

Xacro에는 `control_rate=500`, `max_effort=1000`이 있지만 top-level
`aidin_hand2.launch.py`는 두 값을 CLI argument로 declare·forward하지 않습니다. 변경하려면
custom description/launch가 필요합니다. Runtime 공통 max effort는
`/{side}_hand_control/set_max_effort` topic으로 바꿀 수 있습니다.

Controller manager update rate와 SDK control rate는 함께 설계하십시오.

## 11. Message의 NaN과 validity

`CommandState`에서 현재 mode/type이 선택하지 않은 nested input과 output field는 NaN일 수
있습니다. `controller_input_mode`, `controller_output_type`, `selected_source`를 validity
discriminator로 사용하십시오.

NaN을 0으로 바꾸면 “not applicable”과 실제 target 0을 구분할 수 없습니다. Strict JSON,
database와 ML pipeline은 nullable 또는 validity mask를 사용해야 합니다.

## 12. Optional MANUS dependency 선언

CMake는 `manus_ros2_msgs`를 optional로 찾지만 `aidin_hand2_examples/package.xml`은
`<depend>`로 선언합니다. 일반 환경에서 `rosdep`이 unresolved key로 실패하면 다음처럼
skip합니다.

```bash
rosdep install --from-paths src/aidin-hand2-ros2 --ignore-src --recursive \
  --rosdistro humble --skip-keys "aidin_hand2 manus_ros2_msgs" -y
```

Glove를 사용할 때만 호환 message package와 publisher를 workspace에 추가하십시오.

## 13. GUI·rosbridge security

`gui_bridge.launch.py`의 rosbridge는 remote topic publish와 service call surface를 엽니다.
Repository는 authentication, TLS와 per-command authorization을 구성하지 않습니다.
신뢰할 수 없는 network에 직접 공개하지 마십시오.

## 14. Version·license 상태

`aidin_hand2_description`은 version 0.0.1, Apache-2.0을 선언하지만 나머지 package는
version 0.1.0, license `TODO`입니다. Repository root에 `LICENSE`가 없습니다. 전체
repository의 권리 조건을 한 package field에서 추론하지 마십시오.

## 15. Controller Manager integration debt

실제·mock launch는 `robot_description` parameter를 `ros2_control_node`에 직접 전달하므로
Humble에서 deprecation warning이 날 수 있습니다. 현재 즉시 failure는 아니지만 이후
ros2_control migration 항목입니다.

또한 실제 launch는 hand별 argument를 `OpaqueFunction` 안에서 declare하므로
`ros2 launch ... --show-args`가 `config`만 표시할 수 있습니다. 전체 key는 installed
`hand_bringup.yaml`과 [Launch reference](02_launch_reference.md)를 함께 확인하십시오.

## 16. Deployment acceptance

| 제한 | 수용 여부 | 우회책 owner | 검증 evidence |
|---|---|---|---|
| Command timeout 없음 |  |  |  |
| Auto reconnect stale state |  |  |  |
| Default auto motion |  |  |  |
| Dynamic state schema |  |  |  |
| Broadcaster layout coupling |  |  |  |
| ROS test·CI 없음 |  |  |  |
| License 미정 |  |  |  |
| Rosbridge 인증 없음 |  |  |  |

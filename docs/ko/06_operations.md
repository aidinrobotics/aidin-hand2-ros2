# 운영과 복구

이 문서는 실제 hardware의 startup, health monitoring, stop과 communication recovery를 다룹니다. SDK가 정의하는 command 지속성과 통신 두절 위험은 [SDK 안전과 fault 대응](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/12_safety.md)를 함께 적용하십시오.

## 1. Runtime architecture

한 hand에는 두 개의 control loop가 있습니다.

```text
ROS controller_manager loop (기본 500 Hz)
  read → controllers update → write
             │
             ▼
SDK RT loop (기본 500 Hz, SCHED_FIFO 90 요청)
  CAN RX → state/kinematics → command → CAN TX
```

ROS loop는 controller와 hardware interface를 갱신하고 SDK loop는 실제 CAN을 처리합니다. 둘의 rate, CPU affinity와 priority를 독립적으로 관측해야 합니다.

## 2. Startup sequence

권장 production startup gate:

1. Host boot service가 CAN-FD를 올립니다.
2. CAN state가 `ERROR-ACTIVE`이고 RX frame이 증가합니다.
3. ROS domain과 expected package version을 확인합니다.
4. `auto_home=false`로 bringup합니다.
5. Hardware component와 controller state를 확인합니다.
6. Diagnostics lifecycle `Running`, actuator fault와 freshness를 확인합니다.
7. Max effort를 낮추고 joint target과 commissioning speed를 담은 complete command를 준비합니다.
8. Operator 또는 supervisor가 homing을 승인합니다.
9. `homed=true`를 확인합니다.
10. Command producer를 activate합니다.

ROS process가 뜬 사실과 operation-ready를 분리하십시오.

## 3. Lifecycle operation

### ros2_control callback과 SDK operation

| ros2_control callback | SDK operation | 의미 |
|---|---|---|
| `on_configure` | `create()` + `connect()` | Resource 생성, 첫 CAN frame 확인, listen-only |
| `on_activate` | `run()` | Drive enable, 이후 read cycle에서 optional homing |
| `on_deactivate` | `stop()` | Blocking quick stop |
| `on_cleanup` | `disconnect()` + `destroy()` | 통신·resource 해제 |

Wrapper는 SDK config의 blocking `auto_home`을 강제로 끕니다. ROS `auto_home`은 `start_homing()`을 한 번 trigger하므로 controller manager executor를 block하지 않습니다.

### Listen-only와 enable

`on_configure`는 SDK `connect()`까지 수행하여 첫 frame을 기다리고 `Connected` listen-only 상태를 만듭니다. `on_activate`가 SDK `run()`을 호출해 drive를 enable합니다.

Top-level launch는 hardware를 자동 activate하므로 일반 bringup에서는 configure와 activate 사이에 operator pause가 없습니다. Commissioning 안전은 `auto_home=false`, 낮은 effort와 external interlock으로 확보합니다.

### Runtime `stop`·`run`

```bash
ros2 service call \
  /left_hand_control/stop \
  std_srvs/srv/Trigger

ros2 service call \
  /left_hand_control/run \
  std_srvs/srv/Trigger
```

`stop` service는 plugin의 `started_` latch도 false로 만들어 이후 write를 skip합니다. `run`은 current mode command interface를 현재 state 또는 안전 초기값으로 seed합니다.

### Hardware lifecycle

Controller manager의 hardware component transition도 가능합니다.

```bash
ros2 control set_hardware_component_state \
  left_hand_control inactive

ros2 control set_hardware_component_state \
  left_hand_control active
```

전환 전에 command controller를 deactivate하십시오. 설치된 Humble CLI에서 정확한 syntax는 `ros2 control set_hardware_component_state --help`로 확인합니다.

## 4. Homing operation

```bash
ros2 service call \
  /left_hand_control/home \
  std_srvs/srv/Trigger
```

Wrapper가 SDK `start_homing()`만 trigger하므로 service는 즉시 반환합니다.

Wrapper는 SDK `HandConfig::auto_home`을 항상 `false`로 두고, ROS `auto_home` argument는 이렇게
동작합니다.

```text
on_activate → SDK run()
             ↓
다음 read/write cycle 에서 homed=false 확인
             ↓
SDK start_homing() 1회 trigger
             ↓
homing 중·homed=false 동안 command write 억제
```

SDK blocking `home()` 을 쓰지 않으므로 controller manager executor 가 멈추지 않습니다.

```bash
ros2 topic echo \
  /left_diagnostics_broadcaster/hand_diagnostics
```

Completion 조건:

- `homed=true`
- Lifecycle `Running`
- Actuator fault empty
- Header와 control cycle freshness 정상

Homing 중 또는 homed false일 때 hardware plugin은 command write를 조용히 skip합니다. Publisher는 성공적으로 publish할 수 있지만 hand에는 command가 전달되지 않습니다.

Homing 후 command producer가 최근 target을 다시 보낼지, operator가 새 target을 승인할지 정책을 명시하십시오. Latched application target이 자동으로 재개되는 설계는 위험할 수 있습니다.

## 5. Health monitoring

### 반드시 함께 볼 signal

| Signal | 판단 |
|---|---|
| Hardware component state | ros2_control lifecycle |
| Controller state | Command source active 여부 |
| SDK lifecycle | 실제 hand control state |
| `HandState.header.stamp` | RX observation freshness |
| `control_cycles` | SDK loop liveness |
| `deadline_misses` | Timing health |
| `homed` | Motion command gate |
| `actuator_enabled/fault` | Drive health |
| ROS·SDK log | 사건 원인 |
| SocketCAN statistics | Bus health |

### Freshness

Auto reconnect 중 마지막 `HandState`가 broadcaster rate로 반복될 수 있습니다.

Supervisor example logic:

```text
if now_monotonic - last_changed_observation_stamp > threshold:
    mark STALE
if control_cycles did not increase over threshold:
    mark SDK_LOOP_STALLED
if lifecycle != Running:
    inhibit command
if homed == false:
    inhibit motion command
```

ROS time이 simulation 또는 clock jump의 영향을 받을 수 있으면 node-local monotonic receive time도 기록합니다.

### Diagnostics level의 경계

`/diagnostics` level 규칙은 셋뿐입니다.

| 조건 | Level |
|---|---|
| Lifecycle `Faulted` | `ERROR` |
| Lifecycle `Running` + actuator fault 존재 | `WARN` (일부 mask 후 동작) |
| 그 외 | `OK` |

따라서 다음은 모두 `OK`로 나옵니다.

- `homed=false`
- Deadline miss 급증
- Timestamp stale
- Actuator가 expected 상태와 다르게 disabled
- Auto reconnect transition 전후

`HandDiagnostics`에도 last exception text, reconnect attempt count, state freshness boolean,
command age, composite operation-ready field가 없습니다. Operation-ready 판정은 supervisor가
lifecycle·homed·actuator fault·stamp·control cycle을 합성해서 해야 합니다.

### Command timeout이 없다

ROS basic controller와 SDK 어디에도 application command age watchdog이 없습니다. Publisher가
사라져도 마지막 reference·command가 유지되며, DDS graph disconnect나 topic silence가 자동
`stop()`을 일으키지 않습니다.

> [!WARNING]
> 상위 controller 또는 supervisor에 monotonic command-age watchdog과 독립적인 stop 정책을
> 두십시오. Wrapper는 이 보호를 제공하지 않습니다.

## 6. Realtime 운영

SDK loop는 `SCHED_FIFO` priority 90과 memory lock을 best-effort로 요청합니다. 실패하면 warning 후 non-RT로 계속됩니다.

Controller manager의 Humble documentation은 update loop priority 기본 50과 realtime group·limit 설정을 설명합니다. [Controller Manager user documentation](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)을 기준으로 합니다.

권장 배치 예:

| Thread | CPU | Priority |
|---|---:|---:|
| Left SDK RT | 4 | 90 |
| Right SDK RT | 6 | 90 |
| Controller manager | 별도 CPU | 50 또는 측정 기반 설정 |
| DDS, logger, general work | 나머지 | non-RT |

Default는 `-1`(미설정)입니다. 코어를 지정하기 전에 `lscpu --extended`로 topology를 확인합니다.

```bash
ps -Leo pid,tid,cls,rtprio,psr,comm |
  grep -E 'ros2_control|controller|aidin'
```

500 Hz 목표 period는 2 ms입니다. `last_period_ms`, `last_compute_ms`와 miss 증가율을 기록합니다.

## 7. Communication loss

SDK는 RX silence 약 100 ms를 감지하고 `Faulted`로 전이합니다. 제어 중이었으면 quick stop을
보내며 도달을 판정합니다. CAN이 끊겨 quick-stop frame이 도달하지 않으면 drive가 마지막 torque를
유지할 수 있습니다.

### Wrapper 동작

Wrapper는 `auto_reconnect` 설정과 무관하게 lifecycle이 `Running`이 아니면 command를 보내지
않고 `OK`를 반환합니다. Hardware component를 active로 유지해야 SDK의 복구 loop가 살아 있고,
같은 controller manager의 다른 component와 그 interface를 claim한 controller(`joint_state_broadcaster`
등)가 함께 내려가지 않습니다.

```text
통신 두절
  ↓
SDK Faulted → reconnect retry
  ↓                         ├─ topic은 stale snapshot 발행 가능
  ↓                         └─ command write skip
재연결 성공
  ↓
auto_reconnect_home 정책
  ↓
Running·homed 후 write 재개
```

Default YAML은 timeout 0, 즉 무제한이고 reconnect home true입니다. 통신이 예상치 못한 시점에 돌아오면 homing motion이 시작될 수 있습니다.

## 8. Manual recovery

1. 외부 안전 상태를 확보합니다.
2. 전원·배선·SocketCAN error와 drive fault 원인을 제거합니다.
3. Auto reconnect가 동작 중이면 중복 manual call 전에 lifecycle를 확인합니다.
4. `Faulted`에서 reconnect를 호출합니다.

```bash
ros2 service call \
  /left_hand_control/reconnect \
  std_srvs/srv/Trigger
```

성공 응답은 통신 복구이며 control은 재개하지 않습니다.

```bash
ros2 service call \
  /left_hand_control/run \
  std_srvs/srv/Trigger
```

Reconnect는 SDK homed를 false로 reset합니다. Auto-home policy를 확인하거나 수동 home을 수행합니다.

```bash
ros2 service call \
  /left_hand_control/home \
  std_srvs/srv/Trigger
```

`homed=true` 후 command controller와 target을 재승인합니다.

## 9. Controller failure와 mode recovery

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Mode switch가 실패하면:

1. 두 command controller가 동시에 activate되지 않았는지 확인합니다.
2. 이전 controller를 deactivate합니다.
3. Hardware가 Running·homed인지 확인합니다.
4. 새 controller를 activate합니다.
5. Safe initial command를 보냅니다.

`--strict`를 사용하여 일부 transition만 성공한 상태를 성공으로 취급하지 않습니다.

## 10. Log

Hardware plugin은 SDK callback을 `/rosout`에 중계합니다.

```bash
ros2 topic echo /rosout
journalctl -u <robot-service> -b --no-pager
```

SDK event message에는 side와 cycle context가 포함될 수 있습니다. ROS wrapper catch path는 같은 failure를 중복 설명하지 않고 SDK exception footprint를 원인 기록으로 사용합니다.

Incident에 보존할 것:

- Launch CLI와 config YAML
- `ros2 control` list 3종
- HandState·HandDiagnostics time series
- `/diagnostics`와 `/rosout`
- SocketCAN statistics와 CAN capture
- Kernel·systemd journal
- SDK·wrapper commit과 kernel version

## 11. Graceful shutdown

권장:

```bash
ros2 control switch_controllers \
  --deactivate left_joint_position_controller \
  --strict

ros2 service call \
  /left_hand_control/stop \
  std_srvs/srv/Trigger
```

양손이면 각 side에 반복합니다. 그다음 launch process에 `SIGINT`를 보내 cleanup이 실행되게 합니다.

Shutdown 완료 기준:

- Stop service success 또는 hardware deactivate success
- Controller inactive
- SDK lifecycle Stopped 또는 component cleanup
- Process 종료와 CAN owner 해제
- Critical stop-confirmation failure 없음

`SIGKILL`, terminal close 또는 network disconnect만으로 drive가 안전하게 멈췄다고 가정하지 마십시오.

## 12. Supervisor policy 예시

| Condition | Command gate | Recovery |
|---|---|---|
| Lifecycle not Running | block | 상태별 run/reconnect |
| Homed false | block motion | 승인된 home |
| Observation stale | block + external stop 판단 | CAN·SDK 조사 |
| Deadline miss rate 초과 | derate/stop | RT load 조사 |
| Any actuator fault | mode별 block | 원인 제거 후 승인 |
| Reconnect count 초과 | block | operator intervention |
| ROS command stale | stop | producer restart·승인 |

SDK와 wrapper는 reconnect 횟수, application command age와 operation-ready boolean을 제공하지 않습니다. Supervisor가 상태를 합성해야 합니다.

## 13. Production checklist

- [ ] Normal default 대신 review된 config file을 사용합니다.
- [ ] Auto-home과 reconnect-home은 의도적으로 결정했습니다.
- [ ] Reconnect timeout은 risk와 supervisor policy에 맞습니다.
- [ ] Topic freshness를 stamp·cycle 변화로 감시합니다.
- [ ] Command producer watchdog이 있습니다.
- [ ] SDK·controller manager RT thread의 실제 class·CPU를 확인했습니다.
- [ ] Controller mode switch를 strict transaction으로 수행합니다.
- [ ] Manual recovery는 reconnect → run → home → command 승인 순서입니다.
- [ ] Graceful shutdown과 external E-stop을 시험했습니다.

# Services

hardware component는 자기 node에 `std_srvs/srv/Trigger` service 4개를 제공합니다. node 이름은 매크로의 `name`
parameter 값이고 기본 launch에서는 `/{side}_hand_control`입니다.
`~/run`·`~/stop`·`~/reconnect` service는 SDK `Hand`의 lifecycle을 전이하고 `~/home` service는 homing을
시작합니다. 이 문서는 각 service의 계약, homing, `Faulted` 복구, 종료 절차를 설명합니다. lifecycle
상태의 뜻은 [1.2 Lifecycle](06_controllers.md#12-lifecycle)에 있습니다. service는 로봇 핸드 backend에만
있고 mock과 isaac에는 없습니다.

`{side}`는 `left` 또는 `right`입니다.

## Contents

&nbsp;&nbsp;[**1. Service contract**](#1-service-contract)<br>
&nbsp;&nbsp;[**2. run and stop**](#2-run-and-stop)<br>
&nbsp;&nbsp;[**3. home**](#3-home)<br>
&nbsp;&nbsp;[**4. reconnect**](#4-reconnect)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Manual recovery](#41-manual-recovery)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Automatic recovery](#42-automatic-recovery)<br>
&nbsp;&nbsp;[**5. Shutdown**](#5-shutdown)

## 1. Service contract

service는 hardware component가 만든 node에서 별도 thread의 executor가 처리하므로 controller_manager의
루프를 막지 않습니다. `~/run`·`~/stop`·`~/reconnect` service는 SDK가 전이를 확인할 때까지 대기한 뒤
반환하고, `~/home` service는 시작만 하고 즉시 반환합니다.

| Service | Precondition | Postcondition | Waits for | Timeout | Success message |
|---|---|---|---|---|---|
| `~/run` | `Connected` · `Running` · `Stopped` | `Running` | actuator enable | 4000 ms | `running` |
| `~/stop` | `Running` · `Stopped` | `Stopped` | actuator quick stop | 500 ms | `stopped` |
| `~/home` | `Connected` · `Running` · `Stopped` | `Running` | 없음. 시작만 확인 | 없음 | `homing started — poll diagnostics 'homing_state'` |
| `~/reconnect` | `Faulted` | `Connected` | 첫 state 수신 | 300 ms | `reconnected — call ~/run to resume control` |

표에 없는 state에서 호출하거나 제한 시간 안에 확인되지 않으면 `success` 필드가 `false`이고 `message`
필드에 SDK 예외 문구가 그대로 들어갑니다. 이미 postcondition을 만족하는 state에서 호출하면(`Running`에서
`~/run`, `Stopped`에서 `~/stop`) 아무 동작 없이 성공을 반환합니다. 문구별 조치는 SDK 문서의
[Error messages](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/15_error_messages.md)에
있습니다.

호출은 인자 없이 합니다.

```bash
ros2 service call /left_hand_control/run std_srvs/srv/Trigger
```

응답은 다음 형태입니다.

```text
response:
std_srvs.srv.Trigger_Response(success=True, message='running')
```

## 2. run and stop

`~/run` service는 actuator를 enable해 `Running`으로, `~/stop` service는 quick stop으로 `Stopped`로
전이합니다. 둘은 component를 active로 둔 채 제어만 켜고 끄는 수단이며, hardware component의
activate·deactivate와 같은 SDK 함수를 호출합니다.

`~/stop` service 뒤에는 wrapper가 command 전송을 멈추고, `~/run` service가 성공하면 다시 시작합니다.
`Stopped`에서 `~/run` service로 복귀하면 SDK가 재개 시점의 자세를 유지하는 command에서 시작하므로 자세가
무너지지 않습니다. 필요한 자세는 다시 전송해야 합니다.

`auto_home=true`이고 `homing_state` 값이 `Succeeded`가 아니면 `~/run` service 뒤 homing이 다시
시작됩니다. homing 중 `~/stop` service로 멈춘 뒤 `~/run` service를 호출하는 경우가 그렇습니다.

> [!WARNING]
> `~/stop` service가 500 ms 안에 quick stop을 확인하지 못하면 실패를 반환하고 `lifecycle` 값은
> `Running`으로 남습니다. actuator가 마지막 command를 유지하고 있을 수 있으므로, 다시 호출하기보다 로봇
> 핸드의 전원을 차단하십시오.

## 3. home

homing은 각 finger를 hard stop까지 밀어 hard stop 지점을 원점으로 삼는 절차입니다. `homing_state` 값이
`Succeeded`가 되기 전에는 wrapper가 command를 전송하지 않고, `~/home` service는 homing을 시작한 뒤 즉시
반환합니다.

> [!WARNING]
> finger가 완전히 펴지지 못하게 막혀 있으면 막힌 지점이 hard stop으로 인식되어 원점이 어긋난 채
> 성공으로 보고됩니다. homing 전에 주변을 비우고 완료까지 접촉하지 마십시오.

`~/home` service를 호출합니다.

```bash
ros2 service call /left_hand_control/home std_srvs/srv/Trigger
```

완료는 `homing_state` 값으로 확인합니다. 값을 계속 읽습니다.

```bash
ros2 topic echo /left_diagnostics_broadcaster/hand_diagnostics --field homing_state
```

`InProgress`가 `Succeeded`로 바뀌면 완료이고 500 Hz에서 최대 22 s 걸립니다.

- 성공하면 SDK가 원점 자세를 유지하는 command를 적용합니다. 이어서 필요한 command를 전송하십시오.
- `Failed`이면 `actuator_fault_name` 필드에서 실패한 actuator를 확인하고 원인을 제거한 뒤 `~/home`
  service를 다시 호출합니다.
- homing 중에는 `hand_state` topic의 `command_state.selected_source` 값이 `3`(homing)입니다.
- `~/reconnect` service는 `homing_state` 값을 `NotRun`으로 되돌리므로 homing을 다시 해야 합니다.

`auto_home` parameter는 `~/home` service를 대신 호출하는 설정입니다. `~/run` service 성공 뒤 첫 cycle에서
`homing_state` 값이 `Succeeded`가 아니면 homing이 한 번 시작되고, `~/home` service와 마찬가지로 시작만
하므로 controller_manager 루프는 멈추지 않습니다.

## 4. reconnect

`Faulted`는 통신 오류나 제어·통신 루프 예외로 SDK가 스스로 제어를 종료한 상태이고, 벗어나는 수단은
`~/reconnect` service 하나입니다. wrapper는 `Faulted`에서도 hardware component를 active로 유지하고 command
전송만 멈춥니다. component를 내리면 같은 controller_manager의 다른 component와, 다른 component의
interface를 claim한 `joint_state_broadcaster`가 함께 내려가기 때문입니다.

통신이 끊기면 SDK가 약 100 ms 뒤 `Faulted`로 전이하며 quick stop을 전송합니다. CAN이 끊겨 quick stop
frame이 닿지 않으면 drive가 마지막 토크를 유지할 수 있습니다.

### 4.1 Manual recovery

수동 복구는 `Faulted`를 확인한 뒤 service 셋을 순서대로 호출하는 절차입니다.

1. 전원·배선·SocketCAN 오류와 drive fault의 원인을 제거합니다.
2. `~/reconnect` service를 호출합니다. 성공하면 `Connected`이고 통신만 복구된 상태입니다.
3. `~/run` service를 호출합니다. 성공하면 `Running`이고 actuator가 enable됩니다.
4. `~/home` service를 호출하고 `homing_state` 값이 `Succeeded`가 될 때까지 기다립니다.
   `auto_home=true`이면 `~/run` service 뒤 자동으로 시작됩니다.
5. command controller가 active인지 확인하고 필요한 command를 다시 전송합니다.

2·3·4단계의 호출은 다음 셋입니다.

```bash
ros2 service call /left_hand_control/reconnect std_srvs/srv/Trigger
ros2 service call /left_hand_control/run std_srvs/srv/Trigger
ros2 service call /left_hand_control/home std_srvs/srv/Trigger
```

`~/reconnect` service는 `Faulted`가 아니면 실패합니다. 통신이 살아 있는 상태에서 연결을 다시 수립하려면
[1.2 Lifecycle](06_controllers.md#12-lifecycle)의 component inactive → active 전이를 씁니다.

### 4.2 Automatic recovery

`auto_reconnect=true`이면 SDK의 제어·통신 루프가 스스로 재연결을 반복하고, 연결이 복구되면 `~/run`
service 없이 `Running`으로 복귀합니다. `auto_reconnect_home=true`이면 재개 전에 homing도 수행합니다.

- 자동 복구는 통신 오류에만 동작합니다. 제어·통신 루프 예외로 `Faulted`가 되면 `~/reconnect` service를
  직접 호출해야 합니다.
- `auto_reconnect_home=false`이면 복구 뒤 `homing_state` 값이 `NotRun`이므로 `~/home` service를 호출해야
  command가 전송됩니다.
- `auto_reconnect_timeout_ms` 값이 `0`이면 제한 없이 재시도합니다. 오래 복구 중인 상태를 정상으로
  오해하지 않도록 `lifecycle` 값과 `control_cycles` 값을 함께 감시하십시오. 시한을 넘기면 `Faulted`로
  남고 `~/reconnect` service를 직접 호출해야 합니다.
- 복구 중에는 broadcaster가 마지막 state를 같은 주기로 반복 발행합니다. 판단 기준은
  [Topics](08_topics.md) 4장에 있습니다.

## 5. Shutdown

정상 종료는 command controller를 deactivate하고 hardware를 정지한 뒤 launch를 `SIGINT`로 끝내는
순서입니다.

```bash
ros2 control switch_controllers --strict --deactivate left_joint_position_controller
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger
```

양손이면 오른손에 대해 반복합니다. 그다음 launch terminal에서 `Ctrl-C`를 누르면 controller_manager가
hardware component를 deactivate·cleanup하여 CAN 연결을 닫습니다.

`~/stop` service가 `stopped`를 반환했고 이후 `lifecycle` 값이 `Stopped`이면 actuator는 무토크입니다.
`SIGKILL`, terminal 닫기, 네트워크 단절로는 quick stop이 확인되지 않으므로 정지 수단으로 쓰지
마십시오.

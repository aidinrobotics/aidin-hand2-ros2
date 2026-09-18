# aidin_hand2_hardware

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

`aidin_hand2_hardware` package는 로봇 핸드·mock 연결을 제공합니다. 이 문서는 로봇 핸드의
제어 시작·정지, homing, 통신 오류 복구와 effort·filter·gain 설정을 설명합니다.
service는 `std_srvs/srv/Trigger` 타입이며 요청 인자는 없습니다. mock에는 제공하지 않습니다.

기본 launch의 service 경로는 `/{side}_hand_control/run`처럼 구성됩니다. `{side}`는 `left` 또는
`right`이고, `~`는 service를 제공하는 node 이름입니다. 통합할 때 매크로의 `name`을 바꾸면
`/<name>/run`처럼 경로도 바뀝니다.

## Contents

&nbsp;&nbsp;[**1. State and service calls**](#1-state-and-service-calls)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Lifecycle](#11-lifecycle)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Service calls](#12-service-calls)<br>
&nbsp;&nbsp;[**2. run and stop**](#2-run-and-stop)<br>
&nbsp;&nbsp;[**3. home**](#3-home)<br>
&nbsp;&nbsp;[**4. reconnect**](#4-reconnect)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Manual recovery](#41-manual-recovery)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Automatic recovery](#42-automatic-recovery)<br>
&nbsp;&nbsp;[**5. Shutdown**](#5-shutdown)<br>
&nbsp;&nbsp;[**6. Runtime settings**](#6-runtime-settings)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.1 Max effort](#61-max-effort)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.2 Joint position controller](#62-joint-position-controller)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.3 Joint impedance controller](#63-joint-impedance-controller)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.4 Initial runtime settings](#64-initial-runtime-settings)

## 1. State and service calls

### 1.1 Lifecycle

service 호출 조건은 `hand_diagnostics.lifecycle` 값으로 확인합니다. 상태의 의미와
controller의 `active`·homing 상태의 차이는
[Lifecycle](../docs/ko/05_control_guide.md#1-lifecycle)에서 먼저 확인하십시오.

### 1.2 Service calls

`~/run`·`~/stop`·`~/reconnect` service는 전이 결과를 확인한 뒤 응답합니다.
`~/home` service는 시작만 확인하고 즉시 응답하므로 완료 상태를 별도로 읽어야 합니다.
표의 호출 조건과 결과는 `lifecycle` 값입니다.

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
전이합니다. launch를 유지한 채 제어를 시작하거나 정지할 때 사용합니다.

`~/stop` service 뒤에는 wrapper가 command 전송을 멈추고, `~/run` service가 성공하면 다시 시작합니다.
`Stopped`에서 재개하면 SDK는 재개 시점의 자세를 목표로 제어를 시작합니다.
원하는 목표는 다시 전송하십시오.

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

`Succeeded`가 출력되면 완료이고 500 Hz에서 최대 22 s 걸립니다.
이 echo 명령을 실행한 터미널에서 `Ctrl-C`로 관측을 끝냅니다.

- 성공하면 SDK가 원점 자세를 유지하는 command를 적용합니다. 이어서 필요한 command를 전송하십시오.
- `Failed`이면 `actuator_fault_name` 필드에서 실패한 actuator를 확인하고 원인을 제거한 뒤 `~/home`
  service를 다시 호출합니다.
- homing 중에는 `hand_state` topic의 `command_state.selected_source` 값이 `3`(homing)입니다.
- `~/reconnect` service는 `homing_state` 값을 `NotRun`으로 되돌리므로 homing을 다시 해야 합니다.

`auto_home` 인자는 제어 시작 뒤 자동으로 homing을 시작하는 설정입니다. `~/run` service 성공 뒤 첫 cycle에서
`homing_state` 값이 `Succeeded`가 아니면 homing이 한 번 시작되고, `~/home` service와 마찬가지로 시작만
하므로 controller_manager 루프는 멈추지 않습니다.

## 4. reconnect

`~/reconnect` service는 `Faulted`에서 통신을 복구합니다. 성공하면 `Connected`가 되므로 제어 시작과
homing을 이어서 수행해야 합니다. 복구 중에도 controller와 broadcaster가 `active`로 표시될 수 있으므로
`hand_diagnostics`의 상태를 확인합니다.

통신이 끊기면 SDK가 약 100 ms 뒤 `Faulted`로 전이하며 quick stop을 전송합니다. CAN이 끊겨 quick stop
frame이 닿지 않으면 drive가 마지막 토크를 유지할 수 있습니다.

### 4.1 Manual recovery

전원·배선·SocketCAN 오류와 drive fault의 원인을 제거하고 `lifecycle` 필드가 `Faulted`인지 확인합니다.
먼저 통신을 복구합니다.

```bash
ros2 service call /left_hand_control/reconnect std_srvs/srv/Trigger
```

`success=True`이고 응답이 `reconnected — call ~/run to resume control`이면 제어를 시작합니다.
실패하면 다음 명령으로 진행하지 말고 응답의 원인을 확인하십시오.

```bash
ros2 service call /left_hand_control/run std_srvs/srv/Trigger
```

`running` 응답을 확인합니다. `auto_home=true`이면 homing이 자동으로 시작됩니다.
`auto_home=false`일 때만 직접 시작합니다.

```bash
ros2 service call /left_hand_control/home std_srvs/srv/Trigger
```

[3. home](#3-home)의 방법으로 `homing_state` 필드가 `Succeeded`인지 확인합니다.
대상 command controller가 `active`인지 확인한 뒤 필요한 목표를 다시 보냅니다.

`~/reconnect` service는 `Faulted`에서만 사용합니다. 단순히 제어를 멈췄다 재개하려면 `~/stop`과 `~/run` service를
사용하십시오. 연결을 완전히 닫고 다시 시작해야 한다면 [5. Shutdown](#5-shutdown)으로 종료한 뒤
launch를 다시 실행합니다.

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
  [4.3 Monitoring](../docs/ko/05_control_guide.md#43-monitoring)에 있습니다.

## 5. Shutdown

정상 종료는 command controller 비활성화 → `~/stop` → launch 종료 순서입니다.
아래 예제는 joint position controller를 사용한 경우입니다. 다른 mode를 사용했다면 활성화된
controller 이름으로 바꾸고, chaining을 사용했다면 상위 controller부터 비활성화하십시오.

```bash
ros2 control switch_controllers --strict --deactivate left_joint_position_controller
ros2 service call /left_hand_control/stop std_srvs/srv/Trigger
```

양손이면 오른손에 대해 반복합니다. 그다음 launch 터미널에서 `Ctrl-C`를 누르면 controller_manager가
CAN 연결을 닫습니다.

`~/stop` service가 `stopped`를 반환했고 이후 `lifecycle` 값이 `Stopped`이면 actuator는 무토크입니다.
`SIGKILL`, 터미널 닫기, 네트워크 단절로는 quick stop이 확인되지 않으므로 정지 수단으로 쓰지
마십시오.

## 6. Runtime settings

effort 상한과 filter·gain은 `/{side}_hand_control` node의 ROS parameter로 바꿉니다.
로봇 핸드 연결에서 제공하며 mock에는 없습니다. 변경한 값은 다음 제어 주기부터 적용됩니다.
정지 중에도 바꿀 수 있고 `~/reconnect` 뒤에도 유지됩니다.

배열 parameter는 길이 1이면 actuator 16개에 같은 값을, 길이 16이면 actuator마다 다른 값을 적용합니다. 다른
길이거나 값이 유효하지 않으면 `ros2 param set`이 실패하고 어느 값도 바뀌지 않습니다.

실행 중 변경한 값은 launch를 다시 실행하면 초기화됩니다. 시작할 때 적용할 값은
[6.4 Initial runtime settings](#64-initial-runtime-settings)처럼 YAML에 저장합니다.

선언된 parameter와 현재 값은 다음으로 읽습니다.

```bash
ros2 param list /left_hand_control
ros2 param get /left_hand_control max_effort
```

### 6.1 Max effort

`max_effort` 값은 actuator로 전송되는 effort의 상한입니다.

| Parameter | Type | Default | Valid |
|---|---|---|---|
| `max_effort` | `double[]` | `[1000.0]` | 길이 1 또는 16. 유한하고 `0` 이상 |

단위는 정격 전류의 0.1%이고 정격 전류는
모든 모터가 400 mA이므로 `1000`(100%)이 400 mA입니다. SDK가 `[0, 2000]` 밖의 값을 `[0, 2000]`으로
제한합니다.

`ros2 param set`으로 쓸 때는 값에 소수점을 붙여야 `double` 배열로 해석됩니다. `[1000]`은 integer 배열이라
거부됩니다. 다음은 actuator 전체에 같은 값을 주는 예입니다.

```bash
ros2 param set /left_hand_control max_effort "[800.0]"
```

다음은 actuator마다 다르게 주는 예이고 값은 예시입니다.

```bash
ros2 param set /left_hand_control max_effort \
  "[1000.0, 1000.0, 1000.0, 1000.0,
     800.0,  800.0,  800.0,
     800.0,  800.0,  800.0,
     800.0,  800.0,  800.0,
     800.0,  800.0,  800.0]"
```

적용 중인 값은 `hand_state` topic의 `command_state.max_effort_pct` 필드로 확인합니다.

### 6.2 Joint position controller

`joint_position_controller` parameter 셋은 SDK의 joint position controller 설정입니다. joint position
목표는 매 cycle 다음 세 단계를 거쳐 actuator position이 됩니다.

1. 직전에 통과한 목표에서 `deadband` 값 이내로 움직인 입력은 무시합니다.
2. 3차 low-pass filter가 적용됩니다. `cutoff_freq` 값보다 빠른 변화가 줄어듭니다.
3. inverse kinematics로 actuator position이 됩니다.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `filter_enabled` | `bool` | `true` | `false`이면 1·2단계를 모두 생략합니다 |
| `cutoff_freq` | `double` | `10.0` | 차단 주파수 [Hz]. 유한하고 `0` 이상. `0`이면 2단계만 생략합니다 |
| `deadband` | `double` | `0.000873` | 무시할 변화량 [rad]. 유한하고 `0` 이상. 기본값은 0.05°이며 `0`이면 1단계만 생략합니다 |

`cutoff_freq` 값을 낮추면 목표 변화가 더 완만해지지만 응답 지연이 커집니다. 높이면 목표 변화가
더 빠르게 전달됩니다. filter는 속도 상한을 지정하는 기능이 아니므로 이동 속도를 제한하려면
상위 application에서 시간에 따른 목표값을 생성해야 합니다.

`deadband` parameter가 필요한 이유는 감속비가 크기 때문입니다. joint에서 미세한 변동이라도 actuator 축에서는
큰 반전이 되어 backlash 구간을 반복해서 통과합니다. 입력에 섞인 변동의 크기를 확인하며 조정하십시오.

다음은 차단 주파수를 20 Hz로 바꾸는 예입니다.

```bash
ros2 param set /left_hand_control joint_position_controller.cutoff_freq 20.0
```

### 6.3 Joint impedance controller

> [!NOTE]
> joint impedance controller는 SDK에서 개발 중이므로 사용하지 마십시오. 아래는 설정값의 의미입니다.

`joint_impedance_controller` parameter 둘은 SDK의 joint impedance controller gain입니다.
joint impedance 목표를 inverse kinematics로 actuator position으로 바꾼 뒤, encoder 공간에서
다음 식으로 effort를 산출합니다.

```text
effort = stiffness × position_error − damping × velocity
```

두 gain은 joint가 아니라 actuator 공간의 값이고 actuator 16개에 대응합니다.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stiffness` | `double[]` | thumb `0.02` × 4, 나머지 finger마다 `0.01`·`0.01`·`0.02` | 위치 오차에 곱하는 gain. 길이 1 또는 16. 유한하고 `0` 이상 |
| `damping` | `double[]` | `1e-5` × 16 | 속도에 곱하는 gain. 길이 1 또는 16. 유한하고 `0` 이상 |

### 6.4 Initial runtime settings

[6. Runtime settings](#6-runtime-settings)의 값을 실행 때마다 적용하려면 `ros2_control_node`에 전달하는
controller 설정 YAML에 node 이름의 블록을 추가합니다. 이름은 매크로의 `name` 값과 같아야 합니다.
다음은 `left_hand_control`의 effort 상한과 joint position 설정을 기본값으로 지정하는 예입니다.
`max_effort`를 YAML에서 생략하면 매크로에서 지정한 초깃값을 사용합니다.

```yaml
left_hand_control:
  ros__parameters:
    max_effort: [1000.0]                # 1개면 actuator 16개에 공통
    joint_position_controller:
      filter_enabled: true
      cutoff_freq: 10.0                 # Hz
      deadband: 0.000873                # rad
```

기본 launch는 [aidin_hand2_bringup/config/controllers.yaml](../aidin_hand2_bringup/config/controllers.yaml)을
읽습니다. 이 파일에는 양손의 초기 설정이 이미 있으므로 해당 블록의 값을 수정합니다.
설치된 파일에 변경이 반영되도록 package를 다시 빌드한 뒤 launch를 다시 실행하십시오.
사용자 로봇에서는 이 블록을 자신의 controller YAML에 추가하고
[7.1 Launch the robot and controllers](../aidin_hand2_bringup/README.ko.md#71-launch-the-robot-and-controllers)처럼 전달합니다.

# aidin_hand2_description

[전체 문서](../README.ko.md#documentation) | [English overview](README.md)

`aidin_hand2_description` package는 로봇 핸드의 xacro·URDF와 mesh를 제공합니다.
이 문서에서는 파일 위치를 확인하고, 사용자 URDF에 로봇 핸드를 부착하며, 매크로 인자를 설정합니다.
모델만 보려면 [3. description.launch.py](#3-descriptionlaunchpy)를 실행합니다. ROS 2 없이 다른
도구에서 열려면 [4. Plain URDF for other tools](#4-plain-urdf-for-other-tools)를 보십시오.

## Contents

&nbsp;&nbsp;[**1. Files**](#1-files)<br>
&nbsp;&nbsp;[**2. Add the robot hand to the URDF**](#2-add-the-robot-hand-to-the-urdf)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Files and macro calls](#21-files-and-macro-calls)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Macro settings](#22-macro-settings)<br>
&nbsp;&nbsp;[**3. description.launch.py**](#3-descriptionlaunchpy)<br>
&nbsp;&nbsp;[**4. Plain URDF for other tools**](#4-plain-urdf-for-other-tools)

## 1. Files

아래 경로는 이 package 디렉터리 기준입니다. 모델은 한 손에 joint 21개와 actuator 16개를 포함합니다.
각 finger의 `joint4`는 four-bar로 같은 finger의 `joint3`에 연결된 수동 joint이므로 actuator가
없습니다. 촉각 센서는 finger마다 셋, 손바닥에 둘, 합쳐 한 손에 링크 17개이고 모두 fixed joint로
붙습니다. actuator가 없고 TF에서 각 패드의 위치를 얻기 위한 것입니다.

| Directory | Purpose |
|---|---|
| [xacro/](xacro) | ROS 2가 모델을 만드는 원본. 최상위 [aidin_hand2.urdf.xacro](xacro/aidin_hand2.urdf.xacro)가 왼손·오른손 모델 매크로와 제어 설정을 모두 불러옵니다 |
| [ros2_control/](ros2_control) | 로봇 핸드·mock 제어 설정 매크로 |
| [urdf/](urdf) | `package://`를 해석하지 않는 도구용 전개본. [4. Plain URDF for other tools](#4-plain-urdf-for-other-tools) |
| [meshes/](meshes) | 손별 visual·collision mesh. 파일 이름이 그 mesh를 그리는 링크 이름과 같습니다 |
| [launch/](launch) | [description.launch.py](launch/description.launch.py)로 모델을 시각화합니다 |

## 2. Add the robot hand to the URDF

기존 로봇의 URDF에 로봇 핸드를 붙일 때 사용합니다. controller와 launch까지 구성하는 순서는
[7. Add to your robot](../aidin_hand2_bringup/README.ko.md#7-add-to-your-robot)에 있습니다.

### 2.1 Files and macro calls

수정할 파일은 사용자의 로봇 URDF입니다. 예제에서는
`my_robot_description/urdf/my_robot.urdf.xacro`를 사용합니다.

`aidin_hand2_description` package는 로봇 핸드의 모델과 제어 설정을 xacro 파일로 제공합니다.
사용자 URDF에서 이 파일을 불러와 사용할 수 있으므로, 제공된 파일의 내용을 직접 고칠 필요는 없습니다.

xacro 매크로는 이름을 붙여 재사용할 수 있게 만든 XML 묶음입니다. 매크로를 정의한 `.xacro` 파일을
`<xacro:include>`로 불러온 뒤, `<xacro:매크로이름 ...>`으로 호출하면 해당 내용이 URDF에 추가됩니다.
로봇 핸드에는 **모델을 추가하는 매크로와 제어 설정을 추가하는 매크로**가 필요합니다.

아래 파일은 모두 이 저장소의 `aidin_hand2_description` package 안에 있습니다.

| Purpose | Macro name | Definition file |
|---|---|---|
| 왼손의 링크·joint·mesh와 부착 위치 | `aidin_hand2_left` | [xacro/aidin_hand2_left.urdf.xacro](xacro/aidin_hand2_left.urdf.xacro) |
| 오른손의 링크·joint·mesh와 부착 위치 | `aidin_hand2_right` | [xacro/aidin_hand2_right.urdf.xacro](xacro/aidin_hand2_right.urdf.xacro) |
| 좌우 구분·CAN 연결 등 제어 설정 | `aidin_hand2_ros2_control` | [ros2_control/aidin_hand2.ros2_control.xacro](ros2_control/aidin_hand2.ros2_control.xacro) |

다음은 양손을 붙이는 예제입니다. 모델 파일은 손마다 하나씩 불러오고, 제어 설정 파일은 좌우가
같은 매크로를 쓰므로 한 번만 불러옵니다. `$(find aidin_hand2_description)`은 설치된 package의
공유 디렉터리를 찾는 xacro 표현식이므로, 사용자의 workspace 절대 경로를 적지 않아도 됩니다.

아래 내용을 기존 URDF의 `<robot>` 요소 안에 추가합니다. 바깥 `<robot>`은 위치를 보여 주기
위한 것으로, 기존 `<robot>` 안에 다시 중첩하지 않습니다. 기존 요소에 `xmlns:xacro`가 없다면 추가하십시오.
예제의 `my_arm_left_tool0`와 `my_arm_right_tool0`는 기존 로봇의 링크 이름으로 바꿉니다.

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- 기존 로봇의 링크·joint 정의는 유지합니다. -->

  <!-- 제공된 매크로 정의를 불러옵니다. -->
  <xacro:include filename="$(find aidin_hand2_description)/xacro/aidin_hand2_left.urdf.xacro"/>
  <xacro:include filename="$(find aidin_hand2_description)/xacro/aidin_hand2_right.urdf.xacro"/>
  <xacro:include filename="$(find aidin_hand2_description)/ros2_control/aidin_hand2.ros2_control.xacro"/>

  <!-- 왼손 모델을 기존 로봇의 링크에 붙입니다. -->
  <xacro:aidin_hand2_left prefix="left_" parent="my_arm_left_tool0">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:aidin_hand2_left>

  <!-- 왼손의 제어 설정을 추가합니다. -->
  <xacro:aidin_hand2_ros2_control
    name="left_hand_control" prefix="left_" hand_side="left" can_interface="can0"
    auto_home="false"
    max_effort="1000"
    control_rate="500"
    rt_cpu_affinity="-1"
    disabled_actuators=""
    auto_reconnect="false"
    auto_reconnect_timeout_ms="0"
    auto_reconnect_home="false"
    use_mock="false"/>

  <!-- 오른손 모델을 기존 로봇의 링크에 붙입니다. -->
  <xacro:aidin_hand2_right prefix="right_" parent="my_arm_right_tool0">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:aidin_hand2_right>

  <!-- 오른손의 제어 설정을 추가합니다. -->
  <xacro:aidin_hand2_ros2_control
    name="right_hand_control" prefix="right_" hand_side="right" can_interface="can1"
    auto_home="false"
    max_effort="1000"
    control_rate="500"
    rt_cpu_affinity="-1"
    disabled_actuators=""
    auto_reconnect="false"
    auto_reconnect_timeout_ms="0"
    auto_reconnect_home="false"
    use_mock="false"/>
</robot>
```

모델의 `parent`는 부착할 기존 로봇의 링크 이름입니다. `<origin>`은 해당 링크 기준의 부착 위치
`xyz` [m]와 회전 `rpy` [rad]입니다. 예제의 `0`은 위치·회전 오프셋이 없는 경우이며 실제 장착 위치에
맞게 바꾸십시오. 같은 손의 모델 매크로와 제어 설정 매크로는 `prefix`를 일치시킵니다.
제어 설정 매크로의 `name`은 hardware component마다 달라야 하므로 좌우를 구별해 지정합니다.

제어 설정 매크로에는 인자 13개를 모두 적었습니다. `name`·`prefix`·`hand_side`·`can_interface` 넷은
필수이고 나머지 아홉은 기본값이 있으므로, 기본값을 그대로 쓸 인자는 생략해도 됩니다. 예제에서
기본값과 다르게 지정한 인자는 `auto_home` 하나이며, 나머지 여덟은 기본값을 그대로 적은 것입니다.
각 인자의 뜻과 기본값은 [2.2 Macro settings](#22-macro-settings)에 있습니다.

예제의 `can0`·`can1`은 예시값입니다. 로봇 핸드가 연결된 CAN interface 이름은 컴퓨터마다 다르므로
실제 이름으로 바꾸십시오.

한 손만 붙이려면 해당 손의 `<xacro:include>`와 모델·제어 설정 호출 한 쌍만 남깁니다.
반대쪽 모델 파일은 불러오지 않아도 됩니다.

### 2.2 Macro settings

`aidin_hand2_ros2_control` 매크로를 호출할 때 지정하는 값입니다.
바꾸려면 URDF를 다시 만들어 launch합니다.
`name`부터 `can_interface`까지 넷이 필수이고 나머지는 기본값이 있습니다.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `name` | string | 필수 | hardware component 이름. service node 이름과 [초기 설정 블록](../aidin_hand2_hardware/README.ko.md#64-initial-runtime-settings)의 이름으로도 쓰입니다 |
| `prefix` | string | 필수 | joint·actuator·interface 이름 접두어. `left_` 또는 `right_` |
| `hand_side` | string | 필수 | `left` 또는 `right`. CAN ID와 kinematics를 결정합니다 |
| `can_interface` | string | 필수 | CAN interface 이름. `auto`는 side의 CAN ID로 채널을 탐색합니다 |
| `auto_home` | bool | `true` | `~/run` service 성공 뒤 `homing_state` 값이 `Succeeded`가 아니면 homing을 한 번 시작합니다 |
| `max_effort` | double | `1000` | effort 상한의 초깃값. 단위는 정격 전류의 0.1%이고 `1000`이 100%. runtime 값은 [6.1 Max effort](../aidin_hand2_hardware/README.ko.md#61-max-effort) |
| `control_rate` | int | `500` | 제어·통신 루프 주파수 [Hz]. controller_manager의 `update_rate` 값과 같게 둡니다 |
| `rt_cpu_affinity` | int | `-1` | 제어·통신 루프 thread를 고정할 CPU 번호. `-1`은 고정 안 함 |
| `disabled_actuators` | string | `''` | 사용하지 않을 actuator index. `"0,1,2,3"`처럼 쉼표로 구분하고 공백은 무시합니다 |
| `auto_reconnect` | bool | `false` | 통신 오류가 발생하면 SDK가 재연결을 반복 시도합니다 |
| `auto_reconnect_timeout_ms` | int | `0` | 자동 재연결 제한 시간 [ms]. `0`은 제한 없음 |
| `auto_reconnect_home` | bool | `false` | 자동 재연결 뒤 `Running`으로 복귀하기 전에 homing을 수행합니다 |
| `use_mock` | bool | `false` | CAN 대신 kinematics mock backend를 씁니다 |

`hand_side` 필드는 `left`·`right` 중 하나여야 하고 `can_interface`는 비워 둘 수 없습니다.
`auto_home` 인자의 시작 조건은 [3. home](../aidin_hand2_hardware/README.ko.md#3-home)에 있습니다.

`disabled_actuators` parameter에 지정한 actuator는 enable하지 않고, fault로 보고하지 않으며, homing에서도
제외합니다. 물리적으로 없는 actuator나 고장으로 당장 쓰지 않을 actuator를 뺄 때 쓰며, 되도록 finger
단위로 지정합니다. index와 actuator 이름의 대응은 다음과 같습니다.

```text
0..3    thumb_actuator0..3
4..6    index_actuator1..3
7..9    middle_actuator1..3
10..12  ring_actuator1..3
13..15  baby_actuator1..3
```

> [!WARNING]
> `auto_home`을 생략하면 기본값 `true`가 적용되어 launch 직후 로봇 핸드가 hard stop까지 움직입니다.
> 첫 실행에서는 예제처럼 `false`를 지정하고, 주변을 확인한 뒤 homing을 시작하십시오.

## 3. description.launch.py

`description.launch.py`는 hardware 없이 URDF만 RViz에 표시합니다. `use_mock:=true`로 `robot_description`을
만들고 `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`를 올립니다.

| Argument | Default | Description |
|---|---|---|
| `use_left_hand` | `true` | 왼손을 표시합니다 |
| `use_right_hand` | `true` | 오른손을 표시합니다 |
| `use_gui` | `true` | `joint_state_publisher_gui`의 slider로 joint를 움직입니다 |

인자 없이 실행하면 양손을 표시합니다.

```bash
ros2 launch aidin_hand2_description description.launch.py
```

한 손만 보려면 반대쪽을 끕니다.

```bash
ros2 launch aidin_hand2_description description.launch.py use_right_hand:=false
```

## 4. Plain URDF for other tools

`package://`를 해석하지 않는 도구에서 로봇 핸드를 여는 파일입니다. MuJoCo·PyBullet·pinocchio와
브라우저 뷰어가 여기에 해당합니다. ROS 2에서는 이 파일을 쓰지 않고 `xacro/aidin_hand2.urdf.xacro`로
모델을 만듭니다. RViz와 MoveIt은 mesh를 `package://`나 `file://`로만 찾으므로, 이 파일을 그대로
쓰면 링크는 나타나도 mesh가 표시되지 않습니다.

| File | Contents |
|---|---|
| [urdf/aidin_hand2_left.urdf](urdf/aidin_hand2_left.urdf) | 왼손 하나. root 링크는 `left_hand_base_link` |
| [urdf/aidin_hand2_right.urdf](urdf/aidin_hand2_right.urdf) | 오른손 하나. root 링크는 `right_hand_base_link` |

mesh 경로는 파일 기준 상대 경로입니다. 다른 곳으로 옮길 때는 `urdf/`와 `meshes/`의 위치 관계를
유지하십시오.

> [!IMPORTANT]
> 각 finger의 `joint4`는 실제 로봇 핸드에서 four-bar로 같은 finger의 `joint3`에 딸려 움직이지만,
> URDF는 이 폐루프를 표현하지 못합니다. 이 파일에서 `joint4`는 독립 joint이므로 사용하는 도구에서
> 구속을 지정하십시오. MuJoCo는 `equality`로, `<mimic>`을 읽는 도구는 mimic으로 지정합니다.

> [!NOTE]
> visual mesh와 collision mesh는 파일 이름이 같고 디렉터리로만 구분됩니다. MuJoCo처럼 mesh 파일
> 이름에서 경로를 떼는 도구에서는 `compiler`의 `strippath` 값을 `false`로 지정해야 두 mesh를
> 구별합니다.

xacro를 고치면 이 파일도 다시 만듭니다.

```bash
xacro xacro/aidin_hand2.urdf.xacro use_left_hand:=false > urdf/aidin_hand2_right.urdf
```

전개한 결과에서 `<ros2_control>` 블록과 `world` 링크·그 고정 joint를 지우고,
`package://aidin_hand2_description/`를 `../`로 바꾸고, `<robot>`의 `name` 값을 `aidin_hand2_right`로
고칩니다. 왼손은 `use_right_hand:=false`로 전개하고 이름을 `aidin_hand2_left`로 고칩니다.

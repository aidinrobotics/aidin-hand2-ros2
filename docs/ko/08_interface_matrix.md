# Interface matrix — 전체 인터페이스 목록

이 문서는 AIDIN Hand Gen2 ROS 2 wrapper 가 노출하는 **모든** ros2_control 인터페이스를
생략 없이 나열합니다. 요약과 설계 배경은 [Interface reference](04_interfaces.md) 를,
여기서는 이름 하나하나를 그대로 확인하십시오.

## 0. 표기 규약

- `{side}` 는 `left` 또는 `right` 입니다. 두 손은 같은 이름 규약을 각자의 prefix 로 가지며,
  아래의 모든 행이 `left_`·`right_` 두 벌로 존재합니다 (예: `left_thumb_actuator0/position_cnt`,
  `right_thumb_actuator0/position_cnt`).
- ros2_control 전체 이름은 `<component>/<interface>` 입니다. component 는 URDF 의
  joint · sensor · gpio 이름이고, chainable controller 의 reference 는 component 자리에
  controller 이름이 옵니다.
- `#` 열은 그 그룹 안에서의 0-기반 순번입니다. `claim #` 열은 해당 broadcaster 가
  `state_interface_configuration()` 으로 claim 하는 순서(= `state_interfaces_` 인덱스)이며,
  broadcaster 가 offset 으로 직접 인덱싱하므로 이 순서가 계약입니다.
- 개수 요약은 [10. 개수 요약](#10-개수-요약) 에 있습니다.

## 1. 문서 구성

| 절 | 내용 |
|---|---|
| [2](#2-하드웨어-command-interface-98-개) | 하드웨어 command interface 98 개 (real · mock 공통) |
| [3](#3-하드웨어-state-interface--actuator--joint) | 하드웨어 state interface — actuator 48 · joint 21 |
| [4](#4-하드웨어-state-interface--tactile-143-개-real-전용) | 하드웨어 state interface — tactile 143 개 |
| [5](#5-하드웨어-state-interface--diagnostics-39-개-real-전용) | 하드웨어 state interface — diagnostics 39 개 |
| [6](#6-하드웨어-state-interface--command-echo-132-개-real-전용) | 하드웨어 state interface — command echo 132 개 |
| [7](#7-하드웨어-state-interface--timestamp-2-개-real-전용) | 하드웨어 state interface — timestamp 2 개 |
| [8](#8-controller-별-claim--reference-전체) | Controller 별 claim · reference 전체 |
| [9](#9-mock-하드웨어-차이) | Mock 하드웨어 차이 |
| [10](#10-개수-요약) | 개수 요약 |
| [11](#11-런타임-확인-명령) | 런타임 확인 명령 |

## 2. 하드웨어 command interface 98 개

Real(`AidinHand2SystemInterface`) 과 mock(`AidinHand2MockSystemInterface`) 이 **동일하게**
export 합니다. mode switch 는 빈 집합(Idle) 또는 아래 네 controller 중 하나가 claim 하는 완전한 집합
+ `command_lock` 만 허용합니다. 부분·혼합 claim 은 거부됩니다.

**값 계약** — command interface 의 `NaN` 은 명령이 아니라 "없음" 을 뜻합니다.

| target 값 | 의미 | hardware 동작 |
|---|---|---|
| 전부 `NaN` | 이번 cycle 명령 없음 | `set_command` 를 부르지 않는다 — SDK 가 직전 명령을 유지 |
| 일부 `NaN` | 그 축을 상위가 점유하지 않음 | 그 축의 직전 명령값으로 메워 전송 |
| 일부 `NaN` + 직전 명령도 없음 | 완성 불가 | 전송하지 않고 경고 (완전한 command 를 한 번 보내야 한다) |
| 전부 유한 | 명령 | 그대로 전송 |

controller 는 입력이 있는 cycle 에만 값을 싣고 소비한 뒤 `NaN` 으로 되돌립니다. 같은 값이 다음
cycle 에 다시 명령으로 나가지 않으므로, homing·stop 처럼 명령의 전제가 바뀌는 구간을 지나도 옛
목표가 되살아나지 않습니다.

### 2.1 Claim-only lock — 1 개

| # | Command interface | 의미 |
|---|---|---|
| 0 | `{side}_hand_control/command_lock` | 값 미사용. mode 상호 배제를 위한 resource claim 전용 |

### 2.2 JointPositionController 가 claim 하는 command interface — 17 개

| # | Command interface | 단위 | 의미 |
|---|---|---|---|
| 0 | `{side}_joint_position_command/target_position_rad.thumb_joint0` | rad | `{side}_thumb_joint0` 목표 각도 |
| 1 | `{side}_joint_position_command/target_position_rad.thumb_joint1` | rad | `{side}_thumb_joint1` 목표 각도 |
| 2 | `{side}_joint_position_command/target_position_rad.thumb_joint2` | rad | `{side}_thumb_joint2` 목표 각도 |
| 3 | `{side}_joint_position_command/target_position_rad.thumb_joint3` | rad | `{side}_thumb_joint3` 목표 각도 |
| 4 | `{side}_joint_position_command/target_position_rad.index_joint1` | rad | `{side}_index_joint1` 목표 각도 |
| 5 | `{side}_joint_position_command/target_position_rad.index_joint2` | rad | `{side}_index_joint2` 목표 각도 |
| 6 | `{side}_joint_position_command/target_position_rad.index_joint3` | rad | `{side}_index_joint3` 목표 각도 |
| 7 | `{side}_joint_position_command/target_position_rad.middle_joint1` | rad | `{side}_middle_joint1` 목표 각도 |
| 8 | `{side}_joint_position_command/target_position_rad.middle_joint2` | rad | `{side}_middle_joint2` 목표 각도 |
| 9 | `{side}_joint_position_command/target_position_rad.middle_joint3` | rad | `{side}_middle_joint3` 목표 각도 |
| 10 | `{side}_joint_position_command/target_position_rad.ring_joint1` | rad | `{side}_ring_joint1` 목표 각도 |
| 11 | `{side}_joint_position_command/target_position_rad.ring_joint2` | rad | `{side}_ring_joint2` 목표 각도 |
| 12 | `{side}_joint_position_command/target_position_rad.ring_joint3` | rad | `{side}_ring_joint3` 목표 각도 |
| 13 | `{side}_joint_position_command/target_position_rad.baby_joint1` | rad | `{side}_baby_joint1` 목표 각도 |
| 14 | `{side}_joint_position_command/target_position_rad.baby_joint2` | rad | `{side}_baby_joint2` 목표 각도 |
| 15 | `{side}_joint_position_command/target_position_rad.baby_joint3` | rad | `{side}_baby_joint3` 목표 각도 |
| 16 | `{side}_joint_position_command/speed_rad_s` | rad/s | 16 관절 공통 속도 상한 (0 = 즉시 추종) |

### 2.3 JointImpedanceController 가 claim 하는 command interface — 48 개

| # | Command interface | 단위 | 의미 |
|---|---|---|---|
| 0 | `{side}_joint_impedance_command/target_position_rad.thumb_joint0` | rad | `{side}_thumb_joint0` 평형 자세 |
| 1 | `{side}_joint_impedance_command/target_position_rad.thumb_joint1` | rad | `{side}_thumb_joint1` 평형 자세 |
| 2 | `{side}_joint_impedance_command/target_position_rad.thumb_joint2` | rad | `{side}_thumb_joint2` 평형 자세 |
| 3 | `{side}_joint_impedance_command/target_position_rad.thumb_joint3` | rad | `{side}_thumb_joint3` 평형 자세 |
| 4 | `{side}_joint_impedance_command/target_position_rad.index_joint1` | rad | `{side}_index_joint1` 평형 자세 |
| 5 | `{side}_joint_impedance_command/target_position_rad.index_joint2` | rad | `{side}_index_joint2` 평형 자세 |
| 6 | `{side}_joint_impedance_command/target_position_rad.index_joint3` | rad | `{side}_index_joint3` 평형 자세 |
| 7 | `{side}_joint_impedance_command/target_position_rad.middle_joint1` | rad | `{side}_middle_joint1` 평형 자세 |
| 8 | `{side}_joint_impedance_command/target_position_rad.middle_joint2` | rad | `{side}_middle_joint2` 평형 자세 |
| 9 | `{side}_joint_impedance_command/target_position_rad.middle_joint3` | rad | `{side}_middle_joint3` 평형 자세 |
| 10 | `{side}_joint_impedance_command/target_position_rad.ring_joint1` | rad | `{side}_ring_joint1` 평형 자세 |
| 11 | `{side}_joint_impedance_command/target_position_rad.ring_joint2` | rad | `{side}_ring_joint2` 평형 자세 |
| 12 | `{side}_joint_impedance_command/target_position_rad.ring_joint3` | rad | `{side}_ring_joint3` 평형 자세 |
| 13 | `{side}_joint_impedance_command/target_position_rad.baby_joint1` | rad | `{side}_baby_joint1` 평형 자세 |
| 14 | `{side}_joint_impedance_command/target_position_rad.baby_joint2` | rad | `{side}_baby_joint2` 평형 자세 |
| 15 | `{side}_joint_impedance_command/target_position_rad.baby_joint3` | rad | `{side}_baby_joint3` 평형 자세 |
| 16 | `{side}_joint_impedance_command/stiffness.thumb_actuator0` | SDK gain | `{side}_thumb_actuator0` 강성 (0 이상) |
| 17 | `{side}_joint_impedance_command/stiffness.thumb_actuator1` | SDK gain | `{side}_thumb_actuator1` 강성 (0 이상) |
| 18 | `{side}_joint_impedance_command/stiffness.thumb_actuator2` | SDK gain | `{side}_thumb_actuator2` 강성 (0 이상) |
| 19 | `{side}_joint_impedance_command/stiffness.thumb_actuator3` | SDK gain | `{side}_thumb_actuator3` 강성 (0 이상) |
| 20 | `{side}_joint_impedance_command/stiffness.index_actuator1` | SDK gain | `{side}_index_actuator1` 강성 (0 이상) |
| 21 | `{side}_joint_impedance_command/stiffness.index_actuator2` | SDK gain | `{side}_index_actuator2` 강성 (0 이상) |
| 22 | `{side}_joint_impedance_command/stiffness.index_actuator3` | SDK gain | `{side}_index_actuator3` 강성 (0 이상) |
| 23 | `{side}_joint_impedance_command/stiffness.middle_actuator1` | SDK gain | `{side}_middle_actuator1` 강성 (0 이상) |
| 24 | `{side}_joint_impedance_command/stiffness.middle_actuator2` | SDK gain | `{side}_middle_actuator2` 강성 (0 이상) |
| 25 | `{side}_joint_impedance_command/stiffness.middle_actuator3` | SDK gain | `{side}_middle_actuator3` 강성 (0 이상) |
| 26 | `{side}_joint_impedance_command/stiffness.ring_actuator1` | SDK gain | `{side}_ring_actuator1` 강성 (0 이상) |
| 27 | `{side}_joint_impedance_command/stiffness.ring_actuator2` | SDK gain | `{side}_ring_actuator2` 강성 (0 이상) |
| 28 | `{side}_joint_impedance_command/stiffness.ring_actuator3` | SDK gain | `{side}_ring_actuator3` 강성 (0 이상) |
| 29 | `{side}_joint_impedance_command/stiffness.baby_actuator1` | SDK gain | `{side}_baby_actuator1` 강성 (0 이상) |
| 30 | `{side}_joint_impedance_command/stiffness.baby_actuator2` | SDK gain | `{side}_baby_actuator2` 강성 (0 이상) |
| 31 | `{side}_joint_impedance_command/stiffness.baby_actuator3` | SDK gain | `{side}_baby_actuator3` 강성 (0 이상) |
| 32 | `{side}_joint_impedance_command/damping.thumb_actuator0` | SDK gain | `{side}_thumb_actuator0` 감쇠 (0 이상) |
| 33 | `{side}_joint_impedance_command/damping.thumb_actuator1` | SDK gain | `{side}_thumb_actuator1` 감쇠 (0 이상) |
| 34 | `{side}_joint_impedance_command/damping.thumb_actuator2` | SDK gain | `{side}_thumb_actuator2` 감쇠 (0 이상) |
| 35 | `{side}_joint_impedance_command/damping.thumb_actuator3` | SDK gain | `{side}_thumb_actuator3` 감쇠 (0 이상) |
| 36 | `{side}_joint_impedance_command/damping.index_actuator1` | SDK gain | `{side}_index_actuator1` 감쇠 (0 이상) |
| 37 | `{side}_joint_impedance_command/damping.index_actuator2` | SDK gain | `{side}_index_actuator2` 감쇠 (0 이상) |
| 38 | `{side}_joint_impedance_command/damping.index_actuator3` | SDK gain | `{side}_index_actuator3` 감쇠 (0 이상) |
| 39 | `{side}_joint_impedance_command/damping.middle_actuator1` | SDK gain | `{side}_middle_actuator1` 감쇠 (0 이상) |
| 40 | `{side}_joint_impedance_command/damping.middle_actuator2` | SDK gain | `{side}_middle_actuator2` 감쇠 (0 이상) |
| 41 | `{side}_joint_impedance_command/damping.middle_actuator3` | SDK gain | `{side}_middle_actuator3` 감쇠 (0 이상) |
| 42 | `{side}_joint_impedance_command/damping.ring_actuator1` | SDK gain | `{side}_ring_actuator1` 감쇠 (0 이상) |
| 43 | `{side}_joint_impedance_command/damping.ring_actuator2` | SDK gain | `{side}_ring_actuator2` 감쇠 (0 이상) |
| 44 | `{side}_joint_impedance_command/damping.ring_actuator3` | SDK gain | `{side}_ring_actuator3` 감쇠 (0 이상) |
| 45 | `{side}_joint_impedance_command/damping.baby_actuator1` | SDK gain | `{side}_baby_actuator1` 감쇠 (0 이상) |
| 46 | `{side}_joint_impedance_command/damping.baby_actuator2` | SDK gain | `{side}_baby_actuator2` 감쇠 (0 이상) |
| 47 | `{side}_joint_impedance_command/damping.baby_actuator3` | SDK gain | `{side}_baby_actuator3` 감쇠 (0 이상) |

### 2.4 ActuatorPositionController 가 claim 하는 command interface — 16 개

| # | Command interface | 단위 | 의미 |
|---|---|---|---|
| 0 | `{side}_actuator_position_command/target_position_cnt.thumb_actuator0` | encoder count | `{side}_thumb_actuator0` 목표 위치 |
| 1 | `{side}_actuator_position_command/target_position_cnt.thumb_actuator1` | encoder count | `{side}_thumb_actuator1` 목표 위치 |
| 2 | `{side}_actuator_position_command/target_position_cnt.thumb_actuator2` | encoder count | `{side}_thumb_actuator2` 목표 위치 |
| 3 | `{side}_actuator_position_command/target_position_cnt.thumb_actuator3` | encoder count | `{side}_thumb_actuator3` 목표 위치 |
| 4 | `{side}_actuator_position_command/target_position_cnt.index_actuator1` | encoder count | `{side}_index_actuator1` 목표 위치 |
| 5 | `{side}_actuator_position_command/target_position_cnt.index_actuator2` | encoder count | `{side}_index_actuator2` 목표 위치 |
| 6 | `{side}_actuator_position_command/target_position_cnt.index_actuator3` | encoder count | `{side}_index_actuator3` 목표 위치 |
| 7 | `{side}_actuator_position_command/target_position_cnt.middle_actuator1` | encoder count | `{side}_middle_actuator1` 목표 위치 |
| 8 | `{side}_actuator_position_command/target_position_cnt.middle_actuator2` | encoder count | `{side}_middle_actuator2` 목표 위치 |
| 9 | `{side}_actuator_position_command/target_position_cnt.middle_actuator3` | encoder count | `{side}_middle_actuator3` 목표 위치 |
| 10 | `{side}_actuator_position_command/target_position_cnt.ring_actuator1` | encoder count | `{side}_ring_actuator1` 목표 위치 |
| 11 | `{side}_actuator_position_command/target_position_cnt.ring_actuator2` | encoder count | `{side}_ring_actuator2` 목표 위치 |
| 12 | `{side}_actuator_position_command/target_position_cnt.ring_actuator3` | encoder count | `{side}_ring_actuator3` 목표 위치 |
| 13 | `{side}_actuator_position_command/target_position_cnt.baby_actuator1` | encoder count | `{side}_baby_actuator1` 목표 위치 |
| 14 | `{side}_actuator_position_command/target_position_cnt.baby_actuator2` | encoder count | `{side}_baby_actuator2` 목표 위치 |
| 15 | `{side}_actuator_position_command/target_position_cnt.baby_actuator3` | encoder count | `{side}_baby_actuator3` 목표 위치 |

### 2.5 ActuatorEffortController 가 claim 하는 command interface — 16 개

| # | Command interface | 단위 | 의미 |
|---|---|---|---|
| 0 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator0` | rated current % | `{side}_thumb_actuator0` 목표 토크 |
| 1 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator1` | rated current % | `{side}_thumb_actuator1` 목표 토크 |
| 2 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator2` | rated current % | `{side}_thumb_actuator2` 목표 토크 |
| 3 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator3` | rated current % | `{side}_thumb_actuator3` 목표 토크 |
| 4 | `{side}_actuator_effort_command/target_effort_pct.index_actuator1` | rated current % | `{side}_index_actuator1` 목표 토크 |
| 5 | `{side}_actuator_effort_command/target_effort_pct.index_actuator2` | rated current % | `{side}_index_actuator2` 목표 토크 |
| 6 | `{side}_actuator_effort_command/target_effort_pct.index_actuator3` | rated current % | `{side}_index_actuator3` 목표 토크 |
| 7 | `{side}_actuator_effort_command/target_effort_pct.middle_actuator1` | rated current % | `{side}_middle_actuator1` 목표 토크 |
| 8 | `{side}_actuator_effort_command/target_effort_pct.middle_actuator2` | rated current % | `{side}_middle_actuator2` 목표 토크 |
| 9 | `{side}_actuator_effort_command/target_effort_pct.middle_actuator3` | rated current % | `{side}_middle_actuator3` 목표 토크 |
| 10 | `{side}_actuator_effort_command/target_effort_pct.ring_actuator1` | rated current % | `{side}_ring_actuator1` 목표 토크 |
| 11 | `{side}_actuator_effort_command/target_effort_pct.ring_actuator2` | rated current % | `{side}_ring_actuator2` 목표 토크 |
| 12 | `{side}_actuator_effort_command/target_effort_pct.ring_actuator3` | rated current % | `{side}_ring_actuator3` 목표 토크 |
| 13 | `{side}_actuator_effort_command/target_effort_pct.baby_actuator1` | rated current % | `{side}_baby_actuator1` 목표 토크 |
| 14 | `{side}_actuator_effort_command/target_effort_pct.baby_actuator2` | rated current % | `{side}_baby_actuator2` 목표 토크 |
| 15 | `{side}_actuator_effort_command/target_effort_pct.baby_actuator3` | rated current % | `{side}_baby_actuator3` 목표 토크 |

## 3. 하드웨어 state interface — actuator · joint

Real 과 mock 이 공통으로 export 하는 physical state 입니다. Real hardware 의
`export_state_interfaces()` 는 actuator 를 `position_cnt` → `velocity_rpm` → `current_ma`
순으로 **actuator 별 인터리브** 하여 등록하고, 그 뒤에 joint 21 개를 등록합니다.
`claim #` 는 `HandStateBroadcaster` 의 claim 순서입니다(그룹별 블록).

### 3.1 Joint position 21 개 — `{joint}/position` (rad)

| # | State interface | 단위 | 구분 | HandState 매핑 | claim # |
|---|---|---|---|---|---|
| 0 | `{side}_thumb_joint0/position` | rad | active | `HandState.joint_position[0]` | 0 |
| 1 | `{side}_thumb_joint1/position` | rad | active | `HandState.joint_position[1]` | 1 |
| 2 | `{side}_thumb_joint2/position` | rad | active | `HandState.joint_position[2]` | 2 |
| 3 | `{side}_thumb_joint3/position` | rad | active | `HandState.joint_position[3]` | 3 |
| 4 | `{side}_thumb_joint4/position` | rad | passive (q4 결합분) | `HandState.joint_position[4]` | 4 |
| 5 | `{side}_index_joint1/position` | rad | active | `HandState.joint_position[5]` | 5 |
| 6 | `{side}_index_joint2/position` | rad | active | `HandState.joint_position[6]` | 6 |
| 7 | `{side}_index_joint3/position` | rad | active | `HandState.joint_position[7]` | 7 |
| 8 | `{side}_index_joint4/position` | rad | passive (q4 결합분) | `HandState.joint_position[8]` | 8 |
| 9 | `{side}_middle_joint1/position` | rad | active | `HandState.joint_position[9]` | 9 |
| 10 | `{side}_middle_joint2/position` | rad | active | `HandState.joint_position[10]` | 10 |
| 11 | `{side}_middle_joint3/position` | rad | active | `HandState.joint_position[11]` | 11 |
| 12 | `{side}_middle_joint4/position` | rad | passive (q4 결합분) | `HandState.joint_position[12]` | 12 |
| 13 | `{side}_ring_joint1/position` | rad | active | `HandState.joint_position[13]` | 13 |
| 14 | `{side}_ring_joint2/position` | rad | active | `HandState.joint_position[14]` | 14 |
| 15 | `{side}_ring_joint3/position` | rad | active | `HandState.joint_position[15]` | 15 |
| 16 | `{side}_ring_joint4/position` | rad | passive (q4 결합분) | `HandState.joint_position[16]` | 16 |
| 17 | `{side}_baby_joint1/position` | rad | active | `HandState.joint_position[17]` | 17 |
| 18 | `{side}_baby_joint2/position` | rad | active | `HandState.joint_position[18]` | 18 |
| 19 | `{side}_baby_joint3/position` | rad | active | `HandState.joint_position[19]` | 19 |
| 20 | `{side}_baby_joint4/position` | rad | passive (q4 결합분) | `HandState.joint_position[20]` | 20 |

### 3.2 Actuator physical state 48 개

| # | State interface | 단위 | HandState 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_thumb_actuator0/position_cnt` | encoder count | `HandState.actuator_position[0]` | 21 |
| 1 | `{side}_thumb_actuator1/position_cnt` | encoder count | `HandState.actuator_position[1]` | 22 |
| 2 | `{side}_thumb_actuator2/position_cnt` | encoder count | `HandState.actuator_position[2]` | 23 |
| 3 | `{side}_thumb_actuator3/position_cnt` | encoder count | `HandState.actuator_position[3]` | 24 |
| 4 | `{side}_index_actuator1/position_cnt` | encoder count | `HandState.actuator_position[4]` | 25 |
| 5 | `{side}_index_actuator2/position_cnt` | encoder count | `HandState.actuator_position[5]` | 26 |
| 6 | `{side}_index_actuator3/position_cnt` | encoder count | `HandState.actuator_position[6]` | 27 |
| 7 | `{side}_middle_actuator1/position_cnt` | encoder count | `HandState.actuator_position[7]` | 28 |
| 8 | `{side}_middle_actuator2/position_cnt` | encoder count | `HandState.actuator_position[8]` | 29 |
| 9 | `{side}_middle_actuator3/position_cnt` | encoder count | `HandState.actuator_position[9]` | 30 |
| 10 | `{side}_ring_actuator1/position_cnt` | encoder count | `HandState.actuator_position[10]` | 31 |
| 11 | `{side}_ring_actuator2/position_cnt` | encoder count | `HandState.actuator_position[11]` | 32 |
| 12 | `{side}_ring_actuator3/position_cnt` | encoder count | `HandState.actuator_position[12]` | 33 |
| 13 | `{side}_baby_actuator1/position_cnt` | encoder count | `HandState.actuator_position[13]` | 34 |
| 14 | `{side}_baby_actuator2/position_cnt` | encoder count | `HandState.actuator_position[14]` | 35 |
| 15 | `{side}_baby_actuator3/position_cnt` | encoder count | `HandState.actuator_position[15]` | 36 |
| 16 | `{side}_thumb_actuator0/velocity_rpm` | rpm | `HandState.actuator_velocity[0]` | 37 |
| 17 | `{side}_thumb_actuator1/velocity_rpm` | rpm | `HandState.actuator_velocity[1]` | 38 |
| 18 | `{side}_thumb_actuator2/velocity_rpm` | rpm | `HandState.actuator_velocity[2]` | 39 |
| 19 | `{side}_thumb_actuator3/velocity_rpm` | rpm | `HandState.actuator_velocity[3]` | 40 |
| 20 | `{side}_index_actuator1/velocity_rpm` | rpm | `HandState.actuator_velocity[4]` | 41 |
| 21 | `{side}_index_actuator2/velocity_rpm` | rpm | `HandState.actuator_velocity[5]` | 42 |
| 22 | `{side}_index_actuator3/velocity_rpm` | rpm | `HandState.actuator_velocity[6]` | 43 |
| 23 | `{side}_middle_actuator1/velocity_rpm` | rpm | `HandState.actuator_velocity[7]` | 44 |
| 24 | `{side}_middle_actuator2/velocity_rpm` | rpm | `HandState.actuator_velocity[8]` | 45 |
| 25 | `{side}_middle_actuator3/velocity_rpm` | rpm | `HandState.actuator_velocity[9]` | 46 |
| 26 | `{side}_ring_actuator1/velocity_rpm` | rpm | `HandState.actuator_velocity[10]` | 47 |
| 27 | `{side}_ring_actuator2/velocity_rpm` | rpm | `HandState.actuator_velocity[11]` | 48 |
| 28 | `{side}_ring_actuator3/velocity_rpm` | rpm | `HandState.actuator_velocity[12]` | 49 |
| 29 | `{side}_baby_actuator1/velocity_rpm` | rpm | `HandState.actuator_velocity[13]` | 50 |
| 30 | `{side}_baby_actuator2/velocity_rpm` | rpm | `HandState.actuator_velocity[14]` | 51 |
| 31 | `{side}_baby_actuator3/velocity_rpm` | rpm | `HandState.actuator_velocity[15]` | 52 |
| 32 | `{side}_thumb_actuator0/current_ma` | mA | `HandState.actuator_current[0]` | 53 |
| 33 | `{side}_thumb_actuator1/current_ma` | mA | `HandState.actuator_current[1]` | 54 |
| 34 | `{side}_thumb_actuator2/current_ma` | mA | `HandState.actuator_current[2]` | 55 |
| 35 | `{side}_thumb_actuator3/current_ma` | mA | `HandState.actuator_current[3]` | 56 |
| 36 | `{side}_index_actuator1/current_ma` | mA | `HandState.actuator_current[4]` | 57 |
| 37 | `{side}_index_actuator2/current_ma` | mA | `HandState.actuator_current[5]` | 58 |
| 38 | `{side}_index_actuator3/current_ma` | mA | `HandState.actuator_current[6]` | 59 |
| 39 | `{side}_middle_actuator1/current_ma` | mA | `HandState.actuator_current[7]` | 60 |
| 40 | `{side}_middle_actuator2/current_ma` | mA | `HandState.actuator_current[8]` | 61 |
| 41 | `{side}_middle_actuator3/current_ma` | mA | `HandState.actuator_current[9]` | 62 |
| 42 | `{side}_ring_actuator1/current_ma` | mA | `HandState.actuator_current[10]` | 63 |
| 43 | `{side}_ring_actuator2/current_ma` | mA | `HandState.actuator_current[11]` | 64 |
| 44 | `{side}_ring_actuator3/current_ma` | mA | `HandState.actuator_current[12]` | 65 |
| 45 | `{side}_baby_actuator1/current_ma` | mA | `HandState.actuator_current[13]` | 66 |
| 46 | `{side}_baby_actuator2/current_ma` | mA | `HandState.actuator_current[14]` | 67 |
| 47 | `{side}_baby_actuator3/current_ma` | mA | `HandState.actuator_current[15]` | 68 |

## 4. 하드웨어 state interface — tactile 143 개 (real 전용)

Mock 은 tactile 을 노출하지 않습니다. 단위는 SDK 원시 taxel 값입니다.

### 4.1 Finger tactile 85 개 — sensor component `{side}_{finger}_sensor`

| # | State interface | HandState 매핑 | claim # |
|---|---|---|---|
| 0 | `{side}_thumb_sensor/tactile_1` | `HandState.tactile_thumb[0]` | 69 |
| 1 | `{side}_thumb_sensor/tactile_2` | `HandState.tactile_thumb[1]` | 70 |
| 2 | `{side}_thumb_sensor/tactile_3` | `HandState.tactile_thumb[2]` | 71 |
| 3 | `{side}_thumb_sensor/tactile_4` | `HandState.tactile_thumb[3]` | 72 |
| 4 | `{side}_thumb_sensor/tactile_5` | `HandState.tactile_thumb[4]` | 73 |
| 5 | `{side}_thumb_sensor/tactile_6` | `HandState.tactile_thumb[5]` | 74 |
| 6 | `{side}_thumb_sensor/tactile_7` | `HandState.tactile_thumb[6]` | 75 |
| 7 | `{side}_thumb_sensor/tactile_8` | `HandState.tactile_thumb[7]` | 76 |
| 8 | `{side}_thumb_sensor/tactile_9` | `HandState.tactile_thumb[8]` | 77 |
| 9 | `{side}_thumb_sensor/tactile_10` | `HandState.tactile_thumb[9]` | 78 |
| 10 | `{side}_thumb_sensor/tactile_11` | `HandState.tactile_thumb[10]` | 79 |
| 11 | `{side}_thumb_sensor/tactile_12` | `HandState.tactile_thumb[11]` | 80 |
| 12 | `{side}_thumb_sensor/tactile_13` | `HandState.tactile_thumb[12]` | 81 |
| 13 | `{side}_thumb_sensor/tactile_14` | `HandState.tactile_thumb[13]` | 82 |
| 14 | `{side}_thumb_sensor/tactile_15` | `HandState.tactile_thumb[14]` | 83 |
| 15 | `{side}_thumb_sensor/tactile_16` | `HandState.tactile_thumb[15]` | 84 |
| 16 | `{side}_thumb_sensor/tactile_17` | `HandState.tactile_thumb[16]` | 85 |
| 17 | `{side}_index_sensor/tactile_1` | `HandState.tactile_index[0]` | 86 |
| 18 | `{side}_index_sensor/tactile_2` | `HandState.tactile_index[1]` | 87 |
| 19 | `{side}_index_sensor/tactile_3` | `HandState.tactile_index[2]` | 88 |
| 20 | `{side}_index_sensor/tactile_4` | `HandState.tactile_index[3]` | 89 |
| 21 | `{side}_index_sensor/tactile_5` | `HandState.tactile_index[4]` | 90 |
| 22 | `{side}_index_sensor/tactile_6` | `HandState.tactile_index[5]` | 91 |
| 23 | `{side}_index_sensor/tactile_7` | `HandState.tactile_index[6]` | 92 |
| 24 | `{side}_index_sensor/tactile_8` | `HandState.tactile_index[7]` | 93 |
| 25 | `{side}_index_sensor/tactile_9` | `HandState.tactile_index[8]` | 94 |
| 26 | `{side}_index_sensor/tactile_10` | `HandState.tactile_index[9]` | 95 |
| 27 | `{side}_index_sensor/tactile_11` | `HandState.tactile_index[10]` | 96 |
| 28 | `{side}_index_sensor/tactile_12` | `HandState.tactile_index[11]` | 97 |
| 29 | `{side}_index_sensor/tactile_13` | `HandState.tactile_index[12]` | 98 |
| 30 | `{side}_index_sensor/tactile_14` | `HandState.tactile_index[13]` | 99 |
| 31 | `{side}_index_sensor/tactile_15` | `HandState.tactile_index[14]` | 100 |
| 32 | `{side}_index_sensor/tactile_16` | `HandState.tactile_index[15]` | 101 |
| 33 | `{side}_index_sensor/tactile_17` | `HandState.tactile_index[16]` | 102 |
| 34 | `{side}_middle_sensor/tactile_1` | `HandState.tactile_middle[0]` | 103 |
| 35 | `{side}_middle_sensor/tactile_2` | `HandState.tactile_middle[1]` | 104 |
| 36 | `{side}_middle_sensor/tactile_3` | `HandState.tactile_middle[2]` | 105 |
| 37 | `{side}_middle_sensor/tactile_4` | `HandState.tactile_middle[3]` | 106 |
| 38 | `{side}_middle_sensor/tactile_5` | `HandState.tactile_middle[4]` | 107 |
| 39 | `{side}_middle_sensor/tactile_6` | `HandState.tactile_middle[5]` | 108 |
| 40 | `{side}_middle_sensor/tactile_7` | `HandState.tactile_middle[6]` | 109 |
| 41 | `{side}_middle_sensor/tactile_8` | `HandState.tactile_middle[7]` | 110 |
| 42 | `{side}_middle_sensor/tactile_9` | `HandState.tactile_middle[8]` | 111 |
| 43 | `{side}_middle_sensor/tactile_10` | `HandState.tactile_middle[9]` | 112 |
| 44 | `{side}_middle_sensor/tactile_11` | `HandState.tactile_middle[10]` | 113 |
| 45 | `{side}_middle_sensor/tactile_12` | `HandState.tactile_middle[11]` | 114 |
| 46 | `{side}_middle_sensor/tactile_13` | `HandState.tactile_middle[12]` | 115 |
| 47 | `{side}_middle_sensor/tactile_14` | `HandState.tactile_middle[13]` | 116 |
| 48 | `{side}_middle_sensor/tactile_15` | `HandState.tactile_middle[14]` | 117 |
| 49 | `{side}_middle_sensor/tactile_16` | `HandState.tactile_middle[15]` | 118 |
| 50 | `{side}_middle_sensor/tactile_17` | `HandState.tactile_middle[16]` | 119 |
| 51 | `{side}_ring_sensor/tactile_1` | `HandState.tactile_ring[0]` | 120 |
| 52 | `{side}_ring_sensor/tactile_2` | `HandState.tactile_ring[1]` | 121 |
| 53 | `{side}_ring_sensor/tactile_3` | `HandState.tactile_ring[2]` | 122 |
| 54 | `{side}_ring_sensor/tactile_4` | `HandState.tactile_ring[3]` | 123 |
| 55 | `{side}_ring_sensor/tactile_5` | `HandState.tactile_ring[4]` | 124 |
| 56 | `{side}_ring_sensor/tactile_6` | `HandState.tactile_ring[5]` | 125 |
| 57 | `{side}_ring_sensor/tactile_7` | `HandState.tactile_ring[6]` | 126 |
| 58 | `{side}_ring_sensor/tactile_8` | `HandState.tactile_ring[7]` | 127 |
| 59 | `{side}_ring_sensor/tactile_9` | `HandState.tactile_ring[8]` | 128 |
| 60 | `{side}_ring_sensor/tactile_10` | `HandState.tactile_ring[9]` | 129 |
| 61 | `{side}_ring_sensor/tactile_11` | `HandState.tactile_ring[10]` | 130 |
| 62 | `{side}_ring_sensor/tactile_12` | `HandState.tactile_ring[11]` | 131 |
| 63 | `{side}_ring_sensor/tactile_13` | `HandState.tactile_ring[12]` | 132 |
| 64 | `{side}_ring_sensor/tactile_14` | `HandState.tactile_ring[13]` | 133 |
| 65 | `{side}_ring_sensor/tactile_15` | `HandState.tactile_ring[14]` | 134 |
| 66 | `{side}_ring_sensor/tactile_16` | `HandState.tactile_ring[15]` | 135 |
| 67 | `{side}_ring_sensor/tactile_17` | `HandState.tactile_ring[16]` | 136 |
| 68 | `{side}_baby_sensor/tactile_1` | `HandState.tactile_baby[0]` | 137 |
| 69 | `{side}_baby_sensor/tactile_2` | `HandState.tactile_baby[1]` | 138 |
| 70 | `{side}_baby_sensor/tactile_3` | `HandState.tactile_baby[2]` | 139 |
| 71 | `{side}_baby_sensor/tactile_4` | `HandState.tactile_baby[3]` | 140 |
| 72 | `{side}_baby_sensor/tactile_5` | `HandState.tactile_baby[4]` | 141 |
| 73 | `{side}_baby_sensor/tactile_6` | `HandState.tactile_baby[5]` | 142 |
| 74 | `{side}_baby_sensor/tactile_7` | `HandState.tactile_baby[6]` | 143 |
| 75 | `{side}_baby_sensor/tactile_8` | `HandState.tactile_baby[7]` | 144 |
| 76 | `{side}_baby_sensor/tactile_9` | `HandState.tactile_baby[8]` | 145 |
| 77 | `{side}_baby_sensor/tactile_10` | `HandState.tactile_baby[9]` | 146 |
| 78 | `{side}_baby_sensor/tactile_11` | `HandState.tactile_baby[10]` | 147 |
| 79 | `{side}_baby_sensor/tactile_12` | `HandState.tactile_baby[11]` | 148 |
| 80 | `{side}_baby_sensor/tactile_13` | `HandState.tactile_baby[12]` | 149 |
| 81 | `{side}_baby_sensor/tactile_14` | `HandState.tactile_baby[13]` | 150 |
| 82 | `{side}_baby_sensor/tactile_15` | `HandState.tactile_baby[14]` | 151 |
| 83 | `{side}_baby_sensor/tactile_16` | `HandState.tactile_baby[15]` | 152 |
| 84 | `{side}_baby_sensor/tactile_17` | `HandState.tactile_baby[16]` | 153 |

### 4.2 Palm tactile 58 개 — sensor component `{side}_palm_sensor`

| # | State interface | HandState 매핑 | claim # |
|---|---|---|---|
| 0 | `{side}_palm_sensor/palm1_upper_1` | `HandState.tactile_palm1_upper[0]` | 154 |
| 1 | `{side}_palm_sensor/palm1_upper_2` | `HandState.tactile_palm1_upper[1]` | 155 |
| 2 | `{side}_palm_sensor/palm1_upper_3` | `HandState.tactile_palm1_upper[2]` | 156 |
| 3 | `{side}_palm_sensor/palm1_upper_4` | `HandState.tactile_palm1_upper[3]` | 157 |
| 4 | `{side}_palm_sensor/palm1_upper_5` | `HandState.tactile_palm1_upper[4]` | 158 |
| 5 | `{side}_palm_sensor/palm1_upper_6` | `HandState.tactile_palm1_upper[5]` | 159 |
| 6 | `{side}_palm_sensor/palm1_upper_7` | `HandState.tactile_palm1_upper[6]` | 160 |
| 7 | `{side}_palm_sensor/palm1_upper_8` | `HandState.tactile_palm1_upper[7]` | 161 |
| 8 | `{side}_palm_sensor/palm1_upper_9` | `HandState.tactile_palm1_upper[8]` | 162 |
| 9 | `{side}_palm_sensor/palm1_upper_10` | `HandState.tactile_palm1_upper[9]` | 163 |
| 10 | `{side}_palm_sensor/palm1_upper_11` | `HandState.tactile_palm1_upper[10]` | 164 |
| 11 | `{side}_palm_sensor/palm1_upper_12` | `HandState.tactile_palm1_upper[11]` | 165 |
| 12 | `{side}_palm_sensor/palm1_upper_13` | `HandState.tactile_palm1_upper[12]` | 166 |
| 13 | `{side}_palm_sensor/palm1_upper_14` | `HandState.tactile_palm1_upper[13]` | 167 |
| 14 | `{side}_palm_sensor/palm1_upper_15` | `HandState.tactile_palm1_upper[14]` | 168 |
| 15 | `{side}_palm_sensor/palm1_upper_16` | `HandState.tactile_palm1_upper[15]` | 169 |
| 16 | `{side}_palm_sensor/palm1_upper_17` | `HandState.tactile_palm1_upper[16]` | 170 |
| 17 | `{side}_palm_sensor/palm1_upper_18` | `HandState.tactile_palm1_upper[17]` | 171 |
| 18 | `{side}_palm_sensor/palm1_upper_19` | `HandState.tactile_palm1_upper[18]` | 172 |
| 19 | `{side}_palm_sensor/palm1_upper_20` | `HandState.tactile_palm1_upper[19]` | 173 |
| 20 | `{side}_palm_sensor/palm1_lower_1` | `HandState.tactile_palm1_lower[0]` | 174 |
| 21 | `{side}_palm_sensor/palm1_lower_2` | `HandState.tactile_palm1_lower[1]` | 175 |
| 22 | `{side}_palm_sensor/palm1_lower_3` | `HandState.tactile_palm1_lower[2]` | 176 |
| 23 | `{side}_palm_sensor/palm1_lower_4` | `HandState.tactile_palm1_lower[3]` | 177 |
| 24 | `{side}_palm_sensor/palm1_lower_5` | `HandState.tactile_palm1_lower[4]` | 178 |
| 25 | `{side}_palm_sensor/palm1_lower_6` | `HandState.tactile_palm1_lower[5]` | 179 |
| 26 | `{side}_palm_sensor/palm1_lower_7` | `HandState.tactile_palm1_lower[6]` | 180 |
| 27 | `{side}_palm_sensor/palm1_lower_8` | `HandState.tactile_palm1_lower[7]` | 181 |
| 28 | `{side}_palm_sensor/palm1_lower_9` | `HandState.tactile_palm1_lower[8]` | 182 |
| 29 | `{side}_palm_sensor/palm1_lower_10` | `HandState.tactile_palm1_lower[9]` | 183 |
| 30 | `{side}_palm_sensor/palm1_lower_11` | `HandState.tactile_palm1_lower[10]` | 184 |
| 31 | `{side}_palm_sensor/palm1_lower_12` | `HandState.tactile_palm1_lower[11]` | 185 |
| 32 | `{side}_palm_sensor/palm1_lower_13` | `HandState.tactile_palm1_lower[12]` | 186 |
| 33 | `{side}_palm_sensor/palm1_lower_14` | `HandState.tactile_palm1_lower[13]` | 187 |
| 34 | `{side}_palm_sensor/palm1_lower_15` | `HandState.tactile_palm1_lower[14]` | 188 |
| 35 | `{side}_palm_sensor/palm1_lower_16` | `HandState.tactile_palm1_lower[15]` | 189 |
| 36 | `{side}_palm_sensor/palm1_lower_17` | `HandState.tactile_palm1_lower[16]` | 190 |
| 37 | `{side}_palm_sensor/palm1_lower_18` | `HandState.tactile_palm1_lower[17]` | 191 |
| 38 | `{side}_palm_sensor/palm1_lower_19` | `HandState.tactile_palm1_lower[18]` | 192 |
| 39 | `{side}_palm_sensor/palm1_lower_20` | `HandState.tactile_palm1_lower[19]` | 193 |
| 40 | `{side}_palm_sensor/palm2_1` | `HandState.tactile_palm2[0]` | 194 |
| 41 | `{side}_palm_sensor/palm2_2` | `HandState.tactile_palm2[1]` | 195 |
| 42 | `{side}_palm_sensor/palm2_3` | `HandState.tactile_palm2[2]` | 196 |
| 43 | `{side}_palm_sensor/palm2_4` | `HandState.tactile_palm2[3]` | 197 |
| 44 | `{side}_palm_sensor/palm2_5` | `HandState.tactile_palm2[4]` | 198 |
| 45 | `{side}_palm_sensor/palm2_6` | `HandState.tactile_palm2[5]` | 199 |
| 46 | `{side}_palm_sensor/palm2_7` | `HandState.tactile_palm2[6]` | 200 |
| 47 | `{side}_palm_sensor/palm2_8` | `HandState.tactile_palm2[7]` | 201 |
| 48 | `{side}_palm_sensor/palm2_9` | `HandState.tactile_palm2[8]` | 202 |
| 49 | `{side}_palm_sensor/palm2_10` | `HandState.tactile_palm2[9]` | 203 |
| 50 | `{side}_palm_sensor/palm2_11` | `HandState.tactile_palm2[10]` | 204 |
| 51 | `{side}_palm_sensor/palm2_12` | `HandState.tactile_palm2[11]` | 205 |
| 52 | `{side}_palm_sensor/palm2_13` | `HandState.tactile_palm2[12]` | 206 |
| 53 | `{side}_palm_sensor/palm2_14` | `HandState.tactile_palm2[13]` | 207 |
| 54 | `{side}_palm_sensor/palm2_15` | `HandState.tactile_palm2[14]` | 208 |
| 55 | `{side}_palm_sensor/palm2_16` | `HandState.tactile_palm2[15]` | 209 |
| 56 | `{side}_palm_sensor/palm2_17` | `HandState.tactile_palm2[16]` | 210 |
| 57 | `{side}_palm_sensor/palm2_18` | `HandState.tactile_palm2[17]` | 211 |

## 5. 하드웨어 state interface — diagnostics 39 개 (real 전용)

gpio component `{side}_diagnostics` 하나에 모여 있으며 `DiagnosticsBroadcaster` 가 전부
claim 합니다. `claim #` 는 `DiagnosticsBroadcaster` 의 `state_interfaces_` 인덱스입니다.
`HandStateBroadcaster` 는 이 39 개를 claim 하지 않습니다.

### 5.1 Hand 전역 7 개

| # | State interface | 단위 · 값 | HandDiagnostics 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_diagnostics/lifecycle` | HandLifecycle ordinal (double) | `HandDiagnostics.lifecycle` (이름 문자열) | 0 |
| 1 | `{side}_diagnostics/nan_command_count` | 누적 개수 | `HandDiagnostics.nan_command_count` | 1 |
| 2 | `{side}_diagnostics/control_cycles` | 누적 개수 | `HandDiagnostics.control_cycles` | 2 |
| 3 | `{side}_diagnostics/deadline_misses` | 누적 개수 | `HandDiagnostics.deadline_misses` | 3 |
| 4 | `{side}_diagnostics/last_period_ms` | ms | `HandDiagnostics.last_period_ms` | 4 |
| 5 | `{side}_diagnostics/last_compute_ms` | ms | `HandDiagnostics.last_compute_ms` | 5 |
| 6 | `{side}_diagnostics/homing_state` | HomingState ordinal (1 = Succeeded) | `HandDiagnostics.homing_state` | 6 |

### 5.2 Actuator enabled 16 개

| # | State interface | 값 | HandDiagnostics 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_diagnostics/enabled_thumb_actuator0` | 1 = enabled, 0 = disabled | `actuator_enabled[0]` | 7 |
| 1 | `{side}_diagnostics/enabled_thumb_actuator1` | 1 = enabled, 0 = disabled | `actuator_enabled[1]` | 8 |
| 2 | `{side}_diagnostics/enabled_thumb_actuator2` | 1 = enabled, 0 = disabled | `actuator_enabled[2]` | 9 |
| 3 | `{side}_diagnostics/enabled_thumb_actuator3` | 1 = enabled, 0 = disabled | `actuator_enabled[3]` | 10 |
| 4 | `{side}_diagnostics/enabled_index_actuator1` | 1 = enabled, 0 = disabled | `actuator_enabled[4]` | 11 |
| 5 | `{side}_diagnostics/enabled_index_actuator2` | 1 = enabled, 0 = disabled | `actuator_enabled[5]` | 12 |
| 6 | `{side}_diagnostics/enabled_index_actuator3` | 1 = enabled, 0 = disabled | `actuator_enabled[6]` | 13 |
| 7 | `{side}_diagnostics/enabled_middle_actuator1` | 1 = enabled, 0 = disabled | `actuator_enabled[7]` | 14 |
| 8 | `{side}_diagnostics/enabled_middle_actuator2` | 1 = enabled, 0 = disabled | `actuator_enabled[8]` | 15 |
| 9 | `{side}_diagnostics/enabled_middle_actuator3` | 1 = enabled, 0 = disabled | `actuator_enabled[9]` | 16 |
| 10 | `{side}_diagnostics/enabled_ring_actuator1` | 1 = enabled, 0 = disabled | `actuator_enabled[10]` | 17 |
| 11 | `{side}_diagnostics/enabled_ring_actuator2` | 1 = enabled, 0 = disabled | `actuator_enabled[11]` | 18 |
| 12 | `{side}_diagnostics/enabled_ring_actuator3` | 1 = enabled, 0 = disabled | `actuator_enabled[12]` | 19 |
| 13 | `{side}_diagnostics/enabled_baby_actuator1` | 1 = enabled, 0 = disabled | `actuator_enabled[13]` | 20 |
| 14 | `{side}_diagnostics/enabled_baby_actuator2` | 1 = enabled, 0 = disabled | `actuator_enabled[14]` | 21 |
| 15 | `{side}_diagnostics/enabled_baby_actuator3` | 1 = enabled, 0 = disabled | `actuator_enabled[15]` | 22 |

### 5.3 Actuator fault 16 개

| # | State interface | 값 | HandDiagnostics 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_diagnostics/fault_thumb_actuator0` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[0]` (이름 문자열) | 23 |
| 1 | `{side}_diagnostics/fault_thumb_actuator1` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[1]` (이름 문자열) | 24 |
| 2 | `{side}_diagnostics/fault_thumb_actuator2` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[2]` (이름 문자열) | 25 |
| 3 | `{side}_diagnostics/fault_thumb_actuator3` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[3]` (이름 문자열) | 26 |
| 4 | `{side}_diagnostics/fault_index_actuator1` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[4]` (이름 문자열) | 27 |
| 5 | `{side}_diagnostics/fault_index_actuator2` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[5]` (이름 문자열) | 28 |
| 6 | `{side}_diagnostics/fault_index_actuator3` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[6]` (이름 문자열) | 29 |
| 7 | `{side}_diagnostics/fault_middle_actuator1` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[7]` (이름 문자열) | 30 |
| 8 | `{side}_diagnostics/fault_middle_actuator2` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[8]` (이름 문자열) | 31 |
| 9 | `{side}_diagnostics/fault_middle_actuator3` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[9]` (이름 문자열) | 32 |
| 10 | `{side}_diagnostics/fault_ring_actuator1` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[10]` (이름 문자열) | 33 |
| 11 | `{side}_diagnostics/fault_ring_actuator2` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[11]` (이름 문자열) | 34 |
| 12 | `{side}_diagnostics/fault_ring_actuator3` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[12]` (이름 문자열) | 35 |
| 13 | `{side}_diagnostics/fault_baby_actuator1` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[13]` (이름 문자열) | 36 |
| 14 | `{side}_diagnostics/fault_baby_actuator2` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[14]` (이름 문자열) | 37 |
| 15 | `{side}_diagnostics/fault_baby_actuator3` | ActuatorFault ordinal (0 = None) | `actuator_fault_name[15]` (이름 문자열) | 38 |

## 6. 하드웨어 state interface — command echo 132 개 (real 전용)

gpio 로 선언되지 않고 real plugin 이 동적으로 export 하는 `{side}_commanded` component 입니다
(xacro 에 없으므로 `list_hardware_interfaces` 로만 보입니다). SDK `CommandedState` variant 를
flat double 경계로 펼친 값이고, `HandStateBroadcaster` 가 `HandState.command_state` 로 다시
조립합니다. 현재 mode 에서 유효하지 않은 typed field 는 NaN 입니다.

### 6.1 Scalar 4 개

| # | State interface | 값 | CommandState 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_commanded/controller_input_mode` | 0 Idle / 1 JointPosition / 2 JointImpedance / 3 ActuatorPosition / 4 ActuatorEffort | `controller_input_mode` | 212 |
| 1 | `{side}_commanded/controller_output_type` | 0 NONE / 1 ACTUATOR_POSITION / 2 ACTUATOR_EFFORT | `controller_output_type` | 213 |
| 2 | `{side}_commanded/selected_source` | 0 NONE / 1 CONTROLLER / 2 QUICK_STOP / 3 HOMING | `selected_source` | 214 |
| 3 | `{side}_commanded/controller_input_speed_rad_s` | rad/s | `joint_position_input.speed_rad_s` | 215 |

### 6.2 Controller input joint target 16 개

| # | State interface | 단위 | CommandState 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_commanded/controller_input_target_position_rad.thumb_joint0` | rad | `joint_position_input.target_position_rad[0]` · `joint_impedance_input.target_position_rad[0]` | 216 |
| 1 | `{side}_commanded/controller_input_target_position_rad.thumb_joint1` | rad | `joint_position_input.target_position_rad[1]` · `joint_impedance_input.target_position_rad[1]` | 217 |
| 2 | `{side}_commanded/controller_input_target_position_rad.thumb_joint2` | rad | `joint_position_input.target_position_rad[2]` · `joint_impedance_input.target_position_rad[2]` | 218 |
| 3 | `{side}_commanded/controller_input_target_position_rad.thumb_joint3` | rad | `joint_position_input.target_position_rad[3]` · `joint_impedance_input.target_position_rad[3]` | 219 |
| 4 | `{side}_commanded/controller_input_target_position_rad.index_joint1` | rad | `joint_position_input.target_position_rad[4]` · `joint_impedance_input.target_position_rad[4]` | 220 |
| 5 | `{side}_commanded/controller_input_target_position_rad.index_joint2` | rad | `joint_position_input.target_position_rad[5]` · `joint_impedance_input.target_position_rad[5]` | 221 |
| 6 | `{side}_commanded/controller_input_target_position_rad.index_joint3` | rad | `joint_position_input.target_position_rad[6]` · `joint_impedance_input.target_position_rad[6]` | 222 |
| 7 | `{side}_commanded/controller_input_target_position_rad.middle_joint1` | rad | `joint_position_input.target_position_rad[7]` · `joint_impedance_input.target_position_rad[7]` | 223 |
| 8 | `{side}_commanded/controller_input_target_position_rad.middle_joint2` | rad | `joint_position_input.target_position_rad[8]` · `joint_impedance_input.target_position_rad[8]` | 224 |
| 9 | `{side}_commanded/controller_input_target_position_rad.middle_joint3` | rad | `joint_position_input.target_position_rad[9]` · `joint_impedance_input.target_position_rad[9]` | 225 |
| 10 | `{side}_commanded/controller_input_target_position_rad.ring_joint1` | rad | `joint_position_input.target_position_rad[10]` · `joint_impedance_input.target_position_rad[10]` | 226 |
| 11 | `{side}_commanded/controller_input_target_position_rad.ring_joint2` | rad | `joint_position_input.target_position_rad[11]` · `joint_impedance_input.target_position_rad[11]` | 227 |
| 12 | `{side}_commanded/controller_input_target_position_rad.ring_joint3` | rad | `joint_position_input.target_position_rad[12]` · `joint_impedance_input.target_position_rad[12]` | 228 |
| 13 | `{side}_commanded/controller_input_target_position_rad.baby_joint1` | rad | `joint_position_input.target_position_rad[13]` · `joint_impedance_input.target_position_rad[13]` | 229 |
| 14 | `{side}_commanded/controller_input_target_position_rad.baby_joint2` | rad | `joint_position_input.target_position_rad[14]` · `joint_impedance_input.target_position_rad[14]` | 230 |
| 15 | `{side}_commanded/controller_input_target_position_rad.baby_joint3` | rad | `joint_position_input.target_position_rad[15]` · `joint_impedance_input.target_position_rad[15]` | 231 |

### 6.3 Actuator 별 7 필드 × 16 = 112 개

Actuator 하나마다 아래 7 개가 **연속** 등록됩니다(인터리브). claim 인덱스는
`232 + 7 × actuator_index + field_offset` 입니다.

| # | State interface | 단위 | CommandState 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator0` | encoder count | `actuator_position_input.target_position_cnt[0]` | 232 |
| 1 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator0` | rated current % | `actuator_effort_input.target_effort_pct[0]` | 233 |
| 2 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator0` | encoder count | `target_position_cnt[0]` | 234 |
| 3 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator0` | rated current % | `target_effort_pct[0]` | 235 |
| 4 | `{side}_commanded/stiffness.thumb_actuator0` | SDK gain | `joint_impedance_input.stiffness[0]` | 236 |
| 5 | `{side}_commanded/damping.thumb_actuator0` | SDK gain | `joint_impedance_input.damping[0]` | 237 |
| 6 | `{side}_commanded/max_effort_pct.thumb_actuator0` | rated current % | `max_effort_pct[0]` | 238 |
| 7 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator1` | encoder count | `actuator_position_input.target_position_cnt[1]` | 239 |
| 8 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator1` | rated current % | `actuator_effort_input.target_effort_pct[1]` | 240 |
| 9 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator1` | encoder count | `target_position_cnt[1]` | 241 |
| 10 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator1` | rated current % | `target_effort_pct[1]` | 242 |
| 11 | `{side}_commanded/stiffness.thumb_actuator1` | SDK gain | `joint_impedance_input.stiffness[1]` | 243 |
| 12 | `{side}_commanded/damping.thumb_actuator1` | SDK gain | `joint_impedance_input.damping[1]` | 244 |
| 13 | `{side}_commanded/max_effort_pct.thumb_actuator1` | rated current % | `max_effort_pct[1]` | 245 |
| 14 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator2` | encoder count | `actuator_position_input.target_position_cnt[2]` | 246 |
| 15 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator2` | rated current % | `actuator_effort_input.target_effort_pct[2]` | 247 |
| 16 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator2` | encoder count | `target_position_cnt[2]` | 248 |
| 17 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator2` | rated current % | `target_effort_pct[2]` | 249 |
| 18 | `{side}_commanded/stiffness.thumb_actuator2` | SDK gain | `joint_impedance_input.stiffness[2]` | 250 |
| 19 | `{side}_commanded/damping.thumb_actuator2` | SDK gain | `joint_impedance_input.damping[2]` | 251 |
| 20 | `{side}_commanded/max_effort_pct.thumb_actuator2` | rated current % | `max_effort_pct[2]` | 252 |
| 21 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator3` | encoder count | `actuator_position_input.target_position_cnt[3]` | 253 |
| 22 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator3` | rated current % | `actuator_effort_input.target_effort_pct[3]` | 254 |
| 23 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator3` | encoder count | `target_position_cnt[3]` | 255 |
| 24 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator3` | rated current % | `target_effort_pct[3]` | 256 |
| 25 | `{side}_commanded/stiffness.thumb_actuator3` | SDK gain | `joint_impedance_input.stiffness[3]` | 257 |
| 26 | `{side}_commanded/damping.thumb_actuator3` | SDK gain | `joint_impedance_input.damping[3]` | 258 |
| 27 | `{side}_commanded/max_effort_pct.thumb_actuator3` | rated current % | `max_effort_pct[3]` | 259 |
| 28 | `{side}_commanded/controller_input_target_position_cnt.index_actuator1` | encoder count | `actuator_position_input.target_position_cnt[4]` | 260 |
| 29 | `{side}_commanded/controller_input_target_effort_pct.index_actuator1` | rated current % | `actuator_effort_input.target_effort_pct[4]` | 261 |
| 30 | `{side}_commanded/controller_output_target_position_cnt.index_actuator1` | encoder count | `target_position_cnt[4]` | 262 |
| 31 | `{side}_commanded/controller_output_target_effort_pct.index_actuator1` | rated current % | `target_effort_pct[4]` | 263 |
| 32 | `{side}_commanded/stiffness.index_actuator1` | SDK gain | `joint_impedance_input.stiffness[4]` | 264 |
| 33 | `{side}_commanded/damping.index_actuator1` | SDK gain | `joint_impedance_input.damping[4]` | 265 |
| 34 | `{side}_commanded/max_effort_pct.index_actuator1` | rated current % | `max_effort_pct[4]` | 266 |
| 35 | `{side}_commanded/controller_input_target_position_cnt.index_actuator2` | encoder count | `actuator_position_input.target_position_cnt[5]` | 267 |
| 36 | `{side}_commanded/controller_input_target_effort_pct.index_actuator2` | rated current % | `actuator_effort_input.target_effort_pct[5]` | 268 |
| 37 | `{side}_commanded/controller_output_target_position_cnt.index_actuator2` | encoder count | `target_position_cnt[5]` | 269 |
| 38 | `{side}_commanded/controller_output_target_effort_pct.index_actuator2` | rated current % | `target_effort_pct[5]` | 270 |
| 39 | `{side}_commanded/stiffness.index_actuator2` | SDK gain | `joint_impedance_input.stiffness[5]` | 271 |
| 40 | `{side}_commanded/damping.index_actuator2` | SDK gain | `joint_impedance_input.damping[5]` | 272 |
| 41 | `{side}_commanded/max_effort_pct.index_actuator2` | rated current % | `max_effort_pct[5]` | 273 |
| 42 | `{side}_commanded/controller_input_target_position_cnt.index_actuator3` | encoder count | `actuator_position_input.target_position_cnt[6]` | 274 |
| 43 | `{side}_commanded/controller_input_target_effort_pct.index_actuator3` | rated current % | `actuator_effort_input.target_effort_pct[6]` | 275 |
| 44 | `{side}_commanded/controller_output_target_position_cnt.index_actuator3` | encoder count | `target_position_cnt[6]` | 276 |
| 45 | `{side}_commanded/controller_output_target_effort_pct.index_actuator3` | rated current % | `target_effort_pct[6]` | 277 |
| 46 | `{side}_commanded/stiffness.index_actuator3` | SDK gain | `joint_impedance_input.stiffness[6]` | 278 |
| 47 | `{side}_commanded/damping.index_actuator3` | SDK gain | `joint_impedance_input.damping[6]` | 279 |
| 48 | `{side}_commanded/max_effort_pct.index_actuator3` | rated current % | `max_effort_pct[6]` | 280 |
| 49 | `{side}_commanded/controller_input_target_position_cnt.middle_actuator1` | encoder count | `actuator_position_input.target_position_cnt[7]` | 281 |
| 50 | `{side}_commanded/controller_input_target_effort_pct.middle_actuator1` | rated current % | `actuator_effort_input.target_effort_pct[7]` | 282 |
| 51 | `{side}_commanded/controller_output_target_position_cnt.middle_actuator1` | encoder count | `target_position_cnt[7]` | 283 |
| 52 | `{side}_commanded/controller_output_target_effort_pct.middle_actuator1` | rated current % | `target_effort_pct[7]` | 284 |
| 53 | `{side}_commanded/stiffness.middle_actuator1` | SDK gain | `joint_impedance_input.stiffness[7]` | 285 |
| 54 | `{side}_commanded/damping.middle_actuator1` | SDK gain | `joint_impedance_input.damping[7]` | 286 |
| 55 | `{side}_commanded/max_effort_pct.middle_actuator1` | rated current % | `max_effort_pct[7]` | 287 |
| 56 | `{side}_commanded/controller_input_target_position_cnt.middle_actuator2` | encoder count | `actuator_position_input.target_position_cnt[8]` | 288 |
| 57 | `{side}_commanded/controller_input_target_effort_pct.middle_actuator2` | rated current % | `actuator_effort_input.target_effort_pct[8]` | 289 |
| 58 | `{side}_commanded/controller_output_target_position_cnt.middle_actuator2` | encoder count | `target_position_cnt[8]` | 290 |
| 59 | `{side}_commanded/controller_output_target_effort_pct.middle_actuator2` | rated current % | `target_effort_pct[8]` | 291 |
| 60 | `{side}_commanded/stiffness.middle_actuator2` | SDK gain | `joint_impedance_input.stiffness[8]` | 292 |
| 61 | `{side}_commanded/damping.middle_actuator2` | SDK gain | `joint_impedance_input.damping[8]` | 293 |
| 62 | `{side}_commanded/max_effort_pct.middle_actuator2` | rated current % | `max_effort_pct[8]` | 294 |
| 63 | `{side}_commanded/controller_input_target_position_cnt.middle_actuator3` | encoder count | `actuator_position_input.target_position_cnt[9]` | 295 |
| 64 | `{side}_commanded/controller_input_target_effort_pct.middle_actuator3` | rated current % | `actuator_effort_input.target_effort_pct[9]` | 296 |
| 65 | `{side}_commanded/controller_output_target_position_cnt.middle_actuator3` | encoder count | `target_position_cnt[9]` | 297 |
| 66 | `{side}_commanded/controller_output_target_effort_pct.middle_actuator3` | rated current % | `target_effort_pct[9]` | 298 |
| 67 | `{side}_commanded/stiffness.middle_actuator3` | SDK gain | `joint_impedance_input.stiffness[9]` | 299 |
| 68 | `{side}_commanded/damping.middle_actuator3` | SDK gain | `joint_impedance_input.damping[9]` | 300 |
| 69 | `{side}_commanded/max_effort_pct.middle_actuator3` | rated current % | `max_effort_pct[9]` | 301 |
| 70 | `{side}_commanded/controller_input_target_position_cnt.ring_actuator1` | encoder count | `actuator_position_input.target_position_cnt[10]` | 302 |
| 71 | `{side}_commanded/controller_input_target_effort_pct.ring_actuator1` | rated current % | `actuator_effort_input.target_effort_pct[10]` | 303 |
| 72 | `{side}_commanded/controller_output_target_position_cnt.ring_actuator1` | encoder count | `target_position_cnt[10]` | 304 |
| 73 | `{side}_commanded/controller_output_target_effort_pct.ring_actuator1` | rated current % | `target_effort_pct[10]` | 305 |
| 74 | `{side}_commanded/stiffness.ring_actuator1` | SDK gain | `joint_impedance_input.stiffness[10]` | 306 |
| 75 | `{side}_commanded/damping.ring_actuator1` | SDK gain | `joint_impedance_input.damping[10]` | 307 |
| 76 | `{side}_commanded/max_effort_pct.ring_actuator1` | rated current % | `max_effort_pct[10]` | 308 |
| 77 | `{side}_commanded/controller_input_target_position_cnt.ring_actuator2` | encoder count | `actuator_position_input.target_position_cnt[11]` | 309 |
| 78 | `{side}_commanded/controller_input_target_effort_pct.ring_actuator2` | rated current % | `actuator_effort_input.target_effort_pct[11]` | 310 |
| 79 | `{side}_commanded/controller_output_target_position_cnt.ring_actuator2` | encoder count | `target_position_cnt[11]` | 311 |
| 80 | `{side}_commanded/controller_output_target_effort_pct.ring_actuator2` | rated current % | `target_effort_pct[11]` | 312 |
| 81 | `{side}_commanded/stiffness.ring_actuator2` | SDK gain | `joint_impedance_input.stiffness[11]` | 313 |
| 82 | `{side}_commanded/damping.ring_actuator2` | SDK gain | `joint_impedance_input.damping[11]` | 314 |
| 83 | `{side}_commanded/max_effort_pct.ring_actuator2` | rated current % | `max_effort_pct[11]` | 315 |
| 84 | `{side}_commanded/controller_input_target_position_cnt.ring_actuator3` | encoder count | `actuator_position_input.target_position_cnt[12]` | 316 |
| 85 | `{side}_commanded/controller_input_target_effort_pct.ring_actuator3` | rated current % | `actuator_effort_input.target_effort_pct[12]` | 317 |
| 86 | `{side}_commanded/controller_output_target_position_cnt.ring_actuator3` | encoder count | `target_position_cnt[12]` | 318 |
| 87 | `{side}_commanded/controller_output_target_effort_pct.ring_actuator3` | rated current % | `target_effort_pct[12]` | 319 |
| 88 | `{side}_commanded/stiffness.ring_actuator3` | SDK gain | `joint_impedance_input.stiffness[12]` | 320 |
| 89 | `{side}_commanded/damping.ring_actuator3` | SDK gain | `joint_impedance_input.damping[12]` | 321 |
| 90 | `{side}_commanded/max_effort_pct.ring_actuator3` | rated current % | `max_effort_pct[12]` | 322 |
| 91 | `{side}_commanded/controller_input_target_position_cnt.baby_actuator1` | encoder count | `actuator_position_input.target_position_cnt[13]` | 323 |
| 92 | `{side}_commanded/controller_input_target_effort_pct.baby_actuator1` | rated current % | `actuator_effort_input.target_effort_pct[13]` | 324 |
| 93 | `{side}_commanded/controller_output_target_position_cnt.baby_actuator1` | encoder count | `target_position_cnt[13]` | 325 |
| 94 | `{side}_commanded/controller_output_target_effort_pct.baby_actuator1` | rated current % | `target_effort_pct[13]` | 326 |
| 95 | `{side}_commanded/stiffness.baby_actuator1` | SDK gain | `joint_impedance_input.stiffness[13]` | 327 |
| 96 | `{side}_commanded/damping.baby_actuator1` | SDK gain | `joint_impedance_input.damping[13]` | 328 |
| 97 | `{side}_commanded/max_effort_pct.baby_actuator1` | rated current % | `max_effort_pct[13]` | 329 |
| 98 | `{side}_commanded/controller_input_target_position_cnt.baby_actuator2` | encoder count | `actuator_position_input.target_position_cnt[14]` | 330 |
| 99 | `{side}_commanded/controller_input_target_effort_pct.baby_actuator2` | rated current % | `actuator_effort_input.target_effort_pct[14]` | 331 |
| 100 | `{side}_commanded/controller_output_target_position_cnt.baby_actuator2` | encoder count | `target_position_cnt[14]` | 332 |
| 101 | `{side}_commanded/controller_output_target_effort_pct.baby_actuator2` | rated current % | `target_effort_pct[14]` | 333 |
| 102 | `{side}_commanded/stiffness.baby_actuator2` | SDK gain | `joint_impedance_input.stiffness[14]` | 334 |
| 103 | `{side}_commanded/damping.baby_actuator2` | SDK gain | `joint_impedance_input.damping[14]` | 335 |
| 104 | `{side}_commanded/max_effort_pct.baby_actuator2` | rated current % | `max_effort_pct[14]` | 336 |
| 105 | `{side}_commanded/controller_input_target_position_cnt.baby_actuator3` | encoder count | `actuator_position_input.target_position_cnt[15]` | 337 |
| 106 | `{side}_commanded/controller_input_target_effort_pct.baby_actuator3` | rated current % | `actuator_effort_input.target_effort_pct[15]` | 338 |
| 107 | `{side}_commanded/controller_output_target_position_cnt.baby_actuator3` | encoder count | `target_position_cnt[15]` | 339 |
| 108 | `{side}_commanded/controller_output_target_effort_pct.baby_actuator3` | rated current % | `target_effort_pct[15]` | 340 |
| 109 | `{side}_commanded/stiffness.baby_actuator3` | SDK gain | `joint_impedance_input.stiffness[15]` | 341 |
| 110 | `{side}_commanded/damping.baby_actuator3` | SDK gain | `joint_impedance_input.damping[15]` | 342 |
| 111 | `{side}_commanded/max_effort_pct.baby_actuator3` | rated current % | `max_effort_pct[15]` | 343 |

## 7. 하드웨어 state interface — timestamp 2 개 (real 전용)

동적 export component `{side}_timestamp` 입니다. SDK RX 관측 시각(wall-clock)을 분해한
값이고 `HandState.header.stamp` 로 흐릅니다. `sec` 이 0 이면 broadcaster 가 자신의
update time 으로 대체합니다.

| # | State interface | 단위 | HandState 매핑 | claim # |
|---|---|---|---|---|
| 0 | `{side}_timestamp/sec` | s | `header.stamp.sec` | 344 |
| 1 | `{side}_timestamp/nanosec` | ns | `header.stamp.nanosec` | 345 |

## 8. Controller 별 claim · reference 전체

`aidin_hand2_controllers` 의 6 종과 `aidin_hand2_examples` 의 상위 controller 를 모두
다룹니다. Controller 인스턴스 이름은 `{side}_` prefix 규약을 따릅니다
(`aidin_hand2_bringup/config/controllers.yaml`).

| Controller | Plugin type | chainable | command claim | state claim | reference export |
|---|---|:---:|---:|---:|---:|
| `{side}_joint_position_controller` | `aidin_hand2_controllers/JointPositionController` | O | 18 | 16 | 17 |
| `{side}_joint_impedance_controller` | `aidin_hand2_controllers/JointImpedanceController` | O | 49 | 16 | 48 |
| `{side}_actuator_position_controller` | `aidin_hand2_controllers/ActuatorPositionController` | O | 17 | 16 | 16 |
| `{side}_actuator_effort_controller` | `aidin_hand2_controllers/ActuatorEffortController` | O | 17 | 0 (NONE) | 16 |
| `{side}_hand_state_broadcaster` | `aidin_hand2_controllers/HandStateBroadcaster` | X | 0 (NONE) | 346 | — |
| `{side}_diagnostics_broadcaster` | `aidin_hand2_controllers/DiagnosticsBroadcaster` | X | 0 (NONE) | 39 | — |
| `{side}_glove_teleop_controller` | `aidin_hand2_examples/GloveTeleopController` | O | 16 (하위 reference) | 0 (NONE) | 16 |
| `{side}_joint_position_upper` | `aidin_hand2_examples/JointPositionUpperController` | O | 17 (하위 reference) | 0 (NONE) | 17 |
| `{side}_joint_impedance_upper` | `aidin_hand2_examples/JointImpedanceUpperController` | O | 48 (하위 reference) | 0 (NONE) | 48 |
| `{side}_actuator_position_upper` | `aidin_hand2_examples/ActuatorPositionUpperController` | O | 16 (하위 reference) | 0 (NONE) | 16 |
| `{side}_actuator_effort_upper` | `aidin_hand2_examples/ActuatorEffortUpperController` | O | 16 (하위 reference) | 0 (NONE) | 16 |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | X | 0 (NONE) | 표준 `position` 전체 | — |

### 8.1 JointPositionController

`command_lock` + JointPosition port 전체를 claim 하고, 같은 개수의 reference 를 export
합니다. 아래 표의 한 행이 reference → hardware command 의 1:1 경로입니다.

| ref # | Reference interface (chainable) | 기록되는 hardware command interface | 단위 | 제약 |
|---|---|---|---|---|
| — | — | `{side}_hand_control/command_lock` | — | claim-only. reference 없음 |
| 0 | `{side}_joint_position_controller/{side}_thumb_joint0/position` | `{side}_joint_position_command/target_position_rad.thumb_joint0` | rad | 유한값 필수 |
| 1 | `{side}_joint_position_controller/{side}_thumb_joint1/position` | `{side}_joint_position_command/target_position_rad.thumb_joint1` | rad | 유한값 필수 |
| 2 | `{side}_joint_position_controller/{side}_thumb_joint2/position` | `{side}_joint_position_command/target_position_rad.thumb_joint2` | rad | 유한값 필수 |
| 3 | `{side}_joint_position_controller/{side}_thumb_joint3/position` | `{side}_joint_position_command/target_position_rad.thumb_joint3` | rad | 유한값 필수 |
| 4 | `{side}_joint_position_controller/{side}_index_joint1/position` | `{side}_joint_position_command/target_position_rad.index_joint1` | rad | 유한값 필수 |
| 5 | `{side}_joint_position_controller/{side}_index_joint2/position` | `{side}_joint_position_command/target_position_rad.index_joint2` | rad | 유한값 필수 |
| 6 | `{side}_joint_position_controller/{side}_index_joint3/position` | `{side}_joint_position_command/target_position_rad.index_joint3` | rad | 유한값 필수 |
| 7 | `{side}_joint_position_controller/{side}_middle_joint1/position` | `{side}_joint_position_command/target_position_rad.middle_joint1` | rad | 유한값 필수 |
| 8 | `{side}_joint_position_controller/{side}_middle_joint2/position` | `{side}_joint_position_command/target_position_rad.middle_joint2` | rad | 유한값 필수 |
| 9 | `{side}_joint_position_controller/{side}_middle_joint3/position` | `{side}_joint_position_command/target_position_rad.middle_joint3` | rad | 유한값 필수 |
| 10 | `{side}_joint_position_controller/{side}_ring_joint1/position` | `{side}_joint_position_command/target_position_rad.ring_joint1` | rad | 유한값 필수 |
| 11 | `{side}_joint_position_controller/{side}_ring_joint2/position` | `{side}_joint_position_command/target_position_rad.ring_joint2` | rad | 유한값 필수 |
| 12 | `{side}_joint_position_controller/{side}_ring_joint3/position` | `{side}_joint_position_command/target_position_rad.ring_joint3` | rad | 유한값 필수 |
| 13 | `{side}_joint_position_controller/{side}_baby_joint1/position` | `{side}_joint_position_command/target_position_rad.baby_joint1` | rad | 유한값 필수 |
| 14 | `{side}_joint_position_controller/{side}_baby_joint2/position` | `{side}_joint_position_command/target_position_rad.baby_joint2` | rad | 유한값 필수 |
| 15 | `{side}_joint_position_controller/{side}_baby_joint3/position` | `{side}_joint_position_command/target_position_rad.baby_joint3` | rad | 유한값 필수 |
| 16 | `{side}_joint_position_controller/{side}_joint_position/speed_rad_s` | `{side}_joint_position_command/speed_rad_s` | rad/s | 0 이상 유한값 |

활성화 시 seed: reference `0..15` 는 아래 state 의 현재값, `16` 은 파라미터 `speed_rad_s`.

State claim 16 개:

| claim # | State interface | 용도 |
|---|---|---|
| 0 | `{side}_thumb_joint0/position` | activation seed |
| 1 | `{side}_thumb_joint1/position` | activation seed |
| 2 | `{side}_thumb_joint2/position` | activation seed |
| 3 | `{side}_thumb_joint3/position` | activation seed |
| 4 | `{side}_index_joint1/position` | activation seed |
| 5 | `{side}_index_joint2/position` | activation seed |
| 6 | `{side}_index_joint3/position` | activation seed |
| 7 | `{side}_middle_joint1/position` | activation seed |
| 8 | `{side}_middle_joint2/position` | activation seed |
| 9 | `{side}_middle_joint3/position` | activation seed |
| 10 | `{side}_ring_joint1/position` | activation seed |
| 11 | `{side}_ring_joint2/position` | activation seed |
| 12 | `{side}_ring_joint3/position` | activation seed |
| 13 | `{side}_baby_joint1/position` | activation seed |
| 14 | `{side}_baby_joint2/position` | activation seed |
| 15 | `{side}_baby_joint3/position` | activation seed |

Standalone topic: `/{side}_joint_position_controller/command`
(`aidin_hand2_msgs/JointPositionCommand` — `target_position_rad[16]`, `speed_rad_s`).
Chained mode 에서는 이 topic 이 reference source 가 아닙니다.

### 8.2 JointImpedanceController

| ref # | Reference interface (chainable) | 기록되는 hardware command interface | 단위 | 제약 |
|---|---|---|---|---|
| — | — | `{side}_hand_control/command_lock` | — | claim-only. reference 없음 |
| 0 | `{side}_joint_impedance_controller/{side}_thumb_joint0/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint0` | rad | 유한값 필수 |
| 1 | `{side}_joint_impedance_controller/{side}_thumb_joint1/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint1` | rad | 유한값 필수 |
| 2 | `{side}_joint_impedance_controller/{side}_thumb_joint2/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint2` | rad | 유한값 필수 |
| 3 | `{side}_joint_impedance_controller/{side}_thumb_joint3/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint3` | rad | 유한값 필수 |
| 4 | `{side}_joint_impedance_controller/{side}_index_joint1/position` | `{side}_joint_impedance_command/target_position_rad.index_joint1` | rad | 유한값 필수 |
| 5 | `{side}_joint_impedance_controller/{side}_index_joint2/position` | `{side}_joint_impedance_command/target_position_rad.index_joint2` | rad | 유한값 필수 |
| 6 | `{side}_joint_impedance_controller/{side}_index_joint3/position` | `{side}_joint_impedance_command/target_position_rad.index_joint3` | rad | 유한값 필수 |
| 7 | `{side}_joint_impedance_controller/{side}_middle_joint1/position` | `{side}_joint_impedance_command/target_position_rad.middle_joint1` | rad | 유한값 필수 |
| 8 | `{side}_joint_impedance_controller/{side}_middle_joint2/position` | `{side}_joint_impedance_command/target_position_rad.middle_joint2` | rad | 유한값 필수 |
| 9 | `{side}_joint_impedance_controller/{side}_middle_joint3/position` | `{side}_joint_impedance_command/target_position_rad.middle_joint3` | rad | 유한값 필수 |
| 10 | `{side}_joint_impedance_controller/{side}_ring_joint1/position` | `{side}_joint_impedance_command/target_position_rad.ring_joint1` | rad | 유한값 필수 |
| 11 | `{side}_joint_impedance_controller/{side}_ring_joint2/position` | `{side}_joint_impedance_command/target_position_rad.ring_joint2` | rad | 유한값 필수 |
| 12 | `{side}_joint_impedance_controller/{side}_ring_joint3/position` | `{side}_joint_impedance_command/target_position_rad.ring_joint3` | rad | 유한값 필수 |
| 13 | `{side}_joint_impedance_controller/{side}_baby_joint1/position` | `{side}_joint_impedance_command/target_position_rad.baby_joint1` | rad | 유한값 필수 |
| 14 | `{side}_joint_impedance_controller/{side}_baby_joint2/position` | `{side}_joint_impedance_command/target_position_rad.baby_joint2` | rad | 유한값 필수 |
| 15 | `{side}_joint_impedance_controller/{side}_baby_joint3/position` | `{side}_joint_impedance_command/target_position_rad.baby_joint3` | rad | 유한값 필수 |
| 16 | `{side}_joint_impedance_controller/{side}_thumb_actuator0/stiffness` | `{side}_joint_impedance_command/stiffness.thumb_actuator0` | SDK gain | 0 이상 유한값 |
| 17 | `{side}_joint_impedance_controller/{side}_thumb_actuator1/stiffness` | `{side}_joint_impedance_command/stiffness.thumb_actuator1` | SDK gain | 0 이상 유한값 |
| 18 | `{side}_joint_impedance_controller/{side}_thumb_actuator2/stiffness` | `{side}_joint_impedance_command/stiffness.thumb_actuator2` | SDK gain | 0 이상 유한값 |
| 19 | `{side}_joint_impedance_controller/{side}_thumb_actuator3/stiffness` | `{side}_joint_impedance_command/stiffness.thumb_actuator3` | SDK gain | 0 이상 유한값 |
| 20 | `{side}_joint_impedance_controller/{side}_index_actuator1/stiffness` | `{side}_joint_impedance_command/stiffness.index_actuator1` | SDK gain | 0 이상 유한값 |
| 21 | `{side}_joint_impedance_controller/{side}_index_actuator2/stiffness` | `{side}_joint_impedance_command/stiffness.index_actuator2` | SDK gain | 0 이상 유한값 |
| 22 | `{side}_joint_impedance_controller/{side}_index_actuator3/stiffness` | `{side}_joint_impedance_command/stiffness.index_actuator3` | SDK gain | 0 이상 유한값 |
| 23 | `{side}_joint_impedance_controller/{side}_middle_actuator1/stiffness` | `{side}_joint_impedance_command/stiffness.middle_actuator1` | SDK gain | 0 이상 유한값 |
| 24 | `{side}_joint_impedance_controller/{side}_middle_actuator2/stiffness` | `{side}_joint_impedance_command/stiffness.middle_actuator2` | SDK gain | 0 이상 유한값 |
| 25 | `{side}_joint_impedance_controller/{side}_middle_actuator3/stiffness` | `{side}_joint_impedance_command/stiffness.middle_actuator3` | SDK gain | 0 이상 유한값 |
| 26 | `{side}_joint_impedance_controller/{side}_ring_actuator1/stiffness` | `{side}_joint_impedance_command/stiffness.ring_actuator1` | SDK gain | 0 이상 유한값 |
| 27 | `{side}_joint_impedance_controller/{side}_ring_actuator2/stiffness` | `{side}_joint_impedance_command/stiffness.ring_actuator2` | SDK gain | 0 이상 유한값 |
| 28 | `{side}_joint_impedance_controller/{side}_ring_actuator3/stiffness` | `{side}_joint_impedance_command/stiffness.ring_actuator3` | SDK gain | 0 이상 유한값 |
| 29 | `{side}_joint_impedance_controller/{side}_baby_actuator1/stiffness` | `{side}_joint_impedance_command/stiffness.baby_actuator1` | SDK gain | 0 이상 유한값 |
| 30 | `{side}_joint_impedance_controller/{side}_baby_actuator2/stiffness` | `{side}_joint_impedance_command/stiffness.baby_actuator2` | SDK gain | 0 이상 유한값 |
| 31 | `{side}_joint_impedance_controller/{side}_baby_actuator3/stiffness` | `{side}_joint_impedance_command/stiffness.baby_actuator3` | SDK gain | 0 이상 유한값 |
| 32 | `{side}_joint_impedance_controller/{side}_thumb_actuator0/damping` | `{side}_joint_impedance_command/damping.thumb_actuator0` | SDK gain | 0 이상 유한값 |
| 33 | `{side}_joint_impedance_controller/{side}_thumb_actuator1/damping` | `{side}_joint_impedance_command/damping.thumb_actuator1` | SDK gain | 0 이상 유한값 |
| 34 | `{side}_joint_impedance_controller/{side}_thumb_actuator2/damping` | `{side}_joint_impedance_command/damping.thumb_actuator2` | SDK gain | 0 이상 유한값 |
| 35 | `{side}_joint_impedance_controller/{side}_thumb_actuator3/damping` | `{side}_joint_impedance_command/damping.thumb_actuator3` | SDK gain | 0 이상 유한값 |
| 36 | `{side}_joint_impedance_controller/{side}_index_actuator1/damping` | `{side}_joint_impedance_command/damping.index_actuator1` | SDK gain | 0 이상 유한값 |
| 37 | `{side}_joint_impedance_controller/{side}_index_actuator2/damping` | `{side}_joint_impedance_command/damping.index_actuator2` | SDK gain | 0 이상 유한값 |
| 38 | `{side}_joint_impedance_controller/{side}_index_actuator3/damping` | `{side}_joint_impedance_command/damping.index_actuator3` | SDK gain | 0 이상 유한값 |
| 39 | `{side}_joint_impedance_controller/{side}_middle_actuator1/damping` | `{side}_joint_impedance_command/damping.middle_actuator1` | SDK gain | 0 이상 유한값 |
| 40 | `{side}_joint_impedance_controller/{side}_middle_actuator2/damping` | `{side}_joint_impedance_command/damping.middle_actuator2` | SDK gain | 0 이상 유한값 |
| 41 | `{side}_joint_impedance_controller/{side}_middle_actuator3/damping` | `{side}_joint_impedance_command/damping.middle_actuator3` | SDK gain | 0 이상 유한값 |
| 42 | `{side}_joint_impedance_controller/{side}_ring_actuator1/damping` | `{side}_joint_impedance_command/damping.ring_actuator1` | SDK gain | 0 이상 유한값 |
| 43 | `{side}_joint_impedance_controller/{side}_ring_actuator2/damping` | `{side}_joint_impedance_command/damping.ring_actuator2` | SDK gain | 0 이상 유한값 |
| 44 | `{side}_joint_impedance_controller/{side}_ring_actuator3/damping` | `{side}_joint_impedance_command/damping.ring_actuator3` | SDK gain | 0 이상 유한값 |
| 45 | `{side}_joint_impedance_controller/{side}_baby_actuator1/damping` | `{side}_joint_impedance_command/damping.baby_actuator1` | SDK gain | 0 이상 유한값 |
| 46 | `{side}_joint_impedance_controller/{side}_baby_actuator2/damping` | `{side}_joint_impedance_command/damping.baby_actuator2` | SDK gain | 0 이상 유한값 |
| 47 | `{side}_joint_impedance_controller/{side}_baby_actuator3/damping` | `{side}_joint_impedance_command/damping.baby_actuator3` | SDK gain | 0 이상 유한값 |

자세 reference 이름(`{side}_{active_joint}/position`)은 JointPositionController 와 같은
형식이라 상위 controller 를 그대로 바꿔 붙일 수 있습니다.

활성화 시 seed: reference `0..15` 는 현재 joint position, `16..31` · `32..47` 은 파라미터
`stiffness` · `damping` (생략 시 SDK `kDefaultStiffness` · `kDefaultDamping`).

State claim 16 개:

| claim # | State interface | 용도 |
|---|---|---|
| 0 | `{side}_thumb_joint0/position` | activation seed |
| 1 | `{side}_thumb_joint1/position` | activation seed |
| 2 | `{side}_thumb_joint2/position` | activation seed |
| 3 | `{side}_thumb_joint3/position` | activation seed |
| 4 | `{side}_index_joint1/position` | activation seed |
| 5 | `{side}_index_joint2/position` | activation seed |
| 6 | `{side}_index_joint3/position` | activation seed |
| 7 | `{side}_middle_joint1/position` | activation seed |
| 8 | `{side}_middle_joint2/position` | activation seed |
| 9 | `{side}_middle_joint3/position` | activation seed |
| 10 | `{side}_ring_joint1/position` | activation seed |
| 11 | `{side}_ring_joint2/position` | activation seed |
| 12 | `{side}_ring_joint3/position` | activation seed |
| 13 | `{side}_baby_joint1/position` | activation seed |
| 14 | `{side}_baby_joint2/position` | activation seed |
| 15 | `{side}_baby_joint3/position` | activation seed |

Standalone topic: `/{side}_joint_impedance_controller/command`
(`aidin_hand2_msgs/JointImpedanceCommand` — `target_position_rad[16]`, `stiffness[16]`,
`damping[16]`).

### 8.3 ActuatorPositionController

| ref # | Reference interface (chainable) | 기록되는 hardware command interface | 단위 | 제약 |
|---|---|---|---|---|
| — | — | `{side}_hand_control/command_lock` | — | claim-only. reference 없음 |
| 0 | `{side}_actuator_position_controller/{side}_thumb_actuator0/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator0` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 1 | `{side}_actuator_position_controller/{side}_thumb_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator1` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 2 | `{side}_actuator_position_controller/{side}_thumb_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator2` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 3 | `{side}_actuator_position_controller/{side}_thumb_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator3` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 4 | `{side}_actuator_position_controller/{side}_index_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.index_actuator1` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 5 | `{side}_actuator_position_controller/{side}_index_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.index_actuator2` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 6 | `{side}_actuator_position_controller/{side}_index_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.index_actuator3` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 7 | `{side}_actuator_position_controller/{side}_middle_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.middle_actuator1` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 8 | `{side}_actuator_position_controller/{side}_middle_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.middle_actuator2` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 9 | `{side}_actuator_position_controller/{side}_middle_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.middle_actuator3` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 10 | `{side}_actuator_position_controller/{side}_ring_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.ring_actuator1` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 11 | `{side}_actuator_position_controller/{side}_ring_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.ring_actuator2` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 12 | `{side}_actuator_position_controller/{side}_ring_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.ring_actuator3` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 13 | `{side}_actuator_position_controller/{side}_baby_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.baby_actuator1` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 14 | `{side}_actuator_position_controller/{side}_baby_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.baby_actuator2` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |
| 15 | `{side}_actuator_position_controller/{side}_baby_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.baby_actuator3` | encoder count | 유한값 필수 (int32 범위는 SDK 검증) |

활성화 시 seed: 현재 `position_cnt`.

State claim 16 개:

| claim # | State interface | 용도 |
|---|---|---|
| 0 | `{side}_thumb_actuator0/position_cnt` | activation seed |
| 1 | `{side}_thumb_actuator1/position_cnt` | activation seed |
| 2 | `{side}_thumb_actuator2/position_cnt` | activation seed |
| 3 | `{side}_thumb_actuator3/position_cnt` | activation seed |
| 4 | `{side}_index_actuator1/position_cnt` | activation seed |
| 5 | `{side}_index_actuator2/position_cnt` | activation seed |
| 6 | `{side}_index_actuator3/position_cnt` | activation seed |
| 7 | `{side}_middle_actuator1/position_cnt` | activation seed |
| 8 | `{side}_middle_actuator2/position_cnt` | activation seed |
| 9 | `{side}_middle_actuator3/position_cnt` | activation seed |
| 10 | `{side}_ring_actuator1/position_cnt` | activation seed |
| 11 | `{side}_ring_actuator2/position_cnt` | activation seed |
| 12 | `{side}_ring_actuator3/position_cnt` | activation seed |
| 13 | `{side}_baby_actuator1/position_cnt` | activation seed |
| 14 | `{side}_baby_actuator2/position_cnt` | activation seed |
| 15 | `{side}_baby_actuator3/position_cnt` | activation seed |

Standalone topic: `/{side}_actuator_position_controller/command`
(`aidin_hand2_msgs/ActuatorPositionCommand` — `target_position_cnt[16]`).

### 8.4 ActuatorEffortController

| ref # | Reference interface (chainable) | 기록되는 hardware command interface | 단위 | 제약 |
|---|---|---|---|---|
| — | — | `{side}_hand_control/command_lock` | — | claim-only. reference 없음 |
| 0 | `{side}_actuator_effort_controller/{side}_thumb_actuator0/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator0` | rated current % | 유한값 필수 |
| 1 | `{side}_actuator_effort_controller/{side}_thumb_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator1` | rated current % | 유한값 필수 |
| 2 | `{side}_actuator_effort_controller/{side}_thumb_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator2` | rated current % | 유한값 필수 |
| 3 | `{side}_actuator_effort_controller/{side}_thumb_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator3` | rated current % | 유한값 필수 |
| 4 | `{side}_actuator_effort_controller/{side}_index_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.index_actuator1` | rated current % | 유한값 필수 |
| 5 | `{side}_actuator_effort_controller/{side}_index_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.index_actuator2` | rated current % | 유한값 필수 |
| 6 | `{side}_actuator_effort_controller/{side}_index_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.index_actuator3` | rated current % | 유한값 필수 |
| 7 | `{side}_actuator_effort_controller/{side}_middle_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.middle_actuator1` | rated current % | 유한값 필수 |
| 8 | `{side}_actuator_effort_controller/{side}_middle_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.middle_actuator2` | rated current % | 유한값 필수 |
| 9 | `{side}_actuator_effort_controller/{side}_middle_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.middle_actuator3` | rated current % | 유한값 필수 |
| 10 | `{side}_actuator_effort_controller/{side}_ring_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.ring_actuator1` | rated current % | 유한값 필수 |
| 11 | `{side}_actuator_effort_controller/{side}_ring_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.ring_actuator2` | rated current % | 유한값 필수 |
| 12 | `{side}_actuator_effort_controller/{side}_ring_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.ring_actuator3` | rated current % | 유한값 필수 |
| 13 | `{side}_actuator_effort_controller/{side}_baby_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.baby_actuator1` | rated current % | 유한값 필수 |
| 14 | `{side}_actuator_effort_controller/{side}_baby_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.baby_actuator2` | rated current % | 유한값 필수 |
| 15 | `{side}_actuator_effort_controller/{side}_baby_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.baby_actuator3` | rated current % | 유한값 필수 |

활성화 시 seed: 전부 `0.0` (무동작). State interface 는 claim 하지 않습니다
(`interface_configuration_type::NONE`).

Standalone topic: `/{side}_actuator_effort_controller/command`
(`aidin_hand2_msgs/ActuatorEffortCommand` — `target_effort_pct[16]`).

### 8.5 HandStateBroadcaster

Command interface 는 claim 하지 않고(`NONE`), state 346 개를 아래 순서로 claim 합니다.
diagnostics 39 개는 제외입니다. 각 항목의 전체 이름은 3·4·6·7 절 표의 `claim #` 로 찾습니다.

| claim # | 블록 | 개수 | 참조 |
|---|---|---|---|
| 0–20 | joint position | 21 | [3.1](#31-joint-position-21-개--jointposition-rad) |
| 21–36 | actuator `position_cnt` | 16 | [3.2](#32-actuator-physical-state-48-개) |
| 37–52 | actuator `velocity_rpm` | 16 | [3.2](#32-actuator-physical-state-48-개) |
| 53–68 | actuator `current_ma` | 16 | [3.2](#32-actuator-physical-state-48-개) |
| 69–153 | finger tactile | 85 | [4.1](#41-finger-tactile-85-개--sensor-component-side_finger_sensor) |
| 154–211 | palm tactile | 58 | [4.2](#42-palm-tactile-58-개--sensor-component-side_palm_sensor) |
| 212–215 | command echo scalar | 4 | [6.1](#61-scalar-4-개) |
| 216–231 | command echo joint target | 16 | [6.2](#62-controller-input-joint-target-16-개) |
| 232–343 | command echo actuator 7 필드 인터리브 | 112 | [6.3](#63-actuator-별-7-필드--16--112-개) |
| 344–345 | timestamp | 2 | [7](#7-하드웨어-state-interface--timestamp-2-개-real-전용) |

발행 topic: `/{side}_hand_state_broadcaster/hand_state` (`aidin_hand2_msgs/HandState`).

### 8.6 DiagnosticsBroadcaster

Command interface 는 claim 하지 않고(`NONE`), diagnostics 39 개만 claim 합니다
(claim 순서 = [5.1](#51-hand-전역-7-개) → [5.2](#52-actuator-enabled-16-개) →
[5.3](#53-actuator-fault-16-개)).

발행 topic: `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) 와
`/{side}_diagnostics_broadcaster/hand_diagnostics` (`aidin_hand2_msgs/HandDiagnostics`).

### 8.7 GloveTeleopController (example)

Chain 최상위입니다. 하위 자세 controller 의 reference 16 개를 command 로 claim 하고,
같은 형식의 reference 16 개를 자기 이름으로 export 합니다(Humble 의 chainable 최소 1 개
요구 충족 겸 확장 대비 — 상위가 소비하지 않습니다). `speed_rad_s` reference 는 claim 하지
않아 하위 controller 가 activation 때 seed 한 속도를 유지합니다.

`{target_controller}` 는 파라미터이며 기본값은 `{side}_joint_position_controller` 입니다.

| # | claim 하는 하위 reference | export 하는 reference | 단위 |
|---|---|---|---|
| 0 | `{target_controller}/{side}_thumb_joint0/position` | `{side}_glove_teleop_controller/{side}_thumb_joint0/position` | rad |
| 1 | `{target_controller}/{side}_thumb_joint1/position` | `{side}_glove_teleop_controller/{side}_thumb_joint1/position` | rad |
| 2 | `{target_controller}/{side}_thumb_joint2/position` | `{side}_glove_teleop_controller/{side}_thumb_joint2/position` | rad |
| 3 | `{target_controller}/{side}_thumb_joint3/position` | `{side}_glove_teleop_controller/{side}_thumb_joint3/position` | rad |
| 4 | `{target_controller}/{side}_index_joint1/position` | `{side}_glove_teleop_controller/{side}_index_joint1/position` | rad |
| 5 | `{target_controller}/{side}_index_joint2/position` | `{side}_glove_teleop_controller/{side}_index_joint2/position` | rad |
| 6 | `{target_controller}/{side}_index_joint3/position` | `{side}_glove_teleop_controller/{side}_index_joint3/position` | rad |
| 7 | `{target_controller}/{side}_middle_joint1/position` | `{side}_glove_teleop_controller/{side}_middle_joint1/position` | rad |
| 8 | `{target_controller}/{side}_middle_joint2/position` | `{side}_glove_teleop_controller/{side}_middle_joint2/position` | rad |
| 9 | `{target_controller}/{side}_middle_joint3/position` | `{side}_glove_teleop_controller/{side}_middle_joint3/position` | rad |
| 10 | `{target_controller}/{side}_ring_joint1/position` | `{side}_glove_teleop_controller/{side}_ring_joint1/position` | rad |
| 11 | `{target_controller}/{side}_ring_joint2/position` | `{side}_glove_teleop_controller/{side}_ring_joint2/position` | rad |
| 12 | `{target_controller}/{side}_ring_joint3/position` | `{side}_glove_teleop_controller/{side}_ring_joint3/position` | rad |
| 13 | `{target_controller}/{side}_baby_joint1/position` | `{side}_glove_teleop_controller/{side}_baby_joint1/position` | rad |
| 14 | `{target_controller}/{side}_baby_joint2/position` | `{side}_glove_teleop_controller/{side}_baby_joint2/position` | rad |
| 15 | `{target_controller}/{side}_baby_joint3/position` | `{side}_glove_teleop_controller/{side}_baby_joint3/position` | rad |

### 8.8 상위 skeleton controller 4 종 (example)

각 skeleton 은 대응 basic controller 의 reference 전체를 claim 하고, **같은 suffix** 를 자기
이름으로 다시 export 합니다. 즉 claim 이름은 `{target_controller}/<suffix>`, export 이름은
`<upper_controller>/<suffix>` 이며 suffix 목록은 8.1–8.4 의 reference 표와 동일합니다.

| Upper controller | `target_controller` 기본 대상 | suffix 목록 | 개수 |
|---|---|---|---|
| `{side}_joint_position_upper` | `{side}_joint_position_controller` | [8.1](#81-jointpositioncontroller) 의 reference 17 개 | 17 |
| `{side}_joint_impedance_upper` | `{side}_joint_impedance_controller` | [8.2](#82-jointimpedancecontroller) 의 reference 48 개 | 48 |
| `{side}_actuator_position_upper` | `{side}_actuator_position_controller` | [8.3](#83-actuatorpositioncontroller) 의 reference 16 개 | 16 |
| `{side}_actuator_effort_upper` | `{side}_actuator_effort_controller` | [8.4](#84-actuatoreffortcontroller) 의 reference 16 개 | 16 |

네 skeleton 모두 state interface 는 claim 하지 않고 `HandState` topic 을 구독해 관측을
얻습니다. `command_lock` 때문에 서로 다른 mode 의 basic controller 를 동시에 활성화할 수
없으므로 한 쌍만 선택해 활성화합니다.

## 9. Mock 하드웨어 차이

`AidinHand2MockSystemInterface` 는 command 98 개를 real 과 **완전히 동일하게** export 하고
mode switch 검증도 같습니다. State 는 physical 69 개만 노출합니다.

| 그룹 | Real | Mock | 비고 |
|---|---|---|---|
| Command interface 98 | O | O | 이름·개수·mode 검증 동일 |
| Joint position 21 | O | O | mock 은 FK 결과 |
| Actuator physical 48 | O | O | mock 은 IK/FK 로 합성 (velocity·current 는 0 고정) |
| Finger tactile 85 | O | X | xacro 에서 `use_mock` 시 제외 |
| Palm tactile 58 | O | X | xacro 에서 `use_mock` 시 제외 |
| Diagnostics 39 | O | X | xacro 에서 `use_mock` 시 제외 |
| Command echo 132 | O | X | real plugin 만 동적 export |
| Timestamp 2 | O | X | real plugin 만 동적 export |
| Runtime service (`run`/`stop`/`home`/`reconnect`) | O | X | mock 은 서비스 노드 없음 |
| `~/set_max_effort` topic | O | X | mock 은 `max_effort` 파라미터로 clamp 만 |

따라서 mock 에서는 `HandStateBroadcaster` 와 `DiagnosticsBroadcaster` 를 spawn 할 수
없습니다(claim 대상 state 부재). 네 basic controller 는 모두 load 가능합니다.

## 10. 개수 요약

| 구분 | 그룹 | 개수 | Real | Mock |
|---|---|---|---|---|
| Command | claim-only lock | 1 | O | O |
| Command | JointPositionController | 17 | O | O |
| Command | JointImpedanceController | 48 | O | O |
| Command | ActuatorPositionController | 16 | O | O |
| Command | ActuatorEffortController | 16 | O | O |
| Command | **합계** | **98** | 98 | 98 |
| State | joint position | 21 | O | O |
| State | actuator physical | 48 | O | O |
| State | finger tactile | 85 | O | X |
| State | palm tactile | 58 | O | X |
| State | diagnostics | 39 | O | X |
| State | command echo | 132 | O | X |
| State | timestamp | 2 | O | X |
| State | **합계** | **385** | 385 | 69 |
| Reference | JointPositionController | 17 | O | O |
| Reference | JointImpedanceController | 48 | O | O |
| Reference | ActuatorPositionController | 16 | O | O |
| Reference | ActuatorEffortController | 16 | O | O |
| Reference | GloveTeleopController (example) | 16 | O | O |
| Reference | 상위 skeleton 4 종 (example) | 17 / 48 / 16 / 16 | O | O |

위 개수는 손 하나 기준입니다. 양손 bringup 이면 `left_`·`right_` 두 벌이므로 두 배가 됩니다.

xacro 가 정적으로 선언하는 것은 physical 69 + tactile 143 + diagnostics 39 = 251 개이고,
command echo 132 개와 timestamp 2 개는 real plugin 이 동적으로 export 합니다.

## 11. 런타임 확인 명령

```bash
# 하드웨어 command·state interface 전체 (동적 export 포함)
ros2 control list_hardware_interfaces

# controller 별 claim·reference (chained 여부 포함)
ros2 control list_controllers -v

# 이름 하나 확인 (예: 왼손 command echo)
ros2 control list_hardware_interfaces | grep left_commanded
```

이 문서의 이름이 위 출력과 다르면 소스가 SSOT 입니다. 순서:
`aidin_hand2_hardware/src/*.cpp` (export 순서) →
`aidin_hand2_description/ros2_control/aidin_hand2.ros2_control.xacro` (정적 선언) →
`aidin_hand2_controllers/src/*.cpp` (claim·reference).

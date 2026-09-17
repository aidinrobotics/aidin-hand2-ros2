# Interface matrix

wrapper가 export하는 모든 `ros2_control` interface를 이름 하나하나 나열하는 참조 문서입니다. 요약과 규칙은
[Controllers](06_controllers.md)에 있고, 이 문서에서는 이름과 개수를 그대로 확인합니다.

- `{side}`는 `left` 또는 `right`입니다. 모든 행이 `left_`·`right_` 두 벌로 존재합니다.
- `ros2_control` 전체 이름은 `<component>/<interface>`입니다. component는 URDF의 joint·sensor·gpio
  이름이고, chainable controller의 reference는 component 자리에 controller 이름이 옵니다.
- `#` 열은 그 그룹 안의 0부터 시작하는 순번입니다. `claim #` 열은 `HandStateBroadcaster`가
  `state_interface_configuration()`으로 claim하는 순서이고, broadcaster가 이 순서로 직접 색인하므로 계약의
  일부입니다.
- 개수는 손 하나 기준입니다. 양손이면 두 배입니다.

## Contents

&nbsp;&nbsp;[**1. Command interfaces (65)**](#1-command-interfaces-65)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 command_lock (1)](#11-command_lock-1)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 joint_position_command (16)](#12-joint_position_command-16)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 joint_impedance_command (16)](#13-joint_impedance_command-16)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 actuator_position_command (16)](#14-actuator_position_command-16)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.5 actuator_effort_command (16)](#15-actuator_effort_command-16)<br>
&nbsp;&nbsp;[**2. Actuator and joint state interfaces (69)**](#2-actuator-and-joint-state-interfaces-69)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Joint position (21)](#21-joint-position-21)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Actuator state (48)](#22-actuator-state-48)<br>
&nbsp;&nbsp;[**3. Tactile state interfaces (143)**](#3-tactile-state-interfaces-143)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 Finger tactile (85)](#31-finger-tactile-85)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 Palm tactile (58)](#32-palm-tactile-58)<br>
&nbsp;&nbsp;[**4. Diagnostics state interfaces (39)**](#4-diagnostics-state-interfaces-39)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Hand-wide fields (7)](#41-hand-wide-fields-7)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Actuator enabled (16)](#42-actuator-enabled-16)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.3 Actuator fault (16)](#43-actuator-fault-16)<br>
&nbsp;&nbsp;[**5. Command echo state interfaces (99)**](#5-command-echo-state-interfaces-99)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.1 Scalars (3)](#51-scalars-3)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2 Controller input joint target (16)](#52-controller-input-joint-target-16)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.3 Per-actuator fields (80)](#53-per-actuator-fields-80)<br>
&nbsp;&nbsp;[**6. Timestamp state interfaces (2)**](#6-timestamp-state-interfaces-2)<br>
&nbsp;&nbsp;[**7. Controller claims and references**](#7-controller-claims-and-references)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.1 Command controllers](#71-command-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.2 HandStateBroadcaster](#72-handstatebroadcaster)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.3 DiagnosticsBroadcaster](#73-diagnosticsbroadcaster)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.4 Upper controller skeletons](#74-upper-controller-skeletons)<br>
&nbsp;&nbsp;[**8. Counts by backend**](#8-counts-by-backend)<br>
&nbsp;&nbsp;[**9. Runtime check**](#9-runtime-check)

## 1. Command interfaces (65)

command interface는 세 backend가 같은 65개를 export합니다. mode 전환은 빈 집합(Idle) 또는
`command_lock`과 한 mode의 port 16개 전부만 받아들이고, 일부·혼합 claim은 거부합니다. NaN 규칙은
[Controllers](06_controllers.md) 5장에 있습니다.

### 1.1 command_lock (1)

| # | Command interface | Description |
|---|---|---|
| 0 | `{side}_hand_control/command_lock` | 값 없음. mode 상호 배제를 위한 claim 전용 |

### 1.2 joint_position_command (16)

`JointPositionController`가 `command_lock`과 함께 claim합니다.

| # | Command interface | Unit | Description |
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

### 1.3 joint_impedance_command (16)

`JointImpedanceController`가 `command_lock`과 함께 claim합니다.

| # | Command interface | Unit | Description |
|---|---|---|---|
| 0 | `{side}_joint_impedance_command/target_position_rad.thumb_joint0` | rad | `{side}_thumb_joint0` 평형 각도 |
| 1 | `{side}_joint_impedance_command/target_position_rad.thumb_joint1` | rad | `{side}_thumb_joint1` 평형 각도 |
| 2 | `{side}_joint_impedance_command/target_position_rad.thumb_joint2` | rad | `{side}_thumb_joint2` 평형 각도 |
| 3 | `{side}_joint_impedance_command/target_position_rad.thumb_joint3` | rad | `{side}_thumb_joint3` 평형 각도 |
| 4 | `{side}_joint_impedance_command/target_position_rad.index_joint1` | rad | `{side}_index_joint1` 평형 각도 |
| 5 | `{side}_joint_impedance_command/target_position_rad.index_joint2` | rad | `{side}_index_joint2` 평형 각도 |
| 6 | `{side}_joint_impedance_command/target_position_rad.index_joint3` | rad | `{side}_index_joint3` 평형 각도 |
| 7 | `{side}_joint_impedance_command/target_position_rad.middle_joint1` | rad | `{side}_middle_joint1` 평형 각도 |
| 8 | `{side}_joint_impedance_command/target_position_rad.middle_joint2` | rad | `{side}_middle_joint2` 평형 각도 |
| 9 | `{side}_joint_impedance_command/target_position_rad.middle_joint3` | rad | `{side}_middle_joint3` 평형 각도 |
| 10 | `{side}_joint_impedance_command/target_position_rad.ring_joint1` | rad | `{side}_ring_joint1` 평형 각도 |
| 11 | `{side}_joint_impedance_command/target_position_rad.ring_joint2` | rad | `{side}_ring_joint2` 평형 각도 |
| 12 | `{side}_joint_impedance_command/target_position_rad.ring_joint3` | rad | `{side}_ring_joint3` 평형 각도 |
| 13 | `{side}_joint_impedance_command/target_position_rad.baby_joint1` | rad | `{side}_baby_joint1` 평형 각도 |
| 14 | `{side}_joint_impedance_command/target_position_rad.baby_joint2` | rad | `{side}_baby_joint2` 평형 각도 |
| 15 | `{side}_joint_impedance_command/target_position_rad.baby_joint3` | rad | `{side}_baby_joint3` 평형 각도 |

### 1.4 actuator_position_command (16)

`ActuatorPositionController`가 `command_lock`과 함께 claim합니다.

| # | Command interface | Unit | Description |
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

### 1.5 actuator_effort_command (16)

`ActuatorEffortController`가 `command_lock`과 함께 claim합니다.

| # | Command interface | Unit | Description |
|---|---|---|---|
| 0 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator0` | 정격 전류의 0.1% | `{side}_thumb_actuator0` 목표 effort |
| 1 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator1` | 정격 전류의 0.1% | `{side}_thumb_actuator1` 목표 effort |
| 2 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator2` | 정격 전류의 0.1% | `{side}_thumb_actuator2` 목표 effort |
| 3 | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator3` | 정격 전류의 0.1% | `{side}_thumb_actuator3` 목표 effort |
| 4 | `{side}_actuator_effort_command/target_effort_pct.index_actuator1` | 정격 전류의 0.1% | `{side}_index_actuator1` 목표 effort |
| 5 | `{side}_actuator_effort_command/target_effort_pct.index_actuator2` | 정격 전류의 0.1% | `{side}_index_actuator2` 목표 effort |
| 6 | `{side}_actuator_effort_command/target_effort_pct.index_actuator3` | 정격 전류의 0.1% | `{side}_index_actuator3` 목표 effort |
| 7 | `{side}_actuator_effort_command/target_effort_pct.middle_actuator1` | 정격 전류의 0.1% | `{side}_middle_actuator1` 목표 effort |
| 8 | `{side}_actuator_effort_command/target_effort_pct.middle_actuator2` | 정격 전류의 0.1% | `{side}_middle_actuator2` 목표 effort |
| 9 | `{side}_actuator_effort_command/target_effort_pct.middle_actuator3` | 정격 전류의 0.1% | `{side}_middle_actuator3` 목표 effort |
| 10 | `{side}_actuator_effort_command/target_effort_pct.ring_actuator1` | 정격 전류의 0.1% | `{side}_ring_actuator1` 목표 effort |
| 11 | `{side}_actuator_effort_command/target_effort_pct.ring_actuator2` | 정격 전류의 0.1% | `{side}_ring_actuator2` 목표 effort |
| 12 | `{side}_actuator_effort_command/target_effort_pct.ring_actuator3` | 정격 전류의 0.1% | `{side}_ring_actuator3` 목표 effort |
| 13 | `{side}_actuator_effort_command/target_effort_pct.baby_actuator1` | 정격 전류의 0.1% | `{side}_baby_actuator1` 목표 effort |
| 14 | `{side}_actuator_effort_command/target_effort_pct.baby_actuator2` | 정격 전류의 0.1% | `{side}_baby_actuator2` 목표 effort |
| 15 | `{side}_actuator_effort_command/target_effort_pct.baby_actuator3` | 정격 전류의 0.1% | `{side}_baby_actuator3` 목표 effort |

## 2. Actuator and joint state interfaces (69)

세 backend가 공통으로 export하는 물리 state입니다. mock은 이 69개만 export하고 `velocity_rpm`과
`current_ma`는 항상 `0`입니다.

### 2.1 Joint position (21)

| # | State interface | Unit | Kind | HandState field | claim # |
|---|---|---|---|---|---|
| 0 | `{side}_thumb_joint0/position` | rad | active | `joint_position[0]` | 0 |
| 1 | `{side}_thumb_joint1/position` | rad | active | `joint_position[1]` | 1 |
| 2 | `{side}_thumb_joint2/position` | rad | active | `joint_position[2]` | 2 |
| 3 | `{side}_thumb_joint3/position` | rad | active | `joint_position[3]` | 3 |
| 4 | `{side}_thumb_joint4/position` | rad | passive. joint3에 결합 | `joint_position[4]` | 4 |
| 5 | `{side}_index_joint1/position` | rad | active | `joint_position[5]` | 5 |
| 6 | `{side}_index_joint2/position` | rad | active | `joint_position[6]` | 6 |
| 7 | `{side}_index_joint3/position` | rad | active | `joint_position[7]` | 7 |
| 8 | `{side}_index_joint4/position` | rad | passive. joint3에 결합 | `joint_position[8]` | 8 |
| 9 | `{side}_middle_joint1/position` | rad | active | `joint_position[9]` | 9 |
| 10 | `{side}_middle_joint2/position` | rad | active | `joint_position[10]` | 10 |
| 11 | `{side}_middle_joint3/position` | rad | active | `joint_position[11]` | 11 |
| 12 | `{side}_middle_joint4/position` | rad | passive. joint3에 결합 | `joint_position[12]` | 12 |
| 13 | `{side}_ring_joint1/position` | rad | active | `joint_position[13]` | 13 |
| 14 | `{side}_ring_joint2/position` | rad | active | `joint_position[14]` | 14 |
| 15 | `{side}_ring_joint3/position` | rad | active | `joint_position[15]` | 15 |
| 16 | `{side}_ring_joint4/position` | rad | passive. joint3에 결합 | `joint_position[16]` | 16 |
| 17 | `{side}_baby_joint1/position` | rad | active | `joint_position[17]` | 17 |
| 18 | `{side}_baby_joint2/position` | rad | active | `joint_position[18]` | 18 |
| 19 | `{side}_baby_joint3/position` | rad | active | `joint_position[19]` | 19 |
| 20 | `{side}_baby_joint4/position` | rad | passive. joint3에 결합 | `joint_position[20]` | 20 |

### 2.2 Actuator state (48)

actuator마다 `position_cnt`·`velocity_rpm`·`current_ma` 셋이 있습니다. `HandStateBroadcaster`는 그룹별로
claim하므로 `claim #`는 세 열이 서로 다른 블록입니다.

| # | Actuator | `position_cnt` (encoder count) | claim # | `velocity_rpm` (rpm) | claim # | `current_ma` (mA) | claim # |
|---|---|---|---|---|---|---|---|
| 0 | `{side}_thumb_actuator0` | `actuator_position[0]` | 21 | `actuator_velocity[0]` | 37 | `actuator_current[0]` | 53 |
| 1 | `{side}_thumb_actuator1` | `actuator_position[1]` | 22 | `actuator_velocity[1]` | 38 | `actuator_current[1]` | 54 |
| 2 | `{side}_thumb_actuator2` | `actuator_position[2]` | 23 | `actuator_velocity[2]` | 39 | `actuator_current[2]` | 55 |
| 3 | `{side}_thumb_actuator3` | `actuator_position[3]` | 24 | `actuator_velocity[3]` | 40 | `actuator_current[3]` | 56 |
| 4 | `{side}_index_actuator1` | `actuator_position[4]` | 25 | `actuator_velocity[4]` | 41 | `actuator_current[4]` | 57 |
| 5 | `{side}_index_actuator2` | `actuator_position[5]` | 26 | `actuator_velocity[5]` | 42 | `actuator_current[5]` | 58 |
| 6 | `{side}_index_actuator3` | `actuator_position[6]` | 27 | `actuator_velocity[6]` | 43 | `actuator_current[6]` | 59 |
| 7 | `{side}_middle_actuator1` | `actuator_position[7]` | 28 | `actuator_velocity[7]` | 44 | `actuator_current[7]` | 60 |
| 8 | `{side}_middle_actuator2` | `actuator_position[8]` | 29 | `actuator_velocity[8]` | 45 | `actuator_current[8]` | 61 |
| 9 | `{side}_middle_actuator3` | `actuator_position[9]` | 30 | `actuator_velocity[9]` | 46 | `actuator_current[9]` | 62 |
| 10 | `{side}_ring_actuator1` | `actuator_position[10]` | 31 | `actuator_velocity[10]` | 47 | `actuator_current[10]` | 63 |
| 11 | `{side}_ring_actuator2` | `actuator_position[11]` | 32 | `actuator_velocity[11]` | 48 | `actuator_current[11]` | 64 |
| 12 | `{side}_ring_actuator3` | `actuator_position[12]` | 33 | `actuator_velocity[12]` | 49 | `actuator_current[12]` | 65 |
| 13 | `{side}_baby_actuator1` | `actuator_position[13]` | 34 | `actuator_velocity[13]` | 50 | `actuator_current[13]` | 66 |
| 14 | `{side}_baby_actuator2` | `actuator_position[14]` | 35 | `actuator_velocity[14]` | 51 | `actuator_current[14]` | 67 |
| 15 | `{side}_baby_actuator3` | `actuator_position[15]` | 36 | `actuator_velocity[15]` | 52 | `actuator_current[15]` | 68 |

state interface 이름은 `{side}_<actuator>/position_cnt`, `{side}_<actuator>/velocity_rpm`,
`{side}_<actuator>/current_ma`이고 표의 셀은 대응하는 `HandState` 필드입니다.

## 3. Tactile state interfaces (143)

로봇 핸드와 isaac backend가 export합니다. 값은 센서가 전송한 16-bit 원시 count이고 isaac에서는 수신한
topic 값입니다.

### 3.1 Finger tactile (85)

sensor component는 `{side}_{finger}_sensor`이고 finger마다 taxel 17개입니다. `HandState` 필드는
`tactile_{finger}[k]`입니다.

| # | State interface | HandState field | claim # |
|---|---|---|---|
| 0 | `{side}_thumb_sensor/tactile_1` | `tactile_thumb[0]` | 69 |
| 1 | `{side}_thumb_sensor/tactile_2` | `tactile_thumb[1]` | 70 |
| 2 | `{side}_thumb_sensor/tactile_3` | `tactile_thumb[2]` | 71 |
| 3 | `{side}_thumb_sensor/tactile_4` | `tactile_thumb[3]` | 72 |
| 4 | `{side}_thumb_sensor/tactile_5` | `tactile_thumb[4]` | 73 |
| 5 | `{side}_thumb_sensor/tactile_6` | `tactile_thumb[5]` | 74 |
| 6 | `{side}_thumb_sensor/tactile_7` | `tactile_thumb[6]` | 75 |
| 7 | `{side}_thumb_sensor/tactile_8` | `tactile_thumb[7]` | 76 |
| 8 | `{side}_thumb_sensor/tactile_9` | `tactile_thumb[8]` | 77 |
| 9 | `{side}_thumb_sensor/tactile_10` | `tactile_thumb[9]` | 78 |
| 10 | `{side}_thumb_sensor/tactile_11` | `tactile_thumb[10]` | 79 |
| 11 | `{side}_thumb_sensor/tactile_12` | `tactile_thumb[11]` | 80 |
| 12 | `{side}_thumb_sensor/tactile_13` | `tactile_thumb[12]` | 81 |
| 13 | `{side}_thumb_sensor/tactile_14` | `tactile_thumb[13]` | 82 |
| 14 | `{side}_thumb_sensor/tactile_15` | `tactile_thumb[14]` | 83 |
| 15 | `{side}_thumb_sensor/tactile_16` | `tactile_thumb[15]` | 84 |
| 16 | `{side}_thumb_sensor/tactile_17` | `tactile_thumb[16]` | 85 |
| 17 | `{side}_index_sensor/tactile_1` | `tactile_index[0]` | 86 |
| 18 | `{side}_index_sensor/tactile_2` | `tactile_index[1]` | 87 |
| 19 | `{side}_index_sensor/tactile_3` | `tactile_index[2]` | 88 |
| 20 | `{side}_index_sensor/tactile_4` | `tactile_index[3]` | 89 |
| 21 | `{side}_index_sensor/tactile_5` | `tactile_index[4]` | 90 |
| 22 | `{side}_index_sensor/tactile_6` | `tactile_index[5]` | 91 |
| 23 | `{side}_index_sensor/tactile_7` | `tactile_index[6]` | 92 |
| 24 | `{side}_index_sensor/tactile_8` | `tactile_index[7]` | 93 |
| 25 | `{side}_index_sensor/tactile_9` | `tactile_index[8]` | 94 |
| 26 | `{side}_index_sensor/tactile_10` | `tactile_index[9]` | 95 |
| 27 | `{side}_index_sensor/tactile_11` | `tactile_index[10]` | 96 |
| 28 | `{side}_index_sensor/tactile_12` | `tactile_index[11]` | 97 |
| 29 | `{side}_index_sensor/tactile_13` | `tactile_index[12]` | 98 |
| 30 | `{side}_index_sensor/tactile_14` | `tactile_index[13]` | 99 |
| 31 | `{side}_index_sensor/tactile_15` | `tactile_index[14]` | 100 |
| 32 | `{side}_index_sensor/tactile_16` | `tactile_index[15]` | 101 |
| 33 | `{side}_index_sensor/tactile_17` | `tactile_index[16]` | 102 |
| 34 | `{side}_middle_sensor/tactile_1` | `tactile_middle[0]` | 103 |
| 35 | `{side}_middle_sensor/tactile_2` | `tactile_middle[1]` | 104 |
| 36 | `{side}_middle_sensor/tactile_3` | `tactile_middle[2]` | 105 |
| 37 | `{side}_middle_sensor/tactile_4` | `tactile_middle[3]` | 106 |
| 38 | `{side}_middle_sensor/tactile_5` | `tactile_middle[4]` | 107 |
| 39 | `{side}_middle_sensor/tactile_6` | `tactile_middle[5]` | 108 |
| 40 | `{side}_middle_sensor/tactile_7` | `tactile_middle[6]` | 109 |
| 41 | `{side}_middle_sensor/tactile_8` | `tactile_middle[7]` | 110 |
| 42 | `{side}_middle_sensor/tactile_9` | `tactile_middle[8]` | 111 |
| 43 | `{side}_middle_sensor/tactile_10` | `tactile_middle[9]` | 112 |
| 44 | `{side}_middle_sensor/tactile_11` | `tactile_middle[10]` | 113 |
| 45 | `{side}_middle_sensor/tactile_12` | `tactile_middle[11]` | 114 |
| 46 | `{side}_middle_sensor/tactile_13` | `tactile_middle[12]` | 115 |
| 47 | `{side}_middle_sensor/tactile_14` | `tactile_middle[13]` | 116 |
| 48 | `{side}_middle_sensor/tactile_15` | `tactile_middle[14]` | 117 |
| 49 | `{side}_middle_sensor/tactile_16` | `tactile_middle[15]` | 118 |
| 50 | `{side}_middle_sensor/tactile_17` | `tactile_middle[16]` | 119 |
| 51 | `{side}_ring_sensor/tactile_1` | `tactile_ring[0]` | 120 |
| 52 | `{side}_ring_sensor/tactile_2` | `tactile_ring[1]` | 121 |
| 53 | `{side}_ring_sensor/tactile_3` | `tactile_ring[2]` | 122 |
| 54 | `{side}_ring_sensor/tactile_4` | `tactile_ring[3]` | 123 |
| 55 | `{side}_ring_sensor/tactile_5` | `tactile_ring[4]` | 124 |
| 56 | `{side}_ring_sensor/tactile_6` | `tactile_ring[5]` | 125 |
| 57 | `{side}_ring_sensor/tactile_7` | `tactile_ring[6]` | 126 |
| 58 | `{side}_ring_sensor/tactile_8` | `tactile_ring[7]` | 127 |
| 59 | `{side}_ring_sensor/tactile_9` | `tactile_ring[8]` | 128 |
| 60 | `{side}_ring_sensor/tactile_10` | `tactile_ring[9]` | 129 |
| 61 | `{side}_ring_sensor/tactile_11` | `tactile_ring[10]` | 130 |
| 62 | `{side}_ring_sensor/tactile_12` | `tactile_ring[11]` | 131 |
| 63 | `{side}_ring_sensor/tactile_13` | `tactile_ring[12]` | 132 |
| 64 | `{side}_ring_sensor/tactile_14` | `tactile_ring[13]` | 133 |
| 65 | `{side}_ring_sensor/tactile_15` | `tactile_ring[14]` | 134 |
| 66 | `{side}_ring_sensor/tactile_16` | `tactile_ring[15]` | 135 |
| 67 | `{side}_ring_sensor/tactile_17` | `tactile_ring[16]` | 136 |
| 68 | `{side}_baby_sensor/tactile_1` | `tactile_baby[0]` | 137 |
| 69 | `{side}_baby_sensor/tactile_2` | `tactile_baby[1]` | 138 |
| 70 | `{side}_baby_sensor/tactile_3` | `tactile_baby[2]` | 139 |
| 71 | `{side}_baby_sensor/tactile_4` | `tactile_baby[3]` | 140 |
| 72 | `{side}_baby_sensor/tactile_5` | `tactile_baby[4]` | 141 |
| 73 | `{side}_baby_sensor/tactile_6` | `tactile_baby[5]` | 142 |
| 74 | `{side}_baby_sensor/tactile_7` | `tactile_baby[6]` | 143 |
| 75 | `{side}_baby_sensor/tactile_8` | `tactile_baby[7]` | 144 |
| 76 | `{side}_baby_sensor/tactile_9` | `tactile_baby[8]` | 145 |
| 77 | `{side}_baby_sensor/tactile_10` | `tactile_baby[9]` | 146 |
| 78 | `{side}_baby_sensor/tactile_11` | `tactile_baby[10]` | 147 |
| 79 | `{side}_baby_sensor/tactile_12` | `tactile_baby[11]` | 148 |
| 80 | `{side}_baby_sensor/tactile_13` | `tactile_baby[12]` | 149 |
| 81 | `{side}_baby_sensor/tactile_14` | `tactile_baby[13]` | 150 |
| 82 | `{side}_baby_sensor/tactile_15` | `tactile_baby[14]` | 151 |
| 83 | `{side}_baby_sensor/tactile_16` | `tactile_baby[15]` | 152 |
| 84 | `{side}_baby_sensor/tactile_17` | `tactile_baby[16]` | 153 |

### 3.2 Palm tactile (58)

sensor component는 `{side}_palm_sensor`이고 영역 셋이 있습니다.

| # | State interface | HandState field | claim # |
|---|---|---|---|
| 0 | `{side}_palm_sensor/palm1_upper_1` | `tactile_palm1_upper[0]` | 154 |
| 1 | `{side}_palm_sensor/palm1_upper_2` | `tactile_palm1_upper[1]` | 155 |
| 2 | `{side}_palm_sensor/palm1_upper_3` | `tactile_palm1_upper[2]` | 156 |
| 3 | `{side}_palm_sensor/palm1_upper_4` | `tactile_palm1_upper[3]` | 157 |
| 4 | `{side}_palm_sensor/palm1_upper_5` | `tactile_palm1_upper[4]` | 158 |
| 5 | `{side}_palm_sensor/palm1_upper_6` | `tactile_palm1_upper[5]` | 159 |
| 6 | `{side}_palm_sensor/palm1_upper_7` | `tactile_palm1_upper[6]` | 160 |
| 7 | `{side}_palm_sensor/palm1_upper_8` | `tactile_palm1_upper[7]` | 161 |
| 8 | `{side}_palm_sensor/palm1_upper_9` | `tactile_palm1_upper[8]` | 162 |
| 9 | `{side}_palm_sensor/palm1_upper_10` | `tactile_palm1_upper[9]` | 163 |
| 10 | `{side}_palm_sensor/palm1_upper_11` | `tactile_palm1_upper[10]` | 164 |
| 11 | `{side}_palm_sensor/palm1_upper_12` | `tactile_palm1_upper[11]` | 165 |
| 12 | `{side}_palm_sensor/palm1_upper_13` | `tactile_palm1_upper[12]` | 166 |
| 13 | `{side}_palm_sensor/palm1_upper_14` | `tactile_palm1_upper[13]` | 167 |
| 14 | `{side}_palm_sensor/palm1_upper_15` | `tactile_palm1_upper[14]` | 168 |
| 15 | `{side}_palm_sensor/palm1_upper_16` | `tactile_palm1_upper[15]` | 169 |
| 16 | `{side}_palm_sensor/palm1_upper_17` | `tactile_palm1_upper[16]` | 170 |
| 17 | `{side}_palm_sensor/palm1_upper_18` | `tactile_palm1_upper[17]` | 171 |
| 18 | `{side}_palm_sensor/palm1_upper_19` | `tactile_palm1_upper[18]` | 172 |
| 19 | `{side}_palm_sensor/palm1_upper_20` | `tactile_palm1_upper[19]` | 173 |
| 20 | `{side}_palm_sensor/palm1_lower_1` | `tactile_palm1_lower[0]` | 174 |
| 21 | `{side}_palm_sensor/palm1_lower_2` | `tactile_palm1_lower[1]` | 175 |
| 22 | `{side}_palm_sensor/palm1_lower_3` | `tactile_palm1_lower[2]` | 176 |
| 23 | `{side}_palm_sensor/palm1_lower_4` | `tactile_palm1_lower[3]` | 177 |
| 24 | `{side}_palm_sensor/palm1_lower_5` | `tactile_palm1_lower[4]` | 178 |
| 25 | `{side}_palm_sensor/palm1_lower_6` | `tactile_palm1_lower[5]` | 179 |
| 26 | `{side}_palm_sensor/palm1_lower_7` | `tactile_palm1_lower[6]` | 180 |
| 27 | `{side}_palm_sensor/palm1_lower_8` | `tactile_palm1_lower[7]` | 181 |
| 28 | `{side}_palm_sensor/palm1_lower_9` | `tactile_palm1_lower[8]` | 182 |
| 29 | `{side}_palm_sensor/palm1_lower_10` | `tactile_palm1_lower[9]` | 183 |
| 30 | `{side}_palm_sensor/palm1_lower_11` | `tactile_palm1_lower[10]` | 184 |
| 31 | `{side}_palm_sensor/palm1_lower_12` | `tactile_palm1_lower[11]` | 185 |
| 32 | `{side}_palm_sensor/palm1_lower_13` | `tactile_palm1_lower[12]` | 186 |
| 33 | `{side}_palm_sensor/palm1_lower_14` | `tactile_palm1_lower[13]` | 187 |
| 34 | `{side}_palm_sensor/palm1_lower_15` | `tactile_palm1_lower[14]` | 188 |
| 35 | `{side}_palm_sensor/palm1_lower_16` | `tactile_palm1_lower[15]` | 189 |
| 36 | `{side}_palm_sensor/palm1_lower_17` | `tactile_palm1_lower[16]` | 190 |
| 37 | `{side}_palm_sensor/palm1_lower_18` | `tactile_palm1_lower[17]` | 191 |
| 38 | `{side}_palm_sensor/palm1_lower_19` | `tactile_palm1_lower[18]` | 192 |
| 39 | `{side}_palm_sensor/palm1_lower_20` | `tactile_palm1_lower[19]` | 193 |
| 40 | `{side}_palm_sensor/palm2_1` | `tactile_palm2[0]` | 194 |
| 41 | `{side}_palm_sensor/palm2_2` | `tactile_palm2[1]` | 195 |
| 42 | `{side}_palm_sensor/palm2_3` | `tactile_palm2[2]` | 196 |
| 43 | `{side}_palm_sensor/palm2_4` | `tactile_palm2[3]` | 197 |
| 44 | `{side}_palm_sensor/palm2_5` | `tactile_palm2[4]` | 198 |
| 45 | `{side}_palm_sensor/palm2_6` | `tactile_palm2[5]` | 199 |
| 46 | `{side}_palm_sensor/palm2_7` | `tactile_palm2[6]` | 200 |
| 47 | `{side}_palm_sensor/palm2_8` | `tactile_palm2[7]` | 201 |
| 48 | `{side}_palm_sensor/palm2_9` | `tactile_palm2[8]` | 202 |
| 49 | `{side}_palm_sensor/palm2_10` | `tactile_palm2[9]` | 203 |
| 50 | `{side}_palm_sensor/palm2_11` | `tactile_palm2[10]` | 204 |
| 51 | `{side}_palm_sensor/palm2_12` | `tactile_palm2[11]` | 205 |
| 52 | `{side}_palm_sensor/palm2_13` | `tactile_palm2[12]` | 206 |
| 53 | `{side}_palm_sensor/palm2_14` | `tactile_palm2[13]` | 207 |
| 54 | `{side}_palm_sensor/palm2_15` | `tactile_palm2[14]` | 208 |
| 55 | `{side}_palm_sensor/palm2_16` | `tactile_palm2[15]` | 209 |
| 56 | `{side}_palm_sensor/palm2_17` | `tactile_palm2[16]` | 210 |
| 57 | `{side}_palm_sensor/palm2_18` | `tactile_palm2[17]` | 211 |

## 4. Diagnostics state interfaces (39)

gpio component `{side}_diagnostics`이고 `DiagnosticsBroadcaster`만 claim합니다. `claim #`는
`DiagnosticsBroadcaster`의 순서입니다. 로봇 핸드와 isaac backend가 export합니다.

### 4.1 Hand-wide fields (7)

| # | State interface | Value | HandDiagnostics field | claim # |
|---|---|---|---|---|
| 0 | `{side}_diagnostics/lifecycle` | `HandLifecycle` ordinal | `lifecycle`. 이름 문자열로 변환 | 0 |
| 1 | `{side}_diagnostics/nan_command_count` | 누적 개수 | `nan_command_count` | 1 |
| 2 | `{side}_diagnostics/control_cycles` | 누적 개수 | `control_cycles` | 2 |
| 3 | `{side}_diagnostics/deadline_misses` | 누적 개수 | `deadline_misses` | 3 |
| 4 | `{side}_diagnostics/last_period_ms` | ms | `last_period_ms` | 4 |
| 5 | `{side}_diagnostics/last_compute_ms` | ms | `last_compute_ms` | 5 |
| 6 | `{side}_diagnostics/homing_state` | `HomingState` ordinal | `homing_state`. 이름 문자열로 변환 | 6 |

### 4.2 Actuator enabled (16)

| # | State interface | Value | HandDiagnostics field | claim # |
|---|---|---|---|---|
| 0 | `{side}_diagnostics/enabled_thumb_actuator0` | `1` enabled, `0` disabled | `actuator_enabled[0]` | 7 |
| 1 | `{side}_diagnostics/enabled_thumb_actuator1` | `1` enabled, `0` disabled | `actuator_enabled[1]` | 8 |
| 2 | `{side}_diagnostics/enabled_thumb_actuator2` | `1` enabled, `0` disabled | `actuator_enabled[2]` | 9 |
| 3 | `{side}_diagnostics/enabled_thumb_actuator3` | `1` enabled, `0` disabled | `actuator_enabled[3]` | 10 |
| 4 | `{side}_diagnostics/enabled_index_actuator1` | `1` enabled, `0` disabled | `actuator_enabled[4]` | 11 |
| 5 | `{side}_diagnostics/enabled_index_actuator2` | `1` enabled, `0` disabled | `actuator_enabled[5]` | 12 |
| 6 | `{side}_diagnostics/enabled_index_actuator3` | `1` enabled, `0` disabled | `actuator_enabled[6]` | 13 |
| 7 | `{side}_diagnostics/enabled_middle_actuator1` | `1` enabled, `0` disabled | `actuator_enabled[7]` | 14 |
| 8 | `{side}_diagnostics/enabled_middle_actuator2` | `1` enabled, `0` disabled | `actuator_enabled[8]` | 15 |
| 9 | `{side}_diagnostics/enabled_middle_actuator3` | `1` enabled, `0` disabled | `actuator_enabled[9]` | 16 |
| 10 | `{side}_diagnostics/enabled_ring_actuator1` | `1` enabled, `0` disabled | `actuator_enabled[10]` | 17 |
| 11 | `{side}_diagnostics/enabled_ring_actuator2` | `1` enabled, `0` disabled | `actuator_enabled[11]` | 18 |
| 12 | `{side}_diagnostics/enabled_ring_actuator3` | `1` enabled, `0` disabled | `actuator_enabled[12]` | 19 |
| 13 | `{side}_diagnostics/enabled_baby_actuator1` | `1` enabled, `0` disabled | `actuator_enabled[13]` | 20 |
| 14 | `{side}_diagnostics/enabled_baby_actuator2` | `1` enabled, `0` disabled | `actuator_enabled[14]` | 21 |
| 15 | `{side}_diagnostics/enabled_baby_actuator3` | `1` enabled, `0` disabled | `actuator_enabled[15]` | 22 |

### 4.3 Actuator fault (16)

| # | State interface | Value | HandDiagnostics field | claim # |
|---|---|---|---|---|
| 0 | `{side}_diagnostics/fault_thumb_actuator0` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[0]`. 이름 문자열로 변환 | 23 |
| 1 | `{side}_diagnostics/fault_thumb_actuator1` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[1]`. 이름 문자열로 변환 | 24 |
| 2 | `{side}_diagnostics/fault_thumb_actuator2` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[2]`. 이름 문자열로 변환 | 25 |
| 3 | `{side}_diagnostics/fault_thumb_actuator3` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[3]`. 이름 문자열로 변환 | 26 |
| 4 | `{side}_diagnostics/fault_index_actuator1` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[4]`. 이름 문자열로 변환 | 27 |
| 5 | `{side}_diagnostics/fault_index_actuator2` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[5]`. 이름 문자열로 변환 | 28 |
| 6 | `{side}_diagnostics/fault_index_actuator3` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[6]`. 이름 문자열로 변환 | 29 |
| 7 | `{side}_diagnostics/fault_middle_actuator1` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[7]`. 이름 문자열로 변환 | 30 |
| 8 | `{side}_diagnostics/fault_middle_actuator2` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[8]`. 이름 문자열로 변환 | 31 |
| 9 | `{side}_diagnostics/fault_middle_actuator3` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[9]`. 이름 문자열로 변환 | 32 |
| 10 | `{side}_diagnostics/fault_ring_actuator1` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[10]`. 이름 문자열로 변환 | 33 |
| 11 | `{side}_diagnostics/fault_ring_actuator2` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[11]`. 이름 문자열로 변환 | 34 |
| 12 | `{side}_diagnostics/fault_ring_actuator3` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[12]`. 이름 문자열로 변환 | 35 |
| 13 | `{side}_diagnostics/fault_baby_actuator1` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[13]`. 이름 문자열로 변환 | 36 |
| 14 | `{side}_diagnostics/fault_baby_actuator2` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[14]`. 이름 문자열로 변환 | 37 |
| 15 | `{side}_diagnostics/fault_baby_actuator3` | `ActuatorFault` ordinal. `0`이 None | `actuator_fault_name[15]`. 이름 문자열로 변환 | 38 |

## 5. Command echo state interfaces (99)

gpio component `{side}_commanded`이고 같은 cycle에 SDK가 적용한 command를 담습니다. `HandState`의
`command_state` 필드로 흐릅니다. 로봇 핸드와 isaac backend가 export합니다.

### 5.1 Scalars (3)

| # | State interface | Value | CommandState field | claim # |
|---|---|---|---|---|
| 0 | `{side}_commanded/controller_input_mode` | `0` idle · `1` joint position · `2` joint impedance · `3` actuator position · `4` actuator effort | `controller_input_mode` | 212 |
| 1 | `{side}_commanded/controller_output_type` | `0` none · `1` actuator position · `2` actuator effort | `controller_output_type` | 213 |
| 2 | `{side}_commanded/selected_source` | `0` none · `1` controller · `2` quick stop · `3` homing | `selected_source` | 214 |

### 5.2 Controller input joint target (16)

joint position과 joint impedance가 같은 echo를 공유하므로 `CommandState`의 두 nested 필드에 같은 값이
들어갑니다.

| # | State interface | Unit | CommandState field | claim # |
|---|---|---|---|---|
| 0 | `{side}_commanded/controller_input_target_position_rad.thumb_joint0` | rad | `joint_position_input.target_position_rad[0]` · `joint_impedance_input.target_position_rad[0]` | 215 |
| 1 | `{side}_commanded/controller_input_target_position_rad.thumb_joint1` | rad | `joint_position_input.target_position_rad[1]` · `joint_impedance_input.target_position_rad[1]` | 216 |
| 2 | `{side}_commanded/controller_input_target_position_rad.thumb_joint2` | rad | `joint_position_input.target_position_rad[2]` · `joint_impedance_input.target_position_rad[2]` | 217 |
| 3 | `{side}_commanded/controller_input_target_position_rad.thumb_joint3` | rad | `joint_position_input.target_position_rad[3]` · `joint_impedance_input.target_position_rad[3]` | 218 |
| 4 | `{side}_commanded/controller_input_target_position_rad.index_joint1` | rad | `joint_position_input.target_position_rad[4]` · `joint_impedance_input.target_position_rad[4]` | 219 |
| 5 | `{side}_commanded/controller_input_target_position_rad.index_joint2` | rad | `joint_position_input.target_position_rad[5]` · `joint_impedance_input.target_position_rad[5]` | 220 |
| 6 | `{side}_commanded/controller_input_target_position_rad.index_joint3` | rad | `joint_position_input.target_position_rad[6]` · `joint_impedance_input.target_position_rad[6]` | 221 |
| 7 | `{side}_commanded/controller_input_target_position_rad.middle_joint1` | rad | `joint_position_input.target_position_rad[7]` · `joint_impedance_input.target_position_rad[7]` | 222 |
| 8 | `{side}_commanded/controller_input_target_position_rad.middle_joint2` | rad | `joint_position_input.target_position_rad[8]` · `joint_impedance_input.target_position_rad[8]` | 223 |
| 9 | `{side}_commanded/controller_input_target_position_rad.middle_joint3` | rad | `joint_position_input.target_position_rad[9]` · `joint_impedance_input.target_position_rad[9]` | 224 |
| 10 | `{side}_commanded/controller_input_target_position_rad.ring_joint1` | rad | `joint_position_input.target_position_rad[10]` · `joint_impedance_input.target_position_rad[10]` | 225 |
| 11 | `{side}_commanded/controller_input_target_position_rad.ring_joint2` | rad | `joint_position_input.target_position_rad[11]` · `joint_impedance_input.target_position_rad[11]` | 226 |
| 12 | `{side}_commanded/controller_input_target_position_rad.ring_joint3` | rad | `joint_position_input.target_position_rad[12]` · `joint_impedance_input.target_position_rad[12]` | 227 |
| 13 | `{side}_commanded/controller_input_target_position_rad.baby_joint1` | rad | `joint_position_input.target_position_rad[13]` · `joint_impedance_input.target_position_rad[13]` | 228 |
| 14 | `{side}_commanded/controller_input_target_position_rad.baby_joint2` | rad | `joint_position_input.target_position_rad[14]` · `joint_impedance_input.target_position_rad[14]` | 229 |
| 15 | `{side}_commanded/controller_input_target_position_rad.baby_joint3` | rad | `joint_position_input.target_position_rad[15]` · `joint_impedance_input.target_position_rad[15]` | 230 |

### 5.3 Per-actuator fields (80)

actuator마다 필드 5개가 이 순서로 interleave됩니다. `claim #`는 `231 + 5 × actuator index + field index`입니다.

| # | State interface | Unit | CommandState field | claim # |
|---|---|---|---|---|
| 0 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator0` | encoder count | `actuator_position_input.target_position_cnt[0]` | 231 |
| 1 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator0` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[0]` | 232 |
| 2 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator0` | encoder count | `target_position_cnt[0]` | 233 |
| 3 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator0` | 정격 전류의 0.1% | `target_effort_pct[0]` | 234 |
| 4 | `{side}_commanded/max_effort_pct.thumb_actuator0` | 정격 전류의 0.1% | `max_effort_pct[0]` | 235 |
| 5 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator1` | encoder count | `actuator_position_input.target_position_cnt[1]` | 236 |
| 6 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator1` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[1]` | 237 |
| 7 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator1` | encoder count | `target_position_cnt[1]` | 238 |
| 8 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator1` | 정격 전류의 0.1% | `target_effort_pct[1]` | 239 |
| 9 | `{side}_commanded/max_effort_pct.thumb_actuator1` | 정격 전류의 0.1% | `max_effort_pct[1]` | 240 |
| 10 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator2` | encoder count | `actuator_position_input.target_position_cnt[2]` | 241 |
| 11 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator2` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[2]` | 242 |
| 12 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator2` | encoder count | `target_position_cnt[2]` | 243 |
| 13 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator2` | 정격 전류의 0.1% | `target_effort_pct[2]` | 244 |
| 14 | `{side}_commanded/max_effort_pct.thumb_actuator2` | 정격 전류의 0.1% | `max_effort_pct[2]` | 245 |
| 15 | `{side}_commanded/controller_input_target_position_cnt.thumb_actuator3` | encoder count | `actuator_position_input.target_position_cnt[3]` | 246 |
| 16 | `{side}_commanded/controller_input_target_effort_pct.thumb_actuator3` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[3]` | 247 |
| 17 | `{side}_commanded/controller_output_target_position_cnt.thumb_actuator3` | encoder count | `target_position_cnt[3]` | 248 |
| 18 | `{side}_commanded/controller_output_target_effort_pct.thumb_actuator3` | 정격 전류의 0.1% | `target_effort_pct[3]` | 249 |
| 19 | `{side}_commanded/max_effort_pct.thumb_actuator3` | 정격 전류의 0.1% | `max_effort_pct[3]` | 250 |
| 20 | `{side}_commanded/controller_input_target_position_cnt.index_actuator1` | encoder count | `actuator_position_input.target_position_cnt[4]` | 251 |
| 21 | `{side}_commanded/controller_input_target_effort_pct.index_actuator1` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[4]` | 252 |
| 22 | `{side}_commanded/controller_output_target_position_cnt.index_actuator1` | encoder count | `target_position_cnt[4]` | 253 |
| 23 | `{side}_commanded/controller_output_target_effort_pct.index_actuator1` | 정격 전류의 0.1% | `target_effort_pct[4]` | 254 |
| 24 | `{side}_commanded/max_effort_pct.index_actuator1` | 정격 전류의 0.1% | `max_effort_pct[4]` | 255 |
| 25 | `{side}_commanded/controller_input_target_position_cnt.index_actuator2` | encoder count | `actuator_position_input.target_position_cnt[5]` | 256 |
| 26 | `{side}_commanded/controller_input_target_effort_pct.index_actuator2` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[5]` | 257 |
| 27 | `{side}_commanded/controller_output_target_position_cnt.index_actuator2` | encoder count | `target_position_cnt[5]` | 258 |
| 28 | `{side}_commanded/controller_output_target_effort_pct.index_actuator2` | 정격 전류의 0.1% | `target_effort_pct[5]` | 259 |
| 29 | `{side}_commanded/max_effort_pct.index_actuator2` | 정격 전류의 0.1% | `max_effort_pct[5]` | 260 |
| 30 | `{side}_commanded/controller_input_target_position_cnt.index_actuator3` | encoder count | `actuator_position_input.target_position_cnt[6]` | 261 |
| 31 | `{side}_commanded/controller_input_target_effort_pct.index_actuator3` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[6]` | 262 |
| 32 | `{side}_commanded/controller_output_target_position_cnt.index_actuator3` | encoder count | `target_position_cnt[6]` | 263 |
| 33 | `{side}_commanded/controller_output_target_effort_pct.index_actuator3` | 정격 전류의 0.1% | `target_effort_pct[6]` | 264 |
| 34 | `{side}_commanded/max_effort_pct.index_actuator3` | 정격 전류의 0.1% | `max_effort_pct[6]` | 265 |
| 35 | `{side}_commanded/controller_input_target_position_cnt.middle_actuator1` | encoder count | `actuator_position_input.target_position_cnt[7]` | 266 |
| 36 | `{side}_commanded/controller_input_target_effort_pct.middle_actuator1` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[7]` | 267 |
| 37 | `{side}_commanded/controller_output_target_position_cnt.middle_actuator1` | encoder count | `target_position_cnt[7]` | 268 |
| 38 | `{side}_commanded/controller_output_target_effort_pct.middle_actuator1` | 정격 전류의 0.1% | `target_effort_pct[7]` | 269 |
| 39 | `{side}_commanded/max_effort_pct.middle_actuator1` | 정격 전류의 0.1% | `max_effort_pct[7]` | 270 |
| 40 | `{side}_commanded/controller_input_target_position_cnt.middle_actuator2` | encoder count | `actuator_position_input.target_position_cnt[8]` | 271 |
| 41 | `{side}_commanded/controller_input_target_effort_pct.middle_actuator2` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[8]` | 272 |
| 42 | `{side}_commanded/controller_output_target_position_cnt.middle_actuator2` | encoder count | `target_position_cnt[8]` | 273 |
| 43 | `{side}_commanded/controller_output_target_effort_pct.middle_actuator2` | 정격 전류의 0.1% | `target_effort_pct[8]` | 274 |
| 44 | `{side}_commanded/max_effort_pct.middle_actuator2` | 정격 전류의 0.1% | `max_effort_pct[8]` | 275 |
| 45 | `{side}_commanded/controller_input_target_position_cnt.middle_actuator3` | encoder count | `actuator_position_input.target_position_cnt[9]` | 276 |
| 46 | `{side}_commanded/controller_input_target_effort_pct.middle_actuator3` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[9]` | 277 |
| 47 | `{side}_commanded/controller_output_target_position_cnt.middle_actuator3` | encoder count | `target_position_cnt[9]` | 278 |
| 48 | `{side}_commanded/controller_output_target_effort_pct.middle_actuator3` | 정격 전류의 0.1% | `target_effort_pct[9]` | 279 |
| 49 | `{side}_commanded/max_effort_pct.middle_actuator3` | 정격 전류의 0.1% | `max_effort_pct[9]` | 280 |
| 50 | `{side}_commanded/controller_input_target_position_cnt.ring_actuator1` | encoder count | `actuator_position_input.target_position_cnt[10]` | 281 |
| 51 | `{side}_commanded/controller_input_target_effort_pct.ring_actuator1` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[10]` | 282 |
| 52 | `{side}_commanded/controller_output_target_position_cnt.ring_actuator1` | encoder count | `target_position_cnt[10]` | 283 |
| 53 | `{side}_commanded/controller_output_target_effort_pct.ring_actuator1` | 정격 전류의 0.1% | `target_effort_pct[10]` | 284 |
| 54 | `{side}_commanded/max_effort_pct.ring_actuator1` | 정격 전류의 0.1% | `max_effort_pct[10]` | 285 |
| 55 | `{side}_commanded/controller_input_target_position_cnt.ring_actuator2` | encoder count | `actuator_position_input.target_position_cnt[11]` | 286 |
| 56 | `{side}_commanded/controller_input_target_effort_pct.ring_actuator2` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[11]` | 287 |
| 57 | `{side}_commanded/controller_output_target_position_cnt.ring_actuator2` | encoder count | `target_position_cnt[11]` | 288 |
| 58 | `{side}_commanded/controller_output_target_effort_pct.ring_actuator2` | 정격 전류의 0.1% | `target_effort_pct[11]` | 289 |
| 59 | `{side}_commanded/max_effort_pct.ring_actuator2` | 정격 전류의 0.1% | `max_effort_pct[11]` | 290 |
| 60 | `{side}_commanded/controller_input_target_position_cnt.ring_actuator3` | encoder count | `actuator_position_input.target_position_cnt[12]` | 291 |
| 61 | `{side}_commanded/controller_input_target_effort_pct.ring_actuator3` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[12]` | 292 |
| 62 | `{side}_commanded/controller_output_target_position_cnt.ring_actuator3` | encoder count | `target_position_cnt[12]` | 293 |
| 63 | `{side}_commanded/controller_output_target_effort_pct.ring_actuator3` | 정격 전류의 0.1% | `target_effort_pct[12]` | 294 |
| 64 | `{side}_commanded/max_effort_pct.ring_actuator3` | 정격 전류의 0.1% | `max_effort_pct[12]` | 295 |
| 65 | `{side}_commanded/controller_input_target_position_cnt.baby_actuator1` | encoder count | `actuator_position_input.target_position_cnt[13]` | 296 |
| 66 | `{side}_commanded/controller_input_target_effort_pct.baby_actuator1` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[13]` | 297 |
| 67 | `{side}_commanded/controller_output_target_position_cnt.baby_actuator1` | encoder count | `target_position_cnt[13]` | 298 |
| 68 | `{side}_commanded/controller_output_target_effort_pct.baby_actuator1` | 정격 전류의 0.1% | `target_effort_pct[13]` | 299 |
| 69 | `{side}_commanded/max_effort_pct.baby_actuator1` | 정격 전류의 0.1% | `max_effort_pct[13]` | 300 |
| 70 | `{side}_commanded/controller_input_target_position_cnt.baby_actuator2` | encoder count | `actuator_position_input.target_position_cnt[14]` | 301 |
| 71 | `{side}_commanded/controller_input_target_effort_pct.baby_actuator2` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[14]` | 302 |
| 72 | `{side}_commanded/controller_output_target_position_cnt.baby_actuator2` | encoder count | `target_position_cnt[14]` | 303 |
| 73 | `{side}_commanded/controller_output_target_effort_pct.baby_actuator2` | 정격 전류의 0.1% | `target_effort_pct[14]` | 304 |
| 74 | `{side}_commanded/max_effort_pct.baby_actuator2` | 정격 전류의 0.1% | `max_effort_pct[14]` | 305 |
| 75 | `{side}_commanded/controller_input_target_position_cnt.baby_actuator3` | encoder count | `actuator_position_input.target_position_cnt[15]` | 306 |
| 76 | `{side}_commanded/controller_input_target_effort_pct.baby_actuator3` | 정격 전류의 0.1% | `actuator_effort_input.target_effort_pct[15]` | 307 |
| 77 | `{side}_commanded/controller_output_target_position_cnt.baby_actuator3` | encoder count | `target_position_cnt[15]` | 308 |
| 78 | `{side}_commanded/controller_output_target_effort_pct.baby_actuator3` | 정격 전류의 0.1% | `target_effort_pct[15]` | 309 |
| 79 | `{side}_commanded/max_effort_pct.baby_actuator3` | 정격 전류의 0.1% | `max_effort_pct[15]` | 310 |

## 6. Timestamp state interfaces (2)

gpio component `{side}_timestamp`입니다. SDK가 state를 관측한 시각(`CLOCK_REALTIME`)을 초와 나노초로
나눈 값이고 `HandState.header.stamp`로 흐릅니다. `double` 하나가 nanosecond 전체를 담을 수 없어
둘로 나뉩니다. `sec`이 `0`이면 broadcaster가 자신의 update 시각으로 대체합니다.

| # | State interface | Unit | HandState field | claim # |
|---|---|---|---|---|
| 0 | `{side}_timestamp/sec` | s | `header.stamp.sec` | 311 |
| 1 | `{side}_timestamp/nanosec` | ns | `header.stamp.nanosec` | 312 |

## 7. Controller claims and references

`aidin_hand2_controllers`의 6개와 `aidin_hand2_examples`의 상위 skeleton 4개입니다. controller 이름은
`aidin_hand2_bringup/config/controllers.yaml`의 `{side}_` 관례입니다.

| Controller | Plugin | Chainable | Command claim | State claim | Reference export |
|---|---|---|---|---|---|
| `{side}_joint_position_controller` | `aidin_hand2_controllers/JointPositionController` | 예 | 17 | 0 | 16 |
| `{side}_joint_impedance_controller` | `aidin_hand2_controllers/JointImpedanceController` | 예 | 17 | 0 | 16 |
| `{side}_actuator_position_controller` | `aidin_hand2_controllers/ActuatorPositionController` | 예 | 17 | 0 | 16 |
| `{side}_actuator_effort_controller` | `aidin_hand2_controllers/ActuatorEffortController` | 예 | 17 | 0 | 16 |
| `{side}_hand_state_broadcaster` | `aidin_hand2_controllers/HandStateBroadcaster` | 아니오 | 0 | 313 | 0 |
| `{side}_diagnostics_broadcaster` | `aidin_hand2_controllers/DiagnosticsBroadcaster` | 아니오 | 0 | 39 | 0 |
| `{side}_joint_position_upper` | `aidin_hand2_examples/JointPositionUpperController` | 예 | 16 (아래 controller의 reference) | 0 | 16 |
| `{side}_joint_impedance_upper` | `aidin_hand2_examples/JointImpedanceUpperController` | 예 | 16 (아래 controller의 reference) | 0 | 16 |
| `{side}_actuator_position_upper` | `aidin_hand2_examples/ActuatorPositionUpperController` | 예 | 16 (아래 controller의 reference) | 0 | 16 |
| `{side}_actuator_effort_upper` | `aidin_hand2_examples/ActuatorEffortUpperController` | 예 | 16 (아래 controller의 reference) | 0 | 16 |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 아니오 | 0 | 모든 `position` | 0 |

### 7.1 Command controllers

command controller 넷은 `command_lock`과 자기 mode의 port 16개를 claim하고, reference 16개를 export합니다.
reference 하나가 hardware command interface 하나에 1:1로 기록됩니다. state interface는 claim하지 않고,
activate에서 reference를 NaN으로 채우며 state에서 seed하지 않습니다.

**JointPositionController**

| ref # | Reference interface | Hardware command interface | Unit |
|---|---|---|---|
| | | `{side}_hand_control/command_lock` | claim 전용 |
| 0 | `{side}_joint_position_controller/{side}_thumb_joint0/position` | `{side}_joint_position_command/target_position_rad.thumb_joint0` | rad |
| 1 | `{side}_joint_position_controller/{side}_thumb_joint1/position` | `{side}_joint_position_command/target_position_rad.thumb_joint1` | rad |
| 2 | `{side}_joint_position_controller/{side}_thumb_joint2/position` | `{side}_joint_position_command/target_position_rad.thumb_joint2` | rad |
| 3 | `{side}_joint_position_controller/{side}_thumb_joint3/position` | `{side}_joint_position_command/target_position_rad.thumb_joint3` | rad |
| 4 | `{side}_joint_position_controller/{side}_index_joint1/position` | `{side}_joint_position_command/target_position_rad.index_joint1` | rad |
| 5 | `{side}_joint_position_controller/{side}_index_joint2/position` | `{side}_joint_position_command/target_position_rad.index_joint2` | rad |
| 6 | `{side}_joint_position_controller/{side}_index_joint3/position` | `{side}_joint_position_command/target_position_rad.index_joint3` | rad |
| 7 | `{side}_joint_position_controller/{side}_middle_joint1/position` | `{side}_joint_position_command/target_position_rad.middle_joint1` | rad |
| 8 | `{side}_joint_position_controller/{side}_middle_joint2/position` | `{side}_joint_position_command/target_position_rad.middle_joint2` | rad |
| 9 | `{side}_joint_position_controller/{side}_middle_joint3/position` | `{side}_joint_position_command/target_position_rad.middle_joint3` | rad |
| 10 | `{side}_joint_position_controller/{side}_ring_joint1/position` | `{side}_joint_position_command/target_position_rad.ring_joint1` | rad |
| 11 | `{side}_joint_position_controller/{side}_ring_joint2/position` | `{side}_joint_position_command/target_position_rad.ring_joint2` | rad |
| 12 | `{side}_joint_position_controller/{side}_ring_joint3/position` | `{side}_joint_position_command/target_position_rad.ring_joint3` | rad |
| 13 | `{side}_joint_position_controller/{side}_baby_joint1/position` | `{side}_joint_position_command/target_position_rad.baby_joint1` | rad |
| 14 | `{side}_joint_position_controller/{side}_baby_joint2/position` | `{side}_joint_position_command/target_position_rad.baby_joint2` | rad |
| 15 | `{side}_joint_position_controller/{side}_baby_joint3/position` | `{side}_joint_position_command/target_position_rad.baby_joint3` | rad |

**JointImpedanceController**

| ref # | Reference interface | Hardware command interface | Unit |
|---|---|---|---|
| | | `{side}_hand_control/command_lock` | claim 전용 |
| 0 | `{side}_joint_impedance_controller/{side}_thumb_joint0/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint0` | rad |
| 1 | `{side}_joint_impedance_controller/{side}_thumb_joint1/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint1` | rad |
| 2 | `{side}_joint_impedance_controller/{side}_thumb_joint2/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint2` | rad |
| 3 | `{side}_joint_impedance_controller/{side}_thumb_joint3/position` | `{side}_joint_impedance_command/target_position_rad.thumb_joint3` | rad |
| 4 | `{side}_joint_impedance_controller/{side}_index_joint1/position` | `{side}_joint_impedance_command/target_position_rad.index_joint1` | rad |
| 5 | `{side}_joint_impedance_controller/{side}_index_joint2/position` | `{side}_joint_impedance_command/target_position_rad.index_joint2` | rad |
| 6 | `{side}_joint_impedance_controller/{side}_index_joint3/position` | `{side}_joint_impedance_command/target_position_rad.index_joint3` | rad |
| 7 | `{side}_joint_impedance_controller/{side}_middle_joint1/position` | `{side}_joint_impedance_command/target_position_rad.middle_joint1` | rad |
| 8 | `{side}_joint_impedance_controller/{side}_middle_joint2/position` | `{side}_joint_impedance_command/target_position_rad.middle_joint2` | rad |
| 9 | `{side}_joint_impedance_controller/{side}_middle_joint3/position` | `{side}_joint_impedance_command/target_position_rad.middle_joint3` | rad |
| 10 | `{side}_joint_impedance_controller/{side}_ring_joint1/position` | `{side}_joint_impedance_command/target_position_rad.ring_joint1` | rad |
| 11 | `{side}_joint_impedance_controller/{side}_ring_joint2/position` | `{side}_joint_impedance_command/target_position_rad.ring_joint2` | rad |
| 12 | `{side}_joint_impedance_controller/{side}_ring_joint3/position` | `{side}_joint_impedance_command/target_position_rad.ring_joint3` | rad |
| 13 | `{side}_joint_impedance_controller/{side}_baby_joint1/position` | `{side}_joint_impedance_command/target_position_rad.baby_joint1` | rad |
| 14 | `{side}_joint_impedance_controller/{side}_baby_joint2/position` | `{side}_joint_impedance_command/target_position_rad.baby_joint2` | rad |
| 15 | `{side}_joint_impedance_controller/{side}_baby_joint3/position` | `{side}_joint_impedance_command/target_position_rad.baby_joint3` | rad |

**ActuatorPositionController**

| ref # | Reference interface | Hardware command interface | Unit |
|---|---|---|---|
| | | `{side}_hand_control/command_lock` | claim 전용 |
| 0 | `{side}_actuator_position_controller/{side}_thumb_actuator0/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator0` | encoder count |
| 1 | `{side}_actuator_position_controller/{side}_thumb_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator1` | encoder count |
| 2 | `{side}_actuator_position_controller/{side}_thumb_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator2` | encoder count |
| 3 | `{side}_actuator_position_controller/{side}_thumb_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.thumb_actuator3` | encoder count |
| 4 | `{side}_actuator_position_controller/{side}_index_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.index_actuator1` | encoder count |
| 5 | `{side}_actuator_position_controller/{side}_index_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.index_actuator2` | encoder count |
| 6 | `{side}_actuator_position_controller/{side}_index_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.index_actuator3` | encoder count |
| 7 | `{side}_actuator_position_controller/{side}_middle_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.middle_actuator1` | encoder count |
| 8 | `{side}_actuator_position_controller/{side}_middle_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.middle_actuator2` | encoder count |
| 9 | `{side}_actuator_position_controller/{side}_middle_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.middle_actuator3` | encoder count |
| 10 | `{side}_actuator_position_controller/{side}_ring_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.ring_actuator1` | encoder count |
| 11 | `{side}_actuator_position_controller/{side}_ring_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.ring_actuator2` | encoder count |
| 12 | `{side}_actuator_position_controller/{side}_ring_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.ring_actuator3` | encoder count |
| 13 | `{side}_actuator_position_controller/{side}_baby_actuator1/position_cnt` | `{side}_actuator_position_command/target_position_cnt.baby_actuator1` | encoder count |
| 14 | `{side}_actuator_position_controller/{side}_baby_actuator2/position_cnt` | `{side}_actuator_position_command/target_position_cnt.baby_actuator2` | encoder count |
| 15 | `{side}_actuator_position_controller/{side}_baby_actuator3/position_cnt` | `{side}_actuator_position_command/target_position_cnt.baby_actuator3` | encoder count |

**ActuatorEffortController**

| ref # | Reference interface | Hardware command interface | Unit |
|---|---|---|---|
| | | `{side}_hand_control/command_lock` | claim 전용 |
| 0 | `{side}_actuator_effort_controller/{side}_thumb_actuator0/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator0` | 정격 전류의 0.1% |
| 1 | `{side}_actuator_effort_controller/{side}_thumb_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator1` | 정격 전류의 0.1% |
| 2 | `{side}_actuator_effort_controller/{side}_thumb_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator2` | 정격 전류의 0.1% |
| 3 | `{side}_actuator_effort_controller/{side}_thumb_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.thumb_actuator3` | 정격 전류의 0.1% |
| 4 | `{side}_actuator_effort_controller/{side}_index_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.index_actuator1` | 정격 전류의 0.1% |
| 5 | `{side}_actuator_effort_controller/{side}_index_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.index_actuator2` | 정격 전류의 0.1% |
| 6 | `{side}_actuator_effort_controller/{side}_index_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.index_actuator3` | 정격 전류의 0.1% |
| 7 | `{side}_actuator_effort_controller/{side}_middle_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.middle_actuator1` | 정격 전류의 0.1% |
| 8 | `{side}_actuator_effort_controller/{side}_middle_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.middle_actuator2` | 정격 전류의 0.1% |
| 9 | `{side}_actuator_effort_controller/{side}_middle_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.middle_actuator3` | 정격 전류의 0.1% |
| 10 | `{side}_actuator_effort_controller/{side}_ring_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.ring_actuator1` | 정격 전류의 0.1% |
| 11 | `{side}_actuator_effort_controller/{side}_ring_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.ring_actuator2` | 정격 전류의 0.1% |
| 12 | `{side}_actuator_effort_controller/{side}_ring_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.ring_actuator3` | 정격 전류의 0.1% |
| 13 | `{side}_actuator_effort_controller/{side}_baby_actuator1/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.baby_actuator1` | 정격 전류의 0.1% |
| 14 | `{side}_actuator_effort_controller/{side}_baby_actuator2/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.baby_actuator2` | 정격 전류의 0.1% |
| 15 | `{side}_actuator_effort_controller/{side}_baby_actuator3/effort_pct` | `{side}_actuator_effort_command/target_effort_pct.baby_actuator3` | 정격 전류의 0.1% |

### 7.2 HandStateBroadcaster

command interface는 claim하지 않고 state 313개를 다음 순서로 claim합니다. diagnostics 39개는 포함하지
않습니다. 각 이름은 2·3·5·6장 표의 `claim #`로 찾습니다.

| claim # | Block | Count | Section |
|---|---|---|---|
| 0–20 | joint position | 21 | [2.1](#21-joint-position-21) |
| 21–36 | actuator `position_cnt` | 16 | [2.2](#22-actuator-state-48) |
| 37–52 | actuator `velocity_rpm` | 16 | [2.2](#22-actuator-state-48) |
| 53–68 | actuator `current_ma` | 16 | [2.2](#22-actuator-state-48) |
| 69–153 | finger tactile | 85 | [3.1](#31-finger-tactile-85) |
| 154–211 | palm tactile | 58 | [3.2](#32-palm-tactile-58) |
| 212–214 | command echo scalar | 3 | [5.1](#51-scalars-3) |
| 215–230 | command echo joint target | 16 | [5.2](#52-controller-input-joint-target-16) |
| 231–310 | command echo per-actuator, 필드 5개 interleave | 80 | [5.3](#53-per-actuator-fields-80) |
| 311–312 | timestamp | 2 | [6](#6-timestamp-state-interfaces-2) |

발행 topic은 `/{side}_hand_state_broadcaster/hand_state`(`aidin_hand2_msgs/HandState`)입니다.

### 7.3 DiagnosticsBroadcaster

command interface는 claim하지 않고 diagnostics 39개만 [4.1](#41-hand-wide-fields-7) →
[4.2](#42-actuator-enabled-16) → [4.3](#43-actuator-fault-16) 순서로 claim합니다. 발행 topic은
`/{side}_diagnostics_broadcaster/hand_diagnostics`(`aidin_hand2_msgs/HandDiagnostics`)입니다.

### 7.4 Upper controller skeletons

skeleton 넷은 `target_controller` parameter가 가리키는 command controller의 reference 16개를 command
interface로 claim하고, 같은 suffix를 자기 이름으로 다시 export합니다. claim 이름은
`{target_controller}/{side}_<name>/<suffix>`, export 이름은 `{upper_controller}/{side}_<name>/<suffix>`이고
suffix는 [7.1](#71-command-controllers)의 reference와 같습니다. state interface는 claim하지 않고 `HandState`
topic을 구독합니다.

## 8. Counts by backend

| Kind | Group | Count | Robot hand | Isaac | Mock |
|---|---|---|---|---|---|
| Command | `command_lock` | 1 | O | O | O |
| Command | mode port 4개 × 16 | 64 | O | O | O |
| Command | **합계** | **65** | 65 | 65 | 65 |
| State | joint position | 21 | O | O | O |
| State | actuator state | 48 | O | O | O. velocity·current는 `0` |
| State | finger tactile | 85 | O | O | X |
| State | palm tactile | 58 | O | O | X |
| State | diagnostics | 39 | O | O | X |
| State | command echo | 99 | O | O | X |
| State | timestamp | 2 | O | O | X |
| State | **합계** | **352** | 352 | 352 | 69 |
| Reference | command controller 4개 × 16 | 64 | O | O | O |
| Service | `~/run` `~/stop` `~/home` `~/reconnect` | 4 | O | X | X |

state interface 352개는 세 backend의 plugin이 `export_state_interfaces()`에서 모두 만듭니다. URDF의 매크로는
그중 actuator·joint 69개, tactile 143개, diagnostics 39개의 251개를 함께 선언하고, mock에서는 tactile과
diagnostics를 선언하지 않습니다.

## 9. Runtime check

실행 중인 system의 interface와 claim을 확인합니다.

```bash
ros2 control list_hardware_interfaces            # command·state interface 전체
ros2 control list_controllers -v                 # controller별 claim과 reference
ros2 control list_hardware_interfaces | grep left_commanded
```

이 문서의 이름이 위 출력과 다르면 소스가 기준입니다. export 순서는 `aidin_hand2_hardware/src/*.cpp`,
정적 선언은 `aidin_hand2_description/ros2_control/aidin_hand2.ros2_control.xacro`, claim과 reference는
`aidin_hand2_controllers/src/*.cpp`에 있습니다.

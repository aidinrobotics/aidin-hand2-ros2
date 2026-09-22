<div align="center">

<a href="https://www.aidinrobotics.co.kr/"><img height="240" src="docs/assets/aidin_hand2_logo.webp" alt="AIDIN Hand Gen2 — AIDIN Robotics"></a>

<h1>AIDIN Hand Gen2 ROS 2</h1>

A `ros2_control` wrapper for controlling AIDIN Hand Gen2 from ROS 2. Send targets to a controller,
read state topics, and use services for homing, stopping and recovery. The wrapper provides a mock
that runs without the robot hand, plus URDF and launch configuration for integration into your robot.

[![version](https://img.shields.io/badge/version-0.6.0-blue)](CHANGELOG.md) [![SDK](https://img.shields.io/badge/SDK-0.6.x-blue)](aidin_hand2.repos) [![ROS 2](https://img.shields.io/badge/ROS%202-Humble-brightgreen)](#system-requirements)

[Install](docs/ko/03_installation.md) | [Documentation](#documentation) | [Changelog](CHANGELOG.md) | [Official Site](https://www.aidinrobotics.co.kr/) | English | [한국어](README.ko.md)

</div>

## System requirements

We verify that the wrapper builds and runs on the configuration below.

| Component | Requirement |
|---|---|
| Operating System | Ubuntu 22.04 |
| ROS 2 | Humble |
| Control framework | `ros2_control` |
| SDK | `aidin_hand2` 0.6.x ([`aidin_hand2.repos`](aidin_hand2.repos)) |
| CAN interface | For the robot hand: USB CAN-FD adapter (SocketCAN), 1 Mbit/s nominal, 5 Mbit/s data phase |

## Architecture

Built on [ros2_control](https://control.ros.org/humble/index.html), this wrapper provides controllers for commanding the robot hand and broadcasters for observing its state.

![AIDIN Hand Gen2 ROS 2 architecture](docs/assets/aidin_hand2_ros2_architecture.webp)

A user node sends a command by one of two paths.

- Publish `sensor_msgs/JointState` directly to a command controller's `~/cmd` topic. The name
  matching rule, the field read and the units are in
  [2. Command message](aidin_hand2_msgs/README.ko.md#2-command-message).
- Write a user controller and publish to the topic and message it defines. The user controller
  processes the command and writes the targets to the command controller's reference interfaces, and
  the command controller enters chained mode and stops reading its own `~/cmd` topic. The reference
  names and the switching order are in
  [6. Chaining](aidin_hand2_controllers/README.ko.md#6-chaining).

On either path, one command controller is active per robot hand.

Because the user controller runs inside the controller_manager, it can read actuator positions, joint
angles and tactile values through state interfaces in the same cycle, without a topic. Observation and
target computation close within one 500 Hz cycle, so the control loop has no topic round trip. The
`aidin_hand2_examples` skeletons are this template, with a single line that scales the input by zero
where the algorithm goes. The state interfaces they can read and the steps to run one are in
[Chainable controller examples](aidin_hand2_examples/EXAMPLE.md).

A user node reads state through the `/joint_states`, `~/hand_state` and `~/hand_diagnostics` topics.
The `~` denotes the name of the node providing a topic or service; for example, `~/cmd` becomes
`/left_joint_position_controller/cmd`. Effort limits, filters and gains are ROS parameters of the
hardware node.

## Terms

The ros2_control terms this documentation uses throughout. If they are new to you, read them here
first. The full definitions are in the [ros2_control documentation](https://control.ros.org/humble/index.html).

| Term | Meaning |
|---|---|
| controller_manager | The node that loads, unloads and runs controllers every cycle. The launch files start it |
| hardware component | The plugin that talks to the hardware, reads state and writes targets. The kinds are System, Actuator and Sensor; the wrapper provides one System per robot hand, which calls the SDK and is named `{side}_hand_control` |
| hardware node | The node the hardware component starts. It offers the `~/run`, `~/stop`, `~/home` and `~/reconnect` services and the effort, filter and gain parameters |
| controller | Runs every cycle on top of the hardware component to produce targets or publish observations |
| broadcaster | A controller that produces no target and only publishes observations |
| `unconfigured` · `inactive` · `active` | The ROS 2 managed node states, held separately by controllers and by hardware components. An `active` controller runs every cycle; an `active` hardware component has torque on the drives so the hand can move. `inactive` means loaded but neither, and `unconfigured` comes before it |
| command interface | Where a controller writes a target. Only one controller can claim it at a time |
| state interface | Where observations are read. Several readers can share one |
| reference interface | The input a command controller opens to an upper controller. A target written here is used instead of the controller's own topic |
| chained mode | The state in which a command controller takes its target from its reference interfaces |
| spawner | The executable that loads a controller into the controller_manager. The launch files run one per controller |

`active` applies to both controllers and hardware components, and neither is the lifecycle the SDK
reports. The table that separates the three is in
[1. Lifecycle](docs/ko/05_control_guide.md#1-lifecycle).

## Getting started

Choose a path below. Installation and mock execution do not require the robot hand or a CAN adapter.
The detailed guides are currently in Korean.

| Goal | Reading order |
|---|---|
| Try without the robot hand | [Installation](docs/ko/03_installation.md) → [1. Mock](docs/ko/04_bringup.md#1-mock) |
| Run the robot hand | [Installation](docs/ko/03_installation.md) → [Real-time kernel setup](docs/ko/01_real_time_kernel_setup.md) and [CAN-FD setup](docs/ko/02_can_fd_setup.md) → [2. Robot hand](docs/ko/04_bringup.md#2-robot-hand) |
| Control a running robot hand | [Control guide](docs/ko/05_control_guide.md) — Check state → home → send targets → observe and stop |
| Integrate into your robot | Verify standalone [Bringup](docs/ko/04_bringup.md) → [Add to your robot](aidin_hand2_bringup/README.ko.md#7-add-to-your-robot) |

The joint names this documentation uses and each joint's rotation direction can be
checked by moving them in the viewer below.

<div align="center">

<a href="https://aidinrobotics.github.io/aidin-hand2-ros2/"><img src="docs/assets/viewer_preview.webp" alt="AIDIN Hand Gen2 joint viewer"></a>

</div>

## Documentation

Follow the common guides for installation, the first run and control. Package READMEs provide configuration and detailed references (in Korean).

### User guides

- [Installation](docs/ko/03_installation.md) — SDK installation and wrapper build
- [Real-time kernel setup](docs/ko/01_real_time_kernel_setup.md) — PREEMPT_RT kernel and real-time permissions
- [CAN-FD setup](docs/ko/02_can_fd_setup.md) — CAN interface setup and receive checks
- [Bringup](docs/ko/04_bringup.md) — First run with mock or robot hand, homing, first command, stop
- [Control guide](docs/ko/05_control_guide.md) — Lifecycle, topic commands, service calls, monitoring, stop, recovery and QoS
- [Troubleshooting](docs/ko/06_troubleshooting.md) — Build, launch, control and communication problems

### Packages

| Package | Guide |
|---|---|
| [aidin_hand2_bringup](aidin_hand2_bringup/README.ko.md) | Launch arguments, defaults, configuration files, adding the robot hand to your robot |
| [aidin_hand2_description](aidin_hand2_description/README.ko.md) | xacro and URDF file locations, macro calls and arguments, RViz preview, the plain URDF |
| [aidin_hand2_controllers](aidin_hand2_controllers/README.ko.md) | Controller selection, commands, switching, chaining, YAML and parameters |
| [aidin_hand2_hardware](aidin_hand2_hardware/README.ko.md) | Homing, stop and recovery services, runtime tuning and initial values |
| [aidin_hand2_msgs](aidin_hand2_msgs/README.ko.md) | Command and state message fields, units, joint and actuator array order |
| [aidin_hand2_examples](aidin_hand2_examples/README.ko.md) | Upper-controller skeletons, source and config files, algorithm integration |

## Related repositories

- [aidin-hand2-sdk](https://github.com/aidinrobotics/aidin-hand2-sdk) — the C++ SDK

Three of the SDK documents matter to a reader of this wrapper.

- [C++ guide](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/en/07_cpp_usage_guide.md) — the lifecycle, homing and command semantics that this wrapper exposes
- [Workspace limits](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/en/14_workspace_limits.md) — the reachable range that the SDK projects a joint target into
- [Error messages](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/en/15_error_messages.md) — the text that a failed service returns

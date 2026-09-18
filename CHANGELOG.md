# Changelog

All notable changes to the AIDIN Hand Gen2 ROS 2 wrapper are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this
project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- **Real-time kernel setup and CAN-FD setup are part of this repository's documentation.** They
  carry the same procedure as the SDK documents, so a reader who installs only the wrapper does
  not switch repositories. The last chapter of the CAN-FD document names the interface through the
  launch argument and the xacro macro instead of `HandConfig`.

### Changed

- **Breaking: the four command controllers take `sensor_msgs/JointState` on `~/cmd`.** A command
  used to be a bare `float64[16]` in a message of this repository, so nothing standard could
  publish it and the reader had to know the wrapper's joint order. The controllers now match
  `name` against the URDF names, `{side}_thumb_joint0` to `{side}_baby_joint3` for the joint
  controllers and `{side}_thumb_actuator0` to `{side}_baby_actuator3` for the actuator controllers,
  in any order. Joint position, joint impedance and actuator position read `position`, actuator
  effort reads `effort`, in rad, encoder count and 0.1 % of rated current as before. A name the
  controller does not own is ignored and a name left out is not commanded this cycle, so a
  `joint_state_publisher_gui` remapped to `~/cmd` drives the mock directly. The first message after
  activation still has to name all 16. An empty `name` with exactly 16 values is taken in the
  wrapper's joint order, as the old messages were. A message whose `name` differs in length from
  the field read or repeats a name is dropped with a warning and the controller stays active.
  Publish `sensor_msgs/msg/JointState` to `/{side}_<mode>_controller/cmd`.
- **The upper controller skeletons take a `~/cmd` `sensor_msgs/JointState` and read state through
  state interfaces.** They had no command input and subscribed to the `HandState` topic, which put a
  topic round trip inside the control loop and needed `aidin_hand2_msgs`. Each skeleton now
  subscribes its own `~/cmd` with the command controllers' name matching, claims the joint,
  actuator and, with the new `read_tactile` parameter, tactile state interfaces, copies them into
  member arrays every update, and forwards the input scaled by zero where the algorithm goes. The
  `hand_state_topic` parameter is gone; `read_tactile` defaults to false because the mock exports
  no tactile interface. `aidin_hand2_examples` no longer depends on `aidin_hand2_msgs`; a skeleton
  needs only `sensor_msgs` and the ros2_control packages, so it can be copied into your own package
  as it is.
- **`CommandState` carries the command echo as flat arrays.** `joint_position_input` is
  `joint_position_input_rad`, `joint_impedance_input` is `joint_impedance_input_rad`,
  `actuator_position_input` is `actuator_position_input_cnt` and `actuator_effort_input` is
  `actuator_effort_input_pct`, each `float64[16]`. Readers of `hand_state.command_state` drop one
  level of nesting.
- **The documentation is organized by ROS 2 interface kind.** Installation, Bringup and
  Integration are procedures in the order a reader does them. Controllers, Topics, Services,
  Parameters and Launch files each describe one kind of interface. Troubleshooting is the
  appendix. The former Interface reference, Bringup example and Operations
  documents are folded into these. The startup gate, supervisor policy and production checklist
  sections are gone, because they described the reader's system rather than the wrapper. The facts
  they carried, that there is no command age watchdog and no composite ready flag, are stated with
  the topics.
- **Stale statements are corrected.** The default `cutoff_freq` is 10 Hz, not 60. The mock launch
  starts RViz by default. A controller fills its references with NaN at activation and seeds
  nothing from state. The README no longer points at SDK documents that do not exist.
- `README.ko.md` uses the same English headings as `README.md` and the SDK's Korean README.
- `aidin_hand2.repos` pins SDK v0.5.2, the release that the linked SDK documents describe. v0.5.1
  had no kinematics choice at configure time.

### Removed

- **Breaking: the `JointPositionCommand`, `JointImpedanceCommand`, `ActuatorPositionCommand` and
  `ActuatorEffortCommand` messages are gone.** Their only reader was the command topics, which now
  take `sensor_msgs/JointState`. `aidin_hand2_msgs` keeps `HandState`, `CommandState` and
  `HandDiagnostics`. A node that only published commands no longer needs this package.
- **Breaking: the Isaac Sim backend is removed.** The `AidinHand2IsaacSystemInterface` plugin,
  `aidin_hand2_isaac.launch.py`, `controllers_isaac.yaml` and the `use_isaac` and `isaac_*`
  arguments of the xacro macro are gone, so drop those arguments from your URDF and your launch
  commands. The robot hand and the mock remain, and `use_mock` selects between them. We plan to add
  the backend again once the wrapper framework is stable.
- **The Interface matrix document.** The wrapper exposes its interfaces at the controller level. A
  command enters through a command controller's `~/cmd` topic or its reference interfaces, and
  the hardware rejects any claim of a command port that is not the whole port together with
  `command_lock`. State is read from the topics of the three broadcasters, whose message fields are
  in the Topics document. No user path therefore reads the 65 hardware command interfaces or the 352
  state interfaces by name, and the document that enumerated them also still listed the interfaces
  that 0.4.0 removed. The interface groups, their counts per backend and a diagram of who exports and
  who claims each kind are in the Controllers document, section 1.3, and the reference interface
  names are in section 4.2. A controller of your own that claims a state interface directly gets the
  names from those patterns, the joint and actuator order in section 2 and
  `ros2 control list_hardware_interfaces` on the running system.

## [0.5.0] - 2026-09-10

Requires SDK 0.5.x, whose lifecycle now reports what the hand reached rather than what was asked
of it. Glove teleop is gone. The URDF joint limits were wrong and are corrected. Read the
**Breaking** entries before upgrading.

### Removed

- **Breaking.** Glove teleop. The sources, the config, the scripts and the `use_glove` wiring in
  `aidin_hand2_bringup` are removed. `aidin_hand2_examples` no longer builds conditionally on
  `manus_ros2_msgs`, so it always builds.
- **Breaking.** The controllers no longer publish `/diagnostics`. Read the hand's health from
  `~/hand_diagnostics` instead, which is the single publisher now.

### Changed

- **Breaking: requires SDK 0.5.x.** `find_package(aidin_hand2 0.5 REQUIRED)` fails at configure
  time against 0.4. The compatibility policy is `SameMinorVersion`.
- **Breaking: `~/reconnect` fails outside a fault.** The service returns the SDK's message, and
  the SDK now allows `reconnect()` only in `Faulted`. Rebuilding a healthy link is no longer
  possible through this service; deactivate and activate the hardware component instead.
- **Breaking: `~/run` blocks until the drives confirm Operation Enabled**, up to 4000 ms, and the
  service reports the failure when they do not. It used to return as soon as the SDK recorded the
  request, so a success now means the hardware got there.
- **Breaking: `~/stop` reports failure when the drives do not confirm the quick stop.** The
  lifecycle stays `Running` in that case, which means the actuators may still hold the last
  command. Power the hand off rather than retrying.
- **A command with a non-finite value is dropped instead of raising an error.** The previous
  command keeps going out and the SDK increments its own counter, so a bad target no longer shows
  up as a failed write. Check the value before you publish it.
- **`aidin_hand2` is no longer declared in any `package.xml`.** It is neither a ROS package nor a
  rosdep key, so no resolver could act on the name and every `rosdep install` needed
  `--skip-keys "aidin_hand2"` to get past it. The requirement lives in each `CMakeLists.txt`,
  where `find_package(aidin_hand2 0.5 REQUIRED)` enforces it at configure time.
- Eigen is no longer looked for in `aidin_hand2_hardware`. The SDK stopped exposing it as a
  dependency, because it is compiled into the prebuilt kinematics library.
- The Isaac topic parameters are always created rather than conditionally, which makes them
  consistent with the other arguments. The defaults live in the xacro.
- The joint and actuator count constants come from the SDK's `description.hpp` instead of being
  declared in `aidin_hand2_controllers` and `aidin_hand2_examples`.

### Added

- `use_left_hand` and `use_right_hand` arguments on the mock launch, with the matching `right_*`
  entries in `controllers_mock.yaml`. The defaults are aligned across the xacro, the launch file
  and the YAML.

### Fixed

- **The URDF joint limits disagreed with the SDK.** The 16 active limits now match the measured
  bounds in the SDK's `joint_clamp.cpp`, and the five passive `q4` limits are derived from the
  `q3` limit through the four-bar linkage. A planner reading the URDF was allowed to ask for
  poses the SDK clamps.

### Documentation

- **The install guide duplicated the SDK's build and install commands, and the copy had gone stale
  in three places.** It passed `-DAIDIN_HAND2_BUILD_WEB_BRIDGE=OFF`, which the SDK does not
  define, installed `libeigen3-dev`, which the SDK stopped needing, and omitted `sudo ldconfig`,
  without which the wrapper builds and then fails to load. The SDK section links the SDK document
  instead and keeps only the check that `find_package` will succeed, so one procedure has one
  home. The thumb ball screw lead is called out there, because building for the wrong lead moves
  two thumb actuators by twice or half the commanded distance.
- The prerequisites and workspace layout sections are gone. The list repeated what the following
  sections install, and the tree was the standard colcon layout. What the reader needs from them,
  the ROS 2 install link and the `COLCON_IGNORE` that keeps colcon out of an SDK clone under
  `src/`, moved to the header and the SDK section. The guide is three sections.
- `rosdep install` no longer carries `--recursive`, which is not an option and fails before rosdep
  does anything, nor `--skip-keys "aidin_hand2"`, which the manifests no longer need. The package
  config check reads the installed version rather than being a bare `test -f`, which printed
  nothing either way.
- **The service contract table described three of the four services incorrectly.** `~/run` and
  `~/stop` block until the drives confirm, and the table now names both timeouts. `~/reconnect`
  succeeds only while the lifecycle is `Faulted`, and manual recovery names the hardware component
  transition for the case where the link is alive.
- **Troubleshooting covers the two failures a first build actually hits.** `ament_cmake` not found
  means the ROS 2 environment was not sourced in that shell, and an SDK in two prefixes means
  `find_package` may pick either one — with the libraries of two releases ending up in one
  process. Section 3 no longer documents a rosdep failure that cannot happen any more.
- Both READMEs carry the centered header and the logo, matching the SDK. The three package
  READMEs and `EXAMPLE.md` are in English, `aidin_hand2_bringup` documents the Isaac launch file
  and its config, and `EXAMPLE.md` no longer points at a TODO that the code had moved past.
- `docs/comment_style.md` records the comment rules, which are the SDK's, and they are applied
  across `aidin_hand2_msgs`, `aidin_hand2_hardware` and `aidin_hand2_controllers`.
- `08_interface_matrix` drops the controller table row and section 8.7 that described glove
  teleop, and 8.8 becomes 8.7.

## [0.4.0] - 2026-08-26

### Added

- Runtime tuning parameters on the hardware component's own node (`<side>_hand_control`):
  `max_effort`, `joint_position_controller.{filter_enabled,cutoff_freq,deadband}` and
  `joint_impedance_controller.{stiffness,damping}`. Array parameters take 1 value (shared by every
  actuator) or 16 (per actuator). Set them at launch through `controllers.yaml` or at runtime with
  `ros2 param set`; a change applies on the next cycle.

### Removed

- **Breaking.** `~/set_max_effort`. The `max_effort` parameter replaces it and takes a per-actuator
  array, so one value no longer has two write paths.
- **Breaking.** `speed_rad_s` and the 32 impedance gain command interfaces. The hardware command
  contract goes from 98 resources to 65, `JointPositionController` exports 16 references instead of
  17, and `JointImpedanceController` 16 instead of 48.
- **Breaking.** `speed_rad_s` from `JointPositionCommand.msg`, and `stiffness`/`damping` from
  `JointImpedanceCommand.msg`. `CommandState` nests both, so its echo shrinks with them.
- The `speed_rad_s` parameter of `JointPositionController` and the `stiffness`/`damping` parameters
  of `JointImpedanceController`.

### Changed

- Requires SDK 0.4.x.
- The mock and Isaac backends apply a joint position target immediately. Their speed rate limit went
  with `speed_rad_s`, and neither emulates the SDK's filter, so a target reaches them unshaped.

## [0.3.2] - 2026-08-26

### Changed

- Requires SDK 0.3.1. `find_package(aidin_hand2 0.3.1 REQUIRED)` fails at configure
  time against 0.3.0, which lacks the hold below.
- `~/run` after `~/stop` no longer leaves the hand torque-free. The hold is the SDK's:
  `run()` commands the observed pose, and the hardware sends nothing until a
  controller does.
- With `auto_home` true and homing incomplete, a run triggers homing again. The
  trigger was armed only on activation and after `~/reconnect`, so a `~/stop` during
  homing needed `~/home` or `~/reconnect` to recover.

## [0.3.1] - 2026-08-25

### Added

- `AidinHand2IsaacSystemInterface` — an Isaac Sim bridge hardware that talks to the
  simulator over ROS 2 topics only. Its command contract (98) and state contract
  (tactile and diagnostics included) match the real hardware, so
  `HandStateBroadcaster` and `DiagnosticsBroadcaster` attach unchanged.
- `use_isaac` (plus `isaac_topic_prefix`, `isaac_joint_state_topic`,
  `isaac_joint_command_topic`, `isaac_tactile_prefix`, `isaac_state_timeout`) on the
  `aidin_hand2_ros2_control` xacro macro. The backend resolves as
  `use_isaac` > `use_mock` > real CAN.
- `aidin_hand2_bringup`: `aidin_hand2_isaac.launch.py` and `config/controllers_isaac.yaml`.

### Removed

- Jazzy support. Humble is the only supported distro; the `jazzy` branch is gone.

## [0.3.0] - 2026-08-24

### Changed

- Requires SDK 0.3.x.
- `HandDiagnostics.homed` replaced by `homing_state`.
- Diagnostics state interface `homed` renamed to `homing_state`.
- `NaN` in a command interface means "no command"; the hardware then holds the
  last command.
- A partial `NaN` is an axis the upstream does not own, filled from its last
  commanded value.
- Controllers write a command only on the cycle they receive one.
- Controllers claim no state interfaces and seed no target on activation.
- Controllers drop their command subscription while in chained mode.

### Fixed

- The hand no longer returns to its pre-homing pose after homing.

## [0.2.0]

### Changed

- Requires SDK 0.2.x. `find_package(aidin_hand2 0.2 REQUIRED)` now fails at
  configure time on a version mismatch instead of at compile time.
- Followed the SDK's five-state `HandLifecycle`: `Connecting` and
  `FaultStopping` no longer exist, and recovery is decided by `Faulted` alone.

### Fixed

- A communication loss no longer takes down other hardware components. The
  non-`Running` gate in `write()` was conditional on `auto_reconnect`, so with
  it off `set_command()` threw in `Faulted` and the resulting `ERROR` pushed
  ros2_control to unconfigure this component — along with every controller
  claiming its interfaces, such as `joint_state_broadcaster`.

## [0.1.0]

- Initial release.

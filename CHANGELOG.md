# Changelog

All notable changes to the AIDIN Hand Gen2 ROS 2 wrapper are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this
project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

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

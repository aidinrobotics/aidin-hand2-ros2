# Changelog

All notable changes to the AIDIN Hand Gen2 ROS 2 wrapper are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this
project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Changed

- `HandDiagnostics.homed` → `homing_state`
  (`NotRun`/`Succeeded`/`InProgress`/`Failed`). The diagnostics state interface
  `homed` became `homing_state`.
- `NaN` in a command interface means "no command": the hardware skips
  `set_command()` and the SDK holds its last command. A partial `NaN` is an axis
  the upstream does not own, filled from what was last commanded for it.
- Controllers write a command only on the cycle they receive one. They claim no
  state interfaces and seed no target on activation.
- Controllers drop their command subscription while in chained mode.

### Fixed

- The hand returned to its pre-homing pose once homing finished. `~/home` and
  `~/stop` followed by `~/run` had the same problem.

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

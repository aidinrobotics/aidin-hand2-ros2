// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/description.hpp>

namespace aidin_hand2
{

// -------------------------------- Lifecycle ---------------------------------

// Connected observes without torque, Running is under control
// Stopped comes from stop(), Faulted from a fault the SDK caught
enum class HandLifecycle {
  Disconnected,
  Connected,
  Running,
  Stopped,
  Faulted,
};

// HandLifecycle name for logs
[[nodiscard]] constexpr const char* to_string(HandLifecycle lifecycle) noexcept
{
  switch (lifecycle) {
    case HandLifecycle::Disconnected: return "Disconnected";
    case HandLifecycle::Connected: return "Connected";
    case HandLifecycle::Running: return "Running";
    case HandLifecycle::Stopped: return "Stopped";
    case HandLifecycle::Faulted: return "Faulted";
  }
  return "Disconnected";
}

// ---------------------------------- Homing ----------------------------------

enum class HomingState {
  NotRun     = 0,
  Succeeded  = 1,
  InProgress = 2,
  Failed     = 3,
};

// HomingState name for logs
[[nodiscard]] constexpr const char* to_string(HomingState state) noexcept
{
  switch (state) {
    case HomingState::NotRun: return "NotRun";
    case HomingState::Succeeded: return "Succeeded";
    case HomingState::InProgress: return "InProgress";
    case HomingState::Failed: return "Failed";
  }
  return "NotRun";
}

// ------------------------------ Actuator fault ------------------------------

// Drive error code 0x603F from the motor drive manual
enum class ActuatorFault : std::uint16_t {
  None = 0x0000,
  OverCurrentError = 0x2310,
  OverVoltageError = 0x3210,
  UnderVoltageError = 0x3220,
  OverTemperatureError = 0x4210,
  CurrentDetectionError = 0x5210,
  SpeedError = 0x7310,
  CommunicationError = 0x7500,
  FollowingError = 0x8611,
  HallSensorError = 0xFF01,
  OverLoadError = 0xFF02,
  PositiveLimitSwitchError = 0xFF03,
  NegativeLimitSwitchError = 0xFF04,
  EmergencySwitchError = 0xFF05,
  Sto1Error = 0xFF06,
  Sto2Error = 0xFF07,
  SerialEncoderChannelAError = 0xFF24,
  SerialEncoderChannelADisconnectedError = 0xFF25,
  SerialEncoderChannelBError = 0xFF26,
  SerialEncoderChannelBDisconnectedError = 0xFF27,
};

// ActuatorFault name for logs
[[nodiscard]] constexpr const char* to_string(ActuatorFault fault) noexcept
{
  switch (fault) {
    case ActuatorFault::None: return "";
    case ActuatorFault::OverCurrentError: return "OverCurrentError";
    case ActuatorFault::OverVoltageError: return "OverVoltageError";
    case ActuatorFault::UnderVoltageError: return "UnderVoltageError";
    case ActuatorFault::OverTemperatureError: return "OverTemperatureError";
    case ActuatorFault::CurrentDetectionError: return "CurrentDetectionError";
    case ActuatorFault::SpeedError: return "SpeedError";
    case ActuatorFault::CommunicationError: return "CommunicationError";
    case ActuatorFault::FollowingError: return "FollowingError";
    case ActuatorFault::HallSensorError: return "HallSensorError";
    case ActuatorFault::OverLoadError: return "OverLoadError";
    case ActuatorFault::PositiveLimitSwitchError: return "PositiveLimitSwitchError";
    case ActuatorFault::NegativeLimitSwitchError: return "NegativeLimitSwitchError";
    case ActuatorFault::EmergencySwitchError: return "EmergencySwitchError";
    case ActuatorFault::Sto1Error: return "Sto1Error";
    case ActuatorFault::Sto2Error: return "Sto2Error";
    case ActuatorFault::SerialEncoderChannelAError: return "SerialEncoderChannelAError";
    case ActuatorFault::SerialEncoderChannelADisconnectedError: return "SerialEncoderChannelADisconnectedError";
    case ActuatorFault::SerialEncoderChannelBError: return "SerialEncoderChannelBError";
    case ActuatorFault::SerialEncoderChannelBDisconnectedError: return "SerialEncoderChannelBDisconnectedError";
  }
  return "Unknown";
}

// ------------------------------- Observation --------------------------------

// Index is actuator 0..15
struct ActuatorState {
  std::array<std::int32_t, kActuatorCount> position_count{};
  std::array<std::int32_t, kActuatorCount> velocity_rpm{};
  std::array<std::int16_t, kActuatorCount> current_mA{};
};

// Straight from the drive statusword and error code, exposed through Diagnostics
struct ActuatorHealth {
  std::array<bool, kActuatorCount> enabled{};
  std::array<ActuatorFault, kActuatorCount> fault{};
};

// Joint angles, velocities and estimated torques from the actuator readings
// Effort is from current_mA and the motor torque constant only
// The passive q4 of each digit follows its q3 in velocity and is NaN in effort
struct JointState {
  std::array<double, kJointCount> position_rad{};
  std::array<double, kJointCount> velocity_rad_s{};
  std::array<double, kJointCount> effort_Nm{};
};

// Raw sensor values, no unit and no normalization
// After Hand::set_tactile_bias(), the difference from the bias
struct TactileState {
  std::array<std::array<std::int32_t, kTactileTaxelsPerFinger>, kFingerCount> fingers{};
  std::array<std::int32_t, kPalmTactileCount> palm{};
};

// ------------------------------- Command echo -------------------------------

struct ActuatorPositionSetpoint {
  std::array<double, kActuatorCount> target_position_cnt{};
};

struct ActuatorEffortSetpoint {
  std::array<double, kActuatorCount> target_effort_pct{};
};

using ControllerOutput =
    std::variant<std::monostate, ActuatorPositionSetpoint, ActuatorEffortSetpoint>;

// Command path taken this cycle, None means no TX
enum class CommandSource {
  None,
  Controller,
  QuickStop,
  Homing,
};

// controller_output is the controller output that reaches the motors
struct CommandedState {
  ControllerCommand controller_input{Idle{}};
  ControllerOutput controller_output{};
  CommandSource selected_source{CommandSource::None};
};

// -------------------------------- Hand state --------------------------------

// timestamp is CLOCK_REALTIME ns, stamped once per cycle whether or not a frame arrived
// It stops advancing in Faulted, and 0 = unset
struct HandState {
  std::int64_t timestamp{};
  ActuatorState actuators{};
  JointState joints{};
  TactileState tactile{};
  CommandedState commanded{};
};

}  // namespace aidin_hand2

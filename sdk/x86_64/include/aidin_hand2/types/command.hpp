// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <type_traits>
#include <variant>

#include <aidin_hand2/export.hpp>
#include <aidin_hand2/types/description.hpp>

namespace aidin_hand2
{

// ------------------------------- Command mode -------------------------------

enum class CommandMode {
  Idle,
  JointPosition,
  JointImpedance,
  ActuatorPosition,
  ActuatorEffort,
};

// --------------------------------- Commands ---------------------------------

struct Idle {};

// Target in rad, clamp() pulls it into the reachable workspace
struct JointPositionCommand {
  std::array<double, kActiveJointCount> target{};

  AIDIN_HAND2_EXPORT void clamp();
};

// Target in rad, clamp() pulls it into the reachable workspace
// Gains live in ControllerConfig
struct JointImpedanceCommand {
  std::array<double, kActiveJointCount> target{};

  AIDIN_HAND2_EXPORT void clamp();
};

// Target in encoder count
struct ActuatorPositionCommand {
  std::array<double, kActuatorCount> target{};
};

// Target in percent of rated current, 2000 = 200%
struct ActuatorEffortCommand {
  std::array<double, kActuatorCount> target{};
};

// ----------------------------- Command variant ------------------------------

using ControllerCommand =
    std::variant<Idle, JointPositionCommand, JointImpedanceCommand,
                 ActuatorPositionCommand, ActuatorEffortCommand>;

inline CommandMode to_command_mode(const ControllerCommand& command)
{
  return std::visit(
      [](const auto& active) -> CommandMode {
        using T = std::decay_t<decltype(active)>;
        if constexpr (std::is_same_v<T, JointPositionCommand>) return CommandMode::JointPosition;
        else if constexpr (std::is_same_v<T, JointImpedanceCommand>) return CommandMode::JointImpedance;
        else if constexpr (std::is_same_v<T, ActuatorPositionCommand>) return CommandMode::ActuatorPosition;
        else if constexpr (std::is_same_v<T, ActuatorEffortCommand>) return CommandMode::ActuatorEffort;
        else return CommandMode::Idle;
      },
      command);
}

}  // namespace aidin_hand2

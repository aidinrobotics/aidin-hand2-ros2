// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include <aidin_hand2/types/state.hpp>

namespace aidin_hand2
{

// SDK and RT loop health, sensor readings are in HandState
struct Diagnostics {
  // What the hand was observed to reach, not what was asked for: a stop() that threw leaves this
  // Running, because the drives never confirmed the quick stop and may hold the last command
  HandLifecycle lifecycle{HandLifecycle::Disconnected};

  // The event that put the hand in Faulted: 404, 405 or 702 of error_codes.hpp, 0 while not Faulted
  // Not the drive faults, which are actuator_health.fault
  std::uint16_t fault_code{0};

  HomingState homing_state{HomingState::NotRun};

  // set_command calls rejected by validation: a NaN or Inf target, or one past the int32 range
  std::uint64_t nan_command_count{0};

  std::uint64_t control_cycles{0};

  // Cycles that overran the target period
  std::uint64_t deadline_misses{0};

  // Last cycle-to-cycle gap and this cycle's processing time
  double last_period_ms{0.0};
  double last_compute_ms{0.0};

  ActuatorHealth actuator_health{};
};

}  // namespace aidin_hand2

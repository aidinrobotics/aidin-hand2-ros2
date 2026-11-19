// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <memory>

#include <aidin_hand2/export.hpp>
#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/diagnostics.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

namespace aidin_hand2
{

class HandCore;

// Non-owning handle to one hand; HandManager owns the resources
class AIDIN_HAND2_EXPORT Hand {
 public:
  // ------------------------------- Connection -------------------------------

  // Open the CAN socket and start the read-only control loop (blocking)
  void connect();

  // Stop the control loop and close the CAN socket (blocking)
  void disconnect();

  // Reopen the CAN socket and restart the control loop, keeping runtime settings (blocking)
  void reconnect();

  // -------------------------------- Operation -------------------------------

  // Enable the drives (non-blocking)
  void run();

  // Quick stop the drives (blocking)
  void stop();

  // Hard stop homing (blocking)
  void home();

  // Hard stop homing (non-blocking)
  void start_homing();

  // True while a homing sequence is running
  [[nodiscard]] bool is_homing() const;

  // -------------------------------- Command ---------------------------------

  // Latch a new target and switch to that controller
  void set_command(const Idle& command);
  void set_command(const JointPositionCommand& command);
  void set_command(const JointImpedanceCommand& command);
  void set_command(const ActuatorPositionCommand& command);
  void set_command(const ActuatorEffortCommand& command);

  // --------------------------------- Config ---------------------------------

  // Cap the drive current, % of rated, 0-2000
  void set_max_effort(double limit);
  void set_max_effort(const std::array<double, kActuatorCount>& limit);

  // The max effort in effect from the next cycle on, after the clamp
  [[nodiscard]] std::array<double, kActuatorCount> get_max_effort() const;

  // Set the position filter and the impedance gains
  void set_controller_config(const ControllerConfig& config);

  // The config in effect from the next cycle on
  [[nodiscard]] ControllerConfig get_controller_config() const;

  // -------------------------------- Tactile ---------------------------------

  // Make the current tactile values zero from the next cycle on (non-blocking)
  void set_tactile_bias();

  // Report the raw tactile values again from the next cycle on (non-blocking)
  void reset_tactile_bias();

  // The reading set_tactile_bias() took, all zero when none is set
  [[nodiscard]] TactileState get_tactile_bias() const;

  // ------------------------------ Observation -------------------------------

  // Last observed actuator, joint, tactile and commanded state
  [[nodiscard]] HandState get_state() const;

  // Lifecycle, homing, loop timing and per-actuator health
  [[nodiscard]] Diagnostics get_diagnostics() const;

  // Active controller mode
  [[nodiscard]] CommandMode get_command_mode() const;

 private:
  Hand(std::weak_ptr<HandCore> core, HandSide side);

  std::weak_ptr<HandCore> core_;

  // For the "[left] " of a failure, which may come after the core is gone
  HandSide side_;

  friend class HandManager;
};

}  // namespace aidin_hand2

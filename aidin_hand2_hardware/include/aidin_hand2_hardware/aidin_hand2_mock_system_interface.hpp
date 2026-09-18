// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef AIDIN_HAND2_HARDWARE__AIDIN_HAND2_MOCK_SYSTEM_INTERFACE_HPP_
#define AIDIN_HAND2_HARDWARE__AIDIN_HAND2_MOCK_SYSTEM_INTERFACE_HPP_

#include <array>
#include <set>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/command.hpp>

#include "hardware_interface/system_interface.hpp"

namespace aidin_hand2_hardware
{

// Kinematic mock, same command interface contract as the real hardware
class AidinHand2MockSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string hand_side_;
  std::string prefix_;

  double command_lock_{};
  std::array<double, aidin_hand2::kActiveJointCount> joint_position_target_rad_{};
  std::array<double, aidin_hand2::kActiveJointCount> joint_impedance_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_position_target_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_effort_target_pct_{};

  // Previous target, filling NaN command entries
  std::array<double, aidin_hand2::kActiveJointCount> held_joint_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> held_actuator_target_cnt_{};

  std::array<double, aidin_hand2::kActuatorCount> actuator_position_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_velocity_rpm_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_current_ma_{};
  std::array<double, aidin_hand2::kJointCount> joint_position_rad_{};
  double max_effort_pct_{1000.0};

  aidin_hand2::CommandMode command_mode_{aidin_hand2::CommandMode::Idle};
  aidin_hand2::CommandMode pending_mode_{aidin_hand2::CommandMode::Idle};
  std::set<std::string> active_command_interfaces_;
  std::set<std::string> pending_command_interfaces_;
  bool pending_mode_switch_valid_{false};
};

}  // namespace aidin_hand2_hardware

#endif  // AIDIN_HAND2_HARDWARE__AIDIN_HAND2_MOCK_SYSTEM_INTERFACE_HPP_

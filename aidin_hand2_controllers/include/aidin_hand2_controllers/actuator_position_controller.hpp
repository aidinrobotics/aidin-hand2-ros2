// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef AIDIN_HAND2_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER_HPP_
#define AIDIN_HAND2_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>

#include "aidin_hand2_controllers/joint_state_command.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace aidin_hand2_controllers
{

// Chainable controller moving a JointState matched by name to the hardware command interface
//
// Standalone takes the command topic, chained takes the exported reference, never both
// update_reference_from_subscribers runs first in a cycle, then update_and_write_commands
class ActuatorPositionController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // The subscription lives only while standalone, on_set_chained_mode drops and restores it
  void subscribe();
  void unsubscribe();
  void drop_buffered_command();

  // Filled in on_configure and read only afterwards
  std::string hand_side_;
  std::vector<std::string> actuator_names_;

  // command_lock, then the 16 target positions
  std::vector<std::string> command_interface_names_;

  // Sequence of the last message moved to the references, 0 for none consumed yet
  std::uint64_t command_sequence_{0};
  std::uint64_t consumed_sequence_{0};

  realtime_tools::RealtimeBuffer<
    JointStateCommand<aidin_hand2::kActuatorCount>> command_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__ACTUATOR_POSITION_CONTROLLER_HPP_

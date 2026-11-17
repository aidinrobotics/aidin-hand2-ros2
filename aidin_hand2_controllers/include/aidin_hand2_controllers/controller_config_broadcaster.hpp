// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef AIDIN_HAND2_CONTROLLERS__CONTROLLER_CONFIG_BROADCASTER_HPP_
#define AIDIN_HAND2_CONTROLLERS__CONTROLLER_CONFIG_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "aidin_hand2_msgs/msg/controller_config.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace aidin_hand2_controllers
{

// Broadcasts the settings the SDK holds on ~/controller_config (aidin_hand2_msgs/ControllerConfig)
//
// Max effort and the joint position and joint impedance controller settings
// Published when a value changes, transient_local so a subscriber that joins later gets the last
class ControllerConfigBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Filled in on_configure and read only afterwards
  std::string hand_side_;

  // The claimed values last published, NaN after activation so the first update publishes
  std::vector<double> published_values_;

  std::shared_ptr<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::ControllerConfig>>
    publisher_;
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__CONTROLLER_CONFIG_BROADCASTER_HPP_

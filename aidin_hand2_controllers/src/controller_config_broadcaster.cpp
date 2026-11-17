// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#include "aidin_hand2_controllers/controller_config_broadcaster.hpp"

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>

// --------------------------- State interface name ---------------------------
//   side       ∈ {left, right}
//   actuator   : thumb_actuator0..3, then {index,middle,ring,baby}_actuator1..3
//   joint      : thumb_joint0..3, then {index,middle,ring,baby}_joint1..3
//
//   state interface : {side}_controller_config/max_effort.{actuator}
//                     {side}_controller_config/joint_position_controller.{field}
//                     {side}_controller_config/joint_impedance_controller.stiffness.{joint}
//                     {side}_controller_config/joint_impedance_controller.{field}
//   published topic : ~/controller_config
//                     (aidin_hand2_msgs/ControllerConfig)
//
//   The claim is max_effort x16, 3 joint position fields, stiffness x16 and 2 joint impedance
//   fields, 37 resources, in the interface order the hardware exports
//   filter_enabled arrives as 0 or 1

namespace aidin_hand2_controllers
{

namespace ah2 = aidin_hand2;

// Interface names without the prefix
// The position in the list is the actuator index
constexpr std::array<const char *, ah2::kActuatorCount> kActuatorBaseNames = {
  "thumb_actuator0",
  "thumb_actuator1",
  "thumb_actuator2",
  "thumb_actuator3",
  "index_actuator1",
  "index_actuator2",
  "index_actuator3",
  "middle_actuator1",
  "middle_actuator2",
  "middle_actuator3",
  "ring_actuator1",
  "ring_actuator2",
  "ring_actuator3",
  "baby_actuator1",
  "baby_actuator2",
  "baby_actuator3",
};

// The position in the list is the active joint index
constexpr std::array<const char *, ah2::kActiveJointCount> kActiveJointBaseNames = {
  "thumb_joint0",
  "thumb_joint1",
  "thumb_joint2",
  "thumb_joint3",
  "index_joint1",
  "index_joint2",
  "index_joint3",
  "middle_joint1",
  "middle_joint2",
  "middle_joint3",
  "ring_joint1",
  "ring_joint2",
  "ring_joint3",
  "baby_joint1",
  "baby_joint2",
  "baby_joint3",
};

// Fields in the interface order the hardware exports
constexpr std::array<const char *, 3> kJointPositionFields = {
  "joint_position_controller.filter_enabled",
  "joint_position_controller.cutoff_freq",
  "joint_position_controller.deadband",
};
constexpr std::array<const char *, 2> kJointImpedanceFields = {
  "joint_impedance_controller.filter_enabled",
  "joint_impedance_controller.cutoff_freq",
};

constexpr char kControllerConfigComponent[] = "controller_config";

namespace
{
// Offsets into the claimed state interfaces
constexpr std::size_t kMaxEffortOffset = 0;
constexpr std::size_t kJointPositionOffset = kMaxEffortOffset + ah2::kActuatorCount;
constexpr std::size_t kStiffnessOffset = kJointPositionOffset + kJointPositionFields.size();
constexpr std::size_t kJointImpedanceOffset = kStiffnessOffset + ah2::kActiveJointCount;
constexpr std::size_t kInterfaceCount = kJointImpedanceOffset + kJointImpedanceFields.size();
}  // namespace

// --------------------------------- Lifecycle --------------------------------

controller_interface::CallbackReturn ControllerConfigBroadcaster::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerConfigBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(
      get_node()->get_logger(), "hand_side must be 'left' or 'right', got '%s'",
      hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // transient_local hands the last settings to a subscriber that joins later, such as rosbag
  auto publisher = get_node()->create_publisher<aidin_hand2_msgs::msg::ControllerConfig>(
    "~/controller_config", rclcpp::QoS(1).reliable().transient_local());
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::ControllerConfig>>(
      publisher);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerConfigBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  published_values_.assign(kInterfaceCount, std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerConfigBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------- Interface configuration -------------------------

controller_interface::InterfaceConfiguration
ControllerConfigBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
ControllerConfigBroadcaster::state_interface_configuration() const
{
  const std::string config = hand_side_ + "_" + kControllerConfigComponent + "/";
  std::vector<std::string> names;
  names.reserve(kInterfaceCount);
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(config + "max_effort." + actuator);
  }
  for (const char * field : kJointPositionFields) {
    names.push_back(config + field);
  }
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(config + "joint_impedance_controller.stiffness." + joint);
  }
  for (const char * field : kJointImpedanceFields) {
    names.push_back(config + field);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

// ---------------------------------- Update ----------------------------------

// Publishes when a claimed value differs from the last published one
// A cycle that cannot take the publisher lock leaves the change for the next cycle
controller_interface::return_type ControllerConfigBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  bool changed = false;
  for (std::size_t i = 0; i < kInterfaceCount; ++i) {
    if (state_interfaces_[i].get_value() != published_values_[i]) {
      changed = true;
      break;
    }
  }
  if (!changed || !publisher_ || !publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }

  const auto value = [this](std::size_t index) { return state_interfaces_[index].get_value(); };
  auto & message = publisher_->msg_;
  message.header.stamp = time;
  message.hand_side = hand_side_;
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    message.max_effort[i] = value(kMaxEffortOffset + i);
  }
  auto & position = message.joint_position_controller;
  position.filter_enabled = value(kJointPositionOffset) != 0.0;
  position.cutoff_freq = value(kJointPositionOffset + 1);
  position.deadband = value(kJointPositionOffset + 2);
  auto & impedance = message.joint_impedance_controller;
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    impedance.stiffness[i] = value(kStiffnessOffset + i);
  }
  impedance.filter_enabled = value(kJointImpedanceOffset) != 0.0;
  impedance.cutoff_freq = value(kJointImpedanceOffset + 1);
  publisher_->unlockAndPublish();

  for (std::size_t i = 0; i < kInterfaceCount; ++i) {
    published_values_[i] = value(i);
  }
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::ControllerConfigBroadcaster, controller_interface::ControllerInterface)

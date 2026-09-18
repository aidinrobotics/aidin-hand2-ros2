// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#include "aidin_hand2_controllers/actuator_position_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <aidin_hand2/types/description.hpp>

#include "rclcpp/qos.hpp"

// ------------------------------ Interface name ------------------------------
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   n          : thumb = 0..3, otherwise 1..3
//
//   command interface  : {side}_hand_control/command_lock              (claim-only)
//                        {side}_actuator_position_command/
//                          target_position_cnt.{finger}_actuator{n}      (encoder count)
//   reference interface: {side}_actuator_position_controller/
//                          {side}_{finger}_actuator{n}/position_cnt      (encoder count)
//   command topic      : /{side}_actuator_position_controller/cmd
//                        (sensor_msgs/JointState, name matched, position read)
//
//   The input moves to the command interface unchanged, no target is generated and no state
//   is read
//   A cycle with no input writes NaN, and a consumed input is reset to NaN at once
//   A NaN target is an actuator no upper controller owns, and the hardware fills it
//   The claim is command_lock x1 and target_position_cnt x16, 17 resources
//   command_lock is claim-only, taken for mode exclusion
//   This is a raw interface, the encoder count is commanded with no kinematics in the way

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
constexpr char kCommandLockInterface[] = "command_lock";
constexpr char kTargetInterface[] = "target_position_cnt";
constexpr char kReferenceInterface[] = "position_cnt";

namespace
{
// Claimed command layout is the lock first, then the 16 targets
// Exported reference layout is the 16 targets alone
constexpr std::size_t kCommandLockCount = 1;

std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterface};
  const std::string component = side + "_actuator_position_command/";
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(component + kTargetInterface + "." + actuator);
  }
  return names;
}
}  // namespace

// --------------------------------- Lifecycle --------------------------------

controller_interface::CallbackReturn ActuatorPositionController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(
      get_node()->get_logger(), "hand_side must be 'left' or 'right', got '%s'",
      hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  actuator_names_.clear();
  for (const char * base : kActuatorBaseNames) {
    actuator_names_.push_back(hand_side_ + "_" + base);
  }
  command_interface_names_ = hardware_command_interfaces(hand_side_);

  drop_buffered_command();
  subscribe();
  return controller_interface::CallbackReturn::SUCCESS;
}

// No target at activation, the references are cleared and any earlier message dropped
controller_interface::CallbackReturn ActuatorPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  drop_buffered_command();
  if (reference_interfaces_.size() != ah2::kActuatorCount) {
    return controller_interface::CallbackReturn::ERROR;
  }
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void ActuatorPositionController::subscribe()
{
  if (command_subscriber_) {
    return;
  }
  command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/cmd", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> message) {
      JointStateCommand<ah2::kActuatorCount> command;
      if (!resolve_joint_state_command(
            *message, actuator_names_, JointStateField::kPosition, command.values)) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 5000,
          "ActuatorPosition command dropped — name and position differ in length, or a name repeats");
        return;
      }
      command.sequence = ++command_sequence_;
      command_buffer_.writeFromNonRT(command);
    });
}

void ActuatorPositionController::unsubscribe()
{
  command_subscriber_.reset();
}

void ActuatorPositionController::drop_buffered_command()
{
  command_buffer_.writeFromNonRT(JointStateCommand<ah2::kActuatorCount>{});
  consumed_sequence_ = 0;
}

// -------------------------- Interface configuration -------------------------

controller_interface::InterfaceConfiguration
ActuatorPositionController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

controller_interface::InterfaceConfiguration
ActuatorPositionController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

std::vector<hardware_interface::CommandInterface>
ActuatorPositionController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(
    ah2::kActuatorCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(ah2::kActuatorCount);
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      actuator_names_[i] + "/" + kReferenceInterface,
      &reference_interfaces_[i]);
  }
  return references;
}

// Chained mode drops the topic input, the reference is the only path in
bool ActuatorPositionController::on_set_chained_mode(bool chained_mode)
{
  if (chained_mode) {
    unsubscribe();
  } else {
    subscribe();
  }
  drop_buffered_command();
  return true;
}

// ---------------------------------- Update ----------------------------------

// Only a message not yet consumed moves to the references
controller_interface::return_type
ActuatorPositionController::update_reference_from_subscribers()
{
  const auto & command = *command_buffer_.readFromRT();
  if (command.sequence == 0 || command.sequence == consumed_sequence_) {
    return controller_interface::return_type::OK;
  }
  consumed_sequence_ = command.sequence;
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    reference_interfaces_[i] = command.values[i];
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type ActuatorPositionController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, ah2::kActuatorCount> target{};
  bool has_target = false;
  bool invalid = false;

  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const double value = reference_interfaces_[i];
    if (std::isnan(value)) {
      target[i] = nan;  // Not owned, the hardware fills it
    } else if (!std::isfinite(value)) {
      target[i] = nan;
      invalid = true;
    } else {
      target[i] = value;
      has_target = true;
    }
  }

  if (invalid) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "ActuatorPosition reference has an Inf value — ignored");
  }

  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    (void)command_interfaces_[kCommandLockCount + i].set_value(
      has_target ? target[i] : nan);
  }

  // Marks the input consumed
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::ActuatorPositionController,
  controller_interface::ChainableControllerInterface)

#include "aidin_hand2_controllers/joint_position_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "rclcpp/qos.hpp"

// ------------------------------ Interface name ------------------------------
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   n          : thumb = 0..3, otherwise 1..3
//
//   command interface  : {side}_hand_control/command_lock              (claim-only)
//                        {side}_joint_position_command/
//                          target_position_rad.{finger}_joint{n}         (rad)
//   reference interface: {side}_joint_position_controller/
//                          {side}_{finger}_joint{n}/position             (rad)
//   command topic      : /{side}_joint_position_controller/command
//                        (aidin_hand2_msgs/JointPositionCommand)
//
//   The input moves to the command interface unchanged, no target is generated and no state
//   is read
//   A cycle with no input writes NaN, and a consumed input is reset to NaN at once
//   A NaN target is a joint no upper controller owns, and the hardware fills it
//   command_lock is claim-only, taken for mode exclusion
//   The target filter belongs to the SDK ControllerConfig, tuned on the hardware node

namespace aidin_hand2_controllers
{

constexpr std::size_t kActiveJointCount = 16;
constexpr const char * kActiveJointBaseNames[kActiveJointCount] = {
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
constexpr const char * kCommandLockInterfaceName = "command_lock";
constexpr const char * kTargetPositionInterfaceName = "target_position_rad";
constexpr const char * kReferencePositionInterfaceName = "position";

namespace
{
constexpr std::size_t kTargetCount = kActiveJointCount;
constexpr std::size_t kReferenceCount = kTargetCount;
// Claimed command layout is [0] lock then [1..16] target position
// Exported reference layout is [0..15] target position
constexpr std::size_t kHardwareTargetOffset = 1;

std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterfaceName};
  const std::string component = side + "_joint_position_command/";
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(component + kTargetPositionInterfaceName + "." + joint);
  }
  return names;
}
}  // namespace

// --------------------------------- Lifecycle --------------------------------

controller_interface::CallbackReturn JointPositionController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(get_node()->get_logger(), "hand_side parameter is invalid");
    return controller_interface::CallbackReturn::ERROR;
  }

  active_joint_names_.clear();
  for (const char * base : kActiveJointBaseNames) {
    active_joint_names_.push_back(hand_side_ + "_" + base);
  }
  command_interface_names_ = hardware_command_interfaces(hand_side_);

  drop_buffered_command();
  subscribe();
  return controller_interface::CallbackReturn::SUCCESS;
}

// No target at activation, the references are cleared and any earlier message dropped
controller_interface::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  drop_buffered_command();
  if (reference_interfaces_.size() != kReferenceCount) {
    return controller_interface::CallbackReturn::ERROR;
  }
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointPositionController::subscribe()
{
  if (command_subscriber_) {
    return;
  }
  command_subscriber_ =
    get_node()->create_subscription<aidin_hand2_msgs::msg::JointPositionCommand>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::JointPositionCommand> message) {
        command_buffer_.writeFromNonRT(message);
      });
}

void JointPositionController::unsubscribe()
{
  command_subscriber_.reset();
}

void JointPositionController::drop_buffered_command()
{
  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::JointPositionCommand>());
  consumed_command_ = nullptr;
}

// -------------------------- Interface configuration -------------------------

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

std::vector<hardware_interface::CommandInterface>
JointPositionController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(kReferenceCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(kReferenceCount);
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      active_joint_names_[i] + "/" + kReferencePositionInterfaceName,
      &reference_interfaces_[i]);
  }
  return references;
}

// Chained mode takes the reference, so the topic input is dropped
bool JointPositionController::on_set_chained_mode(bool chained_mode)
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
JointPositionController::update_reference_from_subscribers()
{
  const auto message = *command_buffer_.readFromRT();
  if (!message || message.get() == consumed_command_) {
    return controller_interface::return_type::OK;
  }
  consumed_command_ = message.get();
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    reference_interfaces_[i] = message->target_position_rad[i];
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type JointPositionController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, kTargetCount> target{};
  bool has_target = false;
  bool invalid = false;

  for (std::size_t i = 0; i < kTargetCount; ++i) {
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
      "JointPosition reference has an Inf value — ignored");
  }

  for (std::size_t i = 0; i < kTargetCount; ++i) {
    (void)command_interfaces_[kHardwareTargetOffset + i].set_value(
      has_target ? target[i] : nan);
  }

  // Marks the input consumed
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::JointPositionController,
  controller_interface::ChainableControllerInterface)

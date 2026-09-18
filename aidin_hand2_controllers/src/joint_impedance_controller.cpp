#include "aidin_hand2_controllers/joint_impedance_controller.hpp"

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
//                        {side}_joint_impedance_command/
//                          target_position_rad.{finger}_joint{n}         (rad)
//   reference interface: {side}_joint_impedance_controller/
//                          {side}_{finger}_joint{n}/position             (rad)
//   command topic      : /{side}_joint_impedance_controller/cmd
//                        (sensor_msgs/JointState, name matched, position read)
//
//   The input moves to the command interface unchanged, no target is generated and no state
//   is read
//   A cycle with no input writes NaN, and a consumed input is reset to NaN at once
//   A NaN target is a joint no upper controller owns, and the hardware fills it
//   The claim is command_lock x1 and target_position_rad x16, 17 resources
//   command_lock is claim-only, taken for mode exclusion
//   The equilibrium target carries the same position name as JointPositionController
//   The impedance gains are tuned on the hardware node, not in this controller

namespace aidin_hand2_controllers
{

namespace ah2 = aidin_hand2;

// Interface names without the prefix
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
constexpr char kCommandLockInterface[] = "command_lock";
constexpr char kTargetInterface[] = "target_position_rad";
constexpr char kReferenceInterface[] = "position";

namespace
{
// Claimed command layout is the lock first, then the 16 targets
// Exported reference layout is the 16 targets alone
constexpr std::size_t kCommandLockCount = 1;

std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterface};
  const std::string component = side + "_joint_impedance_command/";
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(component + kTargetInterface + "." + joint);
  }
  return names;
}
}  // namespace

// --------------------------------- Lifecycle --------------------------------

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(
      get_node()->get_logger(), "hand_side must be 'left' or 'right', got '%s'",
      hand_side_.c_str());
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
controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  drop_buffered_command();
  if (reference_interfaces_.size() != ah2::kActiveJointCount) {
    return controller_interface::CallbackReturn::ERROR;
  }
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointImpedanceController::subscribe()
{
  if (command_subscriber_) {
    return;
  }
  command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/cmd", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> message) {
      JointStateCommand<ah2::kActiveJointCount> command;
      if (!resolve_joint_state_command(
            *message, active_joint_names_, JointStateField::kPosition, command.values)) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 5000,
          "JointImpedance command dropped — name and position differ in length, or a name repeats");
        return;
      }
      command.sequence = ++command_sequence_;
      command_buffer_.writeFromNonRT(command);
    });
}

void JointImpedanceController::unsubscribe()
{
  command_subscriber_.reset();
}

void JointImpedanceController::drop_buffered_command()
{
  command_buffer_.writeFromNonRT(JointStateCommand<ah2::kActiveJointCount>{});
  consumed_sequence_ = 0;
}

// -------------------------- Interface configuration -------------------------

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

std::vector<hardware_interface::CommandInterface>
JointImpedanceController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(
    ah2::kActiveJointCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(ah2::kActiveJointCount);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      active_joint_names_[i] + "/" + kReferenceInterface,
      &reference_interfaces_[i]);
  }
  return references;
}

// Chained mode drops the topic input, the reference is the only path in
bool JointImpedanceController::on_set_chained_mode(bool chained_mode)
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
JointImpedanceController::update_reference_from_subscribers()
{
  const auto & command = *command_buffer_.readFromRT();
  if (command.sequence == 0 || command.sequence == consumed_sequence_) {
    return controller_interface::return_type::OK;
  }
  consumed_sequence_ = command.sequence;
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    reference_interfaces_[i] = command.values[i];
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type JointImpedanceController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, ah2::kActiveJointCount> target{};
  bool has_target = false;
  bool invalid = false;

  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
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
      "JointImpedance reference has an Inf value — ignored");
  }

  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
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
  aidin_hand2_controllers::JointImpedanceController,
  controller_interface::ChainableControllerInterface)

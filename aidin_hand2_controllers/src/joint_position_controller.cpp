#include "aidin_hand2_controllers/joint_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/qos.hpp"

// ── interface name ──────────────────────────────────────────────────────────
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   n          : thumb = 0..3, 그 외 = 1..3
//
//   command interface  : {side}_hand_control/command_lock              (claim-only)
//                        {side}_joint_position_command/
//                          target_position_rad.{finger}_joint{n}         (rad)
//                        {side}_joint_position_command/speed_rad_s       (rad/s)
//   reference interface: {side}_joint_position_controller/
//                          {side}_{finger}_joint{n}/position             (rad)
//                        {side}_joint_position_controller/
//                          {side}_joint_position/speed_rad_s             (rad/s)
//   command topic      : /{side}_joint_position_controller/command
//                        (aidin_hand2_msgs/JointPositionCommand)
//
//   target 16 + speed 1을 항상 완전한 한 묶음으로 command port에 기록한다. command_lock은
//   값으로 쓰지 않고 mode 상호 배제를 위한 resource claim으로만 사용한다.
// ─────────────────────────────────────────────────────────────────────────────

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
constexpr const char * kSpeedInterfaceName = "speed_rad_s";
constexpr const char * kReferencePositionInterfaceName = "position";

namespace
{
constexpr std::size_t kTargetCount = kActiveJointCount;
constexpr std::size_t kSpeedIndex = kTargetCount;
constexpr std::size_t kReferenceCount = kTargetCount + 1;
// claimed command layout: [0] lock, [1..16] target position, [17] speed.
// exported reference layout: [0..15] target position, [16] speed.
constexpr std::size_t kHardwareTargetOffset = 1;  // command_interfaces_[0] = command_lock
constexpr std::size_t kHardwareSpeedIndex = kHardwareTargetOffset + kTargetCount;

std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterfaceName};
  const std::string component = side + "_joint_position_command/";
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(component + kTargetPositionInterfaceName + "." + joint);
  }
  names.push_back(component + kSpeedInterfaceName);
  return names;
}
}  // namespace

// ── lifecycle ────────────────────────────────────────────────────────────────
// Parameter 기본값 선언.
controller_interface::CallbackReturn JointPositionController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  auto_declare<double>("speed_rad_s", 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

// Side·speed를 검증하고 state/command interface와 typed command subscriber를 구성.
controller_interface::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  default_speed_ = get_node()->get_parameter("speed_rad_s").as_double();
  if ((hand_side_ != "left" && hand_side_ != "right") ||
      !std::isfinite(default_speed_) || default_speed_ < 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "hand_side or speed_rad_s parameter is invalid");
    return controller_interface::CallbackReturn::ERROR;
  }

  active_joint_names_.clear();
  state_interface_names_.clear();
  // state interface: active joint position 16개.
  for (const char * base : kActiveJointBaseNames) {
    active_joint_names_.push_back(hand_side_ + "_" + base);
    state_interface_names_.push_back(
      active_joint_names_.back() + "/" + kReferencePositionInterfaceName);
  }
  // command interface: [0] command_lock + [1..16] target_position_rad + [17] speed_rad_s.
  command_interface_names_ = hardware_command_interfaces(hand_side_);

  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::JointPositionCommand>());
  command_subscriber_ =
    get_node()->create_subscription<aidin_hand2_msgs::msg::JointPositionCommand>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::JointPositionCommand> message) {
        command_buffer_.writeFromNonRT(message);
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

// 현재 joint 자세와 기본 speed로 reference 전체를 안전하게 seed.
controller_interface::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::JointPositionCommand>());
  if (reference_interfaces_.size() != kReferenceCount ||
      state_interfaces_.size() != kTargetCount)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    reference_interfaces_[i] = state_interfaces_[i].get_value();
  }
  reference_interfaces_[kSpeedIndex] = default_speed_;
  return controller_interface::CallbackReturn::SUCCESS;
}

// Resource release는 controller_manager가 처리하며 추가 동작 없음.
controller_interface::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ── interface configuration ─────────────────────────────────────────────────
// claim: command_lock + JointPosition hardware command port 17개.
controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

// activation seed용 active joint position state 16개.
controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interface_names_};
}

// 자세 16 + 공통 speed 1 = 17개를 reference로 노출.
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
  references.emplace_back(
    get_node()->get_name(),
    hand_side_ + "_joint_position/" + kSpeedInterfaceName,
    &reference_interfaces_[kSpeedIndex]);
  return references;
}

bool JointPositionController::on_set_chained_mode(bool)
{
  return true;
}

// ── update ──────────────────────────────────────────────────────────────────
// standalone: typed command 하나를 reference 전체([0..15] 자세, [16] speed)에 반영.
controller_interface::return_type
JointPositionController::update_reference_from_subscribers()
{
  const auto message = *command_buffer_.readFromRT();
  if (message) {
    for (std::size_t i = 0; i < kTargetCount; ++i) {
      reference_interfaces_[i] = message->target_position_rad[i];
    }
    reference_interfaces_[kSpeedIndex] = message->speed_rad_s;
  }
  return controller_interface::return_type::OK;
}

// reference_interfaces_의 완전한 17개 입력을 mode 전용 hardware command port에 기록.
controller_interface::return_type JointPositionController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < kReferenceCount; ++i) {
    if (!std::isfinite(reference_interfaces_[i])) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "JointPosition reference contains NaN/Inf");
      return controller_interface::return_type::ERROR;
    }
  }
  if (reference_interfaces_[kSpeedIndex] < 0.0) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "JointPosition speed_rad_s must be non-negative");
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kTargetCount; ++i) {
    (void)command_interfaces_[kHardwareTargetOffset + i].set_value(reference_interfaces_[i]);
  }
  (void)command_interfaces_[kHardwareSpeedIndex].set_value(
    reference_interfaces_[kSpeedIndex]);
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::JointPositionController,
  controller_interface::ChainableControllerInterface)

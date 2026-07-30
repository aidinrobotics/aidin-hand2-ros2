#include "aidin_hand2_controllers/joint_impedance_controller.hpp"

#include <cmath>
#include <limits>

#include <aidin_hand2/types/command.hpp>
#include "rclcpp/qos.hpp"

// ── interface name ──────────────────────────────────────────────────────────
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   joint n    : thumb = 0..3, 그 외 = 1..3
//   actuator n : thumb = 0..3, 그 외 = 1..3
//
//   command interface  : {side}_hand_control/command_lock              (claim-only)
//                        {side}_joint_impedance_command/
//                          target_position_rad.{finger}_joint{n}         (rad)
//                        {side}_joint_impedance_command/
//                          stiffness.{finger}_actuator{n}
//                        {side}_joint_impedance_command/
//                          damping.{finger}_actuator{n}
//   reference interface: {side}_joint_impedance_controller/
//                          {side}_{finger}_joint{n}/position             (rad)
//                        {side}_joint_impedance_controller/
//                          {side}_{finger}_actuator{n}/stiffness
//                        {side}_joint_impedance_controller/
//                          {side}_{finger}_actuator{n}/damping
//   command topic      : /{side}_joint_impedance_controller/command
//                        (aidin_hand2_msgs/JointImpedanceCommand)
//
//   자세 reference는 JointPositionController와 같은 position 이름을 쓴다. target 16 + gain 32를
//   reference로 노출하고, command port에도 48개 전체를 한 update에서 기록한다.
// ─────────────────────────────────────────────────────────────────────────────

namespace aidin_hand2_controllers
{

constexpr std::size_t kActiveJointCount = 16;
constexpr std::size_t kActuatorCount = 16;
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
constexpr const char * kActuatorBaseNames[kActuatorCount] = {
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
constexpr const char * kCommandLockInterfaceName = "command_lock";
constexpr const char * kPositionInterfaceName = "target_position_rad";
constexpr const char * kStiffnessInterfaceName = "stiffness";
constexpr const char * kDampingInterfaceName = "damping";
constexpr const char * kReferencePositionInterfaceName = "position";

namespace
{
constexpr std::size_t kTargetOffset = 0;
constexpr std::size_t kStiffnessOffset = 16;
constexpr std::size_t kDampingOffset = 32;
constexpr std::size_t kReferenceCount = 48;
// claimed command layout: [0] lock, [1..16] target position,
// [17..32] stiffness, [33..48] damping.
// exported reference layout: [0..15] target position,
// [16..31] stiffness, [32..47] damping.
constexpr std::size_t kHardwareOffset = 1;  // command_interfaces_[0] = command_lock

std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterfaceName};
  const std::string component = side + "_joint_impedance_command/";
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(component + kPositionInterfaceName + "." + joint);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(component + kStiffnessInterfaceName + "." + actuator);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(component + kDampingInterfaceName + "." + actuator);
  }
  return names;
}
}  // namespace

// ── lifecycle ────────────────────────────────────────────────────────────────
// Side와 기본 impedance gain parameter를 선언.
controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  auto_declare<std::vector<double>>(
    "stiffness", std::vector<double>(
      aidin_hand2::kDefaultStiffness.begin(), aidin_hand2::kDefaultStiffness.end()));
  auto_declare<std::vector<double>>(
    "damping", std::vector<double>(
      aidin_hand2::kDefaultDamping.begin(), aidin_hand2::kDefaultDamping.end()));
  return controller_interface::CallbackReturn::SUCCESS;
}

// Side·gain을 검증하고 state/command interface와 typed command subscriber를 구성.
controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  const auto stiffness = get_node()->get_parameter("stiffness").as_double_array();
  const auto damping = get_node()->get_parameter("damping").as_double_array();
  if ((hand_side_ != "left" && hand_side_ != "right") ||
      stiffness.size() != kActuatorCount ||
      damping.size() != kActuatorCount)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "invalid hand_side or impedance gain size");
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
  actuator_names_.clear();
  for (const char * base : kActuatorBaseNames) {
    actuator_names_.push_back(hand_side_ + "_" + base);
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!std::isfinite(stiffness[i]) || !std::isfinite(damping[i]) ||
        stiffness[i] < 0.0 || damping[i] < 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "impedance gains must be finite and non-negative");
      return controller_interface::CallbackReturn::ERROR;
    }
    default_stiffness_[i] = stiffness[i];
    default_damping_[i] = damping[i];
  }

  // command interface: [0] command_lock + [1..16] target_position_rad
  // + [17..32] stiffness + [33..48] damping.
  command_interface_names_ = hardware_command_interfaces(hand_side_);
  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::JointImpedanceCommand>());
  command_subscriber_ =
    get_node()->create_subscription<aidin_hand2_msgs::msg::JointImpedanceCommand>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::JointImpedanceCommand> message) {
        command_buffer_.writeFromNonRT(message);
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

// 현재 joint 자세와 parameter gain으로 48개 reference 전체를 seed.
controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::JointImpedanceCommand>());
  if (reference_interfaces_.size() != kReferenceCount ||
      state_interfaces_.size() != kActiveJointCount)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    reference_interfaces_[kTargetOffset + i] = state_interfaces_[i].get_value();
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    reference_interfaces_[kStiffnessOffset + i] = default_stiffness_[i];
    reference_interfaces_[kDampingOffset + i] = default_damping_[i];
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// Resource release는 controller_manager가 처리하며 추가 동작 없음.
controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ── interface configuration ─────────────────────────────────────────────────
// claim: command_lock + JointImpedance hardware command port 48개.
controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

// activation seed용 active joint position state 16개.
controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interface_names_};
}

// 평형 자세 16 + stiffness 16 + damping 16 = 48개를 reference로 노출.
std::vector<hardware_interface::CommandInterface>
JointImpedanceController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(kReferenceCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(kReferenceCount);
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      active_joint_names_[i] + "/" + kReferencePositionInterfaceName,
      &reference_interfaces_[kTargetOffset + i]);
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      actuator_names_[i] + "/" + kStiffnessInterfaceName,
      &reference_interfaces_[kStiffnessOffset + i]);
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      actuator_names_[i] + "/" + kDampingInterfaceName,
      &reference_interfaces_[kDampingOffset + i]);
  }
  return references;
}

bool JointImpedanceController::on_set_chained_mode(bool)
{
  return true;
}

// ── update ──────────────────────────────────────────────────────────────────
// standalone: typed command 하나를 reference 전체(자세 16 + gain 32)에 반영.
controller_interface::return_type
JointImpedanceController::update_reference_from_subscribers()
{
  const auto message = *command_buffer_.readFromRT();
  if (message) {
    for (std::size_t i = 0; i < kActiveJointCount; ++i) {
      reference_interfaces_[kTargetOffset + i] = message->target_position_rad[i];
    }
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      reference_interfaces_[kStiffnessOffset + i] = message->stiffness[i];
      reference_interfaces_[kDampingOffset + i] = message->damping[i];
    }
  }
  return controller_interface::return_type::OK;
}

// reference_interfaces_의 완전한 48개 입력을 mode 전용 hardware command port에 기록.
controller_interface::return_type JointImpedanceController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < kReferenceCount; ++i) {
    if (!std::isfinite(reference_interfaces_[i])) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "JointImpedance reference contains NaN/Inf");
      return controller_interface::return_type::ERROR;
    }
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (reference_interfaces_[kStiffnessOffset + i] < 0.0 ||
        reference_interfaces_[kDampingOffset + i] < 0.0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "JointImpedance gains must be non-negative");
      return controller_interface::return_type::ERROR;
    }
  }
  for (std::size_t i = 0; i < kReferenceCount; ++i) {
    (void)command_interfaces_[kHardwareOffset + i].set_value(reference_interfaces_[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::JointImpedanceController,
  controller_interface::ChainableControllerInterface)

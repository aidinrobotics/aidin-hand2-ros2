#include "aidin_hand2_controllers/actuator_effort_controller.hpp"

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
//                        {side}_actuator_effort_command/
//                          target_effort_pct.{finger}_actuator{n}        (rated %)
//   reference interface: {side}_actuator_effort_controller/
//                          {side}_{finger}_actuator{n}/effort_pct
//   command topic      : /{side}_actuator_effort_controller/command
//                        (aidin_hand2_msgs/ActuatorEffortCommand)
//
//   activation 때 16개 reference를 0%로 seed하고, 매 update에 target_effort_pct 16개 전체를
//   command port에 기록한다. command_lock은 값으로 쓰지 않는다.
// ─────────────────────────────────────────────────────────────────────────────

namespace aidin_hand2_controllers
{

constexpr std::size_t kActuatorCount = 16;
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
constexpr const char * kEffortInterfaceName = "target_effort_pct";
constexpr const char * kReferenceEffortInterfaceName = "effort_pct";

namespace
{
// claimed command layout: [0] lock, [1..16] target effort.
// exported reference layout: [0..15] actuator effort.
constexpr std::size_t kHardwareTargetOffset = 1;  // command_interfaces_[0] = command_lock
std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterfaceName};
  const std::string component = side + "_actuator_effort_command/";
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(component + kEffortInterfaceName + "." + actuator);
  }
  return names;
}
}  // namespace

// ── lifecycle ────────────────────────────────────────────────────────────────
// Hand side parameter를 선언.
controller_interface::CallbackReturn ActuatorEffortController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

// Side를 검증하고 hardware command interface와 typed command subscriber를 구성.
controller_interface::CallbackReturn ActuatorEffortController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(get_node()->get_logger(), "hand_side must be left or right");
    return controller_interface::CallbackReturn::ERROR;
  }
  actuator_names_.clear();
  for (const char * base : kActuatorBaseNames) {
    actuator_names_.push_back(hand_side_ + "_" + base);
  }
  // command interface: [0] command_lock + [1..16] target_effort_pct.
  command_interface_names_ = hardware_command_interfaces(hand_side_);

  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::ActuatorEffortCommand>());
  command_subscriber_ =
    get_node()->create_subscription<aidin_hand2_msgs::msg::ActuatorEffortCommand>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::ActuatorEffortCommand> message) {
        command_buffer_.writeFromNonRT(message);
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

// 안전한 무동작값 0%로 16개 reference 전체를 seed.
controller_interface::CallbackReturn ActuatorEffortController::on_activate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::ActuatorEffortCommand>());
  if (reference_interfaces_.size() != kActuatorCount) {
    return controller_interface::CallbackReturn::ERROR;
  }
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

// Resource release는 controller_manager가 처리하며 추가 동작 없음.
controller_interface::CallbackReturn ActuatorEffortController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ── interface configuration ─────────────────────────────────────────────────
// claim: command_lock + ActuatorEffort hardware command port 16개.
controller_interface::InterfaceConfiguration
ActuatorEffortController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

// Effort mode는 activation seed에 hardware state를 요구하지 않음.
controller_interface::InterfaceConfiguration
ActuatorEffortController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

// actuator effort_pct 16개를 reference로 노출.
std::vector<hardware_interface::CommandInterface>
ActuatorEffortController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(
    kActuatorCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(kActuatorCount);
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      actuator_names_[i] + "/" + kReferenceEffortInterfaceName,
      &reference_interfaces_[i]);
  }
  return references;
}

bool ActuatorEffortController::on_set_chained_mode(bool)
{
  return true;
}

// ── update ──────────────────────────────────────────────────────────────────
// standalone: typed command 하나를 reference 전체([0..15] target_effort_pct)에 반영.
controller_interface::return_type
ActuatorEffortController::update_reference_from_subscribers()
{
  const auto message = *command_buffer_.readFromRT();
  if (message) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      reference_interfaces_[i] = message->target_effort_pct[i];
    }
  }
  return controller_interface::return_type::OK;
}

// reference_interfaces_의 완전한 16개 입력을 actuator-effort hardware port에 기록.
controller_interface::return_type ActuatorEffortController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!std::isfinite(reference_interfaces_[i])) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "ActuatorEffort reference contains NaN/Inf");
      return controller_interface::return_type::ERROR;
    }
    (void)command_interfaces_[kHardwareTargetOffset + i].set_value(reference_interfaces_[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::ActuatorEffortController,
  controller_interface::ChainableControllerInterface)

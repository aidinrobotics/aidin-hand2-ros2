#include "aidin_hand2_controllers/actuator_position_controller.hpp"

#include <cmath>
#include <limits>

#include "rclcpp/qos.hpp"

// ── interface name ──────────────────────────────────────────────────────────
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   n          : thumb = 0..3, 그 외 = 1..3
//
//   command interface  : {side}_hand_control/command_lock              (claim-only)
//                        {side}_actuator_position_command/
//                          target_position_cnt.{finger}_actuator{n}      (encoder count)
//   state interface    : {side}_{finger}_actuator{n}/position_cnt       (encoder count)
//   reference interface: {side}_actuator_position_controller/
//                          {side}_{finger}_actuator{n}/position_cnt
//   command topic      : /{side}_actuator_position_controller/command
//                        (aidin_hand2_msgs/ActuatorPositionCommand)
//
//   activation 때 state의 현재 position_cnt로 16개 reference를 seed하고, 매 update에 16개
//   target_position_cnt 전체를 command port에 기록한다.
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
constexpr const char * kPositionInterfaceName = "target_position_cnt";
constexpr const char * kStatePositionInterfaceName = "position_cnt";
constexpr const char * kReferencePositionInterfaceName = "position_cnt";

namespace
{
// claimed command layout: [0] lock, [1..16] target position.
// state/reference layout: [0..15] actuator position.
constexpr std::size_t kHardwareTargetOffset = 1;  // command_interfaces_[0] = command_lock
std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterfaceName};
  const std::string component = side + "_actuator_position_command/";
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(component + kPositionInterfaceName + "." + actuator);
  }
  return names;
}
}  // namespace

// ── lifecycle ────────────────────────────────────────────────────────────────
// Hand side parameter를 선언.
controller_interface::CallbackReturn ActuatorPositionController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

// Side를 검증하고 actuator state/command interface와 typed command subscriber를 구성.
controller_interface::CallbackReturn ActuatorPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(get_node()->get_logger(), "hand_side must be left or right");
    return controller_interface::CallbackReturn::ERROR;
  }
  actuator_names_.clear();
  state_interface_names_.clear();
  // state interface: actuator position_cnt 16개.
  for (const char * base : kActuatorBaseNames) {
    actuator_names_.push_back(hand_side_ + "_" + base);
    state_interface_names_.push_back(
      actuator_names_.back() + "/" + kStatePositionInterfaceName);
  }
  // command interface: [0] command_lock + [1..16] target_position_cnt.
  command_interface_names_ = hardware_command_interfaces(hand_side_);

  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::ActuatorPositionCommand>());
  command_subscriber_ =
    get_node()->create_subscription<aidin_hand2_msgs::msg::ActuatorPositionCommand>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::ActuatorPositionCommand> message) {
        command_buffer_.writeFromNonRT(message);
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

// 현재 actuator position_cnt로 16개 reference 전체를 seed.
controller_interface::CallbackReturn ActuatorPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(
    std::shared_ptr<aidin_hand2_msgs::msg::ActuatorPositionCommand>());
  if (reference_interfaces_.size() != kActuatorCount ||
      state_interfaces_.size() != kActuatorCount)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    reference_interfaces_[i] = state_interfaces_[i].get_value();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// Resource release는 controller_manager가 처리하며 추가 동작 없음.
controller_interface::CallbackReturn ActuatorPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ── interface configuration ─────────────────────────────────────────────────
// claim: command_lock + ActuatorPosition hardware command port 16개.
controller_interface::InterfaceConfiguration
ActuatorPositionController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

// activation seed용 actuator position_cnt state 16개.
controller_interface::InterfaceConfiguration
ActuatorPositionController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interface_names_};
}

// actuator position_cnt 16개를 reference로 노출.
std::vector<hardware_interface::CommandInterface>
ActuatorPositionController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(
    kActuatorCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(kActuatorCount);
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    references.emplace_back(
      get_node()->get_name(),
      actuator_names_[i] + "/" + kReferencePositionInterfaceName,
      &reference_interfaces_[i]);
  }
  return references;
}

bool ActuatorPositionController::on_set_chained_mode(bool)
{
  return true;
}

// ── update ──────────────────────────────────────────────────────────────────
// standalone: typed command 하나를 reference 전체([0..15] target_position_cnt)에 반영.
controller_interface::return_type
ActuatorPositionController::update_reference_from_subscribers()
{
  const auto message = *command_buffer_.readFromRT();
  if (message) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      reference_interfaces_[i] = message->target_position_cnt[i];
    }
  }
  return controller_interface::return_type::OK;
}

// reference_interfaces_의 완전한 16개 입력을 actuator-position hardware port에 기록.
controller_interface::return_type ActuatorPositionController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!std::isfinite(reference_interfaces_[i])) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "ActuatorPosition reference contains NaN/Inf");
      return controller_interface::return_type::ERROR;
    }
    (void)command_interfaces_[kHardwareTargetOffset + i].set_value(reference_interfaces_[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::ActuatorPositionController,
  controller_interface::ChainableControllerInterface)

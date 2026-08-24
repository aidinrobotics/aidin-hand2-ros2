#include "aidin_hand2_controllers/joint_position_controller.hpp"

#include <algorithm>
#include <array>
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
//   입력을 그대로 command interface 로 옮기기만 한다 — 자체 목표를 만들지 않고 state 도 읽지
//   않는다. 입력이 있는 cycle 에만 값이 실리고 그 외에는 NaN(= 이번 cycle 명령 없음)이다. 소비한
//   입력은 즉시 NaN 으로 되돌려 같은 값이 다음 cycle 에 다시 명령으로 나가지 않게 한다. target 의
//   NaN 은 "그 joint 를 상위가 점유하지 않음"이라 hardware 가 채우고, speed 는 상위가 모를 수 있어
//   NaN 이면 speed_rad_s 파라미터로 대체한다. command_lock 은 값으로 쓰지 않고 mode 상호 배제를
//   위한 resource claim 으로만 쓴다.
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

// Side·speed를 검증하고 command interface와 typed command subscriber를 구성.
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
  for (const char * base : kActiveJointBaseNames) {
    active_joint_names_.push_back(hand_side_ + "_" + base);
  }
  // command interface: [0] command_lock + [1..16] target_position_rad + [17] speed_rad_s.
  command_interface_names_ = hardware_command_interfaces(hand_side_);

  drop_buffered_command();
  subscribe();
  return controller_interface::CallbackReturn::SUCCESS;
}

// 활성화 시점에는 목표가 없다 — reference 를 비우고 활성화 이전 message 는 버린다.
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

// Resource release는 controller_manager가 처리하며 추가 동작 없음.
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

// ── interface configuration ─────────────────────────────────────────────────
// claim: command_lock + JointPosition hardware command interface 17개.
controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

// 입력을 옮기기만 하므로 state 는 claim 하지 않는다.
controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
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

// chained 에서는 상위가 reference 를 쓰므로 topic 입력을 내린다(입력 경로 이중화 방지).
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

// ── update ──────────────────────────────────────────────────────────────────
// standalone: 새로 도착한 typed command 한 건만 reference 로 옮긴다. 같은 message 를 다시 반영하면
// 이미 소비한 명령이 매 cycle 되살아난다.
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
  reference_interfaces_[kSpeedIndex] = message->speed_rad_s;
  return controller_interface::return_type::OK;
}

// reference 를 command interface 로 옮긴다. 입력이 없으면 전부 NaN(= 이번 cycle 명령 없음).
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
      target[i] = nan;  // 상위가 점유하지 않은 joint — hardware 가 채운다
    } else if (!std::isfinite(value)) {
      target[i] = nan;
      invalid = true;
    } else {
      target[i] = value;
      has_target = true;
    }
  }

  double speed = reference_interfaces_[kSpeedIndex];
  if (std::isfinite(speed) && speed < 0.0) {
    invalid = true;
  }
  if (!std::isfinite(speed) || speed < 0.0) {
    speed = default_speed_;  // 상위가 speed 를 모를 수 있다 — 파라미터로 대체
  }

  if (invalid) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "JointPosition reference has an invalid value (Inf or negative speed) — ignored");
  }

  for (std::size_t i = 0; i < kTargetCount; ++i) {
    (void)command_interfaces_[kHardwareTargetOffset + i].set_value(
      has_target ? target[i] : nan);
  }
  (void)command_interfaces_[kHardwareSpeedIndex].set_value(has_target ? speed : nan);

  // 소비 표시 — 다음 cycle 에 상위가 다시 쓰지 않으면 명령 없음이 된다.
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::JointPositionController,
  controller_interface::ChainableControllerInterface)

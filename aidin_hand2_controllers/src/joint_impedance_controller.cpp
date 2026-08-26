#include "aidin_hand2_controllers/joint_impedance_controller.hpp"

#include <cmath>
#include <limits>

#include "rclcpp/qos.hpp"

// ── interface name ──────────────────────────────────────────────────────────
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   joint n    : thumb = 0..3, 그 외 = 1..3
//
//   command interface  : {side}_hand_control/command_lock              (claim-only)
//                        {side}_joint_impedance_command/
//                          target_position_rad.{finger}_joint{n}         (rad)
//   reference interface: {side}_joint_impedance_controller/
//                          {side}_{finger}_joint{n}/position             (rad)
//   command topic      : /{side}_joint_impedance_controller/command
//                        (aidin_hand2_msgs/JointImpedanceCommand)
//
//   자세 reference는 JointPositionController와 같은 position 이름을 쓴다. gain 은 SDK
//   ControllerConfig 소관이라 hardware node parameter 로 조절하며 여기서는 다루지 않는다.
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
constexpr const char * kPositionInterfaceName = "target_position_rad";
constexpr const char * kReferencePositionInterfaceName = "position";

namespace
{
constexpr std::size_t kTargetOffset = 0;
constexpr std::size_t kReferenceCount = 16;
// claimed command layout: [0] lock, [1..16] target position.
// exported reference layout: [0..15] target position.
constexpr std::size_t kHardwareOffset = 1;  // command_interfaces_[0] = command_lock

std::vector<std::string> hardware_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{
    side + "_hand_control/" + kCommandLockInterfaceName};
  const std::string component = side + "_joint_impedance_command/";
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(component + kPositionInterfaceName + "." + joint);
  }
  return names;
}
}  // namespace

// ── lifecycle ────────────────────────────────────────────────────────────────
// Side와 기본 impedance gain parameter를 선언.
controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

// Side를 검증하고 command interface와 typed command subscriber를 구성.
controller_interface::CallbackReturn JointImpedanceController::on_configure(
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

  // command interface: [0] command_lock + [1..16] target_position_rad.
  command_interface_names_ = hardware_command_interfaces(hand_side_);
  drop_buffered_command();
  subscribe();
  return controller_interface::CallbackReturn::SUCCESS;
}

// 활성화 시점에는 목표가 없다 — reference 를 비우고 활성화 이전 message 는 버린다.
controller_interface::CallbackReturn JointImpedanceController::on_activate(
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
  command_subscriber_ =
    get_node()->create_subscription<aidin_hand2_msgs::msg::JointImpedanceCommand>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::JointImpedanceCommand> message) {
        command_buffer_.writeFromNonRT(message);
      });
}

void JointImpedanceController::unsubscribe()
{
  command_subscriber_.reset();
}

void JointImpedanceController::drop_buffered_command()
{
  command_buffer_.writeFromNonRT(std::shared_ptr<aidin_hand2_msgs::msg::JointImpedanceCommand>());
  consumed_command_ = nullptr;
}

// ── interface configuration ─────────────────────────────────────────────────
// claim: command_lock + JointImpedance hardware command port 48개.
controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interface_names_};
}

// 입력을 옮기기만 하므로 state 는 claim 하지 않는다.
controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

// 평형 자세 16개를 reference로 노출.
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
  return references;
}

// chained 에서는 상위가 reference 를 쓰므로 topic 입력을 내린다(입력 경로 이중화 방지).
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

// ── update ──────────────────────────────────────────────────────────────────
// standalone: typed command 하나를 reference(자세 16)에 반영.
controller_interface::return_type
JointImpedanceController::update_reference_from_subscribers()
{
  const auto message = *command_buffer_.readFromRT();
  if (!message || message.get() == consumed_command_) {
    return controller_interface::return_type::OK;
  }
  consumed_command_ = message.get();
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    reference_interfaces_[kTargetOffset + i] = message->target_position_rad[i];
  }
  return controller_interface::return_type::OK;
}

// reference 를 command interface 로 옮긴다. 입력이 없으면 전부 NaN(= 이번 cycle 명령 없음).
controller_interface::return_type JointImpedanceController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, kActiveJointCount> target{};
  bool has_target = false;
  bool invalid = false;

  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    const double value = reference_interfaces_[kTargetOffset + i];
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

  if (invalid) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "JointImpedance reference has an Inf value — ignored");
  }

  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    (void)command_interfaces_[kHardwareOffset + kTargetOffset + i].set_value(
      has_target ? target[i] : nan);
  }
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::JointImpedanceController,
  controller_interface::ChainableControllerInterface)

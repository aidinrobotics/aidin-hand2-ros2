#include "aidin_hand2_controllers/diagnostics_broadcaster.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <aidin_hand2/types/hand_lifecycle.hpp>
#include <aidin_hand2/types/state.hpp>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace aidin_hand2_controllers
{

namespace
{
// state_interfaces_ 순서 — HW export_state_interfaces 의 diagnostics gpio 순서와 일치.
// claim 순서: hand 전역 필드 → actuator enabled(16) → actuator fault(16).
// lifecycle(index 0)은 double(HandLifecycle ordinal) — 여기서 to_string 으로 이름화한다.
const std::array<const char *, 7> kDiagnosticsFields = {
  "lifecycle",
  "nan_command_count",
  "control_cycles",
  "deadline_misses",
  "last_period_ms",
  "last_compute_ms",
  "homed"};

// actuator 이름 (prefix 없음) — DiagnosticStatus key 로도 사용.
const std::array<const char *, 16> kActuatorBaseNames = {
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
  "baby_actuator3"};

// enabled·fault 블록 시작 인덱스 — claim 순서에서 파생(필드 개수가 바뀌어도 자동 추종).
constexpr std::size_t kEnabledOffset = kDiagnosticsFields.size();
constexpr std::size_t kFaultOffset = kEnabledOffset + kActuatorBaseNames.size();

std::string bool_string(double value) { return value != 0.0 ? "true" : "false"; }

// diagnostics state interface 의 lifecycle 값(double = HandLifecycle ordinal) → 이름.
std::string lifecycle_string(double value)
{
  return aidin_hand2::to_string(static_cast<aidin_hand2::HandLifecycle>(static_cast<int>(value)));
}
}  // namespace

controller_interface::CallbackReturn DiagnosticsBroadcaster::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiagnosticsBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(get_node()->get_logger(), "hand_side must be 'left' or 'right' (got '%s')",
                 hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  auto diagnostic_array_publisher =
    get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
      diagnostic_array_publisher);

  auto hand_diagnostics_publisher =
    get_node()->create_publisher<aidin_hand2_msgs::msg::HandDiagnostics>(
      "~/hand_diagnostics", rclcpp::SystemDefaultsQoS());
  hand_diagnostics_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::HandDiagnostics>>(
      hand_diagnostics_publisher);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DiagnosticsBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
DiagnosticsBroadcaster::state_interface_configuration() const
{
  const std::string prefix = hand_side_ + "_";
  const std::string diagnostics = prefix + "diagnostics/";
  std::vector<std::string> names;
  for (const auto * field : kDiagnosticsFields) {
    names.push_back(diagnostics + field);
  }
  for (const auto * actuator : kActuatorBaseNames) {
    names.push_back(diagnostics + "enabled_" + actuator);
  }
  for (const auto * actuator : kActuatorBaseNames) {
    names.push_back(diagnostics + "fault_" + actuator);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn DiagnosticsBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiagnosticsBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DiagnosticsBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // 매 호출 발행 — rate 조절은 yaml 의 표준 per-controller update_rate 파라미터로.
  if (publisher_ && publisher_->trylock()) {
    auto & array = publisher_->msg_;
    array.header.stamp = time;
    array.status.clear();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = hand_side_ + "_hand";
    status.hardware_id = hand_side_;

    const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };

    // SDK Diagnostics 7 필드 — lifecycle(이름) · count 3 · ms 2 · homed(bool).
    add(kDiagnosticsFields[0], lifecycle_string(state_interfaces_[0].get_value()));
    for (std::size_t i = 1; i < 4; ++i) {
      add(kDiagnosticsFields[i],
          std::to_string(static_cast<long long>(state_interfaces_[i].get_value())));
    }
    for (std::size_t i = 4; i < 6; ++i) {
      add(kDiagnosticsFields[i], std::to_string(state_interfaces_[i].get_value()));
    }
    add(kDiagnosticsFields[6], bool_string(state_interfaces_[6].get_value()));

    // per-actuator: enabled 는 항상, fault 는 fault 상태일 때만 (이름).
    bool any_fault = false;
    for (std::size_t i = 0; i < 16; ++i) {
      const std::string base = kActuatorBaseNames[i];
      add(base + ".enabled", bool_string(state_interfaces_[kEnabledOffset + i].get_value()));
      const auto fault = static_cast<aidin_hand2::ActuatorFault>(
        static_cast<std::uint16_t>(state_interfaces_[kFaultOffset + i].get_value()));
      if (fault != aidin_hand2::ActuatorFault::None) {
        add(base + ".fault", aidin_hand2::to_string(fault));
        any_fault = true;
      }
    }

    // DiagnosticStatus.level — lifecycle 과 actuator fault 로 그 자리에서 판정한다(별도 요약 필드
    // 없음). 복구 필요(FaultStopping/Faulted) = ERROR, 정상 운영인데 fault 있음 = WARN(일부 mask 후 동작),
    // 그 외 = OK.
    const auto lifecycle =
      static_cast<aidin_hand2::HandLifecycle>(static_cast<int>(state_interfaces_[0].get_value()));
    const bool recovery_needed = lifecycle == aidin_hand2::HandLifecycle::FaultStopping ||
                                 lifecycle == aidin_hand2::HandLifecycle::Faulted;
    if (recovery_needed) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "recovery needed";
    } else if (any_fault) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "actuator fault (degraded)";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    }

    array.status.push_back(status);
    publisher_->unlockAndPublish();
  }

  // 커스텀 HandDiagnostics — 같은 값을 고정 필드로 (Topic Monitor·프로그램 구독용).
  if (hand_diagnostics_publisher_ && hand_diagnostics_publisher_->trylock()) {
    auto & message = hand_diagnostics_publisher_->msg_;
    message.header.stamp = time;
    message.hand_side = hand_side_;
    message.lifecycle = lifecycle_string(state_interfaces_[0].get_value());
    message.nan_command_count = static_cast<std::uint64_t>(state_interfaces_[1].get_value());
    message.control_cycles = static_cast<std::uint64_t>(state_interfaces_[2].get_value());
    message.deadline_misses = static_cast<std::uint64_t>(state_interfaces_[3].get_value());
    message.last_period_ms = state_interfaces_[4].get_value();
    message.last_compute_ms = state_interfaces_[5].get_value();
    message.homed = state_interfaces_[6].get_value() != 0.0;
    for (std::size_t i = 0; i < 16; ++i) {
      message.actuator_enabled[i] = state_interfaces_[kEnabledOffset + i].get_value() != 0.0;
      const auto fault = static_cast<aidin_hand2::ActuatorFault>(
        static_cast<std::uint16_t>(state_interfaces_[kFaultOffset + i].get_value()));
      message.actuator_fault_name[i] = aidin_hand2::to_string(fault);  // None → ""
    }
    hand_diagnostics_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::DiagnosticsBroadcaster, controller_interface::ControllerInterface)

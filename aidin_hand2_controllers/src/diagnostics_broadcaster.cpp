#include "aidin_hand2_controllers/diagnostics_broadcaster.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

// --------------------------- State interface name ---------------------------
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//   n          : thumb = 0..3, otherwise 1..3
//
//   state interface : {side}_diagnostics/{field}                        (7 fields)
//                     {side}_diagnostics/enabled_{finger}_actuator{n}   (0 or 1)
//                     {side}_diagnostics/fault_{finger}_actuator{n}     (ActuatorFault bits)
//   published topic : ~/hand_diagnostics
//                     (aidin_hand2_msgs/HandDiagnostics)
//
//   The claim is 7 hand-wide fields, then enabled x16, then fault x16, 39 resources
//   The claim order is the interface order the hardware exports, and the offsets below
//   index the claim
//   lifecycle and homing_state arrive as an ordinal and go out as a name
//   An empty fault name is a healthy actuator

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

// Hand-wide fields, in the interface order the hardware exports
constexpr std::array<const char *, 7> kDiagnosticsFields = {
  "lifecycle",
  "nan_command_count",
  "control_cycles",
  "deadline_misses",
  "last_period_ms",
  "last_compute_ms",
  "homing_state",
};

constexpr char kDiagnosticsComponent[] = "diagnostics";
constexpr char kEnabledPrefix[] = "enabled_";
constexpr char kFaultPrefix[] = "fault_";

namespace
{
// Index into kDiagnosticsFields, and into the claimed state interfaces
constexpr std::size_t kLifecycleIndex = 0;
constexpr std::size_t kNanCommandCountIndex = 1;
constexpr std::size_t kControlCyclesIndex = 2;
constexpr std::size_t kDeadlineMissesIndex = 3;
constexpr std::size_t kLastPeriodMsIndex = 4;
constexpr std::size_t kLastComputeMsIndex = 5;
constexpr std::size_t kHomingStateIndex = 6;

// The per-actuator blocks follow the hand-wide fields
constexpr std::size_t kEnabledOffset = kDiagnosticsFields.size();
constexpr std::size_t kFaultOffset = kEnabledOffset + ah2::kActuatorCount;

std::string lifecycle_name(double ordinal)
{
  return ah2::to_string(static_cast<ah2::HandLifecycle>(static_cast<int>(ordinal)));
}

std::string homing_state_name(double ordinal)
{
  return ah2::to_string(static_cast<ah2::HomingState>(static_cast<int>(ordinal)));
}

ah2::ActuatorFault actuator_fault(double bits)
{
  return static_cast<ah2::ActuatorFault>(static_cast<std::uint16_t>(bits));
}
}  // namespace

// --------------------------------- Lifecycle --------------------------------

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
    RCLCPP_ERROR(
      get_node()->get_logger(), "hand_side must be 'left' or 'right', got '%s'",
      hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto publisher = get_node()->create_publisher<aidin_hand2_msgs::msg::HandDiagnostics>(
    "~/hand_diagnostics", rclcpp::SystemDefaultsQoS());
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::HandDiagnostics>>(
      publisher);
  return controller_interface::CallbackReturn::SUCCESS;
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

// -------------------------- Interface configuration -------------------------

controller_interface::InterfaceConfiguration
DiagnosticsBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
DiagnosticsBroadcaster::state_interface_configuration() const
{
  const std::string diagnostics = hand_side_ + "_" + kDiagnosticsComponent + "/";
  std::vector<std::string> names;
  names.reserve(kFaultOffset + ah2::kActuatorCount);
  for (const char * field : kDiagnosticsFields) {
    names.push_back(diagnostics + field);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(diagnostics + kEnabledPrefix + actuator);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(diagnostics + kFaultPrefix + actuator);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

// ---------------------------------- Update ----------------------------------

// Publishes on every call, a cycle that cannot take the publisher lock is skipped
// The rate is the standard per-controller update_rate parameter
controller_interface::return_type DiagnosticsBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }

  auto & message = publisher_->msg_;
  message.header.stamp = time;
  message.hand_side = hand_side_;
  message.lifecycle = lifecycle_name(state_interfaces_[kLifecycleIndex].get_value());
  message.nan_command_count =
    static_cast<std::uint64_t>(state_interfaces_[kNanCommandCountIndex].get_value());
  message.control_cycles =
    static_cast<std::uint64_t>(state_interfaces_[kControlCyclesIndex].get_value());
  message.deadline_misses =
    static_cast<std::uint64_t>(state_interfaces_[kDeadlineMissesIndex].get_value());
  message.last_period_ms = state_interfaces_[kLastPeriodMsIndex].get_value();
  message.last_compute_ms = state_interfaces_[kLastComputeMsIndex].get_value();
  message.homing_state = homing_state_name(state_interfaces_[kHomingStateIndex].get_value());

  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    message.actuator_enabled[i] = state_interfaces_[kEnabledOffset + i].get_value() != 0.0;
    message.actuator_fault_name[i] =
      ah2::to_string(actuator_fault(state_interfaces_[kFaultOffset + i].get_value()));
  }

  publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::DiagnosticsBroadcaster, controller_interface::ControllerInterface)

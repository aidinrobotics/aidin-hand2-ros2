#include "aidin_hand2_hardware/aidin_hand2_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"

// Command interface contract, one hand, 65 resources
//   <side>_hand_control/command_lock                                    x1
//   <side>_joint_position_command/target_position_rad.<active_joint>   x16
//   <side>_joint_impedance_command/target_position_rad.<active_joint>  x16
//   <side>_actuator_position_command/target_position_cnt.<actuator>    x16
//   <side>_actuator_effort_command/target_effort_pct.<actuator>        x16
//
// command_lock is claim-only, and a mode switch takes either an empty set or exactly
// one complete mode set with the lock
// Tuning is not a command, it reaches the SDK from this component's own node parameters
namespace aidin_hand2_hardware
{

namespace
{

// Custom state interfaces carry the unit in the name, the standard joint position does not
constexpr char kPositionCountInterface[] = "position_cnt";
constexpr char kVelocityRpmInterface[] = "velocity_rpm";
constexpr char kCurrentMilliampInterface[] = "current_ma";
constexpr char kEnabledInterface[] = "enabled";
constexpr char kFaultInterface[] = "fault";

constexpr char kPositionInterface[] = "position";

// Hand-wide components, grouped the way diagnostics is
constexpr char kCommandedComponent[] = "commanded";
constexpr char kTimestampComponent[] = "timestamp";

// Command echo, on the <prefix>commanded component
constexpr char kControllerInputModeInterface[] = "controller_input_mode";
constexpr char kControllerOutputTypeInterface[] = "controller_output_type";
constexpr char kSelectedSourceInterface[] = "selected_source";
constexpr char kControllerInputTargetInterface[] = "controller_input_target_position_rad";
constexpr char kControllerInputActuatorPositionInterface[] =
  "controller_input_target_position_cnt";
constexpr char kControllerInputActuatorEffortInterface[] =
  "controller_input_target_effort_pct";
constexpr char kControllerOutputPositionInterface[] = "controller_output_target_position_cnt";
constexpr char kControllerOutputEffortInterface[] = "controller_output_target_effort_pct";
constexpr char kCommandedMaxEffortInterface[] = "max_effort_pct";

// Observation wall-clock for header.stamp, on the <prefix>timestamp component
constexpr char kStampSecInterface[] = "sec";
constexpr char kStampNanosecInterface[] = "nanosec";

// Interface order of the diagnostics gpio, matching the diagnostics_values_ array
// lifecycle goes out as the HandLifecycle ordinal and the broadcaster names it
constexpr std::array<const char *, 7> kDiagnosticsInterfaceNames = {
  "lifecycle",
  "nan_command_count",
  "control_cycles",
  "deadline_misses",
  "last_period_ms",
  "last_compute_ms",
  "homing_state"};

// Finger order of the SDK tactile blocks
constexpr std::array<const char *, 5> kFingerNames = {
  "thumb",
  "index",
  "middle",
  "ring",
  "baby"};

// Interface names without the prefix, in fixed SDK order
// The position in the list is the actuator index, the URDF order does not matter
constexpr std::array<const char *, ah2::kActuatorCount> kActuatorBaseNames = {
  "thumb_actuator0", "thumb_actuator1", "thumb_actuator2", "thumb_actuator3",
  "index_actuator1", "index_actuator2", "index_actuator3",
  "middle_actuator1", "middle_actuator2", "middle_actuator3",
  "ring_actuator1", "ring_actuator2", "ring_actuator3",
  "baby_actuator1", "baby_actuator2", "baby_actuator3"};

constexpr std::array<const char *, ah2::kActiveJointCount> kActiveJointBaseNames = {
  "thumb_joint0", "thumb_joint1", "thumb_joint2", "thumb_joint3",
  "index_joint1", "index_joint2", "index_joint3",
  "middle_joint1", "middle_joint2", "middle_joint3",
  "ring_joint1", "ring_joint2", "ring_joint3",
  "baby_joint1", "baby_joint2", "baby_joint3"};

constexpr std::array<std::size_t, ah2::kActiveJointCount> kActiveToJointIndex = {
  0, 1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19};

// FK joints, the last of each digit being the coupled joint4
constexpr std::array<const char *, ah2::kJointCount> kJointBaseNames = {
  "thumb_joint0",
  "thumb_joint1",
  "thumb_joint2",
  "thumb_joint3",
  "thumb_joint4",
  "index_joint1",
  "index_joint2",
  "index_joint3",
  "index_joint4",
  "middle_joint1",
  "middle_joint2",
  "middle_joint3",
  "middle_joint4",
  "ring_joint1",
  "ring_joint2",
  "ring_joint3",
  "ring_joint4",
  "baby_joint1",
  "baby_joint2",
  "baby_joint3",
  "baby_joint4"};

// Component name a mode writes its targets to
std::string command_component(const std::string & side, ah2::CommandMode mode);

// Every command interface one mode needs, the lock included
std::vector<std::string> mode_command_interfaces(
  const std::string & side, ah2::CommandMode mode, bool include_lock = true);

// Every command interface this component exports
std::vector<std::string> all_command_interfaces(const std::string & side);

// The mode whose complete set the claim matches, Idle when empty and nullopt on a partial claim
std::optional<ah2::CommandMode> exact_mode_for_interfaces(
  const std::string & side, const std::set<std::string> & claimed);

// Whether a command interface array is entirely NaN, which means no command this cycle
template <std::size_t N>
bool all_nan(const std::array<double, N> & values);

// Fills a NaN axis from the previous command, leaving it NaN when there is no previous one
double fill_gap(double value, double previous);

}  // namespace

// ------------------------------- Construction -------------------------------

AidinHand2SystemInterface::~AidinHand2SystemInterface()
{
  stop_service_node();
}

// -------------------------------- Lifecycle ---------------------------------

CallbackReturn AidinHand2SystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Relay SDK logs to ROS logging, once per process
  // The SDK logger is global and tags each line [left] or [right]
  static std::once_flag sdk_log_callback_registered;
  std::call_once(sdk_log_callback_registered, [] {
    ah2::set_log_callback([](ah2::LogLevel level, const std::string & message) {
      const auto sdk_logger = rclcpp::get_logger("aidin_hand2");
      switch (level) {
        case ah2::LogLevel::Trace:
        case ah2::LogLevel::Debug:
          RCLCPP_DEBUG(sdk_logger, "%s", message.c_str());
          break;
        case ah2::LogLevel::Info:
          RCLCPP_INFO(sdk_logger, "%s", message.c_str());
          break;
        case ah2::LogLevel::Warn:
          RCLCPP_WARN(sdk_logger, "%s", message.c_str());
          break;
        case ah2::LogLevel::Error:
          RCLCPP_ERROR(sdk_logger, "%s", message.c_str());
          break;
        default:
          RCLCPP_FATAL(sdk_logger, "%s", message.c_str());
          break;
      }
    });

    // Silence the SDK console sink, leaving the ROS path as the only output
    ah2::set_log_to_console(false);
  });

  // hardware_parameters are all strings
  // An empty value keeps the fallback, a malformed one throws into the catch below
  const auto parameter = [this](const std::string & key) -> std::string {
    const auto found = info_.hardware_parameters.find(key);
    return found == info_.hardware_parameters.end() ? std::string{} : found->second;
  };
  const auto as_int = [&](const std::string & key, int fallback) {
    const std::string value = parameter(key);
    return value.empty() ? fallback : std::stoi(value);
  };
  const auto as_double = [&](const std::string & key, double fallback) {
    const std::string value = parameter(key);
    return value.empty() ? fallback : std::stod(value);
  };

  can_interface_ = parameter("can_interface");
  if (can_interface_.empty()) {
    RCLCPP_FATAL(logger(), "missing hardware parameter: can_interface");
    return CallbackReturn::ERROR;
  }

  const std::string hand_side_text = parameter("hand_side");
  if (hand_side_text == "left") {
    hand_side_ = ah2::HandSide::Left;
  } else if (hand_side_text == "right") {
    hand_side_ = ah2::HandSide::Right;
  } else {
    RCLCPP_FATAL(logger(), "hand_side must be 'left' or 'right', got '%s'", hand_side_text.c_str());
    return CallbackReturn::ERROR;
  }
  hand_side_name_ = hand_side_text;
  prefix_ = hand_side_name_ + "_";

  // xacro emits a bool as "True" or "False"
  const std::string auto_home_text = parameter("auto_home");
  if (auto_home_text != "True" && auto_home_text != "False") {
    RCLCPP_FATAL(logger(), "auto_home must be True/False, got '%s'", auto_home_text.c_str());
    return CallbackReturn::ERROR;
  }
  auto_home_ = auto_home_text == "True";

  // Optional, falling back to the defaults
  auto_reconnect_ = parameter("auto_reconnect") == "True";
  auto_reconnect_home_ = parameter("auto_reconnect_home") == "True";

  try {
    max_effort_ = as_double("max_effort", max_effort_);
    control_rate_ = as_int("control_rate", control_rate_);
    rt_cpu_affinity_ = as_int("rt_cpu_affinity", rt_cpu_affinity_);
    auto_reconnect_timeout_ms_ = as_int("auto_reconnect_timeout_ms", auto_reconnect_timeout_ms_);

    // Comma separated index list such as "0,1,2,3", spaces allowed and the range checked by the SDK
    disabled_actuators_.clear();
    const std::string disabled_text = parameter("disabled_actuators");
    std::string token;
    for (std::size_t i = 0; i <= disabled_text.size(); ++i) {
      const char character = (i < disabled_text.size()) ? disabled_text[i] : ',';
      if (character == ',') {
        if (!token.empty()) disabled_actuators_.push_back(std::stoi(token));
        token.clear();
      } else if (character != ' ') {
        token += character;
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(logger(), "invalid numeric hardware parameter: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    ah2::HandConfig config{can_interface_, hand_side_};
    config.control_rate = control_rate_;
    config.rt_cpu_affinity = rt_cpu_affinity_;
    config.disabled_actuators = disabled_actuators_;
    config.auto_reconnect = auto_reconnect_;
    config.auto_reconnect_timeout_ms = auto_reconnect_timeout_ms_;
    config.auto_reconnect_home = auto_reconnect_home_;
    // Homing is triggered from write() with start_homing(), never by the SDK auto-home
    config.auto_home = false;

    hand_ = manager_.create(config);
    hand_->connect();
    state_ = hand_->get_state();
  } catch (const ah2::Exception &) {
    // The SDK already logged the failure
    if (hand_) {
      manager_.destroy(*hand_);
      hand_.reset();
    }
    return CallbackReturn::ERROR;
  }
  start_service_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  std::string failure_message;
  if (!exec_run(failure_message)) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // stop() cancels a running homing with a quick stop and blocks until the drives confirm
  std::string failure_message;
  if (!exec_stop(failure_message)) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_service_node();
  if (hand_) {
    // destroy() tears down a live connection on its own
    try {
      hand_->disconnect();
    } catch (const ah2::Exception &) {
    }
    try {
      manager_.destroy(*hand_);
    } catch (const ah2::Exception &) {
    }
    hand_.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

// ----------------------------- Interface export -----------------------------

std::vector<hardware_interface::StateInterface>
AidinHand2SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  // Actuator motion and sensing, while enabled and fault go to the diagnostics gpio
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::string joint = prefix_ + kActuatorBaseNames[i];
    interfaces.emplace_back(joint, kPositionCountInterface, &state_.actuators.position_count[i]);
    interfaces.emplace_back(joint, kVelocityRpmInterface, &state_.actuators.velocity_rpm[i]);
    interfaces.emplace_back(joint, kCurrentMilliampInterface, &state_.actuators.current_mA[i]);
  }

  for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
    interfaces.emplace_back(prefix_ + kJointBaseNames[i], kPositionInterface, &joint_position_rad_[i]);
  }

  for (std::size_t finger = 0; finger < ah2::kFingerCount; ++finger) {
    const std::string sensor = prefix_ + kFingerNames[finger] + "_sensor";
    for (std::size_t cell = 0; cell < ah2::kTactileTaxelsPerFinger; ++cell) {
      interfaces.emplace_back(sensor, "tactile_" + std::to_string(cell + 1),
                              &state_.tactile.fingers[finger][cell]);
    }
  }

  // Palm regions, stored flat
  const std::string palm = prefix_ + "palm_sensor";
  std::size_t palm_offset = 0;
  const auto add_palm_region = [&](const char * region_prefix, std::size_t count) {
    for (std::size_t cell = 0; cell < count; ++cell) {
      interfaces.emplace_back(palm, region_prefix + std::to_string(cell + 1),
                              &state_.tactile.palm[palm_offset + cell]);
    }
    palm_offset += count;
  };
  add_palm_region("palm1_upper_", ah2::kPalm1UpperCount);
  add_palm_region("palm1_lower_", ah2::kPalm1LowerCount);
  add_palm_region("palm2_", ah2::kPalm2Count);

  const std::string diagnostics = prefix_ + "diagnostics";
  for (std::size_t i = 0; i < kDiagnosticsInterfaceNames.size(); ++i) {
    interfaces.emplace_back(diagnostics, kDiagnosticsInterfaceNames[i], &diagnostics_values_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      diagnostics, std::string(kEnabledInterface) + "_" + kActuatorBaseNames[i],
      &actuator_enabled_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      diagnostics, std::string(kFaultInterface) + "_" + kActuatorBaseNames[i],
      &actuator_fault_[i]);
  }

  // Command echo, the SDK variant spread over the flat double boundary
  // mode, type and source say which typed field is valid, and the broadcaster rebuilds the message
  const std::string commanded = prefix_ + kCommandedComponent;
  interfaces.emplace_back(commanded, kControllerInputModeInterface, &controller_input_mode_);
  interfaces.emplace_back(commanded, kControllerOutputTypeInterface, &controller_output_type_);
  interfaces.emplace_back(commanded, kSelectedSourceInterface, &selected_source_);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      commanded, std::string(kControllerInputTargetInterface) + "." + kActiveJointBaseNames[i],
      &controller_input_target_rad_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::string suffix = std::string(".") + kActuatorBaseNames[i];
    interfaces.emplace_back(commanded, kControllerInputActuatorPositionInterface + suffix,
                            &controller_input_target_position_cnt_[i]);
    interfaces.emplace_back(commanded, kControllerInputActuatorEffortInterface + suffix,
                            &controller_input_target_effort_pct_[i]);
    interfaces.emplace_back(commanded, kControllerOutputPositionInterface + suffix,
                            &controller_output_target_position_cnt_[i]);
    interfaces.emplace_back(commanded, kControllerOutputEffortInterface + suffix,
                            &controller_output_target_effort_pct_[i]);
    interfaces.emplace_back(commanded, kCommandedMaxEffortInterface + suffix,
                            &commanded_max_effort_pct_[i]);
  }

  const std::string timestamp = prefix_ + kTimestampComponent;
  interfaces.emplace_back(timestamp, kStampSecInterface, &observed_stamp_sec_);
  interfaces.emplace_back(timestamp, kStampNanosecInterface, &observed_stamp_nanosec_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AidinHand2SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  interfaces.emplace_back(
    hand_side_name_ + "_hand_control", "command_lock", &command_lock_);

  const std::string joint_position_component =
    command_component(hand_side_name_, ah2::CommandMode::JointPosition);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_position_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_position_target_rad_[i]);
  }

  const std::string joint_impedance_component =
    command_component(hand_side_name_, ah2::CommandMode::JointImpedance);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_impedance_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_impedance_target_rad_[i]);
  }

  const std::string actuator_position_component =
    command_component(hand_side_name_, ah2::CommandMode::ActuatorPosition);
  const std::string actuator_effort_component =
    command_component(hand_side_name_, ah2::CommandMode::ActuatorEffort);
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      actuator_position_component,
      std::string("target_position_cnt.") + kActuatorBaseNames[i],
      &actuator_position_target_cnt_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      actuator_effort_component,
      std::string("target_effort_pct.") + kActuatorBaseNames[i],
      &actuator_effort_target_pct_[i]);
  }

  return interfaces;
}

// ------------------------------- Command mode -------------------------------

hardware_interface::return_type AidinHand2SystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Apply the start and stop delta to the claim set, then demand one complete mode
  // Neither a suffix guess nor a partial claim is accepted
  const auto all_names = all_command_interfaces(hand_side_name_);
  const std::set<std::string> owned(all_names.begin(), all_names.end());
  pending_command_interfaces_ = active_command_interfaces_;

  for (const std::string & name : stop_interfaces) {
    if (owned.count(name) != 0) pending_command_interfaces_.erase(name);
  }
  for (const std::string & name : start_interfaces) {
    if (owned.count(name) != 0) pending_command_interfaces_.insert(name);
  }

  const auto decoded =
    exact_mode_for_interfaces(hand_side_name_, pending_command_interfaces_);
  if (!decoded) {
    pending_mode_switch_valid_ = false;
    RCLCPP_ERROR(
      logger(),
      "rejected mode switch: command interfaces must be one complete mode port plus command_lock");
    return hardware_interface::return_type::ERROR;
  }

  pending_mode_ = *decoded;
  pending_mode_switch_valid_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2SystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  if (!pending_mode_switch_valid_) return hardware_interface::return_type::ERROR;
  active_command_interfaces_ = pending_command_interfaces_;
  command_mode_ = pending_mode_;
  pending_mode_switch_valid_ = false;

  // No command until a controller writes
  clear_mode_command();
  return hardware_interface::return_type::OK;
}

// ------------------------------- Control loop -------------------------------

hardware_interface::return_type AidinHand2SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!hand_) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    state_ = hand_->get_state();
    joint_position_rad_ = state_.joints.position_rad;

    // NaN marks a field the active input or output type does not carry
    const double unused = std::numeric_limits<double>::quiet_NaN();
    controller_input_target_rad_.fill(unused);
    controller_input_target_position_cnt_.fill(unused);
    controller_input_target_effort_pct_.fill(unused);
    controller_output_target_position_cnt_.fill(unused);
    controller_output_target_effort_pct_.fill(unused);

    controller_input_mode_ = static_cast<double>(
      static_cast<int>(ah2::to_command_mode(state_.commanded.controller_input)));
    if (const auto * input =
        std::get_if<ah2::JointPositionCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_rad_ = input->target;
    } else if (const auto * input =
        std::get_if<ah2::JointImpedanceCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_rad_ = input->target;
    } else if (const auto * input =
        std::get_if<ah2::ActuatorPositionCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_position_cnt_ = input->target;
    } else if (const auto * input =
        std::get_if<ah2::ActuatorEffortCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_effort_pct_ = input->target;
    }

    controller_output_type_ = 0.0;
    if (const auto * output =
        std::get_if<ah2::ActuatorPositionSetpoint>(&state_.commanded.controller_output))
    {
      controller_output_type_ = 1.0;
      controller_output_target_position_cnt_ = output->target_position_cnt;
    } else if (const auto * output =
        std::get_if<ah2::ActuatorEffortSetpoint>(&state_.commanded.controller_output))
    {
      controller_output_type_ = 2.0;
      controller_output_target_effort_pct_ = output->target_effort_pct;
    }
    selected_source_ =
      static_cast<double>(static_cast<int>(state_.commanded.selected_source));
    commanded_max_effort_pct_ = state_.commanded.max_effort_pct;

    // Integer split, a double cannot hold the whole ns count
    observed_stamp_sec_ = static_cast<double>(state_.timestamp / 1000000000LL);
    observed_stamp_nanosec_ = static_cast<double>(state_.timestamp % 1000000000LL);

    // Diagnostics in the gpio interface order
    const ah2::Diagnostics diagnostics = hand_->get_diagnostics();
    diagnostics_values_ = {
      static_cast<double>(static_cast<int>(diagnostics.lifecycle)),
      static_cast<double>(diagnostics.nan_command_count),
      static_cast<double>(diagnostics.control_cycles),
      static_cast<double>(diagnostics.deadline_misses),
      diagnostics.last_period_ms,
      diagnostics.last_compute_ms,
      static_cast<double>(static_cast<int>(diagnostics.homing_state))};

    for (std::size_t actuator = 0; actuator < ah2::kActuatorCount; ++actuator) {
      actuator_enabled_[actuator] = diagnostics.actuator_health.enabled[actuator] ? 1.0 : 0.0;
      actuator_fault_[actuator] = static_cast<double>(
        static_cast<std::uint16_t>(diagnostics.actuator_health.fault[actuator]));
    }
  } catch (const ah2::Exception &) {
    // get_state and get_diagnostics read a lock-free buffer and never throw on a lost link
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!hand_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Ahead of the gates below, tuning lands while the hand is stopped
    apply_tuning_parameters();

    if (!started_.load()) {
      return hardware_interface::return_type::OK;
    }
    if (hand_->get_diagnostics().lifecycle != ah2::HandLifecycle::Running) {
      return hardware_interface::return_type::OK;
    }

    // Trigger homing once per run, non-blocking
    // start_homing() raises the request synchronously and the gate below sees it this cycle
    if (auto_home_ && !hand_->is_homing() &&
        hand_->get_diagnostics().homing_state != ah2::HomingState::Succeeded &&
        !auto_home_triggered_.exchange(true)) {
      hand_->start_homing();
    }

    // An unhomed SDK answers set_command with WrongCallOrder
    // Testing homing_state as well covers the unhomed gap right after a reconnect
    if (hand_->is_homing() ||
        hand_->get_diagnostics().homing_state != ah2::HomingState::Succeeded) {
      return hardware_interface::return_type::OK;
    }

    switch (command_mode_) {
      case ah2::CommandMode::Idle:
        hand_->set_command(ah2::Idle{});
        break;

      case ah2::CommandMode::ActuatorPosition: {
        if (all_nan(actuator_position_target_cnt_)) break;
        ah2::ActuatorPositionCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
          command.target[i] = fill_gap(
            actuator_position_target_cnt_[i], controller_input_target_position_cnt_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::ActuatorEffort: {
        if (all_nan(actuator_effort_target_pct_)) break;
        ah2::ActuatorEffortCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
          command.target[i] = fill_gap(
            actuator_effort_target_pct_[i], controller_input_target_effort_pct_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::JointPosition: {
        if (all_nan(joint_position_target_rad_)) break;
        ah2::JointPositionCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
          command.target[i] = fill_gap(
            joint_position_target_rad_[i], controller_input_target_rad_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::JointImpedance: {
        if (all_nan(joint_impedance_target_rad_)) break;
        ah2::JointImpedanceCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
          command.target[i] = fill_gap(
            joint_impedance_target_rad_[i], controller_input_target_rad_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
    }
  } catch (const ah2::Exception &) {
    // The link was lost between the gate above and here, ~/reconnect recovers it
    return hardware_interface::return_type::OK;
  }
  return hardware_interface::return_type::OK;
}

// ------------------------------- Service node -------------------------------

void AidinHand2SystemInterface::start_service_node()
{
  if (service_node_) {
    return;
  }

  // A hardware component has no node of its own
  service_node_ = std::make_shared<rclcpp::Node>(info_.name);
  run_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/run",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_run(failure_message);
      response->message = response->success ? "running" : failure_message;
    });
  stop_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/stop",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_stop(failure_message);
      response->message = response->success ? "stopped" : failure_message;
    });
  home_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/home",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_home(failure_message);
      response->message = response->success ? "homing started — poll diagnostics 'homing_state'" : failure_message;
    });
  reconnect_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/reconnect",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_reconnect(failure_message);
      response->message = response->success ? "reconnected — call ~/run to resume control" : failure_message;
    });
  declare_tuning_parameters();
  service_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(service_node_);
  service_spin_thread_ = std::thread([this] { service_executor_->spin(); });
}

void AidinHand2SystemInterface::stop_service_node()
{
  // join() waits out a service callback still in flight
  if (service_executor_) {
    service_executor_->cancel();
  }
  if (service_spin_thread_.joinable()) {
    service_spin_thread_.join();
  }
  run_service_.reset();
  stop_service_.reset();
  home_service_.reset();
  reconnect_service_.reset();
  tuning_callback_.reset();
  if (service_executor_ && service_node_) {
    service_executor_->remove_node(service_node_);
  }
  service_executor_.reset();
  service_node_.reset();
}

// ----------------------- Hand action [service thread] -----------------------

bool AidinHand2SystemInterface::exec_run(std::string & failure_message)
{
  try {
    hand_->run();
    started_.store(true);

    // Blank the storage, the SDK holds the current pose until the first command arrives
    clear_mode_command();

    // Re-latch, write() triggers homing again when this run starts unhomed
    auto_home_triggered_.store(false);
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

bool AidinHand2SystemInterface::exec_stop(std::string & failure_message)
{
  try {
    hand_->stop();
    started_.store(false);
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

bool AidinHand2SystemInterface::exec_home(std::string & failure_message)
{
  // Triggers only and returns at once
  // The outcome is observed through diagnostics.homing_state
  try {
    hand_->start_homing();
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

bool AidinHand2SystemInterface::exec_reconnect(std::string & failure_message)
{
  // Rebuilds the link without starting control and drops homing_state to NotRun
  try {
    hand_->reconnect();
    started_.store(false);
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

// ----------------------------- Tuning parameter -----------------------------

void AidinHand2SystemInterface::declare_tuning_parameters()
{
  // Defaults come from the max_effort hardware parameter and the SDK, and launch overrides
  // them through a block named after this node in controllers.yaml
  const ah2::ControllerConfig defaults{};
  staged_max_effort_.fill(max_effort_);
  staged_controller_config_ = defaults;

  // The first write() pushes the declared values
  tuning_dirty_ = true;

  service_node_->declare_parameter("max_effort", std::vector<double>{max_effort_});
  service_node_->declare_parameter(
    "joint_position_controller.filter_enabled", defaults.joint_position_controller.filter_enabled);
  service_node_->declare_parameter(
    "joint_position_controller.cutoff_freq", defaults.joint_position_controller.cutoff_freq);
  service_node_->declare_parameter(
    "joint_position_controller.deadband", defaults.joint_position_controller.deadband);
  service_node_->declare_parameter(
    "joint_impedance_controller.stiffness",
    std::vector<double>(defaults.joint_impedance_controller.stiffness.begin(),
                        defaults.joint_impedance_controller.stiffness.end()));
  service_node_->declare_parameter(
    "joint_impedance_controller.damping",
    std::vector<double>(defaults.joint_impedance_controller.damping.begin(),
                        defaults.joint_impedance_controller.damping.end()));

  // Stage what declare() picked up before attaching the callback, which sees only later changes
  const std::vector<std::string> tuning_names{
    "max_effort",
    "joint_position_controller.filter_enabled",
    "joint_position_controller.cutoff_freq",
    "joint_position_controller.deadband",
    "joint_impedance_controller.stiffness",
    "joint_impedance_controller.damping",
  };
  (void)on_set_tuning_parameters(service_node_->get_parameters(tuning_names));
  tuning_callback_ = service_node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return on_set_tuning_parameters(parameters);
    });
}

rcl_interfaces::msg::SetParametersResult AidinHand2SystemInterface::on_set_tuning_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // An array parameter takes 1 value shared by every actuator or 16 for one each
  // A rejection stops ROS from applying the value, and the range clamp is the SDK's job
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Validate on a copy, one rejected parameter leaves the staging untouched
  std::array<double, ah2::kActuatorCount> max_effort = staged_max_effort_;
  ah2::ControllerConfig config = staged_controller_config_;

  const auto expand = [&result](
    const rclcpp::Parameter & parameter, std::array<double, ah2::kActuatorCount> & target) {
    const std::vector<double> values = parameter.as_double_array();
    if (values.size() != 1 && values.size() != ah2::kActuatorCount) {
      result.successful = false;
      result.reason = parameter.get_name() + " must have 1 or 16 values";
      return;
    }
    for (const double value : values) {
      if (!std::isfinite(value) || value < 0.0) {
        result.successful = false;
        result.reason = parameter.get_name() + " must be finite and >= 0";
        return;
      }
    }
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      target[i] = values.size() == 1 ? values[0] : values[i];
    }
  };
  const auto scalar = [&result](const rclcpp::Parameter & parameter, double & target) {
    const double value = parameter.as_double();
    if (!std::isfinite(value) || value < 0.0) {
      result.successful = false;
      result.reason = parameter.get_name() + " must be finite and >= 0";
      return;
    }
    target = value;
  };

  for (const rclcpp::Parameter & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name == "max_effort") {
      expand(parameter, max_effort);
    } else if (name == "joint_position_controller.filter_enabled") {
      config.joint_position_controller.filter_enabled = parameter.as_bool();
    } else if (name == "joint_position_controller.cutoff_freq") {
      scalar(parameter, config.joint_position_controller.cutoff_freq);
    } else if (name == "joint_position_controller.deadband") {
      scalar(parameter, config.joint_position_controller.deadband);
    } else if (name == "joint_impedance_controller.stiffness") {
      expand(parameter, config.joint_impedance_controller.stiffness);
    } else if (name == "joint_impedance_controller.damping") {
      expand(parameter, config.joint_impedance_controller.damping);
    }
    if (!result.successful) return result;
  }

  std::lock_guard<std::mutex> lock(tuning_mutex_);
  staged_max_effort_ = max_effort;
  staged_controller_config_ = config;
  tuning_dirty_ = true;
  return result;
}

void AidinHand2SystemInterface::apply_tuning_parameters()
{
  std::array<double, ah2::kActuatorCount> max_effort{};
  ah2::ControllerConfig config{};
  {
    std::lock_guard<std::mutex> lock(tuning_mutex_);
    if (!tuning_dirty_) return;
    max_effort = staged_max_effort_;
    config = staged_controller_config_;
    tuning_dirty_ = false;
  }
  hand_->set_max_effort(max_effort);
  hand_->set_controller_config(config);
}

// ------------------------- Command write [CM thread] ------------------------

void AidinHand2SystemInterface::clear_mode_command()
{
  const double unset = std::numeric_limits<double>::quiet_NaN();
  switch (command_mode_) {
    case ah2::CommandMode::ActuatorPosition:
      actuator_position_target_cnt_.fill(unset);
      break;
    case ah2::CommandMode::ActuatorEffort:
      actuator_effort_target_pct_.fill(unset);
      break;
    case ah2::CommandMode::JointPosition:
      joint_position_target_rad_.fill(unset);
      break;
    case ah2::CommandMode::JointImpedance:
      joint_impedance_target_rad_.fill(unset);
      break;
    case ah2::CommandMode::Idle:
      break;
  }
}

void AidinHand2SystemInterface::warn_incomplete_command()
{
  // An axis is NaN with no previous command to fill it, never once claimed
  RCLCPP_WARN_THROTTLE(
    logger(), throttle_clock_, 5000,
    "command has axes that were never commanded — skipped. Send a complete command once.");
}

// --------------------------------- Helpers ----------------------------------

rclcpp::Logger AidinHand2SystemInterface::logger() const
{
  return rclcpp::get_logger(info_.name.empty() ? "aidin_hand2_hardware" : info_.name);
}

namespace
{

std::string command_component(const std::string & side, ah2::CommandMode mode)
{
  switch (mode) {
    case ah2::CommandMode::JointPosition: return side + "_joint_position_command";
    case ah2::CommandMode::JointImpedance: return side + "_joint_impedance_command";
    case ah2::CommandMode::ActuatorPosition: return side + "_actuator_position_command";
    case ah2::CommandMode::ActuatorEffort: return side + "_actuator_effort_command";
    case ah2::CommandMode::Idle: return {};
  }
  return {};
}

std::vector<std::string> mode_command_interfaces(
  const std::string & side, ah2::CommandMode mode, bool include_lock)
{
  std::vector<std::string> names;
  if (mode == ah2::CommandMode::Idle) return names;
  if (include_lock) names.push_back(side + "_hand_control/command_lock");
  const std::string component = command_component(side, mode) + "/";
  if (mode == ah2::CommandMode::JointPosition ||
      mode == ah2::CommandMode::JointImpedance)
  {
    for (const char * joint : kActiveJointBaseNames) {
      names.push_back(component + "target_position_rad." + joint);
    }
  }
  if (mode == ah2::CommandMode::ActuatorPosition) {
    for (const char * actuator : kActuatorBaseNames) {
      names.push_back(component + "target_position_cnt." + actuator);
    }
  } else if (mode == ah2::CommandMode::ActuatorEffort) {
    for (const char * actuator : kActuatorBaseNames) {
      names.push_back(component + "target_effort_pct." + actuator);
    }
  }
  return names;
}

std::vector<std::string> all_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{side + "_hand_control/command_lock"};
  for (const auto mode : {
      ah2::CommandMode::JointPosition, ah2::CommandMode::JointImpedance,
      ah2::CommandMode::ActuatorPosition, ah2::CommandMode::ActuatorEffort})
  {
    const auto mode_names = mode_command_interfaces(side, mode, false);
    names.insert(names.end(), mode_names.begin(), mode_names.end());
  }
  return names;
}

std::optional<ah2::CommandMode> exact_mode_for_interfaces(
  const std::string & side, const std::set<std::string> & claimed)
{
  if (claimed.empty()) return ah2::CommandMode::Idle;
  for (const auto mode : {
      ah2::CommandMode::JointPosition, ah2::CommandMode::JointImpedance,
      ah2::CommandMode::ActuatorPosition, ah2::CommandMode::ActuatorEffort})
  {
    const auto names = mode_command_interfaces(side, mode);
    if (claimed == std::set<std::string>(names.begin(), names.end())) return mode;
  }
  return std::nullopt;
}

template <std::size_t N>
bool all_nan(const std::array<double, N> & values)
{
  for (const double value : values) {
    if (!std::isnan(value)) return false;
  }
  return true;
}

double fill_gap(double value, double previous)
{
  return std::isnan(value) ? previous : value;
}

}  // namespace

}  // namespace aidin_hand2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_hardware::AidinHand2SystemInterface, hardware_interface::SystemInterface)

// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#include "aidin_hand2_hardware/aidin_hand2_mock_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>

#include <aidin_hand2/hand/hand_kinematics.hpp>
#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/state.hpp>

#include "rclcpp/rclcpp.hpp"

// Write path is target -> clamp -> IK -> encoder count -> FK -> joint position
// A mode is accepted only as its exact complete interface set plus the lock
namespace aidin_hand2_hardware
{
namespace
{
namespace ah2 = aidin_hand2;

constexpr char kPositionInterface[] = "position";
constexpr char kPositionCntInterface[] = "position_cnt";
constexpr char kVelocityRpmInterface[] = "velocity_rpm";
constexpr char kCurrentMaInterface[] = "current_ma";

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
  "baby_joint4",
};

// The names the real hardware exports and the broadcasters claim
constexpr std::array<const char *, ah2::kFingerCount> kFingerNames = {
  "thumb", "index", "middle", "ring", "baby"};
constexpr std::array<const char *, 7> kDiagnosticsInterfaceNames = {
  "lifecycle",
  "nan_command_count",
  "control_cycles",
  "deadline_misses",
  "last_period_ms",
  "last_compute_ms",
  "homing_state",
};
constexpr std::size_t kLifecycleField = 0;
constexpr std::size_t kControlCyclesField = 2;
constexpr std::size_t kLastPeriodField = 4;
constexpr std::size_t kHomingStateField = 6;

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
  const std::string & side, ah2::CommandMode mode, bool include_lock = true)
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

// All NaN means no command this cycle, partial NaN means unowned entries
template <std::size_t N>
bool all_nan(const std::array<double, N> & values)
{
  for (const double value : values) {
    if (!std::isnan(value)) return false;
  }
  return true;
}

template <std::size_t N>
bool any_nan(const std::array<double, N> & values)
{
  for (const double value : values) {
    if (std::isnan(value)) return true;
  }
  return false;
}

double fill_gap(double value, double previous)
{
  return std::isnan(value) ? previous : value;
}

}  // namespace

hardware_interface::CallbackReturn AidinHand2MockSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto side = info_.hardware_parameters.find("hand_side");
  hand_side_ = side == info_.hardware_parameters.end() ? "left" : side->second;
  if (hand_side_ != "left" && hand_side_ != "right") {
    return hardware_interface::CallbackReturn::ERROR;
  }
  prefix_ = hand_side_ + "_";
  const auto effort = info_.hardware_parameters.find("max_effort");
  if (effort != info_.hardware_parameters.end()) {
    try {
      max_effort_pct_ = std::clamp(std::stod(effort->second), 0.0, 2000.0);
    } catch (const std::exception &) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const std::array<int, ah2::kActuatorCount> zero_encoder{};
  joint_position_rad_ = ah2::fk_actuator_to_joint(zero_encoder);

  // Up and homed from the start: the mock has no link to bring up and no homing to run
  diagnostics_values_[kLifecycleField] =
    static_cast<double>(static_cast<int>(ah2::HandLifecycle::Running));
  diagnostics_values_[kHomingStateField] =
    static_cast<double>(static_cast<int>(ah2::HomingState::Succeeded));
  actuator_enabled_.fill(1.0);
  commanded_max_effort_pct_.fill(max_effort_pct_);
  echo_command({}, {});
  return hardware_interface::CallbackReturn::SUCCESS;
}

// The command echo the real hardware reads back from the SDK, for a joint position command only
void AidinHand2MockSystemInterface::echo_command(
  const std::optional<std::array<double, ah2::kActiveJointCount>> & joint_target,
  const std::optional<std::array<int, ah2::kActuatorCount>> & encoder)
{
  const double unused = std::numeric_limits<double>::quiet_NaN();
  controller_input_target_rad_.fill(unused);
  controller_input_target_position_cnt_.fill(unused);
  controller_input_target_effort_pct_.fill(unused);
  controller_output_target_position_cnt_.fill(unused);
  controller_output_target_effort_pct_.fill(unused);
  controller_input_mode_ = static_cast<double>(static_cast<int>(command_mode_));
  controller_output_type_ = 0.0;
  selected_source_ = static_cast<double>(static_cast<int>(
    command_mode_ == ah2::CommandMode::Idle ? ah2::CommandSource::None :
    ah2::CommandSource::Controller));
  if (joint_target) controller_input_target_rad_ = *joint_target;
  if (encoder) {
    controller_output_type_ = 1.0;  // ActuatorPositionSetpoint
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      controller_output_target_position_cnt_[i] = static_cast<double>((*encoder)[i]);
    }
  }
}

std::vector<hardware_interface::StateInterface>
AidinHand2MockSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::string actuator = prefix_ + kActuatorBaseNames[i];
    interfaces.emplace_back(actuator, kPositionCntInterface, &actuator_position_cnt_[i]);
    interfaces.emplace_back(actuator, kVelocityRpmInterface, &actuator_velocity_rpm_[i]);
    interfaces.emplace_back(actuator, kCurrentMaInterface, &actuator_current_ma_[i]);
  }
  for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
    interfaces.emplace_back(
      prefix_ + kJointBaseNames[i], kPositionInterface, &joint_position_rad_[i]);
  }

  for (std::size_t finger = 0; finger < ah2::kFingerCount; ++finger) {
    const std::string sensor = prefix_ + kFingerNames[finger] + "_sensor";
    for (std::size_t cell = 0; cell < ah2::kTactileTaxelsPerFinger; ++cell) {
      interfaces.emplace_back(sensor, "tactile_" + std::to_string(cell + 1),
                              &tactile_fingers_[finger][cell]);
    }
  }
  const std::string palm = prefix_ + "palm_sensor";
  std::size_t palm_offset = 0;
  const auto add_palm_region = [&](const char * region_prefix, std::size_t count) {
    for (std::size_t cell = 0; cell < count; ++cell) {
      interfaces.emplace_back(palm, region_prefix + std::to_string(cell + 1),
                              &tactile_palm_[palm_offset + cell]);
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
      diagnostics, std::string("enabled_") + kActuatorBaseNames[i], &actuator_enabled_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      diagnostics, std::string("fault_") + kActuatorBaseNames[i], &actuator_fault_[i]);
  }

  const std::string commanded = prefix_ + "commanded";
  interfaces.emplace_back(commanded, "controller_input_mode", &controller_input_mode_);
  interfaces.emplace_back(commanded, "controller_output_type", &controller_output_type_);
  interfaces.emplace_back(commanded, "selected_source", &selected_source_);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      commanded, std::string("controller_input_target_position_rad.") + kActiveJointBaseNames[i],
      &controller_input_target_rad_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::string suffix = std::string(".") + kActuatorBaseNames[i];
    interfaces.emplace_back(commanded, "controller_input_target_position_cnt" + suffix,
                            &controller_input_target_position_cnt_[i]);
    interfaces.emplace_back(commanded, "controller_input_target_effort_pct" + suffix,
                            &controller_input_target_effort_pct_[i]);
    interfaces.emplace_back(commanded, "controller_output_target_position_cnt" + suffix,
                            &controller_output_target_position_cnt_[i]);
    interfaces.emplace_back(commanded, "controller_output_target_effort_pct" + suffix,
                            &controller_output_target_effort_pct_[i]);
    interfaces.emplace_back(commanded, "max_effort_pct" + suffix, &commanded_max_effort_pct_[i]);
  }

  const std::string timestamp = prefix_ + "timestamp";
  interfaces.emplace_back(timestamp, "sec", &observed_stamp_sec_);
  interfaces.emplace_back(timestamp, "nanosec", &observed_stamp_nanosec_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AidinHand2MockSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    hand_side_ + "_hand_control", "command_lock", &command_lock_);

  const std::string joint_position_component =
    command_component(hand_side_, ah2::CommandMode::JointPosition);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_position_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_position_target_rad_[i]);
  }

  const std::string joint_impedance_component =
    command_component(hand_side_, ah2::CommandMode::JointImpedance);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_impedance_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_impedance_target_rad_[i]);
  }

  const std::string actuator_position_component =
    command_component(hand_side_, ah2::CommandMode::ActuatorPosition);
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      actuator_position_component,
      std::string("target_position_cnt.") + kActuatorBaseNames[i],
      &actuator_position_target_cnt_[i]);
  }
  const std::string actuator_effort_component =
    command_component(hand_side_, ah2::CommandMode::ActuatorEffort);
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      actuator_effort_component,
      std::string("target_effort_pct.") + kActuatorBaseNames[i],
      &actuator_effort_target_pct_[i]);
  }
  return interfaces;
}

hardware_interface::return_type AidinHand2MockSystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto all_names = all_command_interfaces(hand_side_);
  const std::set<std::string> owned(all_names.begin(), all_names.end());
  pending_command_interfaces_ = active_command_interfaces_;
  for (const std::string & name : stop_interfaces) {
    if (owned.count(name) != 0) pending_command_interfaces_.erase(name);
  }
  for (const std::string & name : start_interfaces) {
    if (owned.count(name) != 0) pending_command_interfaces_.insert(name);
  }

  const auto decoded =
    exact_mode_for_interfaces(hand_side_, pending_command_interfaces_);
  if (!decoded) {
    pending_mode_switch_valid_ = false;
    return hardware_interface::return_type::ERROR;
  }
  pending_mode_ = *decoded;
  pending_mode_switch_valid_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2MockSystemInterface::perform_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  if (!pending_mode_switch_valid_) return hardware_interface::return_type::ERROR;
  active_command_interfaces_ = pending_command_interfaces_;
  command_mode_ = pending_mode_;
  pending_mode_switch_valid_ = false;

  // No command until a controller writes
  const double unset = std::numeric_limits<double>::quiet_NaN();
  held_joint_target_rad_.fill(unset);
  held_actuator_target_cnt_.fill(unset);
  switch (command_mode_) {
    case ah2::CommandMode::JointPosition:
      joint_position_target_rad_.fill(unset);
      break;
    case ah2::CommandMode::JointImpedance:
      joint_impedance_target_rad_.fill(unset);
      break;
    case ah2::CommandMode::ActuatorPosition:
      actuator_position_target_cnt_.fill(unset);
      break;
    case ah2::CommandMode::ActuatorEffort:
      actuator_effort_target_pct_.fill(unset);
      break;
    case ah2::CommandMode::Idle:
      break;
  }
  return hardware_interface::return_type::OK;
}

// write() advances the pose. read() only stamps the cycle, as the observation clock the real
// hardware takes from the SDK
hardware_interface::return_type AidinHand2MockSystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const std::int64_t ns = time.nanoseconds();
  observed_stamp_sec_ = static_cast<double>(ns / 1000000000LL);
  observed_stamp_nanosec_ = static_cast<double>(ns % 1000000000LL);
  control_cycles_ += 1.0;
  diagnostics_values_[kControlCyclesField] = control_cycles_;
  diagnostics_values_[kLastPeriodField] = period.seconds() * 1000.0;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2MockSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::array<int, ah2::kActuatorCount> encoder{};
  echo_command({}, {});
  if (command_mode_ == ah2::CommandMode::JointPosition) {
    if (!all_nan(joint_position_target_rad_)) {
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        held_joint_target_rad_[i] = fill_gap(
          joint_position_target_rad_[i], held_joint_target_rad_[i]);
      }
    }
    if (any_nan(held_joint_target_rad_)) {
      return hardware_interface::return_type::OK;  // Target incomplete, hold the pose
    }
    ah2::JointPositionCommand command;
    command.target = held_joint_target_rad_;
    command.clamp();
    encoder = ah2::ik_joint_to_actuator(command.target);
    echo_command(held_joint_target_rad_, encoder);
  } else if (command_mode_ == ah2::CommandMode::JointImpedance) {
    if (!all_nan(joint_impedance_target_rad_)) {
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        held_joint_target_rad_[i] = fill_gap(
          joint_impedance_target_rad_[i], held_joint_target_rad_[i]);
      }
    }
    if (any_nan(held_joint_target_rad_)) {
      return hardware_interface::return_type::OK;
    }
    ah2::JointImpedanceCommand command;
    command.target = held_joint_target_rad_;
    command.clamp();
    encoder = ah2::ik_joint_to_actuator(command.target);
  } else if (command_mode_ == ah2::CommandMode::ActuatorPosition) {
    if (!all_nan(actuator_position_target_cnt_)) {
      for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
        held_actuator_target_cnt_[i] = fill_gap(
          actuator_position_target_cnt_[i], held_actuator_target_cnt_[i]);
      }
    }
    if (any_nan(held_actuator_target_cnt_)) {
      return hardware_interface::return_type::OK;
    }
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      encoder[i] = static_cast<int>(std::lround(held_actuator_target_cnt_[i]));
    }
  } else {
    // Idle and ActuatorEffort hold the pose, the mock has no torque model
    if (command_mode_ == ah2::CommandMode::ActuatorEffort) {
      for (double & effort : actuator_effort_target_pct_) {
        if (std::isnan(effort)) continue;  // Unowned or no command
        effort = std::clamp(effort, -max_effort_pct_, max_effort_pct_);
      }
    }
    return hardware_interface::return_type::OK;
  }

  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    actuator_position_cnt_[i] = static_cast<double>(encoder[i]);
  }
  joint_position_rad_ = ah2::fk_actuator_to_joint(encoder);
  return hardware_interface::return_type::OK;
}

}  // namespace aidin_hand2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_hardware::AidinHand2MockSystemInterface,
  hardware_interface::SystemInterface)

// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#include "aidin_hand2_controllers/hand_state_broadcaster.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>

// --------------------------- State interface name ---------------------------
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//
//   state interface : {side}_{joint}/position                          (rad)
//                     {side}_{joint}/velocity                          (rad/s)
//                     {side}_{active_joint}/effort                     (N·m)
//                     {side}_{actuator}/position_cnt                   (encoder count)
//                     {side}_{actuator}/velocity_rpm                   (rpm)
//                     {side}_{actuator}/current_ma                     (mA)
//                     {side}_{finger}_sensor/tactile_1..17
//                     {side}_palm_sensor/palm1_upper_1..20
//                     {side}_palm_sensor/palm1_lower_1..20
//                     {side}_palm_sensor/palm2_1..18
//                     {side}_commanded/...                             (command echo)
//                     {side}_timestamp/sec, nanosec                    (observation clock)
//   published topic : ~/hand_state
//                     (aidin_hand2_msgs/HandState)
//
//   Claim order, shared by state_interface_configuration and update
//     [0..20]    joint position                          21
//     [21..41]   joint velocity                          21
//     [42..57]   active joint effort                     16
//     [58..73]   actuator position_cnt                   16
//     [74..89]   actuator velocity_rpm                   16
//     [90..105]  actuator current_ma                     16
//     [106..190] finger tactile                          5 x 17 = 85
//     [191..248] palm tactile                            20 + 20 + 18 = 58
//     [249..251] command scalar, mode, type, source      3
//     [252..267] command input joint target              16
//     [268..331] command per-actuator, 4 interleaved     16 x 4 = 64
//     [332..333] timestamp sec, nanosec                  2
//
//   The offsets below index that order, and all_state_interface_names appends in it

namespace aidin_hand2_controllers
{

namespace ah2 = aidin_hand2;
using CommandState = aidin_hand2_msgs::msg::CommandState;

// Interface names without the prefix
// The position in the list is the joint index, the coupled joint4 of each digit included
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

// The position in the list is the active joint index, which the command echo is ordered by
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

// Joint index of each active joint
constexpr std::array<std::size_t, ah2::kActiveJointCount> kActiveToJointIndex = {
  0, 1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19};

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

// The position in the list is the tactile block index
constexpr std::array<const char *, ah2::kFingerCount> kFingerNames = {
  "thumb",
  "index",
  "middle",
  "ring",
  "baby",
};

constexpr char kPositionInterface[] = "position";
constexpr char kVelocityInterface[] = "velocity";
constexpr char kEffortInterface[] = "effort";
constexpr char kPositionCntInterface[] = "position_cnt";
constexpr char kVelocityRpmInterface[] = "velocity_rpm";
constexpr char kCurrentMaInterface[] = "current_ma";

constexpr char kSensorSuffix[] = "_sensor";
constexpr char kPalmSensorComponent[] = "palm_sensor";
constexpr char kTactilePrefix[] = "tactile_";
constexpr char kPalm1UpperRegion[] = "palm1_upper";
constexpr char kPalm1LowerRegion[] = "palm1_lower";
constexpr char kPalm2Region[] = "palm2";

constexpr char kCommandedComponent[] = "commanded";
constexpr char kControllerInputModeInterface[] = "controller_input_mode";
constexpr char kControllerOutputTypeInterface[] = "controller_output_type";
constexpr char kSelectedSourceInterface[] = "selected_source";
constexpr char kControllerInputTargetInterface[] = "controller_input_target_position_rad";
constexpr char kControllerInputActuatorPositionInterface[] =
  "controller_input_target_position_cnt";
constexpr char kControllerInputActuatorEffortInterface[] = "controller_input_target_effort_pct";
constexpr char kControllerOutputPositionInterface[] = "controller_output_target_position_cnt";
constexpr char kControllerOutputEffortInterface[] = "controller_output_target_effort_pct";

constexpr char kTimestampComponent[] = "timestamp";
constexpr char kStampSecInterface[] = "sec";
constexpr char kStampNanosecInterface[] = "nanosec";

namespace
{
constexpr std::size_t kJointOffset = 0;
constexpr std::size_t kJointVelocityOffset = 21;
constexpr std::size_t kJointEffortOffset = 42;
constexpr std::size_t kActuatorPositionOffset = 58;
constexpr std::size_t kActuatorVelocityOffset = 74;
constexpr std::size_t kActuatorCurrentOffset = 90;
constexpr std::size_t kFingerTactileOffset = 106;
constexpr std::size_t kPalmTactileOffset = 191;
constexpr std::size_t kControllerInputModeOffset = 249;
constexpr std::size_t kControllerOutputTypeOffset = 250;
constexpr std::size_t kSelectedSourceOffset = 251;
constexpr std::size_t kControllerInputJointOffset = 252;
constexpr std::size_t kCommandedActuatorOffset = 268;
constexpr std::size_t kCommandedActuatorStride = 4;
constexpr std::size_t kTimestampSecOffset = 332;
constexpr std::size_t kTimestampNanosecOffset = 333;
constexpr std::size_t kStateInterfaceCount = kTimestampNanosecOffset + 1;

void append_finger_tactile(std::vector<std::string> & names, const std::string & sensor)
{
  for (std::size_t cell = 1; cell <= ah2::kTactileTaxelsPerFinger; ++cell) {
    names.push_back(sensor + "/" + kTactilePrefix + std::to_string(cell));
  }
}

void append_palm_region(
  std::vector<std::string> & names, const std::string & sensor, const std::string & region,
  std::size_t count)
{
  for (std::size_t cell = 1; cell <= count; ++cell) {
    names.push_back(sensor + "/" + region + "_" + std::to_string(cell));
  }
}

std::vector<std::string> all_state_interface_names(const std::string & prefix)
{
  std::vector<std::string> names;
  names.reserve(kStateInterfaceCount);

  for (const char * joint : kJointBaseNames) {
    names.push_back(prefix + joint + "/" + kPositionInterface);
  }
  for (const char * joint : kJointBaseNames) {
    names.push_back(prefix + joint + "/" + kVelocityInterface);
  }
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(prefix + joint + "/" + kEffortInterface);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(prefix + actuator + "/" + kPositionCntInterface);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(prefix + actuator + "/" + kVelocityRpmInterface);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(prefix + actuator + "/" + kCurrentMaInterface);
  }

  for (const char * finger : kFingerNames) {
    append_finger_tactile(names, prefix + finger + kSensorSuffix);
  }
  const std::string palm = prefix + kPalmSensorComponent;
  append_palm_region(names, palm, kPalm1UpperRegion, ah2::kPalm1UpperCount);
  append_palm_region(names, palm, kPalm1LowerRegion, ah2::kPalm1LowerCount);
  append_palm_region(names, palm, kPalm2Region, ah2::kPalm2Count);

  const std::string commanded = prefix + kCommandedComponent;
  names.push_back(commanded + "/" + kControllerInputModeInterface);
  names.push_back(commanded + "/" + kControllerOutputTypeInterface);
  names.push_back(commanded + "/" + kSelectedSourceInterface);
  for (const char * joint : kActiveJointBaseNames) {
    names.push_back(commanded + "/" + kControllerInputTargetInterface + "." + joint);
  }
  for (const char * actuator : kActuatorBaseNames) {
    names.push_back(
      commanded + "/" + kControllerInputActuatorPositionInterface + "." + actuator);
    names.push_back(commanded + "/" + kControllerInputActuatorEffortInterface + "." + actuator);
    names.push_back(commanded + "/" + kControllerOutputPositionInterface + "." + actuator);
    names.push_back(commanded + "/" + kControllerOutputEffortInterface + "." + actuator);
  }

  const std::string timestamp = prefix + kTimestampComponent;
  names.push_back(timestamp + "/" + kStampSecInterface);
  names.push_back(timestamp + "/" + kStampNanosecInterface);
  return names;
}
}  // namespace

// --------------------------------- Lifecycle --------------------------------

controller_interface::CallbackReturn HandStateBroadcaster::on_init()
{
  auto_declare<std::string>("hand_side", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HandStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(
      get_node()->get_logger(), "hand_side must be 'left' or 'right', got '%s'",
      hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto publisher = get_node()->create_publisher<aidin_hand2_msgs::msg::HandState>(
    "~/hand_state", rclcpp::SystemDefaultsQoS());
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::HandState>>(
      publisher);
  publisher_->msg_.hand_side = hand_side_;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HandStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HandStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------- Interface configuration -------------------------

controller_interface::InterfaceConfiguration
HandStateBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
HandStateBroadcaster::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          all_state_interface_names(hand_side_ + "_")};
}

// ---------------------------------- Update ----------------------------------

// Publishes on every call, a cycle that cannot take the publisher lock is skipped
// The rate is the standard per-controller update_rate parameter
controller_interface::return_type HandStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }

  auto & message = publisher_->msg_;

  // header.stamp is the wall-clock of the observation, not of this publish
  // sec = 0 is a stamp the hardware has not filled yet, and the publish time stands in
  const auto stamp_sec =
    static_cast<std::int32_t>(state_interfaces_[kTimestampSecOffset].get_value());
  const auto stamp_nanosec =
    static_cast<std::uint32_t>(state_interfaces_[kTimestampNanosecOffset].get_value());
  if (stamp_sec > 0) {
    message.header.stamp.sec = stamp_sec;
    message.header.stamp.nanosec = stamp_nanosec;
  } else {
    message.header.stamp = time;
  }

  for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
    message.joint_position[i] = state_interfaces_[kJointOffset + i].get_value();
    message.joint_velocity[i] = state_interfaces_[kJointVelocityOffset + i].get_value();
  }

  // A coupled joint4 has no effort interface and stays NaN
  message.joint_effort.fill(std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    message.joint_effort[kActiveToJointIndex[i]] =
      state_interfaces_[kJointEffortOffset + i].get_value();
  }
  // A state interface is always a double, so each reading returns to its wire width here
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    message.actuator_position[i] =
      static_cast<std::int32_t>(state_interfaces_[kActuatorPositionOffset + i].get_value());
    message.actuator_velocity[i] =
      static_cast<std::int32_t>(state_interfaces_[kActuatorVelocityOffset + i].get_value());
    message.actuator_current[i] =
      static_cast<std::int16_t>(state_interfaces_[kActuatorCurrentOffset + i].get_value());
  }

  const auto taxel = [this](std::size_t index) {
    return static_cast<std::int32_t>(state_interfaces_[index].get_value());
  };

  for (std::size_t i = 0; i < ah2::kTactileTaxelsPerFinger; ++i) {
    const std::size_t block = kFingerTactileOffset + i;
    const std::size_t stride = ah2::kTactileTaxelsPerFinger;
    message.tactile_thumb[i] = taxel(block + 0 * stride);
    message.tactile_index[i] = taxel(block + 1 * stride);
    message.tactile_middle[i] = taxel(block + 2 * stride);
    message.tactile_ring[i] = taxel(block + 3 * stride);
    message.tactile_baby[i] = taxel(block + 4 * stride);
  }
  for (std::size_t i = 0; i < ah2::kPalm1UpperCount; ++i) {
    message.tactile_palm1_upper[i] = taxel(kPalmTactileOffset + i);
  }
  for (std::size_t i = 0; i < ah2::kPalm1LowerCount; ++i) {
    message.tactile_palm1_lower[i] = taxel(kPalmTactileOffset + ah2::kPalm1UpperCount + i);
  }
  for (std::size_t i = 0; i < ah2::kPalm2Count; ++i) {
    message.tactile_palm2[i] =
      taxel(kPalmTactileOffset + ah2::kPalm1UpperCount + ah2::kPalm1LowerCount + i);
  }

  auto & command = message.command_state;
  command.controller_input_mode =
    static_cast<std::uint8_t>(state_interfaces_[kControllerInputModeOffset].get_value());
  command.controller_output_type =
    static_cast<std::uint8_t>(state_interfaces_[kControllerOutputTypeOffset].get_value());
  command.selected_source =
    static_cast<std::uint8_t>(state_interfaces_[kSelectedSourceOffset].get_value());

  // The hardware exports one array per input and output kind, and the message keeps the active one
  // field is the per-actuator interface: 0 input position, 1 input effort, 2 output position,
  // 3 output effort
  const auto per_actuator = [this](std::size_t field, std::array<double, 16> & target) {
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      target[i] =
        state_interfaces_[kCommandedActuatorOffset + i * kCommandedActuatorStride + field].get_value();
    }
  };

  command.controller_input.fill(std::numeric_limits<double>::quiet_NaN());
  switch (command.controller_input_mode) {
    case CommandState::CONTROLLER_INPUT_JOINT_POSITION:
    case CommandState::CONTROLLER_INPUT_JOINT_IMPEDANCE:
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        command.controller_input[i] = state_interfaces_[kControllerInputJointOffset + i].get_value();
      }
      break;
    case CommandState::CONTROLLER_INPUT_ACTUATOR_POSITION:
      per_actuator(0, command.controller_input);
      break;
    case CommandState::CONTROLLER_INPUT_ACTUATOR_EFFORT:
      per_actuator(1, command.controller_input);
      break;
    default:
      break;
  }

  command.controller_output.fill(std::numeric_limits<double>::quiet_NaN());
  switch (command.controller_output_type) {
    case CommandState::OUTPUT_ACTUATOR_POSITION:
      per_actuator(2, command.controller_output);
      break;
    case CommandState::OUTPUT_ACTUATOR_EFFORT:
      per_actuator(3, command.controller_output);
      break;
    default:
      break;
  }

  publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::HandStateBroadcaster, controller_interface::ControllerInterface)

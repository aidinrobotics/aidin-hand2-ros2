#include "aidin_hand2_controllers/hand_state_broadcaster.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>

// --------------------------- State interface name ---------------------------
//   side       ∈ {left, right}
//   finger     ∈ {thumb, index, middle, ring, baby}
//
//   state interface : {side}_{joint}/position                          (rad)
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
//     [21..36]   actuator position_cnt                   16
//     [37..52]   actuator velocity_rpm                   16
//     [53..68]   actuator current_ma                     16
//     [69..153]  finger tactile                          5 x 17 = 85
//     [154..211] palm tactile                            20 + 20 + 18 = 58
//     [212..214] command scalar, mode, type, source      3
//     [215..230] command input joint target              16
//     [231..310] command per-actuator, 5 interleaved     16 x 5 = 80
//     [311..312] timestamp sec, nanosec                  2
//
//   The offsets below index that order, and all_state_interface_names appends in it

namespace aidin_hand2_controllers
{

namespace ah2 = aidin_hand2;

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
constexpr char kCommandedMaxEffortInterface[] = "max_effort_pct";

constexpr char kTimestampComponent[] = "timestamp";
constexpr char kStampSecInterface[] = "sec";
constexpr char kStampNanosecInterface[] = "nanosec";

namespace
{
constexpr std::size_t kJointOffset = 0;
constexpr std::size_t kActuatorPositionOffset = 21;
constexpr std::size_t kActuatorVelocityOffset = 37;
constexpr std::size_t kActuatorCurrentOffset = 53;
constexpr std::size_t kFingerTactileOffset = 69;
constexpr std::size_t kPalmTactileOffset = 154;
constexpr std::size_t kControllerInputModeOffset = 212;
constexpr std::size_t kControllerOutputTypeOffset = 213;
constexpr std::size_t kSelectedSourceOffset = 214;
constexpr std::size_t kControllerInputJointOffset = 215;
constexpr std::size_t kCommandedActuatorOffset = 231;
constexpr std::size_t kCommandedActuatorStride = 5;
constexpr std::size_t kTimestampSecOffset = 311;
constexpr std::size_t kTimestampNanosecOffset = 312;
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
    names.push_back(commanded + "/" + kCommandedMaxEffortInterface + "." + actuator);
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
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    message.actuator_position[i] = state_interfaces_[kActuatorPositionOffset + i].get_value();
    message.actuator_velocity[i] = state_interfaces_[kActuatorVelocityOffset + i].get_value();
    message.actuator_current[i] = state_interfaces_[kActuatorCurrentOffset + i].get_value();
  }

  for (std::size_t i = 0; i < ah2::kTactileTaxelsPerFinger; ++i) {
    const std::size_t block = kFingerTactileOffset + i;
    const std::size_t stride = ah2::kTactileTaxelsPerFinger;
    message.tactile_thumb[i] = state_interfaces_[block + 0 * stride].get_value();
    message.tactile_index[i] = state_interfaces_[block + 1 * stride].get_value();
    message.tactile_middle[i] = state_interfaces_[block + 2 * stride].get_value();
    message.tactile_ring[i] = state_interfaces_[block + 3 * stride].get_value();
    message.tactile_baby[i] = state_interfaces_[block + 4 * stride].get_value();
  }
  for (std::size_t i = 0; i < ah2::kPalm1UpperCount; ++i) {
    message.tactile_palm1_upper[i] = state_interfaces_[kPalmTactileOffset + i].get_value();
  }
  for (std::size_t i = 0; i < ah2::kPalm1LowerCount; ++i) {
    message.tactile_palm1_lower[i] =
      state_interfaces_[kPalmTactileOffset + ah2::kPalm1UpperCount + i].get_value();
  }
  for (std::size_t i = 0; i < ah2::kPalm2Count; ++i) {
    message.tactile_palm2[i] =
      state_interfaces_[
        kPalmTactileOffset + ah2::kPalm1UpperCount + ah2::kPalm1LowerCount + i].get_value();
  }

  auto & command = message.command_state;
  command.controller_input_mode =
    static_cast<std::uint8_t>(state_interfaces_[kControllerInputModeOffset].get_value());
  command.controller_output_type =
    static_cast<std::uint8_t>(state_interfaces_[kControllerOutputTypeOffset].get_value());
  command.selected_source =
    static_cast<std::uint8_t>(state_interfaces_[kSelectedSourceOffset].get_value());

  // JointPosition and JointImpedance share one echo, the hardware exports the target once
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    const double target = state_interfaces_[kControllerInputJointOffset + i].get_value();
    command.joint_position_input.target_position_rad[i] = target;
    command.joint_impedance_input.target_position_rad[i] = target;
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::size_t base = kCommandedActuatorOffset + i * kCommandedActuatorStride;
    command.actuator_position_input.target_position_cnt[i] =
      state_interfaces_[base + 0].get_value();
    command.actuator_effort_input.target_effort_pct[i] = state_interfaces_[base + 1].get_value();
    command.target_position_cnt[i] = state_interfaces_[base + 2].get_value();
    command.target_effort_pct[i] = state_interfaces_[base + 3].get_value();
    command.max_effort_pct[i] = state_interfaces_[base + 4].get_value();
  }

  publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::HandStateBroadcaster, controller_interface::ControllerInterface)

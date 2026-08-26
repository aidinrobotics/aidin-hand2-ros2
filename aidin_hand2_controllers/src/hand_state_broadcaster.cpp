#include "aidin_hand2_controllers/hand_state_broadcaster.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace aidin_hand2_controllers
{

namespace
{
// state_interfaces_ 순서 (state_interface_configuration 과 update 가 공유):
//   [0..20]    joint position (21)
//   [21..36]   actuator position_cnt (16)
//   [37..52]   actuator velocity_rpm (16)
//   [53..68]   actuator current_ma (16)
//   [69..153]  finger tactile (5 × 17 = 85)
//   [154..211] palm tactile (20 + 20 + 18 = 58)
//   [212..214] command scalar: input mode, output type, selected source (3)
//   [215..230] controller input joint target (16)
//   [231..310] command per-actuator 5필드 인터리브 ×16
//   [311..312] timestamp: sec, nanosec (2)
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

std::vector<std::string> joint_names(const std::string & prefix)
{
  return {
    prefix + "thumb_joint0",
    prefix + "thumb_joint1",
    prefix + "thumb_joint2",
    prefix + "thumb_joint3",
    prefix + "thumb_joint4",
    prefix + "index_joint1",
    prefix + "index_joint2",
    prefix + "index_joint3",
    prefix + "index_joint4",
    prefix + "middle_joint1",
    prefix + "middle_joint2",
    prefix + "middle_joint3",
    prefix + "middle_joint4",
    prefix + "ring_joint1",
    prefix + "ring_joint2",
    prefix + "ring_joint3",
    prefix + "ring_joint4",
    prefix + "baby_joint1",
    prefix + "baby_joint2",
    prefix + "baby_joint3",
    prefix + "baby_joint4",
  };
}

std::vector<std::string> actuator_names(const std::string & prefix)
{
  return {
    prefix + "thumb_actuator0",
    prefix + "thumb_actuator1",
    prefix + "thumb_actuator2",
    prefix + "thumb_actuator3",
    prefix + "index_actuator1",
    prefix + "index_actuator2",
    prefix + "index_actuator3",
    prefix + "middle_actuator1",
    prefix + "middle_actuator2",
    prefix + "middle_actuator3",
    prefix + "ring_actuator1",
    prefix + "ring_actuator2",
    prefix + "ring_actuator3",
    prefix + "baby_actuator1",
    prefix + "baby_actuator2",
    prefix + "baby_actuator3",
  };
}

// active joint 16 (구동분) — 순서 = SDK active joint index. commanded target_joint 노출용.
std::vector<std::string> active_joint_names(const std::string & prefix)
{
  return {
    prefix + "thumb_joint0",
    prefix + "thumb_joint1",
    prefix + "thumb_joint2",
    prefix + "thumb_joint3",
    prefix + "index_joint1",
    prefix + "index_joint2",
    prefix + "index_joint3",
    prefix + "middle_joint1",
    prefix + "middle_joint2",
    prefix + "middle_joint3",
    prefix + "ring_joint1",
    prefix + "ring_joint2",
    prefix + "ring_joint3",
    prefix + "baby_joint1",
    prefix + "baby_joint2",
    prefix + "baby_joint3",
  };
}

// "<prefix><finger>_sensor/tactile_1".."tactile_17" 을 append.
void append_finger_tactile(std::vector<std::string> & names, const std::string & sensor)
{
  for (int cell = 1; cell <= 17; ++cell) {
    names.push_back(sensor + "/tactile_" + std::to_string(cell));
  }
}

// palm region "<prefix>palm_sensor/<region>_1".."_count" 을 append.
void append_palm_region(
  std::vector<std::string> & names, const std::string & sensor, const std::string & region,
  int count)
{
  for (int cell = 1; cell <= count; ++cell) {
    names.push_back(sensor + "/" + region + "_" + std::to_string(cell));
  }
}

std::vector<std::string> all_state_interface_names(const std::string & prefix)
{
  std::vector<std::string> names;
  for (const auto & joint : joint_names(prefix)) {
    names.push_back(joint + "/position");
  }
  const std::vector<std::string> actuators = actuator_names(prefix);
  for (const auto & actuator : actuators) {
    names.push_back(actuator + "/position_cnt");
  }
  for (const auto & actuator : actuators) {
    names.push_back(actuator + "/velocity_rpm");
  }
  for (const auto & actuator : actuators) {
    names.push_back(actuator + "/current_ma");
  }
  for (const char * finger : {"thumb", "index", "middle", "ring", "baby"}) {
    append_finger_tactile(names, prefix + finger + "_sensor");
  }
  const std::string palm = prefix + "palm_sensor";
  append_palm_region(names, palm, "palm1_upper", 20);
  append_palm_region(names, palm, "palm1_lower", 20);
  append_palm_region(names, palm, "palm2", 18);

  // command echo — export_state_interfaces 등록 순서와 정확히 일치해야 한다(offset 인덱싱).
  const std::string commanded = prefix + "commanded";
  names.push_back(commanded + "/controller_input_mode");
  names.push_back(commanded + "/controller_output_type");
  names.push_back(commanded + "/selected_source");
  const std::vector<std::string> active_joints = active_joint_names(prefix);
  for (const auto & joint : active_joints) {
    // joint 는 "<prefix><name>" 이라 target_joint suffix 는 prefix 없는 base 이름을 쓴다.
    names.push_back(
      commanded + "/controller_input_target_position_rad." + joint.substr(prefix.size()));
  }
  for (const auto & actuator : actuators) {
    const std::string base = actuator.substr(prefix.size());  // prefix 뗀 actuator 이름
    names.push_back(commanded + "/controller_input_target_position_cnt." + base);
    names.push_back(commanded + "/controller_input_target_effort_pct." + base);
    names.push_back(commanded + "/controller_output_target_position_cnt." + base);
    names.push_back(commanded + "/controller_output_target_effort_pct." + base);
    names.push_back(commanded + "/max_effort_pct." + base);
  }

  // timestamp — 관측 시점 wall-clock (header.stamp 용).
  const std::string timestamp = prefix + "timestamp";
  names.push_back(timestamp + "/sec");
  names.push_back(timestamp + "/nanosec");
  return names;
}
}  // namespace

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
    RCLCPP_ERROR(get_node()->get_logger(), "hand_side must be 'left' or 'right' (got '%s')",
                 hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  auto realtime_publisher = get_node()->create_publisher<aidin_hand2_msgs::msg::HandState>(
    "~/hand_state", rclcpp::SystemDefaultsQoS());
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::HandState>>(
      realtime_publisher);
  publisher_->msg_.hand_side = hand_side_;
  return controller_interface::CallbackReturn::SUCCESS;
}

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

controller_interface::return_type HandStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // 매 호출 발행 — rate 조절은 yaml 의 표준 per-controller update_rate 파라미터로.
  if (publisher_ && publisher_->trylock()) {
    auto & message = publisher_->msg_;

    // header.stamp = 관측 시점 wall-clock (발행 시각 time 이 아니라 손이 관측된 순간). SDK 가 아직
    // 안 채운 초기값(sec=0)일 땐 발행 시각으로 fallback — 0 stamp 는 ROS 에서 미설정 취급이라 위험.
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

    for (std::size_t i = 0; i < 21; ++i) {
      message.joint_position[i] = state_interfaces_[kJointOffset + i].get_value();
    }
    for (std::size_t i = 0; i < 16; ++i) {
      message.actuator_position[i] = state_interfaces_[kActuatorPositionOffset + i].get_value();
      message.actuator_velocity[i] = state_interfaces_[kActuatorVelocityOffset + i].get_value();
      message.actuator_current[i] = state_interfaces_[kActuatorCurrentOffset + i].get_value();
    }
    for (std::size_t i = 0; i < 17; ++i) {
      message.tactile_thumb[i] = state_interfaces_[kFingerTactileOffset + 0 * 17 + i].get_value();
      message.tactile_index[i] = state_interfaces_[kFingerTactileOffset + 1 * 17 + i].get_value();
      message.tactile_middle[i] = state_interfaces_[kFingerTactileOffset + 2 * 17 + i].get_value();
      message.tactile_ring[i] = state_interfaces_[kFingerTactileOffset + 3 * 17 + i].get_value();
      message.tactile_baby[i] = state_interfaces_[kFingerTactileOffset + 4 * 17 + i].get_value();
    }
    for (std::size_t i = 0; i < 20; ++i) {
      message.tactile_palm1_upper[i] = state_interfaces_[kPalmTactileOffset + i].get_value();
      message.tactile_palm1_lower[i] = state_interfaces_[kPalmTactileOffset + 20 + i].get_value();
    }
    for (std::size_t i = 0; i < 18; ++i) {
      message.tactile_palm2[i] = state_interfaces_[kPalmTactileOffset + 40 + i].get_value();
    }

    auto & command = message.command_state;
    command.controller_input_mode =
      static_cast<std::uint8_t>(state_interfaces_[kControllerInputModeOffset].get_value());
    command.controller_output_type =
      static_cast<std::uint8_t>(state_interfaces_[kControllerOutputTypeOffset].get_value());
    command.selected_source =
      static_cast<std::uint8_t>(state_interfaces_[kSelectedSourceOffset].get_value());
    for (std::size_t i = 0; i < 16; ++i) {
      const double target =
        state_interfaces_[kControllerInputJointOffset + i].get_value();
      command.joint_position_input.target_position_rad[i] = target;
      command.joint_impedance_input.target_position_rad[i] = target;
    }
    for (std::size_t i = 0; i < 16; ++i) {
      const std::size_t base = kCommandedActuatorOffset + i * kCommandedActuatorStride;
      command.actuator_position_input.target_position_cnt[i] =
        state_interfaces_[base + 0].get_value();
      command.actuator_effort_input.target_effort_pct[i] =
        state_interfaces_[base + 1].get_value();
      command.target_position_cnt[i] = state_interfaces_[base + 2].get_value();
      command.target_effort_pct[i] = state_interfaces_[base + 3].get_value();
      command.max_effort_pct[i] = state_interfaces_[base + 4].get_value();
    }

    publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_controllers::HandStateBroadcaster, controller_interface::ControllerInterface)

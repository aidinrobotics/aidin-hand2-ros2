#include "aidin_hand2_hardware/aidin_hand2_isaac_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <aidin_hand2/hand/hand_kinematics.hpp>
#include <aidin_hand2/types/state.hpp>

namespace aidin_hand2_hardware
{
namespace
{
namespace ah2 = aidin_hand2;

constexpr char kPositionInterface[] = "position";
constexpr char kPositionCntInterface[] = "position_cnt";
constexpr char kVelocityRpmInterface[] = "velocity_rpm";
constexpr char kCurrentMaInterface[] = "current_ma";
constexpr char kEnabledInterface[] = "enabled";
constexpr char kFaultInterface[] = "fault";

constexpr char kCommandedComponent[] = "commanded";
constexpr char kTimestampComponent[] = "timestamp";

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

constexpr char kStampSecInterface[] = "sec";
constexpr char kStampNanosecInterface[] = "nanosec";

constexpr std::array<const char *, 7> kDiagnosticsInterfaceNames = {
  "lifecycle",
  "nan_command_count",
  "control_cycles",
  "deadline_misses",
  "last_period_ms",
  "last_compute_ms",
  "homing_state"};

constexpr std::array<const char *, ah2::kFingerCount> kFingerNames = {
  "thumb", "index", "middle", "ring", "baby"};

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

constexpr std::array<const char *, ah2::kJointCount> kJointBaseNames = {
  "thumb_joint0", "thumb_joint1", "thumb_joint2", "thumb_joint3", "thumb_joint4",
  "index_joint1", "index_joint2", "index_joint3", "index_joint4",
  "middle_joint1", "middle_joint2", "middle_joint3", "middle_joint4",
  "ring_joint1", "ring_joint2", "ring_joint3", "ring_joint4",
  "baby_joint1", "baby_joint2", "baby_joint3", "baby_joint4"};

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

// A leaf starting with '/' is absolute, otherwise it is appended to the prefix
std::string join_topic(const std::string & prefix, const std::string & leaf)
{
  if (!leaf.empty() && leaf.front() == '/') return leaf;
  if (prefix.empty()) return "/" + leaf;
  if (prefix.back() == '/') return prefix + leaf;
  return prefix + "/" + leaf;
}

std::string parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key, const std::string & fallback)
{
  const auto found = parameters.find(key);
  if (found == parameters.end() || found->second.empty()) return fallback;
  return found->second;
}

}  // namespace

AidinHand2IsaacSystemInterface::~AidinHand2IsaacSystemInterface()
{
  stop_bridge_node();
}

rclcpp::Logger AidinHand2IsaacSystemInterface::logger() const
{
  return rclcpp::get_logger("AidinHand2IsaacSystemInterface");
}

hardware_interface::CallbackReturn AidinHand2IsaacSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & parameters = info_.hardware_parameters;
  hand_side_ = parameter_or(parameters, "hand_side", "left");
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(logger(), "hand_side must be 'left' or 'right' (got '%s')", hand_side_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  prefix_ = hand_side_ + "_";

  const auto timeout = parameters.find("state_timeout");
  if (timeout != parameters.end() && !timeout->second.empty()) {
    try {
      state_timeout_ = std::stod(timeout->second);
    } catch (const std::exception &) {
      RCLCPP_ERROR(
        logger(), "state_timeout must be a number (got '%s')", timeout->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto effort = parameters.find("max_effort");
  if (effort != parameters.end() && !effort->second.empty()) {
    try {
      max_effort_pct_ = std::clamp(std::stod(effort->second), 0.0, 2000.0);
    } catch (const std::exception &) {
      RCLCPP_ERROR(logger(), "max_effort must be a number (got '%s')", effort->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // The state topic is shared with the rest of the robot by name matching, the command is not
  const std::string topic_prefix = parameter_or(parameters, "topic_prefix", "/isaac");
  joint_state_topic_ = join_topic(
    topic_prefix, parameter_or(parameters, "joint_state_topic", "joint_states"));
  joint_command_topic_ = join_topic(
    topic_prefix, parameter_or(parameters, "joint_command_topic", "hand_command"));
  tactile_prefix_ = join_topic(
    topic_prefix, parameter_or(parameters, "tactile_prefix", "tactile"));
  node_name_ = parameter_or(
    parameters, "node_name", "aidin_hand2_isaac_hardware_" + hand_side_);

  for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
    joint_name_to_index_.emplace(prefix_ + kJointBaseNames[i], i);
  }

  commanded_max_effort_pct_.fill(max_effort_pct_);
  actuator_enabled_.fill(1.0);   // No drive health in Isaac, always healthy
  actuator_fault_.fill(0.0);

  // Pose until the first Isaac state, FK of encoder 0
  const std::array<int, ah2::kActuatorCount> zero_encoder{};
  joint_position_rad_ = ah2::fk_actuator_to_joint(zero_encoder);
  rx_joint_position_rad_ = joint_position_rad_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AidinHand2IsaacSystemInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!start_bridge_node()) return hardware_interface::CallbackReturn::ERROR;
  RCLCPP_INFO(
    logger(), "%s hand Isaac bridge: state '%s', command '%s', tactile '%s/*'",
    hand_side_.c_str(), joint_state_topic_.c_str(), joint_command_topic_.c_str(),
    tactile_prefix_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AidinHand2IsaacSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_bridge_node();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AidinHand2IsaacSystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  control_cycles_ = 0;
  deadline_misses_ = 0;
  nan_command_count_ = 0;
  last_period_ms_ = 0.0;
  last_compute_ms_ = 0.0;
  state_clock_seeded_ = false;
  activated_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AidinHand2IsaacSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  activated_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool AidinHand2IsaacSystemInterface::start_bridge_node()
{
  if (node_) return true;
  try {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger(), "rclcpp is not initialized — cannot start the Isaac bridge node");
      return false;
    }
    node_ = std::make_shared<rclcpp::Node>(node_name_);

    // Sensor QoS, to match Isaac publishing best-effort
    const auto sensor_qos = rclcpp::SensorDataQoS();
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, sensor_qos,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_joint_state(msg); });

    for (std::size_t finger = 0; finger < ah2::kFingerCount; ++finger) {
      const std::string topic =
        join_topic(tactile_prefix_, prefix_ + kFingerNames[finger] + "_sensor");
      finger_tactile_subs_[finger] =
        node_->create_subscription<std_msgs::msg::Float64MultiArray>(
          topic, sensor_qos,
          [this, finger](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
            on_finger_tactile(finger, msg);
          });
    }
    palm_tactile_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      join_topic(tactile_prefix_, prefix_ + "palm_sensor"), sensor_qos,
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        on_palm_tactile(msg);
      });

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_command_topic_, rclcpp::QoS(1));

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "failed to start the Isaac bridge node: %s", error.what());
    stop_bridge_node();
    return false;
  }
  return true;
}

void AidinHand2IsaacSystemInterface::stop_bridge_node()
{
  if (executor_) executor_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  if (executor_ && node_) executor_->remove_node(node_);
  executor_.reset();
  joint_command_pub_.reset();
  palm_tactile_sub_.reset();
  for (auto & sub : finger_tactile_subs_) sub.reset();
  joint_state_sub_.reset();
  node_.reset();
}

void AidinHand2IsaacSystemInterface::on_joint_state(
  const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  const std::size_t count = std::min(msg->name.size(), msg->position.size());
  std::lock_guard<std::mutex> guard(rx_mutex_);
  bool matched = false;
  for (std::size_t i = 0; i < count; ++i) {
    const auto found = joint_name_to_index_.find(msg->name[i]);
    if (found == joint_name_to_index_.end()) continue;  // Another hand or the rest of the robot
    rx_joint_position_rad_[found->second] = msg->position[i];
    matched = true;
  }
  if (!matched) return;  // Nothing for this hand, so rx_seq_ stays put
  rx_stamp_ns_ =
    static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
  rx_joint_valid_ = true;
  ++rx_seq_;
}

void AidinHand2IsaacSystemInterface::on_finger_tactile(
  std::size_t finger, const std_msgs::msg::Float64MultiArray::ConstSharedPtr & msg)
{
  const std::size_t count = std::min<std::size_t>(msg->data.size(), ah2::kTactileTaxelsPerFinger);
  std::lock_guard<std::mutex> guard(rx_mutex_);
  for (std::size_t cell = 0; cell < count; ++cell) {
    rx_finger_tactile_[finger][cell] = msg->data[cell];
  }
}

void AidinHand2IsaacSystemInterface::on_palm_tactile(
  const std_msgs::msg::Float64MultiArray::ConstSharedPtr & msg)
{
  const std::size_t count = std::min<std::size_t>(msg->data.size(), ah2::kPalmTactileCount);
  std::lock_guard<std::mutex> guard(rx_mutex_);
  for (std::size_t cell = 0; cell < count; ++cell) {
    rx_palm_tactile_[cell] = msg->data[cell];
  }
}

std::vector<hardware_interface::StateInterface>
AidinHand2IsaacSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  // Actuator state, velocity and current stay 0
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
      interfaces.emplace_back(
        sensor, "tactile_" + std::to_string(cell + 1), &finger_tactile_[finger][cell]);
    }
  }

  // The three palm regions share one flat array
  const std::string palm = prefix_ + "palm_sensor";
  std::size_t palm_offset = 0;
  const auto add_palm_region = [&](const char * region_prefix, std::size_t count) {
    for (std::size_t cell = 0; cell < count; ++cell) {
      interfaces.emplace_back(
        palm, region_prefix + std::to_string(cell + 1), &palm_tactile_[palm_offset + cell]);
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

  // Command echo flattened to doubles
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

  // Observation stamp from the Isaac message header
  const std::string timestamp = prefix_ + kTimestampComponent;
  interfaces.emplace_back(timestamp, kStampSecInterface, &observed_stamp_sec_);
  interfaces.emplace_back(timestamp, kStampNanosecInterface, &observed_stamp_nanosec_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AidinHand2IsaacSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(hand_side_ + "_hand_control", "command_lock", &command_lock_);

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

hardware_interface::return_type AidinHand2IsaacSystemInterface::prepare_command_mode_switch(
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

  const auto decoded = exact_mode_for_interfaces(hand_side_, pending_command_interfaces_);
  if (!decoded) {
    pending_mode_switch_valid_ = false;
    return hardware_interface::return_type::ERROR;
  }
  pending_mode_ = *decoded;
  pending_mode_switch_valid_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2IsaacSystemInterface::perform_command_mode_switch(
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

hardware_interface::return_type AidinHand2IsaacSystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  bool fresh = false;
  bool linked = false;
  {
    std::lock_guard<std::mutex> guard(rx_mutex_);
    joint_position_rad_ = rx_joint_position_rad_;
    finger_tactile_ = rx_finger_tactile_;
    palm_tactile_ = rx_palm_tactile_;
    linked = rx_joint_valid_;
    fresh = rx_seq_ != last_seen_rx_seq_;
    last_seen_rx_seq_ = rx_seq_;
    if (linked) {
      observed_stamp_sec_ = static_cast<double>(rx_stamp_ns_ / 1000000000LL);
      observed_stamp_nanosec_ = static_cast<double>(rx_stamp_ns_ % 1000000000LL);
    }
  }

  // Isaac reports no actuator count, the IK of the observed pose fills it
  std::array<double, ah2::kActiveJointCount> active_rad{};
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    active_rad[i] = joint_position_rad_[kActiveToJointIndex[i]];
  }
  const auto encoder = ah2::ik_joint_to_actuator(active_rad);
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    actuator_position_cnt_[i] = static_cast<double>(encoder[i]);
  }

  ++control_cycles_;
  last_period_ms_ = period.seconds() * 1000.0;

  // Staleness is elapsed time, not missed cycles
  const double now = time.seconds();
  if (fresh || !state_clock_seeded_) {
    last_state_seconds_ = now;
    state_clock_seeded_ = true;
  }
  const bool stale = state_timeout_ > 0.0 && (now - last_state_seconds_) > state_timeout_;
  if (stale) ++deadline_misses_;

  // Disconnected before the first state and while it is stale, Running once activated
  const ah2::HandLifecycle lifecycle =
    (!linked || stale) ? ah2::HandLifecycle::Disconnected
                       : (activated_ ? ah2::HandLifecycle::Running
                                     : ah2::HandLifecycle::Connected);
  diagnostics_values_ = {
    static_cast<double>(static_cast<int>(lifecycle)),
    static_cast<double>(nan_command_count_),
    static_cast<double>(control_cycles_),
    static_cast<double>(deadline_misses_),
    last_period_ms_,
    last_compute_ms_,
    // Isaac needs no homing, the origin always counts as established
    static_cast<double>(static_cast<int>(ah2::HomingState::Succeeded))};
  return hardware_interface::return_type::OK;
}

bool AidinHand2IsaacSystemInterface::resolve_target_encoder(
  std::array<int, ah2::kActuatorCount> & encoder)
{
  if (command_mode_ == ah2::CommandMode::JointPosition) {
    if (!all_nan(joint_position_target_rad_)) {
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        held_joint_target_rad_[i] =
          fill_gap(joint_position_target_rad_[i], held_joint_target_rad_[i]);
      }
    }
    if (any_nan(held_joint_target_rad_)) return false;  // Target incomplete
    ah2::JointPositionCommand command;
    command.target = held_joint_target_rad_;
    command.clamp();
    controller_input_target_rad_ = command.target;
    encoder = ah2::ik_joint_to_actuator(command.target);
    return true;
  }

  if (command_mode_ == ah2::CommandMode::JointImpedance) {
    if (!all_nan(joint_impedance_target_rad_)) {
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        held_joint_target_rad_[i] =
          fill_gap(joint_impedance_target_rad_[i], held_joint_target_rad_[i]);
      }
    }
    if (any_nan(held_joint_target_rad_)) return false;
    ah2::JointImpedanceCommand command;
    command.target = held_joint_target_rad_;
    command.clamp();
    controller_input_target_rad_ = command.target;
    encoder = ah2::ik_joint_to_actuator(command.target);
    return true;
  }

  if (command_mode_ == ah2::CommandMode::ActuatorPosition) {
    if (!all_nan(actuator_position_target_cnt_)) {
      for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
        held_actuator_target_cnt_[i] =
          fill_gap(actuator_position_target_cnt_[i], held_actuator_target_cnt_[i]);
      }
    }
    if (any_nan(held_actuator_target_cnt_)) return false;
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      encoder[i] = static_cast<int>(std::lround(held_actuator_target_cnt_[i]));
    }
    controller_input_target_position_cnt_ = held_actuator_target_cnt_;
    return true;
  }

  return false;  // Idle and ActuatorEffort do not move the pose
}

void AidinHand2IsaacSystemInterface::publish_joint_command(
  const rclcpp::Time & time, const std::array<double, ah2::kJointCount> & target_rad)
{
  if (!joint_command_pub_) return;
  sensor_msgs::msg::JointState message;
  message.header.stamp = time;
  message.name.reserve(ah2::kJointCount);
  message.position.reserve(ah2::kJointCount);
  // All 21 go out, a coupled joint4 is driven where Isaac models it as an independent joint
  // and ignored where Isaac models it as a constraint
  for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
    message.name.push_back(prefix_ + kJointBaseNames[i]);
    message.position.push_back(target_rad[i]);
  }
  joint_command_pub_->publish(message);
}

hardware_interface::return_type AidinHand2IsaacSystemInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  const auto compute_start = std::chrono::steady_clock::now();

  const double unused = std::numeric_limits<double>::quiet_NaN();
  controller_input_target_rad_.fill(unused);
  controller_input_target_position_cnt_.fill(unused);
  controller_input_target_effort_pct_.fill(unused);
  controller_output_target_position_cnt_.fill(unused);
  controller_output_target_effort_pct_.fill(unused);
  controller_input_mode_ = static_cast<double>(static_cast<int>(command_mode_));
  controller_output_type_ = 0.0;
  selected_source_ = static_cast<double>(static_cast<int>(ah2::CommandSource::None));

  if (command_mode_ == ah2::CommandMode::ActuatorEffort) {
    // No torque model, the pose stays put and only the command is echoed
    for (double & effort : actuator_effort_target_pct_) {
      if (std::isnan(effort)) continue;  // Unowned or no command
      effort = std::clamp(effort, -max_effort_pct_, max_effort_pct_);
    }
    controller_input_target_effort_pct_ = actuator_effort_target_pct_;
    controller_output_target_effort_pct_ = actuator_effort_target_pct_;
    controller_output_type_ = 2.0;
    if (!all_nan(actuator_effort_target_pct_)) {
      selected_source_ = static_cast<double>(static_cast<int>(ah2::CommandSource::Controller));
    }
  } else {
    std::array<int, ah2::kActuatorCount> encoder{};
    if (resolve_target_encoder(encoder)) {
      std::array<double, ah2::kActuatorCount> encoder_cnt{};
      for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
        encoder_cnt[i] = static_cast<double>(encoder[i]);
      }
      controller_output_target_position_cnt_ = encoder_cnt;
      controller_output_type_ = 1.0;
      selected_source_ = static_cast<double>(static_cast<int>(ah2::CommandSource::Controller));
      publish_joint_command(time, ah2::fk_actuator_to_joint(encoder));
    } else if (command_mode_ != ah2::CommandMode::Idle) {
      ++nan_command_count_;  // Target incomplete, nothing sent this cycle
    }
  }

  commanded_max_effort_pct_.fill(max_effort_pct_);
  last_compute_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - compute_start).count();
  return hardware_interface::return_type::OK;
}

}  // namespace aidin_hand2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_hardware::AidinHand2IsaacSystemInterface,
  hardware_interface::SystemInterface)

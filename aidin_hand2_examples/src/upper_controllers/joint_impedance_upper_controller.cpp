// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <aidin_hand2/types/description.hpp>

#include "controller_interface/chainable_controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Chain interface
//   claims and exports <side>_<active_joint>/position x16 on the JointImpedanceController
// Command input
//   subscribes ~/cmd as sensor_msgs/JointState matched by name, the command controller contract
//   resolve_joint_state_command below is the same rule the command controllers apply
// State input
//   claims the hand's state interfaces and copies them into the members below every update
//   tactile is claimed only with read_tactile, the mock exports none
// Template behavior
//   scales the input by zero and forwards it, replace the WRITE block with the algorithm
namespace aidin_hand2_examples
{
namespace ah2 = aidin_hand2;

namespace
{

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

// The position in the list is the active joint index
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

constexpr std::array<const char *, ah2::kFingerCount> kFingerNames = {
  "thumb",
  "index",
  "middle",
  "ring",
  "baby",
};

// A JointState resolved by name into the controller's own order, NaN for a name not sent
// sequence 0 is no message yet
template <std::size_t N>
struct JointStateCommand
{
  std::uint64_t sequence{0};
  std::array<double, N> values{};
};

enum class JointStateField { kPosition, kEffort };

// An empty name takes the N values in the controller's own order
// Otherwise false when name differs in length from the field read or repeats a name
// A name the controller does not own is skipped
template <std::size_t N>
bool resolve_joint_state_command(
  const sensor_msgs::msg::JointState & msg, const std::vector<std::string> & names,
  JointStateField field, std::array<double, N> & out)
{
  const auto & values = field == JointStateField::kPosition ? msg.position : msg.effort;
  if (msg.name.empty()) {
    if (values.size() != N) {
      return false;
    }
    std::copy(values.begin(), values.end(), out.begin());
    return true;
  }
  if (msg.name.size() != values.size()) {
    return false;
  }
  out.fill(std::numeric_limits<double>::quiet_NaN());
  std::array<bool, N> seen{};
  for (std::size_t k = 0; k < msg.name.size(); ++k) {
    for (std::size_t i = 0; i < N; ++i) {
      if (msg.name[k] != names[i]) {
        continue;
      }
      if (seen[i]) {
        return false;
      }
      seen[i] = true;
      out[i] = values[k];
      break;
    }
  }
  return true;
}

// Claim order of the state interfaces, read_state() indexes it
constexpr std::size_t kJointOffset = 0;
constexpr std::size_t kActuatorPositionOffset = kJointOffset + ah2::kJointCount;
constexpr std::size_t kActuatorVelocityOffset = kActuatorPositionOffset + ah2::kActuatorCount;
constexpr std::size_t kActuatorCurrentOffset = kActuatorVelocityOffset + ah2::kActuatorCount;
constexpr std::size_t kFingerTactileOffset = kActuatorCurrentOffset + ah2::kActuatorCount;
constexpr std::size_t kPalmTactileOffset =
  kFingerTactileOffset + ah2::kFingerCount * ah2::kTactileTaxelsPerFinger;

}  // namespace

class JointImpedanceUpperController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::string>("hand_side", "");
    auto_declare<std::string>("target_controller", "");
    auto_declare<bool>("read_tactile", false);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    const std::string side = get_node()->get_parameter("hand_side").as_string();
    target_controller_ = get_node()->get_parameter("target_controller").as_string();
    read_tactile_ = get_node()->get_parameter("read_tactile").as_bool();
    if ((side != "left" && side != "right") || target_controller_.empty()) {
      return controller_interface::CallbackReturn::ERROR;
    }

    command_names_.clear();
    reference_suffixes_.clear();
    for (const char * joint : kActiveJointBaseNames) {
      command_names_.push_back(side + "_" + joint);
      reference_suffixes_.push_back(side + "_" + joint + "/position");
    }
    lower_reference_names_.clear();
    for (const auto & suffix : reference_suffixes_) {
      lower_reference_names_.push_back(target_controller_ + "/" + suffix);
    }

    state_interface_names_.clear();
    for (const char * joint : kJointBaseNames) {
      state_interface_names_.push_back(side + "_" + joint + "/position");
    }
    for (const char * interface : {"position_cnt", "velocity_rpm", "current_ma"}) {
      for (const char * actuator : kActuatorBaseNames) {
        state_interface_names_.push_back(side + "_" + actuator + "/" + interface);
      }
    }
    if (read_tactile_) {
      for (const char * finger : kFingerNames) {
        for (std::size_t k = 1; k <= ah2::kTactileTaxelsPerFinger; ++k) {
          state_interface_names_.push_back(
            side + "_" + finger + "_sensor/tactile_" + std::to_string(k));
        }
      }
      const std::string palm = side + "_palm_sensor/";
      for (std::size_t k = 1; k <= ah2::kPalm1UpperCount; ++k) {
        state_interface_names_.push_back(palm + "palm1_upper_" + std::to_string(k));
      }
      for (std::size_t k = 1; k <= ah2::kPalm1LowerCount; ++k) {
        state_interface_names_.push_back(palm + "palm1_lower_" + std::to_string(k));
      }
      for (std::size_t k = 1; k <= ah2::kPalm2Count; ++k) {
        state_interface_names_.push_back(palm + "palm2_" + std::to_string(k));
      }
    }

    command_buffer_.writeFromNonRT(JointStateCommand<ah2::kActiveJointCount>{});
    command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "~/cmd", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> message) {
        JointStateCommand<ah2::kActiveJointCount> command;
        if (!resolve_joint_state_command(
              *message, command_names_, JointStateField::kPosition, command.values)) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 5000,
            "JointImpedance upper command dropped — name and position differ in length, or a name repeats");
          return;
        }
        command.sequence = ++command_sequence_;
        command_buffer_.writeFromNonRT(command);
      });
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    if (state_interfaces_.size() != state_interface_names_.size()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    command_buffer_.writeFromNonRT(JointStateCommand<ah2::kActiveJointCount>{});
    consumed_sequence_ = 0;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
    joint_position_rad_.fill(nan);
    actuator_position_cnt_.fill(nan);
    actuator_velocity_rpm_.fill(nan);
    actuator_current_ma_.fill(nan);
    for (auto & finger : tactile_finger_) {
      finger.fill(nan);
    }
    tactile_palm1_upper_.fill(nan);
    tactile_palm1_lower_.fill(nan);
    tactile_palm2_.fill(nan);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      lower_reference_names_};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      state_interface_names_};
  }

protected:
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override
  {
    reference_interfaces_.assign(
      reference_suffixes_.size(), std::numeric_limits<double>::quiet_NaN());
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(reference_suffixes_.size());
    for (std::size_t i = 0; i < reference_suffixes_.size(); ++i) {
      interfaces.emplace_back(
        get_node()->get_name(), reference_suffixes_[i], &reference_interfaces_[i]);
    }
    return interfaces;
  }

  bool on_set_chained_mode(bool) override {return true;}

  // Standalone only, a message not yet consumed moves to the references
  controller_interface::return_type update_reference_from_subscribers() override
  {
    const auto & command = *command_buffer_.readFromRT();
    if (command.sequence == 0 || command.sequence == consumed_sequence_) {
      return controller_interface::return_type::OK;
    }
    consumed_sequence_ = command.sequence;
    for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
      reference_interfaces_[i] = command.values[i];
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // ------------------------------------ READ ------------------------------------
    // state interfaces -> the hand state members at the end of this file
    read_state();

    // ------------------------------------ WRITE -----------------------------------
    // Write the algorithm here, the input is in reference_interfaces_[0..15] (target_position_rad)
    // The template scales it by zero, so any command sends zero
    std::array<double, ah2::kActiveJointCount> target{};
    for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
      target[i] = 0.0 * reference_interfaces_[i];
    }

    // ----------------------------------- FORWARD ----------------------------------
    // A complete finite target goes to the command controller below, else nothing this cycle
    const bool has_any_target = std::any_of(
      target.begin(), target.end(), [](double value) {return std::isfinite(value);});
    if (!has_any_target) {
      return controller_interface::return_type::OK;
    }
    const bool has_complete_target = std::all_of(
      target.begin(), target.end(), [](double value) {return std::isfinite(value);});
    if (!has_complete_target) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "JointImpedance upper target dropped — not all 16 values are finite");
      std::fill(
        reference_interfaces_.begin(), reference_interfaces_.end(),
        std::numeric_limits<double>::quiet_NaN());
      return controller_interface::return_type::OK;
    }
    for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
      (void)command_interfaces_[i].set_value(target[i]);
    }
    std::fill(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
    return controller_interface::return_type::OK;
  }

private:
  void read_state()
  {
    for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
      joint_position_rad_[i] = state_interfaces_[kJointOffset + i].get_value();
    }
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      actuator_position_cnt_[i] = state_interfaces_[kActuatorPositionOffset + i].get_value();
      actuator_velocity_rpm_[i] = state_interfaces_[kActuatorVelocityOffset + i].get_value();
      actuator_current_ma_[i] = state_interfaces_[kActuatorCurrentOffset + i].get_value();
    }
    if (!read_tactile_) {
      return;
    }
    for (std::size_t finger = 0; finger < ah2::kFingerCount; ++finger) {
      for (std::size_t k = 0; k < ah2::kTactileTaxelsPerFinger; ++k) {
        tactile_finger_[finger][k] = state_interfaces_[
          kFingerTactileOffset + finger * ah2::kTactileTaxelsPerFinger + k].get_value();
      }
    }
    std::size_t index = kPalmTactileOffset;
    for (double & value : tactile_palm1_upper_) {
      value = state_interfaces_[index++].get_value();
    }
    for (double & value : tactile_palm1_lower_) {
      value = state_interfaces_[index++].get_value();
    }
    for (double & value : tactile_palm2_) {
      value = state_interfaces_[index++].get_value();
    }
  }

  std::string target_controller_;
  bool read_tactile_{false};
  std::vector<std::string> command_names_;
  std::vector<std::string> reference_suffixes_;
  std::vector<std::string> lower_reference_names_;
  std::vector<std::string> state_interface_names_;

  std::uint64_t command_sequence_{0};
  std::uint64_t consumed_sequence_{0};
  realtime_tools::RealtimeBuffer<JointStateCommand<ah2::kActiveJointCount>> command_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;

  // ======================= Hand state, refreshed by read_state() every update =======================
  // Index order is the SDK order, see kJointBaseNames and kActuatorBaseNames above
  std::array<double, ah2::kJointCount> joint_position_rad_{};          // rad, passive joint4 included
  std::array<double, ah2::kActuatorCount> actuator_position_cnt_{};    // encoder count
  std::array<double, ah2::kActuatorCount> actuator_velocity_rpm_{};    // rpm
  std::array<double, ah2::kActuatorCount> actuator_current_ma_{};      // mA
  // Raw sensor counts, NaN unless read_tactile is set
  std::array<std::array<double, ah2::kTactileTaxelsPerFinger>, ah2::kFingerCount> tactile_finger_{};
  std::array<double, ah2::kPalm1UpperCount> tactile_palm1_upper_{};
  std::array<double, ah2::kPalm1LowerCount> tactile_palm1_lower_{};
  std::array<double, ah2::kPalm2Count> tactile_palm2_{};
  // =================================================================================================
};

}  // namespace aidin_hand2_examples

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_examples::JointImpedanceUpperController,
  controller_interface::ChainableControllerInterface)

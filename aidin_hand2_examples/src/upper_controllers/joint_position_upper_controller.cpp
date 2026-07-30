#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "aidin_hand2_msgs/msg/hand_state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "realtime_tools/realtime_buffer.hpp"

// Chain interface
//   claims:
//     <target_controller>/<side>_<active_joint>/position ×16
//     <target_controller>/<side>_joint_position/speed_rad_s
//   exports the same suffixes under this controller name.
// State input
//   subscribes to HandState and copies the complete message into hand_state_ every update.
// Template behavior
//   does not generate a value. No reference is forwarded until a still-higher controller supplies
//   the complete finite set, so the lower basic controller keeps its activation seed until then.
namespace aidin_hand2_examples
{
namespace
{
constexpr std::array<const char *, 16> kActiveJointBaseNames = {
  "thumb_joint0", "thumb_joint1", "thumb_joint2", "thumb_joint3",
  "index_joint1", "index_joint2", "index_joint3",
  "middle_joint1", "middle_joint2", "middle_joint3",
  "ring_joint1", "ring_joint2", "ring_joint3",
  "baby_joint1", "baby_joint2", "baby_joint3"};
}

class JointPositionUpperController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::string>("hand_side", "");
    auto_declare<std::string>("target_controller", "");
    auto_declare<std::string>("hand_state_topic", "");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    const std::string side = get_node()->get_parameter("hand_side").as_string();
    target_controller_ = get_node()->get_parameter("target_controller").as_string();
    std::string state_topic = get_node()->get_parameter("hand_state_topic").as_string();
    if ((side != "left" && side != "right") || target_controller_.empty()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    if (state_topic.empty()) {
      state_topic = "/" + side + "_hand_state_broadcaster/hand_state";
    }

    reference_suffixes_.clear();
    for (const char * joint : kActiveJointBaseNames) {
      reference_suffixes_.push_back(side + "_" + joint + "/position");
    }
    reference_suffixes_.push_back(side + "_joint_position/speed_rad_s");
    lower_reference_names_.clear();
    for (const auto & suffix : reference_suffixes_) {
      lower_reference_names_.push_back(target_controller_ + "/" + suffix);
    }

    hand_state_buffer_.writeFromNonRT(
      std::shared_ptr<aidin_hand2_msgs::msg::HandState>());
    hand_state_subscriber_ =
      get_node()->create_subscription<aidin_hand2_msgs::msg::HandState>(
      state_topic, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<aidin_hand2_msgs::msg::HandState> message) {
        hand_state_buffer_.writeFromNonRT(message);
      });
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    hand_state_buffer_.writeFromNonRT(
      std::shared_ptr<aidin_hand2_msgs::msg::HandState>());
    has_hand_state_ = false;
    std::fill(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
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
    return {controller_interface::interface_configuration_type::NONE, {}};
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

  controller_interface::return_type update_reference_from_subscribers() override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const auto state = *hand_state_buffer_.readFromRT();
    if (state) {
      hand_state_ = *state;  // joint, actuator, tactile, command_state 전부 보존
      has_hand_state_ = true;
    }

    // TODO(user algorithm):
    //   hand_state_와 has_hand_state_를 읽고 reference_interfaces_[0..15]에
    //   target_position_rad, reference_interfaces_[16]에 speed_rad_s를 완전한 한 묶음으로
    //   기록한다.

    const bool has_any_reference = std::any_of(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      [](double value) {return std::isfinite(value);});
    if (!has_any_reference) {
      return controller_interface::return_type::OK;
    }
    const bool has_complete_reference = std::all_of(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      [](double value) {return std::isfinite(value);});
    if (!has_complete_reference || reference_interfaces_.back() < 0.0) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "JointPosition upper reference must be complete, finite, and use non-negative speed");
      return controller_interface::return_type::ERROR;
    }
    for (std::size_t i = 0; i < reference_interfaces_.size(); ++i) {
      (void)command_interfaces_[i].set_value(reference_interfaces_[i]);
    }
    return controller_interface::return_type::OK;
  }

private:
  std::string target_controller_;
  std::vector<std::string> reference_suffixes_;
  std::vector<std::string> lower_reference_names_;
  realtime_tools::RealtimeBuffer<
    std::shared_ptr<aidin_hand2_msgs::msg::HandState>> hand_state_buffer_;
  rclcpp::Subscription<aidin_hand2_msgs::msg::HandState>::SharedPtr hand_state_subscriber_;
  aidin_hand2_msgs::msg::HandState hand_state_{};
  bool has_hand_state_{false};
};

}  // namespace aidin_hand2_examples

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_examples::JointPositionUpperController,
  controller_interface::ChainableControllerInterface)

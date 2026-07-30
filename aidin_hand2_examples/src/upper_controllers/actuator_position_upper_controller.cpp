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
//   claims/exports <side>_<actuator>/position_cnt ×16 for the
//   ActuatorPosition basic controller.
// State input
//   subscribes to HandState and copies the complete message into hand_state_ every update.
// Template behavior
//   no values are generated; a higher controller's complete finite reference set is forwarded.
namespace aidin_hand2_examples
{
namespace
{
constexpr std::array<const char *, 16> kActuatorBaseNames = {
  "thumb_actuator0", "thumb_actuator1", "thumb_actuator2", "thumb_actuator3",
  "index_actuator1", "index_actuator2", "index_actuator3",
  "middle_actuator1", "middle_actuator2", "middle_actuator3",
  "ring_actuator1", "ring_actuator2", "ring_actuator3",
  "baby_actuator1", "baby_actuator2", "baby_actuator3"};
}

class ActuatorPositionUpperController : public controller_interface::ChainableControllerInterface
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
    for (const char * actuator : kActuatorBaseNames) {
      reference_suffixes_.push_back(side + "_" + actuator + "/position_cnt");
    }
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
    //   hand_state_를 읽고 reference_interfaces_[0..15]에 target_position_cnt를 기록한다.

    const bool has_any_reference = std::any_of(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      [](double value) {return std::isfinite(value);});
    if (!has_any_reference) {
      return controller_interface::return_type::OK;
    }
    const bool has_complete_reference = std::all_of(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      [](double value) {return std::isfinite(value);});
    if (!has_complete_reference) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "ActuatorPosition upper reference must be complete and finite");
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
  aidin_hand2_examples::ActuatorPositionUpperController,
  controller_interface::ChainableControllerInterface)

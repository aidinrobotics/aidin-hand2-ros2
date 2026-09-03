#ifndef AIDIN_HAND2_CONTROLLERS__ACTUATOR_EFFORT_CONTROLLER_HPP_
#define AIDIN_HAND2_CONTROLLERS__ACTUATOR_EFFORT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "aidin_hand2_msgs/msg/actuator_effort_command.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace aidin_hand2_controllers
{

// Chainable controller moving an ActuatorEffort typed command to the hardware command interface
//
// Standalone takes the command topic, chained takes the exported reference, never both
// update_reference_from_subscribers runs first in a cycle, then update_and_write_commands
class ActuatorEffortController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // The subscription lives only while standalone, on_set_chained_mode drops and restores it
  void subscribe();
  void unsubscribe();
  void drop_buffered_command();

  // Filled in on_configure and read only afterwards
  std::string hand_side_;
  std::vector<std::string> actuator_names_;

  // command_lock, then the 16 target efforts
  std::vector<std::string> command_interface_names_;

  // Message already moved to the references, nullptr for none consumed yet
  const void * consumed_command_{nullptr};

  realtime_tools::RealtimeBuffer<
    std::shared_ptr<aidin_hand2_msgs::msg::ActuatorEffortCommand>> command_buffer_;
  rclcpp::Subscription<aidin_hand2_msgs::msg::ActuatorEffortCommand>::SharedPtr
    command_subscriber_;
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__ACTUATOR_EFFORT_CONTROLLER_HPP_

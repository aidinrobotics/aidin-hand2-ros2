#ifndef AIDIN_HAND2_CONTROLLERS__HAND_STATE_BROADCASTER_HPP_
#define AIDIN_HAND2_CONTROLLERS__HAND_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "aidin_hand2_msgs/msg/hand_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace aidin_hand2_controllers
{

// Broadcasts the observation snapshot on ~/hand_state (aidin_hand2_msgs/HandState)
//
// Joint position 21, actuator position, velocity and current 16 each, tactile 143,
// the command echo and the observation timestamp
// Fault and enabled are not here, DiagnosticsBroadcaster carries those
class HandStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Filled in on_configure and read only afterwards
  std::string hand_side_;

  std::shared_ptr<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::HandState>> publisher_;
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__HAND_STATE_BROADCASTER_HPP_

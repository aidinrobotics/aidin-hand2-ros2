#ifndef AIDIN_HAND2_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define AIDIN_HAND2_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "aidin_hand2_msgs/msg/joint_impedance_command.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace aidin_hand2_controllers
{

// JointImpedance typed command를 hardware command port로 옮기는 chainable adapter.
class JointImpedanceController : public controller_interface::ChainableControllerInterface
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
  std::string hand_side_;
  std::vector<std::string> active_joint_names_;
  std::vector<std::string> actuator_names_;
  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  std::array<double, 16> default_stiffness_{};  // 파라미터 초기값 (on_activate 시 reference 주입)
  std::array<double, 16> default_damping_{};

  realtime_tools::RealtimeBuffer<
    std::shared_ptr<aidin_hand2_msgs::msg::JointImpedanceCommand>> command_buffer_;
  rclcpp::Subscription<aidin_hand2_msgs::msg::JointImpedanceCommand>::SharedPtr command_subscriber_;
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_

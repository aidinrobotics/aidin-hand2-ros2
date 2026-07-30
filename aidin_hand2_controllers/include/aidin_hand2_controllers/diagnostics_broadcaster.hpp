#ifndef AIDIN_HAND2_CONTROLLERS__DIAGNOSTICS_BROADCASTER_HPP_
#define AIDIN_HAND2_CONTROLLERS__DIAGNOSTICS_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "aidin_hand2_msgs/msg/hand_diagnostics.hpp"
#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace aidin_hand2_controllers
{

// SDK Diagnostics(health·comm·rt·cycle 통계) + per-actuator fault(이름)·enabled 를 두 형태로 발행:
//   /diagnostics (표준 DiagnosticArray) — rqt_runtime_monitor·aggregator 등 표준 진단 도구용
//   ~/hand_diagnostics (커스텀 HandDiagnostics) — 고정 필드, Topic Monitor·프로그램 구독용
// 운영/진단 전용 — 학습 관측은 HandStateBroadcaster.
class DiagnosticsBroadcaster : public controller_interface::ControllerInterface
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
  std::string hand_side_;

  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
    publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<aidin_hand2_msgs::msg::HandDiagnostics>>
    hand_diagnostics_publisher_;
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__DIAGNOSTICS_BROADCASTER_HPP_

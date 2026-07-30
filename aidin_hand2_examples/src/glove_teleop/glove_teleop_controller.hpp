#ifndef AIDIN_HAND2_EXAMPLES__GLOVE_TELEOP__GLOVE_TELEOP_CONTROLLER_HPP_
#define AIDIN_HAND2_EXAMPLES__GLOVE_TELEOP__GLOVE_TELEOP_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace aidin_hand2_examples
{

// AIDIN Hand Gen2 active_joint 수 (하위 controller 와 동일 고정 구조).
inline constexpr std::size_t kActiveJointCount = 16;

// MANUS 글러브를 AIDIN Hand Gen2 로 실시간 텔레오퍼하는 chainable controller (예제).
//
// 데이터 경로: [manus_ros2 노드] --manus_glove_N(토픽)--> [이 controller] --reference--> [하위 자세
// controller] --> hardware. 즉 이 controller 는 하위 JointPosition/JointImpedance controller 가
// 노출한 자세 reference interface 를 claim 하여 chain 을 이룬다. 토픽으로 자세를 되쏘지 않고
// reference 에 직접 기입하므로 controller_manager update 주기 안에서 한 번에 흐른다.
//
// 매핑: MANUS ergonomics(손가락 관절별 정규 stretch/spread, 0~1) 를 active_joint 16 목표(rad)로
// 변환한다. 각 active_joint 는 하나의 ergonomics type 에 대응하고 rad = value * scale + offset 으로
// 스케일한다(scale·offset 은 파라미터 — 실기기 캘리브레이션 값). 대응이 비지정인 joint 는 NaN 을
// 기입해 하위 controller 가 직전 값을 유지하게 한다.
//
// 실행: controller_manager 에 spawn (config/glove_teleop/glove_teleop_controller.yaml 참고).
//   ros2 run controller_manager spawner right_glove_teleop_controller
class GloveTeleopController : public controller_interface::ChainableControllerInterface
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
  // active_joint 하나의 매핑 — ergonomics 하나에 대한 선형 변환.
  //   target_rad = (ergonomics_value_deg · scale + offset_deg) · π/180
  // ergonomics_type 이 빈 문자열이면 지령 안 함(NaN → 하위 controller 직전값 유지).
  struct JointMapping
  {
    std::string ergonomics_type;  // MANUS ergonomics type 문자열 (예: "IndexMCPStretch")
    double scale{0.0};            // 방향+배율(단위 없음)
    double offset{0.0};           // deg (코드가 π/180 적용)
  };

  std::string hand_side_;                                     // "left" | "right"
  std::string target_controller_;                            // 하위 controller 이름 (reference 소유자)
  std::string glove_topic_;                                   // 구독할 manus_glove_N 토픽
  double quantize_step_deg_{0.0};                            // 관절 목표를 이 격자(deg)로 반올림, 0 이면 비활성
  std::array<JointMapping, kActiveJointCount> mappings_;      // active_joint index → ergonomics 매핑
  std::vector<std::string> reference_interface_names_;        // claim 할 하위 reference 이름 (16)

  realtime_tools::RealtimeBuffer<std::shared_ptr<manus_ros2_msgs::msg::ManusGlove>> glove_buffer_;
  rclcpp::Subscription<manus_ros2_msgs::msg::ManusGlove>::SharedPtr glove_subscriber_;
};

}  // namespace aidin_hand2_examples

#endif  // AIDIN_HAND2_EXAMPLES__GLOVE_TELEOP__GLOVE_TELEOP_CONTROLLER_HPP_

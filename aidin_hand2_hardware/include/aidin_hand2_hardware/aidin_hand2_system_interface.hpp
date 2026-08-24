#pragma once

#include <array>
#include <atomic>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"

#include <aidin_hand2/aidin_hand2.hpp>

namespace aidin_hand2_hardware
{

namespace ah2 = aidin_hand2;

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// AIDIN Hand Gen2 SDK 를 ros2_control 에 어댑트하는 SystemInterface.
// protocol·kinematics·RT loop 는 SDK 소유 — 여기는 snapshot 복사와 command 조립만 한다.
// lifecycle 매핑: configure=create+connect / activate=run(+auto_home) / deactivate=stop /
// cleanup=disconnect+destroy. run/stop/home/reconnect 는 자체 service 로도 노출(전용 스레드).
// command mode 는 controller 가 claim 한 인터페이스 군이 결정한다 (perform_command_mode_switch).
//
// interface 이름은 SDK 도메인이 고정(actuator 16 · joint 21 · active 16)이라 URDF 를 파싱하지
// 않고 hardware 가 아는 고정 순서(prefix 만 붙임)로 export·매핑한다. URDF 순서는 무관하며,
// 이름이 고정 목록에 없으면 export 시 무시된다.
class AidinHand2SystemInterface : public hardware_interface::SystemInterface
{
public:
  ~AidinHand2SystemInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger() const;

  // ---- run/stop/home/reconnect (service 콜백 스레드에서 동기 실행 — read/write 루프와 별개) ----
  bool exec_run(std::string & failure_message);
  bool exec_stop(std::string & failure_message);
  bool exec_home(std::string & failure_message);
  bool exec_reconnect(std::string & failure_message);
  void start_service_node();
  void stop_service_node();

  // ---- SDK 세션 ----
  ah2::HandManager manager_;
  std::optional<ah2::Hand> hand_;

  // ---- 파라미터 ----
  std::string hand_side_name_;   // "left" / "right"
  std::string prefix_;           // "<side>_" — state 이름 앞에 붙임
  std::string can_interface_;
  ah2::HandSide hand_side_{ah2::HandSide::Left};
  bool auto_home_{true};
  // 아래 기본값은 SDK default 와 동일 (HandConfig / kDefaultMaxEffort)
  // rated current % (1000 = 100%). on_configure 초기 적용 + ~/set_max_effort 토픽 런타임 갱신 공용.
  // write(RT 스레드)와 토픽 콜백(service_node 스레드)이 함께 접근해 atomic. 안전 한계값이라 모드 무관 유지.
  std::atomic<double> max_effort_{1000.0};
  int control_rate_{500};        // RT loop Hz
  int rt_cpu_affinity_{-1};      // 코어 pin, -1 = 미설정
  std::vector<int> disabled_actuators_{};  // 미가동 actuator index (콤마 구분 파라미터 파싱 결과)
  bool auto_reconnect_{false};              // 통신 두절 시 SDK 자동 재수립
  int auto_reconnect_timeout_ms_{0};        // 재수립 포기 상한 [ms], 0 = 무제한
  bool auto_reconnect_home_{false};         // 재수립 복귀 시 run 전 homing

  // ---- state 저장소 (state interface 가 가리키는 메모리) ----
  ah2::HandState state_{};
  std::array<double, ah2::kJointCount> joint_position_rad_{};
  std::array<double, ah2::kActuatorCount> actuator_enabled_{};
  std::array<double, ah2::kActuatorCount> actuator_fault_{};
  std::array<double, 7> diagnostics_values_{};

  // ---- command state·timestamp 저장소 ----
  double controller_input_mode_{};
  double controller_output_type_{};
  double selected_source_{};
  std::array<double, ah2::kActiveJointCount> controller_input_target_rad_{};
  double controller_input_speed_rad_s_{};
  std::array<double, ah2::kActuatorCount> controller_input_stiffness_{};
  std::array<double, ah2::kActuatorCount> controller_input_damping_{};
  std::array<double, ah2::kActuatorCount> controller_input_target_position_cnt_{};
  std::array<double, ah2::kActuatorCount> controller_input_target_effort_pct_{};
  std::array<double, ah2::kActuatorCount> controller_output_target_position_cnt_{};
  std::array<double, ah2::kActuatorCount> controller_output_target_effort_pct_{};
  std::array<double, ah2::kActuatorCount> commanded_max_effort_pct_{};
  double observed_stamp_sec_{};      // HandState.timestamp 를 sec/nanosec 로 분해 (double 정밀도 손실 회피)
  double observed_stamp_nanosec_{};

  // ---- command 저장소 (command interface 가 가리키는 메모리) ----
  double command_lock_{};
  std::array<double, ah2::kActiveJointCount> joint_position_target_rad_{};
  double joint_position_speed_rad_s_{0.0};
  std::array<double, ah2::kActiveJointCount> joint_impedance_target_rad_{};
  std::array<double, ah2::kActuatorCount> joint_impedance_stiffness_{};
  std::array<double, ah2::kActuatorCount> joint_impedance_damping_{};
  std::array<double, ah2::kActuatorCount> actuator_position_target_cnt_{};
  std::array<double, ah2::kActuatorCount> actuator_effort_target_pct_{};
  double last_applied_max_effort_{1000.0};     // write 스레드 전용 — 직전 적용값(변경 시에만 SDK 재호출)

  // ---- command mode (정확한 command-port claim 집합에서 파생) ----
  ah2::CommandMode command_mode_{ah2::CommandMode::Idle};
  ah2::CommandMode pending_mode_{ah2::CommandMode::Idle};
  std::set<std::string> active_command_interfaces_;
  std::set<std::string> pending_command_interfaces_;
  bool pending_mode_switch_valid_{false};

  // ---- 제어 상태 ----
  std::atomic<bool> started_{false};       // SDK start 상태 — false 면 write 가 command 미전송
  // 원점 상태는 wrapper 가 추적하지 않는다 — SDK diagnostics.homing_state 가 단일 출처(read 가 관측).
  // auto-home 1회 트리거 래치 — 원점 미확정이면 start_homing() 을 한 번만 걸도록. on_activate·
  // exec_reconnect 가 리셋해 다시 세운다.
  std::atomic<bool> auto_home_triggered_{false};

  // ---- service node (hardware 자체 노드 — ~/run·~/stop·~/home·~/reconnect 노출, 전용 spin 스레드) ----
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_service_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr max_effort_sub_;  // ~/set_max_effort
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;
  std::thread service_spin_thread_;
};

}  // namespace aidin_hand2_hardware

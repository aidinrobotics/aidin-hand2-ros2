#ifndef AIDIN_HAND2_HARDWARE__AIDIN_HAND2_ISAAC_SYSTEM_INTERFACE_HPP_
#define AIDIN_HAND2_HARDWARE__AIDIN_HAND2_ISAAC_SYSTEM_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/description.hpp>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace aidin_hand2_hardware
{

// Isaac Sim 과 ROS 2 토픽으로만 통신하는 SystemInterface.
//
//   read()  : Isaac 이 발행한 sensor_msgs/JointState (+ 촉각 토픽) 를 state interface 로 옮긴다.
//   write() : 실 hardware·mock 과 같은 규칙으로 command 를 해석해 목표 joint 자세(21)를
//             sensor_msgs/JointState 로 Isaac 에 발행한다.
//
// command interface 계약(98개)과 state interface 계약은 실 hardware 와 동일하다 — mock 과 달리
// 촉각 143개와 diagnostics 39개도 내보내므로 hand_state_broadcaster · diagnostics_broadcaster 가
// 그대로 붙는다. Isaac 에 대응물이 없는 값의 출처는 다음과 같다:
//   - actuator position_cnt : Isaac joint 자세를 SDK IK 로 되돌린 값
//   - actuator velocity_rpm / current_ma : 0 (mock 과 동일 — Isaac 에 actuator 모델이 없다)
//   - diagnostics : 브리지가 직접 채운다 (lifecycle·control_cycles·deadline_misses 등)
//
// 구독/발행은 하드웨어 전용 노드(node_name 파라미터)에서 이뤄지고 콜백은 전용 executor 스레드가
// 처리한다. read()/write() 는 뮤텍스로 보호된 버퍼만 복사하므로 제어 루프를 막지 않는다.
class AidinHand2IsaacSystemInterface : public hardware_interface::SystemInterface
{
public:
  AidinHand2IsaacSystemInterface() = default;
  ~AidinHand2IsaacSystemInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

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

  bool start_bridge_node();
  void stop_bridge_node();

  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);
  void on_finger_tactile(
    std::size_t finger, const std_msgs::msg::Float64MultiArray::ConstSharedPtr & msg);
  void on_palm_tactile(const std_msgs::msg::Float64MultiArray::ConstSharedPtr & msg);

  // write() 가 고른 목표 actuator count. 명령이 아직 완전하지 않으면 false 를 돌려준다.
  bool resolve_target_encoder(
    const rclcpp::Duration & period, std::array<int, aidin_hand2::kActuatorCount> & encoder);
  void publish_joint_command(const rclcpp::Time & time,
                             const std::array<double, aidin_hand2::kJointCount> & target_rad);

  // ---- 파라미터 ----
  std::string hand_side_;   // "left" / "right"
  std::string prefix_;      // "<side>_"
  std::string node_name_;
  std::string joint_state_topic_;
  std::string joint_command_topic_;
  std::string tactile_prefix_;
  double max_effort_pct_{1000.0};
  // Isaac 상태가 이 시간을 넘겨 갱신되지 않으면 링크가 끊긴 것으로 본다 [s]. 0 이하면 검사 안 함.
  // 시뮬레이터는 제어 루프보다 느리게 발행하는 것이 정상이라 "매 cycle 새 메시지" 를 기준 삼으면
  // 안 된다 — 그래서 cycle 수가 아니라 경과 시간으로 판정한다.
  double state_timeout_{0.1};

  // ---- 브리지 노드 (전용 spin 스레드) ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr,
             aidin_hand2::kFingerCount> finger_tactile_subs_{};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr palm_tactile_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  // ---- 수신 버퍼 (콜백 스레드 ↔ read()) ----
  std::mutex rx_mutex_;
  std::unordered_map<std::string, std::size_t> joint_name_to_index_;
  std::array<double, aidin_hand2::kJointCount> rx_joint_position_rad_{};
  std::array<std::array<double, aidin_hand2::kTactileTaxelsPerFinger>,
             aidin_hand2::kFingerCount> rx_finger_tactile_{};
  std::array<double, aidin_hand2::kPalmTactileCount> rx_palm_tactile_{};
  std::int64_t rx_stamp_ns_{0};
  std::uint64_t rx_seq_{0};          // joint state 수신 카운터 — read() 가 신선도 판정에 쓴다
  std::uint64_t last_seen_rx_seq_{0};
  bool rx_joint_valid_{false};

  // ---- state 저장소 (state interface 가 가리키는 메모리) ----
  std::array<double, aidin_hand2::kActuatorCount> actuator_position_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_velocity_rpm_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_current_ma_{};
  std::array<double, aidin_hand2::kJointCount> joint_position_rad_{};
  std::array<std::array<double, aidin_hand2::kTactileTaxelsPerFinger>,
             aidin_hand2::kFingerCount> finger_tactile_{};
  std::array<double, aidin_hand2::kPalmTactileCount> palm_tactile_{};
  std::array<double, 7> diagnostics_values_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_enabled_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_fault_{};
  double observed_stamp_sec_{};
  double observed_stamp_nanosec_{};

  // ---- command echo state 저장소 (실 hardware 의 <prefix>commanded 와 동일 계약) ----
  double controller_input_mode_{};
  double controller_output_type_{};
  double selected_source_{};
  std::array<double, aidin_hand2::kActiveJointCount> controller_input_target_rad_{};
  double controller_input_speed_rad_s_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_input_stiffness_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_input_damping_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_input_target_position_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_input_target_effort_pct_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_output_target_position_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_output_target_effort_pct_{};
  std::array<double, aidin_hand2::kActuatorCount> commanded_max_effort_pct_{};

  // ---- command 저장소 (command interface 가 가리키는 메모리) ----
  double command_lock_{};
  std::array<double, aidin_hand2::kActiveJointCount> joint_position_target_rad_{};
  double joint_position_speed_rad_s_{};
  std::array<double, aidin_hand2::kActiveJointCount> joint_impedance_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> joint_impedance_stiffness_{};
  std::array<double, aidin_hand2::kActuatorCount> joint_impedance_damping_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_position_target_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_effort_target_pct_{};

  // 직전까지 받은 목표 — command interface 의 NaN(명령 없음·미점유)을 메워 완전한 목표로 유지한다.
  std::array<double, aidin_hand2::kActiveJointCount> held_joint_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> held_actuator_target_cnt_{};
  std::array<double, aidin_hand2::kActiveJointCount> slew_position_rad_{};
  bool slew_seeded_{false};

  // ---- command mode ----
  aidin_hand2::CommandMode command_mode_{aidin_hand2::CommandMode::Idle};
  aidin_hand2::CommandMode pending_mode_{aidin_hand2::CommandMode::Idle};
  std::set<std::string> active_command_interfaces_;
  std::set<std::string> pending_command_interfaces_;
  bool pending_mode_switch_valid_{false};

  // ---- diagnostics 집계 (브리지가 직접 채운다) ----
  std::atomic<bool> activated_{false};
  std::uint64_t control_cycles_{0};
  std::uint64_t deadline_misses_{0};   // 상태가 state_timeout 을 넘겨 낡은 채 돈 read() 횟수
  double last_state_seconds_{0.0};     // 마지막으로 새 상태를 받은 read() 시각 [s]
  bool state_clock_seeded_{false};
  std::uint64_t nan_command_count_{0}; // 목표가 불완전해 명령을 못 낸 write() 횟수
  double last_period_ms_{0.0};
  double last_compute_ms_{0.0};
};

}  // namespace aidin_hand2_hardware

#endif  // AIDIN_HAND2_HARDWARE__AIDIN_HAND2_ISAAC_SYSTEM_INTERFACE_HPP_

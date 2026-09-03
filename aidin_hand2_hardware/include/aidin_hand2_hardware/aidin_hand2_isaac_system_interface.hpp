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

// ros2_control SystemInterface over Isaac Sim, ROS 2 topics instead of CAN
//   read()   Isaac's joint state and tactile topics into the state interfaces
//   write()  the target joint pose out as sensor_msgs/JointState
//
// The command and state interface contracts match the real hardware, and what Isaac has no
// counterpart for comes from
//   actuator position_cnt              IK of the Isaac joint pose
//   actuator velocity_rpm, current_ma  0
//   diagnostics                        this bridge
//
// The topics are served by a node named by the node_name parameter with its own executor
// thread, and read() and write() copy mutex-protected buffers only
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

  // False while the target is still incomplete
  bool resolve_target_encoder(std::array<int, aidin_hand2::kActuatorCount> & encoder);
  void publish_joint_command(const rclcpp::Time & time,
                             const std::array<double, aidin_hand2::kJointCount> & target_rad);

  // --------------------- Config [all threads, read only] --------------------

  // Parsed in on_init from the URDF hardware parameters

  std::string hand_side_;   // "left" / "right"
  std::string prefix_;      // "<side>_"
  std::string node_name_;
  std::string joint_state_topic_;
  std::string joint_command_topic_;
  std::string tactile_prefix_;
  double max_effort_pct_{1000.0};
  // Seconds without a new Isaac state before the link counts as lost, 0 or less never checks
  double state_timeout_{0.1};

  // ----------------------- Bridge node [bridge thread] ----------------------

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr,
             aidin_hand2::kFingerCount> finger_tactile_subs_{};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr palm_tactile_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  // -------------------- Single thread: plain [CM thread] --------------------

  // Backing memory of the exported state interfaces, refilled by read()

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

  // Command echo flattened to doubles
  double controller_input_mode_{};
  double controller_output_type_{};
  double selected_source_{};
  std::array<double, aidin_hand2::kActiveJointCount> controller_input_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_input_target_position_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_input_target_effort_pct_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_output_target_position_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> controller_output_target_effort_pct_{};
  std::array<double, aidin_hand2::kActuatorCount> commanded_max_effort_pct_{};

  // Backing memory of the exported command interfaces, written by the controllers
  double command_lock_{};
  std::array<double, aidin_hand2::kActiveJointCount> joint_position_target_rad_{};
  std::array<double, aidin_hand2::kActiveJointCount> joint_impedance_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_position_target_cnt_{};
  std::array<double, aidin_hand2::kActuatorCount> actuator_effort_target_pct_{};

  // Previous target, filling NaN command entries
  std::array<double, aidin_hand2::kActiveJointCount> held_joint_target_rad_{};
  std::array<double, aidin_hand2::kActuatorCount> held_actuator_target_cnt_{};

  // Mode in effect and the one prepare_command_mode_switch validated
  aidin_hand2::CommandMode command_mode_{aidin_hand2::CommandMode::Idle};
  aidin_hand2::CommandMode pending_mode_{aidin_hand2::CommandMode::Idle};
  std::set<std::string> active_command_interfaces_;
  std::set<std::string> pending_command_interfaces_;
  bool pending_mode_switch_valid_{false};

  // Diagnostics this bridge fills
  std::atomic<bool> activated_{false};
  std::uint64_t control_cycles_{0};

  // read() calls that ran on a state older than state_timeout_
  std::uint64_t deadline_misses_{0};

  // Seconds at the read() that last saw a new state
  double last_state_seconds_{0.0};
  bool state_clock_seeded_{false};

  // write() calls that sent nothing on an incomplete target
  std::uint64_t nan_command_count_{0};

  double last_period_ms_{0.0};
  double last_compute_ms_{0.0};

  // --------------- Cross thread: buffer [bridge -> CM thread] ---------------

  std::mutex rx_mutex_;
  std::unordered_map<std::string, std::size_t> joint_name_to_index_;
  std::array<double, aidin_hand2::kJointCount> rx_joint_position_rad_{};
  std::array<std::array<double, aidin_hand2::kTactileTaxelsPerFinger>,
             aidin_hand2::kFingerCount> rx_finger_tactile_{};
  std::array<double, aidin_hand2::kPalmTactileCount> rx_palm_tactile_{};
  std::int64_t rx_stamp_ns_{0};

  // Incremented on every joint state, read() compares the two to spot a new one
  std::uint64_t rx_seq_{0};
  std::uint64_t last_seen_rx_seq_{0};
  bool rx_joint_valid_{false};
};

}  // namespace aidin_hand2_hardware

#endif  // AIDIN_HAND2_HARDWARE__AIDIN_HAND2_ISAAC_SYSTEM_INTERFACE_HPP_

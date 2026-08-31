#pragma once

#include <array>
#include <atomic>
#include <mutex>
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
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <aidin_hand2/aidin_hand2.hpp>

namespace aidin_hand2_hardware
{

namespace ah2 = aidin_hand2;

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ros2_control SystemInterface over the AIDIN Hand Gen2 SDK
// The SDK owns the protocol, the kinematics and the RT loop, this copies snapshots
// and assembles commands
//
// Lifecycle
//   on_configure   create + connect
//   on_activate    run
//   on_deactivate  stop
//   on_cleanup     disconnect + destroy
//
// run, stop, home and reconnect are also exposed as services on this component's own node
// Interface names follow a fixed SDK order with the side prefix, so the URDF is never parsed
class AidinHand2SystemInterface : public hardware_interface::SystemInterface
{
public:
  ~AidinHand2SystemInterface() override;

  // ------------------------------- Lifecycle --------------------------------

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  // ---------------------------- Interface export ----------------------------

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ------------------------------ Command mode ------------------------------

  // The claimed command interface set decides the mode
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // ------------------------------ Control loop ------------------------------

  // Copy the SDK snapshot into the state storage
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Apply staged tuning, then hand the active mode command to the SDK
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // =============================== Functions ================================

  // ------------------------------ Service node ------------------------------

  void start_service_node();
  void stop_service_node();

  // ---------------------- Hand action [service thread] ----------------------

  // Run synchronously on the service callback thread, apart from the read and write loop
  // A failure message goes back in the service response, since the SDK already logged it
  bool exec_run(std::string & failure_message);
  bool exec_stop(std::string & failure_message);
  bool exec_home(std::string & failure_message);
  bool exec_reconnect(std::string & failure_message);

  // ---------------------------- Tuning parameter ----------------------------

  // Declare max_effort and the SDK ControllerConfig on the service node
  void declare_tuning_parameters();

  // Validate and stage [service thread]
  rcl_interfaces::msg::SetParametersResult on_set_tuning_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // Push a dirty staging to the SDK [CM thread]
  void apply_tuning_parameters();

  // ----------------------- Command write [CM thread] ------------------------

  // Blank the command storage of the current mode, NaN meaning no command
  void clear_mode_command();

  void warn_incomplete_command();

  // -------------------------------- Helpers ---------------------------------

  rclcpp::Logger logger() const;

  // =============================== Variables ================================

  // -------------------- Config [all threads, read only] ---------------------

  // Parsed in on_init from the URDF hardware parameters

  // "left" or "right"
  std::string hand_side_name_;

  // "<side>_", prepended to every interface name
  std::string prefix_;

  std::string can_interface_;
  ah2::HandSide hand_side_{ah2::HandSide::Left};
  bool auto_home_{true};

  // Defaults below match HandConfig and kDefaultMaxEffort

  // Rated current %, 1000 being 100%, and the node parameter default
  double max_effort_{1000.0};

  // RT loop Hz
  int control_rate_{500};

  // CPU to pin the RT loop to, -1 for none
  int rt_cpu_affinity_{-1};

  // Actuator indices left unpowered
  std::vector<int> disabled_actuators_{};

  bool auto_reconnect_{false};

  // 0 for no limit
  int auto_reconnect_timeout_ms_{0};

  bool auto_reconnect_home_{false};

  // -------------------- SDK session [CM, service thread] --------------------

  ah2::HandManager manager_;
  std::optional<ah2::Hand> hand_;

  // ----------------- Single thread: observation [CM thread] -----------------

  // Backing memory of the exported state interfaces, refilled by read()

  ah2::HandState state_{};
  std::array<double, ah2::kJointCount> joint_position_rad_{};
  std::array<double, ah2::kActuatorCount> actuator_enabled_{};
  std::array<double, ah2::kActuatorCount> actuator_fault_{};
  std::array<double, 7> diagnostics_values_{};

  // Command echo, the SDK variant flattened to doubles
  double controller_input_mode_{};
  double controller_output_type_{};
  double selected_source_{};
  std::array<double, ah2::kActiveJointCount> controller_input_target_rad_{};
  std::array<double, ah2::kActuatorCount> controller_input_target_position_cnt_{};
  std::array<double, ah2::kActuatorCount> controller_input_target_effort_pct_{};
  std::array<double, ah2::kActuatorCount> controller_output_target_position_cnt_{};
  std::array<double, ah2::kActuatorCount> controller_output_target_effort_pct_{};
  std::array<double, ah2::kActuatorCount> commanded_max_effort_pct_{};

  // HandState.timestamp split in two, since a double cannot hold the ns count
  double observed_stamp_sec_{};
  double observed_stamp_nanosec_{};

  // ------------------- Single thread: command [CM thread] -------------------

  // Backing memory of the exported command interfaces, written by the controllers

  // Claim-only, no controller reads or writes the value
  double command_lock_{};

  std::array<double, ah2::kActiveJointCount> joint_position_target_rad_{};
  std::array<double, ah2::kActiveJointCount> joint_impedance_target_rad_{};
  std::array<double, ah2::kActuatorCount> actuator_position_target_cnt_{};
  std::array<double, ah2::kActuatorCount> actuator_effort_target_pct_{};

  // Mode in effect and the one prepare_command_mode_switch validated
  ah2::CommandMode command_mode_{ah2::CommandMode::Idle};
  ah2::CommandMode pending_mode_{ah2::CommandMode::Idle};
  std::set<std::string> active_command_interfaces_;
  std::set<std::string> pending_command_interfaces_;
  bool pending_mode_switch_valid_{false};

  // Throttles the write path warning
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // -------------- Cross thread: atomic [CM <-> service thread] --------------

  // Homing is not tracked here, diagnostics.homing_state is the single source

  // False after stop, which keeps write() from sending commands
  std::atomic<bool> started_{false};

  // Latched by exec_run, so write() triggers homing once per run
  std::atomic<bool> auto_home_triggered_{false};

  // ----------- Cross thread: mutex [service thread -> CM thread] ------------

  // The parameter callback validates and stages, write() applies a dirty staging

  std::mutex tuning_mutex_;
  std::array<double, ah2::kActuatorCount> staged_max_effort_{};
  ah2::ControllerConfig staged_controller_config_{};
  bool tuning_dirty_{false};

  // --------------------- Service node [service thread] ----------------------

  // This component's own node, exposing ~/run, ~/stop, ~/home and ~/reconnect

  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr tuning_callback_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;
  std::thread service_spin_thread_;
};

}  // namespace aidin_hand2_hardware

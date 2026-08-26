#include "aidin_hand2_hardware/aidin_hand2_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"

// Hardware command interface contract (one hand, 65 resources)
//   <side>_hand_control/command_lock                                      ×1
//   <side>_joint_position_command/target_position_rad.<active_joint>     ×16
//   <side>_joint_impedance_command/target_position_rad.<active_joint>    ×16
//   <side>_actuator_position_command/target_position_cnt.<actuator>      ×16
//   <side>_actuator_effort_command/target_effort_pct.<actuator>          ×16
// command_lock is claim-only. A mode switch accepts only an empty set (Idle)
// or exactly one complete mode set including the lock.
// Tuning (max_effort, JointPosition filter, JointImpedance gains) is not a command — it lives on
// this component's own node as parameters and reaches the SDK through set_controller_config().
namespace aidin_hand2_hardware
{

namespace
{

// 표준 joint position 을 제외한 custom state interface 는 단위를 이름에 붙인다.
constexpr char kPositionCountInterface[] = "position_cnt";
constexpr char kVelocityRpmInterface[] = "velocity_rpm";
constexpr char kCurrentMilliampInterface[] = "current_ma";
constexpr char kEnabledInterface[] = "enabled";
constexpr char kFaultInterface[] = "fault";

constexpr char kPositionInterface[] = "position";

// command echo·timestamp component 이름 (hand 전역 상태 — diagnostics 처럼 한 component 로 묶음)
constexpr char kCommandedComponent[] = "commanded";
constexpr char kTimestampComponent[] = "timestamp";

// command echo scalar 인터페이스 (component = <prefix>commanded)
constexpr char kControllerInputModeInterface[] = "controller_input_mode";
constexpr char kControllerOutputTypeInterface[] = "controller_output_type";
constexpr char kSelectedSourceInterface[] = "selected_source";
constexpr char kControllerInputTargetInterface[] = "controller_input_target_position_rad";
constexpr char kControllerInputActuatorPositionInterface[] =
  "controller_input_target_position_cnt";
constexpr char kControllerInputActuatorEffortInterface[] =
  "controller_input_target_effort_pct";
constexpr char kControllerOutputPositionInterface[] = "controller_output_target_position_cnt";
constexpr char kControllerOutputEffortInterface[] = "controller_output_target_effort_pct";
constexpr char kCommandedMaxEffortInterface[] = "max_effort_pct";      // actuator

// timestamp 인터페이스 (component = <prefix>timestamp) — 관측 시점 wall-clock, header.stamp 용
constexpr char kStampSecInterface[] = "sec";
constexpr char kStampNanosecInterface[] = "nanosec";

// diagnostics gpio 의 state interface 순서 = diagnostics_values_ 배열 순서.
// lifecycle 은 double(HandLifecycle ordinal) 로 나가고 broadcaster 가 이름화한다.
// 종합 판정(healthy)은 두지 않는다 — 소비자가 lifecycle 과 actuator fault 로 직접 판단한다.
constexpr std::array<const char *, 7> kDiagnosticsInterfaceNames = {
  "lifecycle",
  "nan_command_count",
  "control_cycles",
  "deadline_misses",
  "last_period_ms",
  "last_compute_ms",
  "homing_state"};

// tactile finger 순서 (SDK flat array 블록 순서). thumb 뒤 long finger 4개.
constexpr std::array<const char *, 5> kFingerNames = {
  "thumb",
  "index",
  "middle",
  "ring",
  "baby"};

// interface 이름(prefix 제외) — SDK 도메인 고정 순서. 이름/인덱스 매핑은 이 목록의 순서 그 자체다.
// actuator 16: thumb 4 + long finger 3×4.
constexpr std::array<const char *, ah2::kActuatorCount> kActuatorBaseNames = {
  "thumb_actuator0", "thumb_actuator1", "thumb_actuator2", "thumb_actuator3",
  "index_actuator1", "index_actuator2", "index_actuator3",
  "middle_actuator1", "middle_actuator2", "middle_actuator3",
  "ring_actuator1", "ring_actuator2", "ring_actuator3",
  "baby_actuator1", "baby_actuator2", "baby_actuator3"};

constexpr std::array<const char *, ah2::kActiveJointCount> kActiveJointBaseNames = {
  "thumb_joint0", "thumb_joint1", "thumb_joint2", "thumb_joint3",
  "index_joint1", "index_joint2", "index_joint3",
  "middle_joint1", "middle_joint2", "middle_joint3",
  "ring_joint1", "ring_joint2", "ring_joint3",
  "baby_joint1", "baby_joint2", "baby_joint3"};

constexpr std::array<std::size_t, ah2::kActiveJointCount> kActiveToJointIndex = {
  0, 1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19};

// joint 21 (FK state) — 각 digit 마지막이 q4 결합분. 순서 = SDK joint index.
constexpr std::array<const char *, ah2::kJointCount> kJointBaseNames = {
  "thumb_joint0",
  "thumb_joint1",
  "thumb_joint2",
  "thumb_joint3",
  "thumb_joint4",
  "index_joint1",
  "index_joint2",
  "index_joint3",
  "index_joint4",
  "middle_joint1",
  "middle_joint2",
  "middle_joint3",
  "middle_joint4",
  "ring_joint1",
  "ring_joint2",
  "ring_joint3",
  "ring_joint4",
  "baby_joint1",
  "baby_joint2",
  "baby_joint3",
  "baby_joint4"};

std::string command_component(const std::string & side, ah2::CommandMode mode)
{
  switch (mode) {
    case ah2::CommandMode::JointPosition: return side + "_joint_position_command";
    case ah2::CommandMode::JointImpedance: return side + "_joint_impedance_command";
    case ah2::CommandMode::ActuatorPosition: return side + "_actuator_position_command";
    case ah2::CommandMode::ActuatorEffort: return side + "_actuator_effort_command";
    case ah2::CommandMode::Idle: return {};
  }
  return {};
}

std::vector<std::string> mode_command_interfaces(
  const std::string & side, ah2::CommandMode mode, bool include_lock = true)
{
  std::vector<std::string> names;
  if (mode == ah2::CommandMode::Idle) return names;
  if (include_lock) names.push_back(side + "_hand_control/command_lock");
  const std::string component = command_component(side, mode) + "/";
  if (mode == ah2::CommandMode::JointPosition ||
      mode == ah2::CommandMode::JointImpedance)
  {
    for (const char * joint : kActiveJointBaseNames) {
      names.push_back(component + "target_position_rad." + joint);
    }
  }
  if (mode == ah2::CommandMode::ActuatorPosition) {
    for (const char * actuator : kActuatorBaseNames) {
      names.push_back(component + "target_position_cnt." + actuator);
    }
  } else if (mode == ah2::CommandMode::ActuatorEffort) {
    for (const char * actuator : kActuatorBaseNames) {
      names.push_back(component + "target_effort_pct." + actuator);
    }
  }
  return names;
}

std::vector<std::string> all_command_interfaces(const std::string & side)
{
  std::vector<std::string> names{side + "_hand_control/command_lock"};
  for (const auto mode : {
      ah2::CommandMode::JointPosition, ah2::CommandMode::JointImpedance,
      ah2::CommandMode::ActuatorPosition, ah2::CommandMode::ActuatorEffort})
  {
    const auto mode_names = mode_command_interfaces(side, mode, false);
    names.insert(names.end(), mode_names.begin(), mode_names.end());
  }
  return names;
}

std::optional<ah2::CommandMode> exact_mode_for_interfaces(
  const std::string & side, const std::set<std::string> & claimed)
{
  if (claimed.empty()) return ah2::CommandMode::Idle;
  for (const auto mode : {
      ah2::CommandMode::JointPosition, ah2::CommandMode::JointImpedance,
      ah2::CommandMode::ActuatorPosition, ah2::CommandMode::ActuatorEffort})
  {
    const auto names = mode_command_interfaces(side, mode);
    if (claimed == std::set<std::string>(names.begin(), names.end())) return mode;
  }
  return std::nullopt;
}


// command interface 의 NaN 은 "이번 cycle 명령 없음"(전체) 또는 "상위가 그 축을 점유하지 않음"
// (일부)을 뜻한다. 전자면 set_command 를 부르지 않아 SDK 가 직전 명령을 유지하고, 후자면 빈 자리를
// 채워 완전한 command 로 만든다.
template <std::size_t N>
bool all_nan(const std::array<double, N> & values)
{
  for (const double value : values) {
    if (!std::isnan(value)) return false;
  }
  return true;
}

// 빈 자리는 직전 명령값으로만 채운다 — 그 축을 지금 그대로 두라는 뜻이다. 직전 명령도 없으면
// 채울 값이 없다(NaN 유지). hardware 가 목표를 지어내지 않는다.
double fill_gap(double value, double previous)
{
  return std::isnan(value) ? previous : value;
}

}  // namespace

AidinHand2SystemInterface::~AidinHand2SystemInterface()
{
  stop_service_node();
}

rclcpp::Logger AidinHand2SystemInterface::logger() const
{
  return rclcpp::get_logger(info_.name.empty() ? "aidin_hand2_hardware" : info_.name);
}

CallbackReturn AidinHand2SystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // SDK 로그를 ROS 로깅으로 중계 — 프로세스 전역 1회 (left/right 인터페이스 2개가 같은
  // controller_manager 에 뜨므로). SDK 로거가 전역이라 고정 이름 로거를 쓰고, hand 구분은
  // SDK 가 본문에 넣는 [left]/[right] 태그로 된다. 해제는 안 함 — 프로세스 수명과 같다.
  static std::once_flag sdk_log_callback_registered;
  std::call_once(sdk_log_callback_registered, [] {
    ah2::set_log_callback([](ah2::LogLevel level, const std::string & message) {
      const auto sdk_logger = rclcpp::get_logger("aidin_hand2");
      switch (level) {
        case ah2::LogLevel::Trace:
        case ah2::LogLevel::Debug:
          RCLCPP_DEBUG(sdk_logger, "%s", message.c_str());
          break;
        case ah2::LogLevel::Info:
          RCLCPP_INFO(sdk_logger, "%s", message.c_str());
          break;
        case ah2::LogLevel::Warn:
          RCLCPP_WARN(sdk_logger, "%s", message.c_str());
          break;
        case ah2::LogLevel::Error:
          RCLCPP_ERROR(sdk_logger, "%s", message.c_str());
          break;
        default:
          RCLCPP_FATAL(sdk_logger, "%s", message.c_str());
          break;
      }
    });

    // SDK 의 stderr 직접 출력 차단 — 콘솔엔 ROS 경로로만 나가게 (이중 출력 방지)
    ah2::set_log_to_console(false);
  });

  // ---- 파라미터 ---- (hardware_parameters 는 전부 string 이라 숫자는 변환 필요)
  const auto parameter = [this](const std::string & key) -> std::string {
    const auto found = info_.hardware_parameters.find(key);
    return found == info_.hardware_parameters.end() ? std::string{} : found->second;
  };
  // 빈 값(미지정)이면 fallback 유지. 잘못된 숫자는 stoi/stod 가 throw → 아래 try 가 FATAL 처리.
  const auto as_int = [&](const std::string & key, int fallback) {
    const std::string value = parameter(key);
    return value.empty() ? fallback : std::stoi(value);
  };
  const auto as_double = [&](const std::string & key, double fallback) {
    const std::string value = parameter(key);
    return value.empty() ? fallback : std::stod(value);
  };

  can_interface_ = parameter("can_interface");
  if (can_interface_.empty()) {
    RCLCPP_FATAL(logger(), "missing hardware parameter: can_interface");
    return CallbackReturn::ERROR;
  }

  const std::string hand_side_text = parameter("hand_side");
  if (hand_side_text == "left") {
    hand_side_ = ah2::HandSide::Left;
  } else if (hand_side_text == "right") {
    hand_side_ = ah2::HandSide::Right;
  } else {
    RCLCPP_FATAL(logger(), "hand_side must be 'left' or 'right', got '%s'", hand_side_text.c_str());
    return CallbackReturn::ERROR;
  }
  hand_side_name_ = hand_side_text;
  prefix_ = hand_side_name_ + "_";

  // xacro 는 bool 을 "True"/"False" 로 방출한다. 그 두 값만 받는다.
  const std::string auto_home_text = parameter("auto_home");
  if (auto_home_text != "True" && auto_home_text != "False") {
    RCLCPP_FATAL(logger(), "auto_home must be True/False, got '%s'", auto_home_text.c_str());
    return CallbackReturn::ERROR;
  }
  auto_home_ = auto_home_text == "True";

  // auto_reconnect 3종은 선택적(없으면 기본값) — 통신 두절 시 SDK 자동 재수립 정책.
  auto_reconnect_ = parameter("auto_reconnect") == "True";
  auto_reconnect_home_ = parameter("auto_reconnect_home") == "True";

  try {
    max_effort_ = as_double("max_effort", max_effort_);
    control_rate_ = as_int("control_rate", control_rate_);
    rt_cpu_affinity_ = as_int("rt_cpu_affinity", rt_cpu_affinity_);
    auto_reconnect_timeout_ms_ = as_int("auto_reconnect_timeout_ms", auto_reconnect_timeout_ms_);

    // 미가동 actuator — 콤마 구분 index 목록(예: "0,1,2,3"). 빈 값 = 전부 가동. 공백 허용.
    // 잘못된 토큰은 stoi 가 throw → 아래 catch 가 FATAL 처리(범위 검증은 SDK 가 담당).
    disabled_actuators_.clear();
    const std::string disabled_text = parameter("disabled_actuators");
    std::string token;
    for (std::size_t i = 0; i <= disabled_text.size(); ++i) {
      const char character = (i < disabled_text.size()) ? disabled_text[i] : ',';
      if (character == ',') {
        if (!token.empty()) disabled_actuators_.push_back(std::stoi(token));
        token.clear();
      } else if (character != ' ') {
        token += character;
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(logger(), "invalid numeric hardware parameter: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AidinHand2SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  // actuator 16 — position/velocity/current (모션/센싱). enable/fault 는 진단이라 diagnostics gpio 로.
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::string joint = prefix_ + kActuatorBaseNames[i];
    interfaces.emplace_back(joint, kPositionCountInterface, &state_.actuators.position_count[i]);
    interfaces.emplace_back(joint, kVelocityRpmInterface, &state_.actuators.velocity_rpm[i]);
    interfaces.emplace_back(joint, kCurrentMilliampInterface, &state_.actuators.current_mA[i]);
  }

  // joint 21 — FK position (rad).
  for (std::size_t i = 0; i < ah2::kJointCount; ++i) {
    interfaces.emplace_back(prefix_ + kJointBaseNames[i], kPositionInterface, &joint_position_rad_[i]);
  }

  // tactile — finger 5 x 17.
  for (std::size_t finger = 0; finger < ah2::kFingerCount; ++finger) {
    const std::string sensor = prefix_ + kFingerNames[finger] + "_sensor";
    for (std::size_t cell = 0; cell < ah2::kTactileTaxelsPerFinger; ++cell) {
      interfaces.emplace_back(sensor, "tactile_" + std::to_string(cell + 1),
                              &state_.tactile.fingers[finger][cell]);
    }
  }

  // palm — 3 region 58 (upper 20 + lower 20 + palm2 18), flat 저장.
  const std::string palm = prefix_ + "palm_sensor";
  std::size_t palm_offset = 0;
  const auto add_palm_region = [&](const char * region_prefix, std::size_t count) {
    for (std::size_t cell = 0; cell < count; ++cell) {
      interfaces.emplace_back(palm, region_prefix + std::to_string(cell + 1),
                              &state_.tactile.palm[palm_offset + cell]);
    }
    palm_offset += count;
  };
  add_palm_region("palm1_upper_", ah2::kPalm1UpperCount);
  add_palm_region("palm1_lower_", ah2::kPalm1LowerCount);
  add_palm_region("palm2_", ah2::kPalm2Count);

  // diagnostics gpio — hand 전역 12 + per-actuator enabled 16 + fault 16.
  const std::string diagnostics = prefix_ + "diagnostics";
  for (std::size_t i = 0; i < kDiagnosticsInterfaceNames.size(); ++i) {
    interfaces.emplace_back(diagnostics, kDiagnosticsInterfaceNames[i], &diagnostics_values_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      diagnostics, std::string(kEnabledInterface) + "_" + kActuatorBaseNames[i],
      &actuator_enabled_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      diagnostics, std::string(kFaultInterface) + "_" + kActuatorBaseNames[i],
      &actuator_fault_[i]);
  }

  // command state — SDK variant 를 ros2_control 의 flat double 경계로 펼친다. mode/type/source 가
  // 어떤 typed field 가 유효한지 규정하고, broadcaster 가 다시 typed ROS message 로 조립한다.
  const std::string commanded = prefix_ + kCommandedComponent;
  interfaces.emplace_back(commanded, kControllerInputModeInterface, &controller_input_mode_);
  interfaces.emplace_back(commanded, kControllerOutputTypeInterface, &controller_output_type_);
  interfaces.emplace_back(commanded, kSelectedSourceInterface, &selected_source_);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      commanded, std::string(kControllerInputTargetInterface) + "." + kActiveJointBaseNames[i],
      &controller_input_target_rad_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    const std::string suffix = std::string(".") + kActuatorBaseNames[i];
    interfaces.emplace_back(commanded, kControllerInputActuatorPositionInterface + suffix,
                            &controller_input_target_position_cnt_[i]);
    interfaces.emplace_back(commanded, kControllerInputActuatorEffortInterface + suffix,
                            &controller_input_target_effort_pct_[i]);
    interfaces.emplace_back(commanded, kControllerOutputPositionInterface + suffix,
                            &controller_output_target_position_cnt_[i]);
    interfaces.emplace_back(commanded, kControllerOutputEffortInterface + suffix,
                            &controller_output_target_effort_pct_[i]);
    interfaces.emplace_back(commanded, kCommandedMaxEffortInterface + suffix,
                            &commanded_max_effort_pct_[i]);
  }

  // 관측 timestamp — sec/nanosec 로 분해된 wall-clock (header.stamp 용).
  const std::string timestamp = prefix_ + kTimestampComponent;
  interfaces.emplace_back(timestamp, kStampSecInterface, &observed_stamp_sec_);
  interfaces.emplace_back(timestamp, kStampNanosecInterface, &observed_stamp_nanosec_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AidinHand2SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  // command_lock 은 claim-only mutex다. 어느 controller 도 값을 읽거나 쓰지 않는다.
  interfaces.emplace_back(
    hand_side_name_ + "_hand_control", "command_lock", &command_lock_);

  const std::string joint_position_component =
    command_component(hand_side_name_, ah2::CommandMode::JointPosition);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_position_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_position_target_rad_[i]);
  }

  const std::string joint_impedance_component =
    command_component(hand_side_name_, ah2::CommandMode::JointImpedance);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_impedance_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_impedance_target_rad_[i]);
  }

  const std::string actuator_position_component =
    command_component(hand_side_name_, ah2::CommandMode::ActuatorPosition);
  const std::string actuator_effort_component =
    command_component(hand_side_name_, ah2::CommandMode::ActuatorEffort);
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      actuator_position_component,
      std::string("target_position_cnt.") + kActuatorBaseNames[i],
      &actuator_position_target_cnt_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      actuator_effort_component,
      std::string("target_effort_pct.") + kActuatorBaseNames[i],
      &actuator_effort_target_pct_[i]);
  }

  return interfaces;
}

CallbackReturn AidinHand2SystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // configure = create(자원 할당) + connect(통신 수립·첫 수신 확인, ~300ms blocking). 여기까지는
  // 관측만 — 드라이브 enable 은 activate(run) 몫. 통신이 살아 있어 diagnostics 를 읽을 수 있다.
  try {
    ah2::HandConfig config{can_interface_, hand_side_};
    config.control_rate = control_rate_;
    config.rt_cpu_affinity = rt_cpu_affinity_;
    config.disabled_actuators = disabled_actuators_;
    config.auto_reconnect = auto_reconnect_;
    config.auto_reconnect_timeout_ms = auto_reconnect_timeout_ms_;
    config.auto_reconnect_home = auto_reconnect_home_;
    // SDK 의 blocking auto-home 은 절대 쓰지 않는다 — run() 이 homing 으로 CM executor 를 막지 않게.
    // homing 은 wrapper 가 RT read/write 에서 start_homing()(non-blocking) 으로 직접 건다(auto_home_ 파라미터).
    config.auto_home = false;

    hand_ = manager_.create(config);
    hand_->connect();
    state_ = hand_->get_state();
  } catch (const ah2::Exception &) {
    // 실패 로그는 SDK 가 이미 [exception] <ErrorCode>: <msg> 로 남긴다(한 실패 한 화자). 여기선
    // 자원 정리 + lifecycle 실패 반환만. (set_log_callback 중계로 /rosout 에 이미 나감.)
    if (hand_) {
      manager_.destroy(*hand_);
      hand_.reset();
    }
    return CallbackReturn::ERROR;
  }
  start_service_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // activate = run (기본). 이후 ~/stop 으로 멈추고 ~/run 으로 다시 시작할 수 있다.
  // 실패 메시지는 SDK 가 [exception] 로그로 이미 남긴다(한 실패 한 화자) — 여기선 lifecycle 실패 반환만.
  // auto-home 트리거 래치는 exec_run 이 세운다(모든 run 경로 공용).
  std::string failure_message;
  if (!exec_run(failure_message)) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // homing 진행 중이어도 별도 대기가 필요 없다 — stop() 이 quick stop 으로 homing 을 중단시키고
  // 정지 확인까지 블로킹한다(SDK). 그 뒤 is_homing() 은 한 cycle 안에 false 로 가라앉는다.
  std::string failure_message;
  if (!exec_stop(failure_message)) {
    // 실패 로그는 SDK [exception] 이 유일 기록 — 여기선 반환만.
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  // cleanup = disconnect(통신 종료) + destroy(자원 파기). destroy 는 연결 상태면 정지 확인·종료까지
  // 하므로, disconnect 는 정상 종료 경로에서만 성공하고 실패해도 destroy 가 마무리한다.
  stop_service_node();
  if (hand_) {
    // disconnect/destroy 실패는 SDK 가 [exception] 로그로 남긴다(한 실패 한 화자). disconnect 실패해도
    // destroy 로 마무리하고, destroy 실패도 삼켜 cleanup 을 진행한다(재출력 없이 흐름만).
    try {
      hand_->disconnect();
    } catch (const ah2::Exception &) {
    }
    try {
      manager_.destroy(*hand_);
    } catch (const ah2::Exception &) {
    }
    hand_.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AidinHand2SystemInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

// start/stop delta 를 현재 claim 집합에 적용한 다음, 정확히 한 mode 의 완전한 port+lock 집합인지
// 검증한다. suffix 추론이나 부분 claim 은 허용하지 않는다.
hardware_interface::return_type AidinHand2SystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto all_names = all_command_interfaces(hand_side_name_);
  const std::set<std::string> owned(all_names.begin(), all_names.end());
  pending_command_interfaces_ = active_command_interfaces_;

  for (const std::string & name : stop_interfaces) {
    if (owned.count(name) != 0) pending_command_interfaces_.erase(name);
  }
  for (const std::string & name : start_interfaces) {
    if (owned.count(name) != 0) pending_command_interfaces_.insert(name);
  }

  const auto decoded =
    exact_mode_for_interfaces(hand_side_name_, pending_command_interfaces_);
  if (!decoded) {
    pending_mode_switch_valid_ = false;
    RCLCPP_ERROR(
      logger(),
      "rejected mode switch: command interfaces must be one complete mode port plus command_lock");
    return hardware_interface::return_type::ERROR;
  }

  pending_mode_ = *decoded;
  pending_mode_switch_valid_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2SystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  if (!pending_mode_switch_valid_) return hardware_interface::return_type::ERROR;
  active_command_interfaces_ = pending_command_interfaces_;
  command_mode_ = pending_mode_;
  pending_mode_switch_valid_ = false;

  clear_mode_command();  // controller 가 처음 쓰기 전까지는 명령 없음
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!hand_) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    // 관측 snapshot — SDK HandState 복사 후 joint position(rad) 은 state interface 저장소로.
    state_ = hand_->get_state();
    joint_position_rad_ = state_.joints.position_rad;  // SDK·ROS 둘 다 rad

    // SDK variant → ros2_control flat state interface. NaN 은 해당 typed input/output 에서
    // 유효하지 않은 필드임을 뜻하며 broadcaster 가 mode/type 로 다시 구조화한다.
    const double unused = std::numeric_limits<double>::quiet_NaN();
    controller_input_target_rad_.fill(unused);
    controller_input_target_position_cnt_.fill(unused);
    controller_input_target_effort_pct_.fill(unused);
    controller_output_target_position_cnt_.fill(unused);
    controller_output_target_effort_pct_.fill(unused);

    controller_input_mode_ = static_cast<double>(
      static_cast<int>(ah2::to_command_mode(state_.commanded.controller_input)));
    if (const auto * input =
        std::get_if<ah2::JointPositionCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_rad_ = input->target;
    } else if (const auto * input =
        std::get_if<ah2::JointImpedanceCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_rad_ = input->target;
    } else if (const auto * input =
        std::get_if<ah2::ActuatorPositionCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_position_cnt_ = input->target;
    } else if (const auto * input =
        std::get_if<ah2::ActuatorEffortCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_effort_pct_ = input->target;
    }

    controller_output_type_ = 0.0;
    if (const auto * output =
        std::get_if<ah2::ActuatorPositionSetpoint>(&state_.commanded.controller_output))
    {
      controller_output_type_ = 1.0;
      controller_output_target_position_cnt_ = output->target_position_cnt;
    } else if (const auto * output =
        std::get_if<ah2::ActuatorEffortSetpoint>(&state_.commanded.controller_output))
    {
      controller_output_type_ = 2.0;
      controller_output_target_effort_pct_ = output->target_effort_pct;
    }
    selected_source_ =
      static_cast<double>(static_cast<int>(state_.commanded.selected_source));
    commanded_max_effort_pct_ = state_.commanded.max_effort_pct;

    // 관측 timestamp(ns) → sec/nanosec 분해. 정수 나눗셈이라 double 반올림 없이 header.stamp 로 흐른다.
    observed_stamp_sec_ = static_cast<double>(state_.timestamp / 1000000000LL);
    observed_stamp_nanosec_ = static_cast<double>(state_.timestamp % 1000000000LL);

    // 진단 snapshot — SDK Diagnostics 를 gpio state 순서로 double 화. lifecycle 은 enum→double
    // (ordinal), broadcaster 가 to_string 으로 이름화(command echo mode/source 와 동일 방식).
    const ah2::Diagnostics diagnostics = hand_->get_diagnostics();
    diagnostics_values_ = {
      static_cast<double>(static_cast<int>(diagnostics.lifecycle)),
      static_cast<double>(diagnostics.nan_command_count),
      static_cast<double>(diagnostics.control_cycles),
      static_cast<double>(diagnostics.deadline_misses),
      diagnostics.last_period_ms,
      diagnostics.last_compute_ms,
      static_cast<double>(static_cast<int>(diagnostics.homing_state))};

    // per-actuator enable·fault
    for (std::size_t actuator = 0; actuator < ah2::kActuatorCount; ++actuator) {
      actuator_enabled_[actuator] = diagnostics.actuator_health.enabled[actuator] ? 1.0 : 0.0;
      actuator_fault_[actuator] = static_cast<double>(
        static_cast<std::uint16_t>(diagnostics.actuator_health.fault[actuator]));
    }
  } catch (const ah2::Exception &) {
    // SDK [exception] 로그가 유일 기록(한 실패 한 화자). get_state/get_diagnostics 는 통신 두절로
    // 던지지 않으므로(lock-free 버퍼 읽기) 여기 오면 핸들 무효(destroy 후) — 복구 불가라 fail-fast.
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AidinHand2SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!hand_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // tuning 은 lifecycle·homing 과 무관하게 먼저 적용한다 — ~/stop 중에 바꿔도 반영된다.
    apply_tuning_parameters();

    if (!started_.load()) {
      return hardware_interface::return_type::OK;  // ~/stop 으로 멈춘 상태 — command 미전송 (재개는 ~/run)
    }
    if (hand_->get_diagnostics().lifecycle != ah2::HandLifecycle::Running) {
      return hardware_interface::return_type::OK;
    }

    // auto-home — 명령을 내보내는 이 write 단계에서 non-blocking 으로 건다(스레드 없음). 원점 미확정이면
    // start_homing() 을 1회만(래치). start_homing() 은 request_homing_ 을 동기적으로 세우고 즉시 반환하므로,
    // 바로 아래 억제 체크의 is_homing() 이 이 cycle 부터 true → command 유출 없음. 실제 homing 명령은
    // SDK RT loop 가 보낸다(여기 write 아님). start_homing()/get_diagnostics() 예외는 아래 catch 가 받는다.
    if (auto_home_ && !hand_->is_homing() &&
        hand_->get_diagnostics().homing_state != ah2::HomingState::Succeeded &&
        !auto_home_triggered_.exchange(true)) {
      hand_->start_homing();
    }

    // homing 중이거나 원점 미확정이면 command 미전송. homing 중: FSM 간섭 방지. 미확정: SDK 가 set_command
    // 를 거부(WrongCallOrder)하므로 매 cycle 그 예외를 내는 대신 조용히 넘긴다. is_homing() 만으로는
    // "homing 도 아닌데 아직 미확정" 구간(재연결 직후, auto_home 실패)이 빠진다. homing_state 가 Succeeded 로
    // 돌아오면(~/home·재homing·~/reconnect) 자연히 재개된다.
    if (hand_->is_homing() ||
        hand_->get_diagnostics().homing_state != ah2::HomingState::Succeeded) {
      return hardware_interface::return_type::OK;
    }

    switch (command_mode_) {
      case ah2::CommandMode::Idle:
        hand_->set_command(ah2::Idle{});
        break;

      case ah2::CommandMode::ActuatorPosition: {
        if (all_nan(actuator_position_target_cnt_)) break;
        ah2::ActuatorPositionCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
          command.target[i] = fill_gap(
            actuator_position_target_cnt_[i], controller_input_target_position_cnt_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::ActuatorEffort: {
        if (all_nan(actuator_effort_target_pct_)) break;
        ah2::ActuatorEffortCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
          command.target[i] = fill_gap(
            actuator_effort_target_pct_[i], controller_input_target_effort_pct_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::JointPosition: {
        if (all_nan(joint_position_target_rad_)) break;
        ah2::JointPositionCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
          command.target[i] = fill_gap(
            joint_position_target_rad_[i], controller_input_target_rad_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::JointImpedance: {
        if (all_nan(joint_impedance_target_rad_)) break;
        ah2::JointImpedanceCommand command;
        bool complete = true;
        for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
          command.target[i] = fill_gap(
            joint_impedance_target_rad_[i], controller_input_target_rad_[i]);
          if (std::isnan(command.target[i])) complete = false;
        }
        if (!complete) {
          warn_incomplete_command();
          break;
        }
        hand_->set_command(command);
        break;
      }
    }
  } catch (const ah2::Exception &) {
    // SDK [exception] 로그가 유일 기록. 위 lifecycle 게이트를 스쳐 지난 race(체크 후 두절)라 OK 로
    // 흘려 컴포넌트를 내리지 않는다 — 명령 미전송이 곧 fail-safe 이고 복구는 ~/reconnect 로 한다.
    return hardware_interface::return_type::OK;
  }
  return hardware_interface::return_type::OK;
}

// 현재 mode 의 command 저장소를 비운다(NaN = 명령 없음). mode 전환·재개 직후처럼 상위가 아직
// 아무것도 쓰지 않은 구간에서 옛 값이 명령으로 나가지 않게 한다.
// 일부 축이 NaN 인데 그 축에 직전 명령도 없어 명령을 완성할 수 없을 때. 상위가 그 축을 한 번도
// 점유하지 않았다는 뜻이라, 채우지 않고 그 cycle 을 건너뛴다(hardware 가 목표를 지어내지 않는다).
void AidinHand2SystemInterface::warn_incomplete_command()
{
  RCLCPP_WARN_THROTTLE(
    logger(), throttle_clock_, 5000,
    "command has axes that were never commanded — skipped. Send a complete command once.");
}

void AidinHand2SystemInterface::clear_mode_command()
{
  const double unset = std::numeric_limits<double>::quiet_NaN();
  switch (command_mode_) {
    case ah2::CommandMode::ActuatorPosition:
      actuator_position_target_cnt_.fill(unset);
      break;
    case ah2::CommandMode::ActuatorEffort:
      actuator_effort_target_pct_.fill(unset);
      break;
    case ah2::CommandMode::JointPosition:
      joint_position_target_rad_.fill(unset);
      break;
    case ah2::CommandMode::JointImpedance:
      joint_impedance_target_rad_.fill(unset);
      break;
    case ah2::CommandMode::Idle:
      break;
  }
}

bool AidinHand2SystemInterface::exec_run(std::string & failure_message)
{
  try {
    hand_->run();
    started_.store(true);

    // run 시점에 상위 명령은 없다 — 저장소를 비워 옛 목표가 다시 나가지 않게 한다. 상위가 첫 명령을
    // 줄 때까지의 자세 유지는 SDK 몫이다(stop→run 이면 run() 이 현재 자세 hold 를 세운다).
    clear_mode_command();
    // auto-home 1회 트리거 래치를 세운다 — homing 이 완료되지 않은 채 run 하면(첫 run·reconnect 후
    // run·homing 중 stop 후 run) write 가 start_homing() 을 다시 건다. 완료돼 있으면 아무 일도 없다.
    auto_home_triggered_.store(false);
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

bool AidinHand2SystemInterface::exec_stop(std::string & failure_message)
{
  try {
    hand_->stop();
    started_.store(false);  // write 가 이후 command 를 보내지 않도록 (재가동은 ~/run)
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

bool AidinHand2SystemInterface::exec_home(std::string & failure_message)
{
  // homing 트리거만 하고 즉시 반환(non-blocking) — CM executor 를 막지 않는다. 직전 명령 Idle 리셋은 SDK 가
  // 트리거 시점에 하고, 완료 관측은 diagnostics.homing_state 로 한다(wrapper 는 상태를 세우지 않음).
  // 재진입(이미 homing 중 재호출)은 SDK 가 처리한다(request 재설정). check_allowed(Home) 게이트가 상태 검증.
  try {
    hand_->start_homing();
    return true;
  } catch (const ah2::Exception & exception) {
    // 서비스 응답 메시지로 돌려준다(로그 아님 — 로그는 SDK [exception] 이 유일 기록).
    failure_message = exception.what();
    return false;
  }
}

bool AidinHand2SystemInterface::exec_reconnect(std::string & failure_message)
{
  // 통신 두절·실패 복구 — 통신만 재수립하고 제어는 시작하지 않는다(started_=false). reconnect 가
  // homing_state 를 NotRun 으로 내리므로 homing 도 다시 해야 한다. 제어 시작은 ~/run 이 맡는다
  // (auto_home 트리거 래치도 exec_run 이 세운다).
  try {
    hand_->reconnect();
    started_.store(false);
    return true;
  } catch (const ah2::Exception & exception) {
    failure_message = exception.what();
    return false;
  }
}

void AidinHand2SystemInterface::start_service_node()
{
  if (service_node_) {
    return;
  }

  // hardware 컴포넌트는 기본 node 가 없다 — start/stop/home 은 제어 루프 밖 관리 동작이라
  // 자체 node 를 만들어 ~/run·~/stop·~/home service 로 노출한다 (전용 spin 스레드).
  service_node_ = std::make_shared<rclcpp::Node>(info_.name);
  run_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/run",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_run(failure_message);
      response->message = response->success ? "running" : failure_message;
    });
  stop_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/stop",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_stop(failure_message);
      response->message = response->success ? "stopped" : failure_message;
    });
  home_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/home",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_home(failure_message);
      // start_homing 은 트리거만 — 완료를 기다리지 않는다. 완료는 diagnostics 의 homing_state 로 관측한다.
      response->message = response->success ? "homing started — poll diagnostics 'homing_state'" : failure_message;
    });
  reconnect_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/reconnect",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
           const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
      std::string failure_message;
      response->success = exec_reconnect(failure_message);
      response->message = response->success ? "reconnected — call ~/run to resume control" : failure_message;
    });
  declare_tuning_parameters();
  service_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(service_node_);
  service_spin_thread_ = std::thread([this] { service_executor_->spin(); });
}

// 런타임 tuning parameter 선언 — max_effort 와 SDK ControllerConfig. mode·claim 과 무관한 값이라
// controller 가 아니라 이 hardware 노드가 소유한다(run/stop 서비스와 같은 자리). 기본값은 xacro
// hardware_parameter(max_effort)와 SDK 기본값이고, launch 는 controllers.yaml 에 이 노드 이름으로
// 블록을 두어 덮는다. declare 시점에 override 가 반영되므로 별도 초기 적용 경로는 두지 않는다.
void AidinHand2SystemInterface::declare_tuning_parameters()
{
  const ah2::ControllerConfig defaults{};
  staged_max_effort_.fill(max_effort_);
  staged_controller_config_ = defaults;
  tuning_dirty_ = true;  // 첫 write 가 선언된 값을 그대로 SDK 에 적용한다

  service_node_->declare_parameter("max_effort", std::vector<double>{max_effort_});
  service_node_->declare_parameter(
    "joint_position_controller.filter_enabled", defaults.joint_position_controller.filter_enabled);
  service_node_->declare_parameter(
    "joint_position_controller.cutoff_freq", defaults.joint_position_controller.cutoff_freq);
  service_node_->declare_parameter(
    "joint_position_controller.deadband", defaults.joint_position_controller.deadband);
  service_node_->declare_parameter(
    "joint_impedance_controller.stiffness",
    std::vector<double>(defaults.joint_impedance_controller.stiffness.begin(),
                        defaults.joint_impedance_controller.stiffness.end()));
  service_node_->declare_parameter(
    "joint_impedance_controller.damping",
    std::vector<double>(defaults.joint_impedance_controller.damping.begin(),
                        defaults.joint_impedance_controller.damping.end()));

  // declare 로 들어온 override 를 staging 에 반영한 뒤 콜백을 붙인다 — 콜백은 이후 변경만 받는다.
  const std::vector<std::string> tuning_names{
    "max_effort",
    "joint_position_controller.filter_enabled",
    "joint_position_controller.cutoff_freq",
    "joint_position_controller.deadband",
    "joint_impedance_controller.stiffness",
    "joint_impedance_controller.damping",
  };
  (void)on_set_tuning_parameters(service_node_->get_parameters(tuning_names));
  tuning_callback_ = service_node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return on_set_tuning_parameters(parameters);
    });
}

// parameter 검증 + staging 반영. 배열은 길이 1(전체 공통) 또는 16(actuator 별)만 받는다. 거부하면
// ROS 가 값을 반영하지 않으므로 잘못된 값이 SDK 까지 가지 않는다. 범위 clamp 는 SDK 몫.
rcl_interfaces::msg::SetParametersResult AidinHand2SystemInterface::on_set_tuning_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 검증은 staging 사본에 먼저 적용한다 — 한 parameter 라도 거부되면 아무것도 바뀌지 않는다.
  std::array<double, ah2::kActuatorCount> max_effort = staged_max_effort_;
  ah2::ControllerConfig config = staged_controller_config_;

  const auto expand = [&result](
    const rclcpp::Parameter & parameter, std::array<double, ah2::kActuatorCount> & target) {
    const std::vector<double> values = parameter.as_double_array();
    if (values.size() != 1 && values.size() != ah2::kActuatorCount) {
      result.successful = false;
      result.reason = parameter.get_name() + " must have 1 or 16 values";
      return;
    }
    for (const double value : values) {
      if (!std::isfinite(value) || value < 0.0) {
        result.successful = false;
        result.reason = parameter.get_name() + " must be finite and >= 0";
        return;
      }
    }
    for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
      target[i] = values.size() == 1 ? values[0] : values[i];
    }
  };
  const auto scalar = [&result](const rclcpp::Parameter & parameter, double & target) {
    const double value = parameter.as_double();
    if (!std::isfinite(value) || value < 0.0) {
      result.successful = false;
      result.reason = parameter.get_name() + " must be finite and >= 0";
      return;
    }
    target = value;
  };

  for (const rclcpp::Parameter & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name == "max_effort") {
      expand(parameter, max_effort);
    } else if (name == "joint_position_controller.filter_enabled") {
      config.joint_position_controller.filter_enabled = parameter.as_bool();
    } else if (name == "joint_position_controller.cutoff_freq") {
      scalar(parameter, config.joint_position_controller.cutoff_freq);
    } else if (name == "joint_position_controller.deadband") {
      scalar(parameter, config.joint_position_controller.deadband);
    } else if (name == "joint_impedance_controller.stiffness") {
      expand(parameter, config.joint_impedance_controller.stiffness);
    } else if (name == "joint_impedance_controller.damping") {
      expand(parameter, config.joint_impedance_controller.damping);
    }
    if (!result.successful) return result;
  }

  std::lock_guard<std::mutex> lock(tuning_mutex_);
  staged_max_effort_ = max_effort;
  staged_controller_config_ = config;
  tuning_dirty_ = true;
  return result;
}

// staging 을 SDK 로 넘긴다 — 바뀐 cycle 에만. write 스레드에서만 부른다.
void AidinHand2SystemInterface::apply_tuning_parameters()
{
  std::array<double, ah2::kActuatorCount> max_effort{};
  ah2::ControllerConfig config{};
  {
    std::lock_guard<std::mutex> lock(tuning_mutex_);
    if (!tuning_dirty_) return;
    max_effort = staged_max_effort_;
    config = staged_controller_config_;
    tuning_dirty_ = false;
  }
  hand_->set_max_effort(max_effort);
  hand_->set_controller_config(config);
}

void AidinHand2SystemInterface::stop_service_node()
{
  // 진행 중인 서비스 콜백이 있으면 끝날 때까지 join 이 대기한다.
  if (service_executor_) {
    service_executor_->cancel();
  }
  if (service_spin_thread_.joinable()) {
    service_spin_thread_.join();
  }
  run_service_.reset();
  stop_service_.reset();
  home_service_.reset();
  reconnect_service_.reset();
  tuning_callback_.reset();
  if (service_executor_ && service_node_) {
    service_executor_->remove_node(service_node_);
  }
  service_executor_.reset();
  service_node_.reset();
}

}  // namespace aidin_hand2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_hardware::AidinHand2SystemInterface, hardware_interface::SystemInterface)

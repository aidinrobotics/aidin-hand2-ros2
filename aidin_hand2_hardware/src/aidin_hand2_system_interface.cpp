#include "aidin_hand2_hardware/aidin_hand2_system_interface.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"

// Hardware command interface contract (one hand, 98 resources)
//   <side>_hand_control/command_lock                                      ×1
//   <side>_joint_position_command/target_position_rad.<active_joint>     ×16
//   <side>_joint_position_command/speed_rad_s                             ×1
//   <side>_joint_impedance_command/target_position_rad.<active_joint>    ×16
//   <side>_joint_impedance_command/{stiffness,damping}.<actuator>        ×32
//   <side>_actuator_position_command/target_position_cnt.<actuator>      ×16
//   <side>_actuator_effort_command/target_effort_pct.<actuator>          ×16
// command_lock is claim-only. A mode switch accepts only an empty set (Idle)
// or exactly one complete mode set including the lock.
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
constexpr char kControllerInputSpeedInterface[] = "controller_input_speed_rad_s";
constexpr char kControllerInputTargetInterface[] = "controller_input_target_position_rad";
constexpr char kControllerInputActuatorPositionInterface[] =
  "controller_input_target_position_cnt";
constexpr char kControllerInputActuatorEffortInterface[] =
  "controller_input_target_effort_pct";
constexpr char kControllerOutputPositionInterface[] = "controller_output_target_position_cnt";
constexpr char kControllerOutputEffortInterface[] = "controller_output_target_effort_pct";
constexpr char kCommandedStiffnessInterface[] = "stiffness";           // actuator
constexpr char kCommandedDampingInterface[] = "damping";               // actuator
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
  if (mode == ah2::CommandMode::JointPosition) {
    names.push_back(component + "speed_rad_s");
  } else if (mode == ah2::CommandMode::JointImpedance) {
    for (const char * actuator : kActuatorBaseNames) {
      names.push_back(component + "stiffness." + actuator);
    }
    for (const char * actuator : kActuatorBaseNames) {
      names.push_back(component + "damping." + actuator);
    }
  } else if (mode == ah2::CommandMode::ActuatorPosition) {
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
  joint_impedance_stiffness_ = ah2::kDefaultStiffness;
  joint_impedance_damping_ = ah2::kDefaultDamping;

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
    max_effort_.store(as_double("max_effort", max_effort_.load()));
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
  interfaces.emplace_back(commanded, kControllerInputSpeedInterface, &controller_input_speed_rad_s_);
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
    interfaces.emplace_back(commanded, kCommandedStiffnessInterface + suffix,
                            &controller_input_stiffness_[i]);
    interfaces.emplace_back(commanded, kCommandedDampingInterface + suffix,
                            &controller_input_damping_[i]);
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
  interfaces.emplace_back(
    joint_position_component, "speed_rad_s", &joint_position_speed_rad_s_);

  const std::string joint_impedance_component =
    command_component(hand_side_name_, ah2::CommandMode::JointImpedance);
  for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
    interfaces.emplace_back(
      joint_impedance_component,
      std::string("target_position_rad.") + kActiveJointBaseNames[i],
      &joint_impedance_target_rad_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      joint_impedance_component,
      std::string("stiffness.") + kActuatorBaseNames[i],
      &joint_impedance_stiffness_[i]);
  }
  for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
    interfaces.emplace_back(
      joint_impedance_component,
      std::string("damping.") + kActuatorBaseNames[i],
      &joint_impedance_damping_[i]);
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
    hand_->set_max_effort(max_effort_.load());
    last_applied_max_effort_ = max_effort_.load();  // write 재적용 게이트 기준 초기화
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
  // activate = run (기본). 이후 ~/stop 으로 멈추고 ~/run 으로 재개할 수 있다.
  // 실패 메시지는 SDK 가 [exception] 로그로 이미 남긴다(한 실패 한 화자) — 여기선 lifecycle 실패 반환만.
  std::string failure_message;
  if (!exec_run(failure_message)) {  // exec_run 이 state_ 갱신 + mode 별 command seed 수행
    return CallbackReturn::ERROR;
  }
  // auto-home 은 스레드 없이 RT read 에서 non-blocking 으로 건다(start_homing). 여기선 이번 활성화
  // 구간의 1회 트리거 래치만 다시 세운다 — 원점 미확정(homing_state != Succeeded)을 보면 한 번
  // start_homing() 을 걸고, 완료는 homing_state 로 관측한다. homing 중 write 는 command 를 억제한다.
  auto_home_triggered_.store(false);
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

  // controller 첫 update 전에도 port 는 완전하고 finite 하다.
  switch (command_mode_) {
    case ah2::CommandMode::ActuatorPosition:
      actuator_position_target_cnt_ = state_.actuators.position_count;
      break;
    case ah2::CommandMode::ActuatorEffort:
      actuator_effort_target_pct_.fill(0.0);
      break;
    case ah2::CommandMode::JointPosition:
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        joint_position_target_rad_[i] = state_.joints.position_rad[kActiveToJointIndex[i]];
      }
      break;
    case ah2::CommandMode::JointImpedance:
      for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
        joint_impedance_target_rad_[i] = state_.joints.position_rad[kActiveToJointIndex[i]];
      }
      break;
    case ah2::CommandMode::Idle:
      break;
  }
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
    controller_input_speed_rad_s_ = unused;
    controller_input_stiffness_.fill(unused);
    controller_input_damping_.fill(unused);
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
      controller_input_speed_rad_s_ = input->speed_rad_s;
    } else if (const auto * input =
        std::get_if<ah2::JointImpedanceCommand>(&state_.commanded.controller_input))
    {
      controller_input_target_rad_ = input->target;
      controller_input_stiffness_ = input->gains.stiffness;
      controller_input_damping_ = input->gains.damping;
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

  if (!started_.load()) {
    return hardware_interface::return_type::OK;  // ~/stop 으로 멈춘 상태 — command 미전송 (재개는 ~/run)
  }

  try {
    // 비-Running 게이트 — 이때 start_homing()/set_command() 를 부르면 예외가 나고, 그 예외로 ERROR 를
    // 올리면 ros2_control 이 하드웨어 컴포넌트를 내린다(unconfigured). 그러면 같은 CM 의 다른 컴포넌트·
    // joint_state_broadcaster 까지 함께 죽고 SDK 의 auto_reconnect 도 무력화되므로, 조용히 넘겨(OK)
    // 컴포넌트를 active 로 유지한다. Running 복귀 시 다음 cycle 부터 제어가 자연히 재개된다.
    // Faulted 로 굳은 경우도 명령 미전송(fail-safe)과 같은 의미이고 복구는 ~/reconnect 로 한다.
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

    // max_effort 런타임 적용 — 모드와 무관(전 actuator 공통 안전 한계). 변경됐을 때만 SDK 재호출
    // (config/latch 라 매 cycle 호출 불필요). ~/set_max_effort 토픽이 목표값을 갱신한다.
    const double target_max_effort = max_effort_.load();
    if (target_max_effort != last_applied_max_effort_) {
      hand_->set_max_effort(target_max_effort);
      last_applied_max_effort_ = target_max_effort;
    }

    switch (command_mode_) {
      case ah2::CommandMode::Idle:
        hand_->set_command(ah2::Idle{});
        break;

      case ah2::CommandMode::ActuatorPosition: {
        ah2::ActuatorPositionCommand command;
        command.target = actuator_position_target_cnt_;
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::ActuatorEffort: {
        ah2::ActuatorEffortCommand command;
        command.target = actuator_effort_target_pct_;
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::JointPosition: {
        ah2::JointPositionCommand command;
        command.target = joint_position_target_rad_;
        command.speed_rad_s = joint_position_speed_rad_s_;
        hand_->set_command(command);
        break;
      }
      case ah2::CommandMode::JointImpedance: {
        ah2::JointImpedanceCommand command;
        command.target = joint_impedance_target_rad_;
        command.gains.stiffness = joint_impedance_stiffness_;
        command.gains.damping = joint_impedance_damping_;
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

bool AidinHand2SystemInterface::exec_run(std::string & failure_message)
{
  try {
    hand_->run();
    started_.store(true);

    // 재개 안전 초기값 — 현재 mode 의 command 저장소를 현재 자세로(effort 는 0). stop 중 손이
    // 옮겨졌거나 상위가 명령을 멈췄어도 재개 시 튀지 않게. controller 가 command 를 주면 덮인다.
    // (perform 과 동일 로직 — mode 안 바뀌는 재개는 perform 이 안 불리므로 여기서도 채운다.)
    state_ = hand_->get_state();
    switch (command_mode_) {
      case ah2::CommandMode::ActuatorPosition:
        actuator_position_target_cnt_ = state_.actuators.position_count;
        break;
      case ah2::CommandMode::ActuatorEffort:
        actuator_effort_target_pct_.fill(0.0);
        break;
      case ah2::CommandMode::JointPosition:
        for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
          joint_position_target_rad_[i] = state_.joints.position_rad[kActiveToJointIndex[i]];
        }
        break;
      case ah2::CommandMode::JointImpedance:
        for (std::size_t i = 0; i < ah2::kActiveJointCount; ++i) {
          joint_impedance_target_rad_[i] = state_.joints.position_rad[kActiveToJointIndex[i]];
        }
        break;
      case ah2::CommandMode::Idle:
        break;
    }
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
  // 통신 두절·실패 복구 — 통신만 재수립하고 제어(run)는 재개하지 않는다(started_=false). 원점은
  // 재수립 중 소실됐을 수 있어 재확인 대상이 된다. 복귀 후 제어는 ~/run(auto_home 시 homing 포함).
  try {
    hand_->reconnect();
    started_.store(false);
    // 재수립 후 원점을 다시 잡아야 하므로(reconnect 가 homing_state 를 NotRun 으로 내린다) 트리거를 다시 세운다.
    // 이게 없으면 다음 ~/run 후 read 가 이미 소진된 래치 탓에 start_homing 을 다시 걸지 않는다.
    auto_home_triggered_.store(false);
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
  // max_effort 런타임 갱신 — 안전 한계값(rated current %). 콜백은 값만 저장하고, write 가 변경 시
  // SDK 적용(범위 clamp 는 SDK set_max_effort 내부 소관). 모드·claim 과 무관해 controller 대신 이
  // hardware 노드가 직접 받는다(start/stop 서비스와 동일 위치).
  max_effort_sub_ = service_node_->create_subscription<std_msgs::msg::Float64>(
    "~/set_max_effort", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64 & message) { max_effort_.store(message.data); });
  service_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(service_node_);
  service_spin_thread_ = std::thread([this] { service_executor_->spin(); });
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
  max_effort_sub_.reset();
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

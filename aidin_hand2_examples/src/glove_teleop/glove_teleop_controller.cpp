#include "glove_teleop/glove_teleop_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

#include "rclcpp/qos.hpp"

// ── chain 구조 ──────────────────────────────────────────────────────
//   [manus_ros2] --manus_glove_N(topic)--> [GloveTeleopController]
//        ergonomics(0~1) --map--> active_joint 목표(rad)
//        --> 하위 자세 controller 의 reference interface 에 기입
//        --> hardware
//
//   claim 하는 reference 이름(하위 JointPosition/JointImpedance controller 노출):
//     {target_controller}/{side}_{finger}_joint{n}/position       (16, 자세)
//   speed reference 는 claim 하지 않는다 — 하위 controller가 activation 때 seed한 speed를 유지한다.
//
//   이 controller 는 chain 최상위라 자신의 reference 는 노출하지 않는다(on_export = 빈 목록).
// ────────────────────────────────────────────────────────────────────

namespace aidin_hand2_examples
{

namespace
{
// active_joint 16 의 base 이름 (하위 controller 와 동일 순서·규약).
constexpr const char * kActiveJointBaseNames[kActiveJointCount] = {
  "thumb_joint0",  "thumb_joint1",  "thumb_joint2",  "thumb_joint3",
  "index_joint1",  "index_joint2",  "index_joint3",
  "middle_joint1", "middle_joint2", "middle_joint3",
  "ring_joint1",   "ring_joint2",   "ring_joint3",
  "baby_joint1",   "baby_joint2",   "baby_joint3",
};

// active_joint index → 기본 ergonomics type. 실기기 캘리브 전 시작점이며 yaml 로 재정의한다.
// 관절 정체(URDF limit): thumb joint1=MCP flex, joint2=MCP abd, joint0=CMC abd, joint3=DIP flex.
// long finger joint1=abduction, joint2=MCP flex, joint3=PIP flex.
constexpr const char * kDefaultErgonomicsType[kActiveJointCount] = {
  "ThumbMCPSpread",   "ThumbMCPStretch",  "ThumbMCPSpread",   "ThumbDIPStretch",
  "IndexSpread",      "IndexMCPStretch",  "IndexPIPStretch",
  "MiddleSpread",     "MiddleMCPStretch", "MiddlePIPStretch",
  "RingSpread",       "RingMCPStretch",   "RingPIPStretch",
  "PinkySpread",      "PinkyMCPStretch",  "PinkyPIPStretch",
};

}  // namespace

// ── lifecycle ───────────────────────────────────────────────────────
controller_interface::CallbackReturn GloveTeleopController::on_init()
{
  auto_declare<std::string>("hand_side", "");
  auto_declare<std::string>("target_controller", "");
  auto_declare<std::string>("glove_topic", "");

  // 관절 목표를 반올림할 격자(deg). 0 이면 비활성. 미세 떨림 억제용.
  auto_declare<double>("quantize_step_deg", 0.0);

  // 매핑 파라미터: joint 별 ergonomics_type·scale·offset. 미선언 시 위 기본값.
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    const std::string joint = kActiveJointBaseNames[i];
    auto_declare<std::string>("mapping." + joint + ".ergonomics_type", kDefaultErgonomicsType[i]);
    auto_declare<double>("mapping." + joint + ".scale", 0.0);
    auto_declare<double>("mapping." + joint + ".offset", 0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GloveTeleopController::on_configure(
  const rclcpp_lifecycle::State &)
{
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(get_node()->get_logger(), "hand_side must be 'left' or 'right' (got '%s')",
                 hand_side_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const std::string prefix = hand_side_ + "_";

  // 하위 자세 controller 이름 — 미지정 시 side 기본값(joint position controller).
  target_controller_ = get_node()->get_parameter("target_controller").as_string();
  if (target_controller_.empty()) {
    target_controller_ = prefix + "joint_position_controller";
  }

  // 구독할 글러브 토픽 — 미지정 시 manus_glove_0.
  glove_topic_ = get_node()->get_parameter("glove_topic").as_string();
  if (glove_topic_.empty()) {
    glove_topic_ = "manus_glove_0";
  }

  quantize_step_deg_ = get_node()->get_parameter("quantize_step_deg").as_double();

  // 매핑·claim 대상 reference 이름 구성.
  reference_interface_names_.clear();
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    const std::string joint = prefix + kActiveJointBaseNames[i];
    const std::string base = kActiveJointBaseNames[i];

    mappings_[i].ergonomics_type =
      get_node()->get_parameter("mapping." + base + ".ergonomics_type").as_string();
    mappings_[i].scale  = get_node()->get_parameter("mapping." + base + ".scale").as_double();
    mappings_[i].offset = get_node()->get_parameter("mapping." + base + ".offset").as_double();

    // chain: 하위 controller 의 자세 reference interface 를 claim.
    reference_interface_names_.push_back(target_controller_ + "/" + joint + "/position");
  }

  glove_buffer_.writeFromNonRT(std::shared_ptr<manus_ros2_msgs::msg::ManusGlove>());
  glove_subscriber_ = get_node()->create_subscription<manus_ros2_msgs::msg::ManusGlove>(
    glove_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<manus_ros2_msgs::msg::ManusGlove> message) {
      glove_buffer_.writeFromNonRT(message);
    });

  RCLCPP_INFO(get_node()->get_logger(),
              "glove teleop: '%s' -> reference of '%s' (%zu joints)",
              glove_topic_.c_str(), target_controller_.c_str(), kActiveJointCount);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GloveTeleopController::on_activate(
  const rclcpp_lifecycle::State &)
{
  glove_buffer_.writeFromNonRT(std::shared_ptr<manus_ros2_msgs::msg::ManusGlove>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GloveTeleopController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ── interface configuration ─────────────────────────────────────────
// 하위 controller 의 자세 reference 16 을 command interface 로 claim → chain 성립.
controller_interface::InterfaceConfiguration
GloveTeleopController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, reference_interface_names_};
}

controller_interface::InterfaceConfiguration
GloveTeleopController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

// active_joint 16 자세를 reference 로 노출 — 하위 controller 와 동일한 형식이라, 나중에 상위
// controller(궤적 생성기 등)가 glove_teleop 을 chain 으로 구동할 수 있다. 현재 update 는 글러브
// 데이터로만 지령하고 이 reference 를 소비하진 않는다(형식·확장 대비). Humble 은 chainable 이면
// reference 최소 1개를 요구하므로 이 노출이 configure 통과에도 필요하다.
std::vector<hardware_interface::CommandInterface>
GloveTeleopController::on_export_reference_interfaces()
{
  reference_interfaces_.assign(kActiveJointCount, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> references;
  references.reserve(kActiveJointCount);
  const std::string prefix = hand_side_ + "_";
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    references.emplace_back(hardware_interface::CommandInterface(
      get_node()->get_name(), prefix + kActiveJointBaseNames[i] + "/position",
      &reference_interfaces_[i]));
  }
  return references;
}

bool GloveTeleopController::on_set_chained_mode(bool)
{
  return true;
}

// ── update ──────────────────────────────────────────────────────────
// standalone 경로 없음(항상 글러브 토픽 구독) — 여기서는 할 일 없음.
controller_interface::return_type GloveTeleopController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

// 최신 글러브 데이터를 매핑해 하위 controller 의 자세 reference 에 기입.
// ergonomics 가 없거나 매핑 type 이 빈 joint 는 NaN → 하위 controller 가 직전 값 유지.
controller_interface::return_type GloveTeleopController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto message = *glove_buffer_.readFromRT();
  if (!message) {
    return controller_interface::return_type::OK;
  }

  // ergonomics type → value 조회용 (메시지당 한 번 구성).
  // MANUS ergonomics value 는 degree 단위(실측: 손 편 상태 stretch ~30~40°). scale·offset 은
  // degree 도메인에서 적용하고(scale=방향+배율, offset=deg) 마지막에 한 번 rad 로 변환한다:
  //   rad_target = (value_deg · scale + offset_deg) · π/180
  std::unordered_map<std::string, double> ergonomics_deg;
  ergonomics_deg.reserve(message->ergonomics.size());
  for (const auto & entry : message->ergonomics) {
    ergonomics_deg.emplace(entry.type, entry.value);
  }

  constexpr double kDegToRad = M_PI / 180.0;

  // 관절 목표 반올림 격자(rad). deg 파라미터를 rad 로 환산해 둔다. 0 이면 반올림 안 함.
  const double quantize_step_rad = quantize_step_deg_ * kDegToRad;

  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    const auto & mapping = mappings_[i];
    double target = std::numeric_limits<double>::quiet_NaN();

    // 단순 선형: target_rad = (value_deg · scale + offset_deg) · π/180.
    if (!mapping.ergonomics_type.empty()) {
      const auto found = ergonomics_deg.find(mapping.ergonomics_type);
      if (found != ergonomics_deg.end()) {
        target = (found->second * mapping.scale + mapping.offset) * kDegToRad;

        // 미세 떨림 억제: 목표를 격자로 반올림해 정지 시 값이 계속 흔들리지 않게 한다.
        if (quantize_step_rad > 0.0) {
          target = std::round(target / quantize_step_rad) * quantize_step_rad;
        }
      }
    }
    if (!std::isnan(target)) {
      (void)command_interfaces_[i].set_value(target);
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace aidin_hand2_examples

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aidin_hand2_examples::GloveTeleopController,
  controller_interface::ChainableControllerInterface)

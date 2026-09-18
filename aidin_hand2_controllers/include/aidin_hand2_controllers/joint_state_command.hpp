#ifndef AIDIN_HAND2_CONTROLLERS__JOINT_STATE_COMMAND_HPP_
#define AIDIN_HAND2_CONTROLLERS__JOINT_STATE_COMMAND_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

namespace aidin_hand2_controllers
{

// A JointState resolved by name into the controller's own order, NaN for a name not sent
// sequence 0 is no message yet
template <std::size_t N>
struct JointStateCommand
{
  std::uint64_t sequence{0};
  std::array<double, N> values{};
};

enum class JointStateField { kPosition, kEffort };

// An empty name takes the N values in the controller's own order
// Otherwise false when name differs in length from the field read or repeats a name
// A name the controller does not own is skipped
template <std::size_t N>
bool resolve_joint_state_command(
  const sensor_msgs::msg::JointState & msg, const std::vector<std::string> & names,
  JointStateField field, std::array<double, N> & out)
{
  const auto & values = field == JointStateField::kPosition ? msg.position : msg.effort;
  if (msg.name.empty()) {
    if (values.size() != N) {
      return false;
    }
    std::copy(values.begin(), values.end(), out.begin());
    return true;
  }
  if (msg.name.size() != values.size()) {
    return false;
  }
  out.fill(std::numeric_limits<double>::quiet_NaN());
  std::array<bool, N> seen{};
  for (std::size_t k = 0; k < msg.name.size(); ++k) {
    for (std::size_t i = 0; i < N; ++i) {
      if (msg.name[k] != names[i]) {
        continue;
      }
      if (seen[i]) {
        return false;
      }
      seen[i] = true;
      out[i] = values[k];
      break;
    }
  }
  return true;
}

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__JOINT_STATE_COMMAND_HPP_

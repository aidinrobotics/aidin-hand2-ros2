// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef AIDIN_HAND2_CONTROLLERS__WARN_THROTTLE_HPP_
#define AIDIN_HAND2_CONTROLLERS__WARN_THROTTLE_HPP_

#include <chrono>

namespace aidin_hand2_controllers
{

// Lets one warning through at most once per period, per object
// RCLCPP_*_THROTTLE keeps its time per call site, which both hands in one process would share
// Not thread safe, each one belongs to the subscriber callback or to update
class WarnThrottle
{
public:
  explicit WarnThrottle(std::chrono::milliseconds period) : period_(period) {}

  // True when the period has passed since the last true, and on the first call
  bool allow()
  {
    const auto now = std::chrono::steady_clock::now();
    if (armed_ && now - last_ < period_) {
      return false;
    }
    armed_ = true;
    last_ = now;
    return true;
  }

private:
  std::chrono::steady_clock::duration period_;
  std::chrono::steady_clock::time_point last_{};
  bool armed_{false};
};

}  // namespace aidin_hand2_controllers

#endif  // AIDIN_HAND2_CONTROLLERS__WARN_THROTTLE_HPP_

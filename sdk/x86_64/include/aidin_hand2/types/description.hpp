// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

namespace aidin_hand2
{

// --------------------------------- Constants --------------------------------

// Fixed structural constants of the AIDIN Hand Gen2
inline constexpr std::size_t kActuatorCount          = 16;
inline constexpr std::size_t kJointCount             = 21;
inline constexpr std::size_t kActiveJointCount       = 16;

// The 4-bar coupled q4 of each digit
inline constexpr std::size_t kPassiveJointCount      = 5;

// Fingertip positions as xyz
inline constexpr std::size_t kTaskCount              = 15;

// Thumb 6-D pose with 4 fingertip positions
inline constexpr std::size_t kOrientedTaskCount      = 18;

inline constexpr std::size_t kFingerCount            = 5;
inline constexpr std::size_t kTactileTaxelsPerFinger = 17;
inline constexpr std::size_t kPalmTactileCount       = 58;
inline constexpr std::size_t kPalm1UpperCount        = 20;
inline constexpr std::size_t kPalm1LowerCount        = 20;
inline constexpr std::size_t kPalm2Count             = 18;

// ----------------------------------- Enums ----------------------------------

enum class HandSide {
  Left,
  Right,
};

enum class Finger {
  Thumb,
  Index,
  Middle,
  Ring,
  Baby,
};

// -------------------------------- Hand model --------------------------------

// The robot hand's model, given with the hand
enum class HandModel {
  Gen2,
  Gen2B,
};

// HandModel name for launch arguments and settings
[[nodiscard]] constexpr const char* to_string(HandModel model) noexcept
{
  switch (model) {
    case HandModel::Gen2: return "gen2";
    case HandModel::Gen2B: return "gen2b";
  }
  return "gen2";
}

// "gen2" or "gen2b" in any letter case, nullopt for anything else
[[nodiscard]] constexpr std::optional<HandModel> parse_hand_model(std::string_view text) noexcept
{
  const auto matches = [text](std::string_view name) {
    if (text.size() != name.size()) return false;
    for (std::size_t i = 0; i < name.size(); ++i) {
      const char lower =
          (text[i] >= 'A' && text[i] <= 'Z') ? static_cast<char>(text[i] - 'A' + 'a') : text[i];
      if (lower != name[i]) return false;
    }
    return true;
  };
  if (matches("gen2")) return HandModel::Gen2;
  if (matches("gen2b")) return HandModel::Gen2B;
  return std::nullopt;
}

}  // namespace aidin_hand2

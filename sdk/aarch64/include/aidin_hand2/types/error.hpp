// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <aidin_hand2/export.hpp>
#include <aidin_hand2/types/description.hpp>  // HandSide

namespace aidin_hand2
{

// --------------------------------- Error kind -------------------------------

// SDK operation failure, grouped by what the caller should check
enum class ErrorKind {
  None = 0,

  // Fix the call sequence or the arguments
  InvalidArgument,
  WrongCallOrder,

  // Check the environment or the device
  InterfaceUnavailable,
  CommunicationLost,
  HardwareFault,

  // Fix the cause in Diagnostics::fault_code, then reconnect()
  HandFaulted,

  // A fault in the SDK itself
  ControlLoopFault,
  UnexpectedError,
};

// ErrorKind name for logs
[[nodiscard]] constexpr const char* to_string(ErrorKind kind) noexcept
{
  switch (kind) {
    case ErrorKind::None: return "None";
    case ErrorKind::InvalidArgument: return "InvalidArgument";
    case ErrorKind::WrongCallOrder: return "WrongCallOrder";
    case ErrorKind::InterfaceUnavailable: return "InterfaceUnavailable";
    case ErrorKind::CommunicationLost: return "CommunicationLost";
    case ErrorKind::HardwareFault: return "HardwareFault";
    case ErrorKind::HandFaulted: return "HandFaulted";
    case ErrorKind::ControlLoopFault: return "ControlLoopFault";
    case ErrorKind::UnexpectedError: return "UnexpectedError";
  }
  return "UnexpectedError";
}

// The kind an error code belongs to, which is its hundreds digit
//   1xx InvalidArgument, 2xx WrongCallOrder, 3xx InterfaceUnavailable, 4xx CommunicationLost,
//   5xx HardwareFault, 6xx HandFaulted, 7xx ControlLoopFault, 9xx UnexpectedError, 0 None
[[nodiscard]] constexpr ErrorKind kind_of(std::uint16_t code) noexcept
{
  switch (code / 100) {
    case 0: return ErrorKind::None;
    case 1: return ErrorKind::InvalidArgument;
    case 2: return ErrorKind::WrongCallOrder;
    case 3: return ErrorKind::InterfaceUnavailable;
    case 4: return ErrorKind::CommunicationLost;
    case 5: return ErrorKind::HardwareFault;
    case 6: return ErrorKind::HandFaulted;
    case 7: return ErrorKind::ControlLoopFault;
  }
  return ErrorKind::UnexpectedError;
}

namespace detail
{

// "[E<code>] <message>", or the message alone for code 0
[[nodiscard]] inline std::string coded_message(std::uint16_t code, std::string message)
{
  if (code == 0) return message;
  return "[E" + std::to_string(code) + "] " + message;
}

// "[left] " or "[right] ", in front of every message about one hand, log line or exception
[[nodiscard]] constexpr const char* hand_prefix(HandSide side) noexcept
{
  return side == HandSide::Left ? "[left] " : "[right] ";
}

}  // namespace detail

// --------------------------------- Exception --------------------------------

// Thrown by every public call
class AIDIN_HAND2_EXPORT Exception : public std::runtime_error {
 public:
  // An SDK failure, whose what() is "[E<code>] <message>"
  Exception(std::uint16_t code, std::string message)
  : std::runtime_error(detail::coded_message(code, std::move(message))), kind_(kind_of(code)), code_(code)
  {
  }

  // An SDK failure about one hand, whose what() is "[left] [E<code>] <message>"
  Exception(HandSide side, std::uint16_t code, std::string message)
  : std::runtime_error(detail::hand_prefix(side) + detail::coded_message(code, std::move(message))),
    kind_(kind_of(code)),
    code_(code)
  {
  }

  // An application's own failure, with code 0 and the message as what()
  Exception(ErrorKind kind, std::string message)
  : std::runtime_error(std::move(message)), kind_(kind)
  {
  }

  [[nodiscard]] ErrorKind kind() const noexcept
  {
    return kind_;
  }

  // The number of this failure in the error message table, 0 for an application's own
  [[nodiscard]] std::uint16_t code() const noexcept
  {
    return code_;
  }

 private:
  ErrorKind kind_{ErrorKind::UnexpectedError};
  std::uint16_t code_{0};
};

}  // namespace aidin_hand2

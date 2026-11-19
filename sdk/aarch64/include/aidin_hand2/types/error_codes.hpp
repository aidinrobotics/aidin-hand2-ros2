// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

// One number per cause, the value Exception::code(), Diagnostics::fault_code and the log line carry
// The hundreds digit is the ErrorKind. Within a kind the numbers follow the order the SDK checks
// along create, connect, run, home, command and config, stop, disconnect, reconnect and destroy,
// with the realtime events after the calls. Features left out of the documentation come last
// A number is never changed or reused, and a new cause takes the next free number of its kind

namespace aidin_hand2::error_code
{

// ------------------------- E1xx InvalidArgument --------------------------

constexpr std::uint16_t kInterfaceNameEmpty          = 101;
constexpr std::uint16_t kHandSideInvalid             = 102;
constexpr std::uint16_t kHandModelInvalid            = 103;

constexpr std::uint16_t kMaxEffortInvalid            = 104;
constexpr std::uint16_t kPositionControllerInvalid   = 105;
constexpr std::uint16_t kImpedanceStiffnessInvalid   = 106;
constexpr std::uint16_t kImpedanceCutoffInvalid      = 107;

// -------------------------- E2xx WrongCallOrder --------------------------

constexpr std::uint16_t kAlreadyConnected            = 201;
constexpr std::uint16_t kNotConnected                = 202;
constexpr std::uint16_t kHomingInterrupted           = 203;
constexpr std::uint16_t kNotRunning                  = 204;
constexpr std::uint16_t kNotHomed                    = 205;
constexpr std::uint16_t kNotFaulted                  = 206;
constexpr std::uint16_t kHandDestroyed               = 207;

// ----------------------- E3xx InterfaceUnavailable -----------------------

constexpr std::uint16_t kInterfaceNotFound           = 301;
constexpr std::uint16_t kSocketPermissionDenied      = 302;
constexpr std::uint16_t kCanNotSupported             = 303;
constexpr std::uint16_t kCanFdNotConfigured          = 304;
constexpr std::uint16_t kInterfaceDown               = 305;
constexpr std::uint16_t kInterfaceOpenFailed         = 306;
constexpr std::uint16_t kOtherMaster                 = 307;

// interface_name "auto", not documented
constexpr std::uint16_t kAutoNoInterface             = 308;
constexpr std::uint16_t kAutoNoHand                  = 309;

// ------------------------ E4xx CommunicationLost -------------------------

constexpr std::uint16_t kNoFirstFrame                = 401;
constexpr std::uint16_t kLinkFailed                  = 402;
constexpr std::uint16_t kQuickStopUnconfirmed        = 403;
constexpr std::uint16_t kReceiveLost                 = 404;
constexpr std::uint16_t kTransmitFailed              = 405;
constexpr std::uint16_t kAutoReconnectTimeout        = 406;

// -------------------------- E5xx HardwareFault ---------------------------

constexpr std::uint16_t kEnableTimeout               = 501;
constexpr std::uint16_t kHomingFailed                = 502;
constexpr std::uint16_t kQuickStopTimeout            = 503;
constexpr std::uint16_t kActuatorFault               = 504;

// --------------------------- E6xx HandFaulted ----------------------------

// Every call refused in Faulted, whatever put the hand there
constexpr std::uint16_t kHandFaulted                 = 601;

// ------------------------- E7xx ControlLoopFault -------------------------

constexpr std::uint16_t kThreadStartFailed           = 701;
constexpr std::uint16_t kControlLoopTerminated       = 702;
constexpr std::uint16_t kEventLoggingTerminated      = 703;

// ------------------------- E9xx UnexpectedError -------------------------

constexpr std::uint16_t kUnexpectedException         = 901;
constexpr std::uint16_t kInternalState               = 902;

}  // namespace aidin_hand2::error_code

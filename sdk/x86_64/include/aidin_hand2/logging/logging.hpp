// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <functional>
#include <string>

#include <aidin_hand2/export.hpp>

namespace aidin_hand2
{

// -------------------------------- Log level ---------------------------------

enum class LogLevel {
  Trace,
  Debug,
  Info,
  Warn,
  Error,
  Critical,
  Off,
};

// ----------------------------------- Sinks ----------------------------------

// Set the console level, the file and callback sinks always keep Debug
AIDIN_HAND2_EXPORT void set_log_level(LogLevel level);

// Turn console output on or off
AIDIN_HAND2_EXPORT void set_log_to_console(bool enabled);

// Write to a rotating file: 5 MiB each, 3 rotated files plus the current one
// A path that cannot be opened is reported as a warning, not an exception
AIDIN_HAND2_EXPORT void set_log_to_file(const std::string& path);

// Flush every sink
AIDIN_HAND2_EXPORT void flush_log();

// --------------------------------- Callback ---------------------------------

// Forward every line from Debug up, independent of the console and file sinks
// One callback at a time, nullptr clears it
using LogCallback = std::function<void(LogLevel, const std::string&)>;
AIDIN_HAND2_EXPORT void set_log_callback(LogCallback callback);

}  // namespace aidin_hand2

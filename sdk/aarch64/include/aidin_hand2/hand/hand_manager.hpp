// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <vector>

#include <aidin_hand2/export.hpp>
#include <aidin_hand2/hand/hand.hpp>
#include <aidin_hand2/types/config.hpp>

namespace aidin_hand2
{

class HandCore;

// Owns every HandCore and is the only place they are created or destroyed
class AIDIN_HAND2_EXPORT HandManager {
 public:
  // -------------------------------- Lifetime --------------------------------

  HandManager();
  ~HandManager();

  HandManager(HandManager&& other) noexcept;
  HandManager& operator=(HandManager&& other) noexcept;

  HandManager(const HandManager&) = delete;
  HandManager& operator=(const HandManager&) = delete;

  // --------------------------------- Hands ----------------------------------

  // Validate the config and allocate a core
  [[nodiscard]] Hand create(const HandConfig& config);

  // Close the core and drop it from the registry, invalidating the handle (blocking)
  void destroy(Hand& hand);

  // Close every core and empty the registry (blocking)
  void destroy_all();

 private:
  std::vector<std::shared_ptr<HandCore>> cores_;
};

}  // namespace aidin_hand2

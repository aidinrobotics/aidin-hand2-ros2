// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/error.hpp>
#include <aidin_hand2/types/error_codes.hpp>

namespace aidin_hand2
{

// ------------------------------- Hand config --------------------------------

struct HandConfig {
  // Throws E101, E102 or E103 of the error message table
  HandConfig(std::string interface_name, HandSide hand_side, HandModel hand_model)
  : interface_name(std::move(interface_name)), hand_side(hand_side), hand_model(hand_model)
  {
    if (this->interface_name.empty()) {
      throw Exception(error_code::kInterfaceNameEmpty,
                      "cannot create HandConfig: interface_name must not be empty");
    }
    if (hand_side != HandSide::Left && hand_side != HandSide::Right) {
      throw Exception(error_code::kHandSideInvalid,
                      "cannot create HandConfig: hand_side must be HandSide::Left or HandSide::Right");
    }
    if (hand_model != HandModel::Gen2 && hand_model != HandModel::Gen2B) {
      throw Exception(error_code::kHandModelInvalid,
                      "cannot create HandConfig: hand_model must be HandModel::Gen2 or HandModel::Gen2B");
    }
  }

  // "can0", or "auto" to scan for this hand's channel
  std::string interface_name;
  HandSide    hand_side;
  HandModel   hand_model;

  bool auto_home{true};

  // -1 = unset
  int rt_cpu_affinity{-1};
  bool auto_reconnect{false};

  // 0 = no limit
  int auto_reconnect_timeout_ms{0};
  bool auto_reconnect_home{false};
  std::vector<int> disabled_actuators{};
};

// ---------------------------- Controller config -----------------------------

// Runtime settable with set_controller_config()
struct ControllerConfig {
  struct JointPositionController {
    bool filter_enabled{true};

    // Hz
    double cutoff_freq{10.0};

    // rad
    double deadband{0.000873};
  };
  struct JointImpedanceController {
    // Per active joint, in the order of JointImpedanceCommand::target
    // 0..1, 1 = the joint's hard stiffness
    std::array<double, kActiveJointCount> stiffness{
        0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8,
        0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};

    // Target low-pass, the joint position controller's without the deadband
    bool filter_enabled{true};

    // Hz
    double cutoff_freq{10.0};
  };

  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};

}  // namespace aidin_hand2

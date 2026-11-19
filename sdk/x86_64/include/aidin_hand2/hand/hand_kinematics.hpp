// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

#include <aidin_hand2/export.hpp>
#include <aidin_hand2/types/description.hpp>

namespace aidin_hand2
{
// Angles in rad, actuators in encoder count, hand_model the HandConfig::hand_model of the hand

// ---------------------------- Forward kinematics ----------------------------

// Actuator encoders to joint angles
AIDIN_HAND2_EXPORT std::array<double, kJointCount> fk_actuator_to_joint(
    HandModel hand_model, const std::array<int, kActuatorCount>& encoder);

// Actuator encoders to fingertip positions (not using)
AIDIN_HAND2_EXPORT std::array<double, kTaskCount> fk_actuator_to_task(
    HandModel hand_model, const std::array<int, kActuatorCount>& encoder);

// ---------------------------- Inverse kinematics ----------------------------

// Joint angles to actuator encoders
AIDIN_HAND2_EXPORT std::array<int, kActuatorCount> ik_joint_to_actuator(
    HandModel hand_model, const std::array<double, kActiveJointCount>& active_joint);

// Fingertip positions to joint angles (not using)
AIDIN_HAND2_EXPORT std::array<double, kActiveJointCount> ik_task_to_joint(
    HandModel hand_model, const std::array<double, kTaskCount>& task);

// Thumb pose and fingertip positions to joint angles (not using)
AIDIN_HAND2_EXPORT std::array<double, kActiveJointCount> ik_oriented_task_to_joint(
    HandModel hand_model, const std::array<double, kOrientedTaskCount>& oriented_task);

}  // namespace aidin_hand2

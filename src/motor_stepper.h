// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file motor_stepper.h
/// @brief Class that handles stepper motor control.

#pragma once

#include <Arduino.h>
#include <stepper_driver.h>

#include "common_types.h"
#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Motor Stepper class.
class MotorStepper {
 public:

  /// @brief Construct a Motor Stepper object.
  MotorStepper(mt::StepperDriver& stepper_driver, Configuration& configuration = Configuration::GetInstance());

  /// @brief Destroy the Motor Stepper object.
  ~MotorStepper();

  /// @brief Actuate the motor based on the current action and control mode.
  /// @param control_mode The control mode.
  /// @param control_action The control action.
  /// @param status_output The status message to return, indicating the current action.
  void Actuate(common::ControlMode control_mode, common::ControlAction control_action,
               String& status_output); ///< This must be called repeatedly.

  /// @brief Log/report the general status of the motor based on the current control mode.
  /// @param control_mode The control mode.
  void LogGeneralStatus(common::ControlMode control_mode) const;

  /// @brief Get the homing status.
  /// @return The homing status.
  bool homing() const;

 private:

  /// @brief Configuration settings.
  Configuration& configuration_;

  // Stepper motor driver.
  mt::StepperDriver& stepper_driver_;
  
  // Control flags and indicator variables.
  mt::StepperDriver::MotionDirection motion_direction_ = configuration_.kDefaultMotionDirection_; ///< Variable to keep track of the motion direction.
  mt::StepperDriver::MotionType motion_type_ = mt::StepperDriver::MotionType::kRelative; ///< Variable to keep track of the motion type.
  float sweep_direction_ = static_cast<float>(motion_direction_); ///< Variable to keep track of the sweep direction.
  uint8_t sweep_angle_index_ = configuration_.kDefaultSweepAngleIndex_; ///< Index to keep track of the sweep angle set from the lookup table.
  uint8_t speed_index_ = configuration_.kDefaultSpeedIndex_; ///< Index to keep track of the motor speed set from the lookup table.
  mt::StepperDriver::MotionStatus motion_status_ = mt::StepperDriver::MotionStatus::kIdle; ///< Variable to keep track of the motion status.
  bool allow_motion_ = false; ///< Flag to control the motion toggle based on user input.
  float homing_angle_degrees_ = 0.0F; ///< Variable to store the angular position (degrees) for homing.
  bool homing_ = false; ///< Flag to keep track of the homing status.
};

} // namespace mtmotor_jig
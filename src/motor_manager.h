// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file motor_manager.h
/// @brief Class that handles motor control.

#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <Arduino.h>
#include <stepper_driver.h>

#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Motor Manager class.
class MotorManager {
 public:

  /// @brief Construct a Motor Manager object.
  MotorManager();

  /// @brief Destroy the Motor Manager object.
  ~MotorManager();

  /// @brief Initialise the motor.
  void Begin(); ///< This must be called only once.

  /// @brief Actuate the motor based on the current action and control mode.
  /// @param control_mode The control mode.
  /// @param control_action The control action.
  /// @param status_output The status message to return, indicating the current action.
  void Actuate(Configuration::ControlMode control_mode, Configuration::ControlAction control_action,
               String& status_output); ///< This must be called repeatedly.

  /// @brief Log/report the general status of the motor based on the current control mode.
  /// @param control_mode The control mode.
  void LogGeneralStatus(Configuration::ControlMode control_mode) const;

  /// @brief Get the homing status.
  /// @return The homing status.
  bool homing() const;

 private:

  /// @brief Configuration settings.
  Configuration& configuration_ = Configuration::GetInstance();

  // Stepper motor driver.
  mt::StepperDriver stepper_driver_{configuration_.kMotorDriverPulPin_,
                                    configuration_.kMotorDriverDirPin_,
                                    configuration_.kMotorDriverEnaPin_,
                                    configuration_.kMicrostepMode_,
                                    configuration_.kFullStepAngle_degrees_,
                                    configuration_.kGearRatio_};

  // Control flags and indicator variables.
  mt::StepperDriver::MotionDirection motion_direction_ = configuration_.kDefaultMotionDirection_; ///< Variable to keep track of the motion direction.
  mt::StepperDriver::MotionType motion_type_ = mt::StepperDriver::MotionType::kRelative; ///< Variable to keep track of the motion type.
  float sweep_direction_ = static_cast<float>(motion_direction_); ///< Variable to keep track of the sweep direction.
  uint8_t sweep_angle_index_ = configuration_.kDefaultSweepAngleIndex_; ///< Index to keep track of the sweep angle set from the lookup table.
  uint8_t speed_index_ = configuration_.kDefaultSpeedIndex_; ///< Index to keep track of the motor speed set from the lookup table.
  mt::StepperDriver::MotionStatus motion_status_ = mt::StepperDriver::MotionStatus::kIdle; ///< Variable to keep track of the motion status.
  bool allow_motion_ = false; ///< Variable to keep track of the motion toggle based on user input.
  float homing_angle_degrees_ = 0.0F; ///< Variable to store the angular position (degrees) for homing.
  bool homing_ = false; ///< Variable to keep track of the homing status.
};

} // namespace mtmotor_jig

#endif // MOTOR_MANAGER_H_
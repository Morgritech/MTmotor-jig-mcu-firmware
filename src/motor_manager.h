// Copyright (C) 2024 Morgritech
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
  void Begin();

  /// @brief Actuate the motor.
  void Actuate(Configuration::ControlMode control_mode, Configuration::ControlAction control_action);

  /// @brief Log/report the general status of the motor.
  void LogGeneralStatus(Configuration::ControlMode control_mode) const;

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
  //mt::StepperDriver::MotionDirection previous_motion_direction_ = configuration_.kDefaultMotionDirection_; // Variable to keep track of the previously set motion direction.
  mt::StepperDriver::MotionType motion_type_ = mt::StepperDriver::MotionType::kRelative; ///< Variable to keep track of the motion type.
  float sweep_direction_ = static_cast<float>(motion_direction_); ///< Variable to keep track of the sweep direction.
  uint8_t sweep_angle_index_ = configuration_.kDefaultSweepAngleIndex_; ///< Index to keep track of the sweep angle set from the lookup table.
  uint8_t speed_index_ = configuration_.kDefaultSpeedIndex_; ///< Index to keep track of the motor speed set from the lookup table.
  mt::StepperDriver::MotionStatus motion_status_ = mt::StepperDriver::MotionStatus::kIdle; ///< Variable to keep track of the motion status.
};

} // namespace mtmotor_jig

#endif // MOTOR_MANAGER_H_
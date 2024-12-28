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
};

} // namespace mtmotor_jig

#endif // MOTOR_MANAGER_H_
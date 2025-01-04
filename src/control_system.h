// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file control_system.h
/// @brief Class that links sensor inputs (buttons, serial, etc.) to actuator outputs (display, steppers, etc.).

#ifndef CONTROL_SYSTEM_H_
#define CONTROL_SYSTEM_H_

#include <Arduino.h>

#include "configuration.h"
#include "input_manager.h"
#include "motor_manager.h"
#include "display_manager.h"

namespace mtmotor_jig {

/// @brief The Control System class.
class ControlSystem {
 public:

  /// @brief Construct a Control System object. 
  ControlSystem();

  /// @brief Destroy the Control System object.
  ~ControlSystem();

  /// @brief Initialise the hardware (Serial port, logging, pins, etc.).
  void Begin(); ///< This must be called only once.

  /// @brief Check inputs and trigger outputs/actions.
  void CheckAndProcess(); ///< This must be called repeatedly.

 private:

  /// @brief Log/report the general status of the control system.
  void LogGeneralStatus() const;

  /// @brief Configuration settings.
  Configuration& configuration_ = Configuration::GetInstance();

  // Sensors and actuators, inputs and outputs.
  InputManager inputs_; ///< The User inputs (encoder, buttons, serial, etc.).
  MotorManager motor_; ///< The Motor drive system.
  DisplayManager display_; ///< The display (LCD).

  // Control flags and indicator variables.
  Configuration::ControlMode control_mode_ = configuration_.kDefaultControlMode_; ///< Variable to keep track of the control system mode.
  Configuration::ControlMode previous_control_mode_ = configuration_.kDefaultControlMode_; ///< Variable to keep track of the previously set control system mode.
  Configuration::ControlAction control_action_ = Configuration::ControlAction::kIdle; ///< Variable to keep track of the control actions from user inputs.
  String status_ = ""; ///< Variable to keep track of the status message to display.
};

} // namespace mtmotor_jig

#endif // CONTROL_SYSTEM_H_
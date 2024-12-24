// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file control_system.h
/// @brief Class that links sensor inputs (buttons, serial, etc.) to actuator outputs (display, steppers, etc.).

#ifndef CONTROL_SYSTEM_H_
#define CONTROL_SYSTEM_H_

#include <Arduino.h>
#include <momentary_button.h>
#include <stepper_driver.h>

#include "configuration.h"
#include "display.h"

namespace mtmotor_jig {

/// @brief The Control System class.
class ControlSystem {
 public:

  /// @brief Construct a Control System object. 
  ControlSystem();

  /// @brief Destroy the Control System object.
  ~ControlSystem();

  /// @brief Initialise the hardware (Serial port, logging, pins, etc.).
  void Begin() const;

  /// @brief Check inputs and trigger outputs/actions.
  void CheckAndProcess(); ///< This must be called periodically.

 private:

  /// @brief Log/report the general status of the control system.
  void LogGeneralStatus() const;

  /// @brief Configuration settings.
  Configuration& configuration_ = Configuration::GetInstance();

};

} // namespace mtmotor_jig

#endif // CONTROL_SYSTEM_H_
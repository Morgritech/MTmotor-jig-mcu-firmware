// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_manager.h
/// @brief Class that handles user input (buttons, serial, etc.).

#pragma once

#include <Arduino.h>

#include "configuration.h"
#include "input.h"

namespace mtmotor_jig {

/// @brief The Input Manager class.
class InputManager {
 public:
 
  /// @brief Construct an Input Manager object.
  InputManager(InputInterface& inputs);

  /// @brief Destroy the Input Manager object.
  ~InputManager();

  /// @brief Check for user input based on the current control mode.
  /// @param control_mode The control mode.
  /// @return The control action.
  Configuration::ControlAction Check(Configuration::ControlMode control_mode); ///< This must be called repeatedly.

 private:

  // Buttons to control the motor.
  InputInterface& inputs_; ///< The inputs.
};

} // namespace mtmotor_jig
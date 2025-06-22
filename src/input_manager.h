// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_manager.h
/// @brief Class that handles user input (buttons, serial, etc.).

#pragma once

#include <Arduino.h>
#include <momentary_button.h>
#include <rotary_encoder.h>

#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Input Manager class.
class InputManager {
 public:
 
  /// @brief Construct an Input Manager object.
  InputManager(Configuration& configuration, mt::RotaryEncoder& encoder_dial, mt::MomentaryButton& encoder_button, 
               mt::MomentaryButton& controller_button, mt::MomentaryButton& limit_switch);

  /// @brief Destroy the Input Manager object.
  ~InputManager();

  /// @brief Initialise the inputs.
  void Begin(); ///< This must be called only once.

  /// @brief Check for user input based on the current control mode.
  /// @param control_mode The control mode.
  /// @return The control action.
  Configuration::ControlAction Check(Configuration::ControlMode control_mode); ///< This must be called repeatedly.

 private:

  /// @brief Configuration settings.
  Configuration& configuration_;

  // Buttons to control the motor.
  mt::RotaryEncoder& encoder_dial_; ///< Encoder dial to control mode selection.
  mt::MomentaryButton& encoder_button_; ///< Button to control motor direction or angle.
  mt::MomentaryButton& controller_button_; ///< Button to control motor speed.
  mt::MomentaryButton& limit_switch_; ///< Limit switch to manipulate the motor with respect to a soft home position.
};

} // namespace mtmotor_jig
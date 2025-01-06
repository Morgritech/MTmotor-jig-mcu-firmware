// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_manager.h
/// @brief Class that handles user input (buttons, serial, etc.).

#ifndef INPUT_MANAGER_H_
#define INPUT_MANAGER_H_

#include <Arduino.h>
#include <momentary_button.h>
#include <rotary_encoder.h>

#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Input Manager class.
class InputManager {
 public:
 
  /// @brief Construct an Input Manager object.
  InputManager();

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
  Configuration& configuration_ = Configuration::GetInstance();

  // Buttons to control the motor.
  mt::RotaryEncoder encoder_dial_{configuration_.kEncoderContactAPin_,
                                  configuration_.kEncoderContactBPin_,
                                  configuration_.kEncoderDetents_,
                                  configuration_.kEncoderMaxRotationAngle_degrees_}; ///< Encoder dial to control mode selection.
  mt::MomentaryButton encoder_button_{configuration_.kEncoderButtonPin_,
                                       configuration_.kUnpressedPinState_,
                                       configuration_.kDebouncePeriod_ms_,
                                       configuration_.kShortPressPeriod_ms_,
                                       configuration_.kLongPressPeriod_ms_}; ///< Button to control motor direction or angle.
  mt::MomentaryButton controller_button_{configuration_.kControllerButtonPin_,
                                       configuration_.kUnpressedPinState_,
                                       configuration_.kDebouncePeriod_ms_,
                                       configuration_.kShortPressPeriod_ms_,
                                       configuration_.kLongPressPeriod_ms_}; ///< Button to control motor speed.
  mt::MomentaryButton limit_switch_{configuration_.kLimitSwitchPin_,
                                    configuration_.kUnpressedPinState_,
                                    configuration_.kDebouncePeriod_ms_,
                                    configuration_.kShortPressPeriod_ms_,
                                    configuration_.kLongPressPeriod_ms_}; ///< Limit switch to manipulate the motor with respect to a soft home position.
};

} // namespace mtmotor_jig

#endif // INPUT_MANAGER_H_
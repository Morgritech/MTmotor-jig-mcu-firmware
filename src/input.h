// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input.h
/// @brief Class for creating configurable inputs (buttons, serial, etc.).

#pragma once

#include <Arduino.h>
#include <momentary_button.h>
#include <rotary_encoder.h>

#include "configuration.h"
#include "input_interface.h"

namespace mtmotor_jig {
/*
template<typename InputType>
struct InputTraits {
  typeid(InputType) type_id_; ///< The type of the input.
  auto typeid = typeid(InputType)
};
*/

/// @brief The Input class.
template <typename InputType>
class Input : public InputInterface {
 public:
 
  /// @brief Construct an Input object.
  Input(Configuration& configuration, InputType& input);

  /// @brief Destroy the Input object.
  ~Input();

  /// @brief Initialise the input.
  void Begin() override; ///< This must be called only once.

  /// @brief Check for user input based on the current control mode.
  /// @param control_mode The control mode.
  /// @return The control action.
  Configuration::ControlAction Check([[maybe_unused]] Configuration::ControlMode control_mode) override; ///< This must be called repeatedly.

 private:

  /// @brief Configuration settings.
  Configuration& configuration_;

  /// @brief The input.
  InputType& input_;
};

} // namespace mtmotor_jig
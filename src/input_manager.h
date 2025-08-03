// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_manager.h
/// @brief Class that handles user input (encoders, buttons, serial, etc.).

#pragma once

#include <Arduino.h>

#include "common_types.h"
#include "input_interface.h"

namespace mtmotor_jig {

/// @brief The Input Manager class.
class InputManager {
 public:
 
  /// @brief Construct an Input Manager object.
  /// @tparam Size The size of the input items.
  /// @param inputs The input items.
  template <size_t Size>
  InputManager::InputManager(InputInterface* (&inputs)[Size]) : inputs_size_(Size) { // Initialised here to avoid explicit instantiation.
    for (size_t i = 0; i < Size; i++) {
      inputs_[i] = inputs[i];
    }
  }

  // Comment out the above and uncomment this if you want to use explicit instantiation in the .cpp file.
  //template <size_t Size>
  //InputManager(InputInterface* (&inputs)[Size]);

  /// @brief Destroy the Input Manager object.
  ~InputManager();

  /// @brief Check for user input.
  /// @param control_mode The control mode.
  /// @return The control action.
  common::ControlAction CheckAndProcess(common::ControlMode control_mode) const; ///< This must be called repeatedly.

 private:

  /// @brief The inputs.
  InputInterface* inputs_[4];

  /// @brief The size of the inputs.
  size_t inputs_size_;
};

} // namespace mtmotor_jig
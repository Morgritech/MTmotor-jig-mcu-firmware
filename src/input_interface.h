// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_manager.h
/// @brief Class that defines the interface for generic inputs (buttons, serial, etc.).

#pragma once

#include <Arduino.h>

#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Input Interface class.
class InputInterface {
 public:
 
  /// @brief Destroy the Input Interface object.
  ~InputInterface() = default;

  /// @brief Initialise the inputs.
  virtual void Begin() = 0; ///< This must be called only once.

  /// @brief Check for user input based on the current control mode.
  /// @param control_mode The control mode.
  /// @return The control action.
  virtual Configuration::ControlAction Check(Configuration::ControlMode control_mode) = 0; ///< This must be called repeatedly.
};

} // namespace mtmotor_jig
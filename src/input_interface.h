// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_interface.h
/// @brief Class that defines the interface for generic inputs (buttons, serial, etc.).

#pragma once

#include <Arduino.h>

#include "input_types.h"

namespace mtmotor_jig {

/// @brief The Input Interface class.
class InputInterface {
 public:
 
  /// @brief Destroy the Input Interface object.
  ~InputInterface() = default;

  /// @brief Check for user input.
  /// @return The input event.
  virtual inputs::Event Check() = 0; ///< This must be called repeatedly.
};

} // namespace mtmotor_jig
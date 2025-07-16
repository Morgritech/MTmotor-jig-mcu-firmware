// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input.h
/// @brief Class to create configurable hardware inputs (encoders, buttons, etc.).

#pragma once

#include <Arduino.h>

#include "common_types.h"
#include "input_types.h"
#include "input_interface.h"

namespace mtmotor_jig {

/// @brief The Input class.
template <typename InputType>
class Input : public InputInterface {
 public:

  /// @brief Construct an Input object.
  Input(common::InputId id, InputType& input);

  /// @brief Destroy the Input object.
  ~Input();

  /// @brief Check for user input.
  /// @return The control action.
  inputs::Event Check() override; ///< This must be called repeatedly.

 private:

  /// @brief The input ID.
  common::InputId id_;

  /// @brief The input.
  InputType& input_;
};

} // namespace mtmotor_jig
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

#include "common_types.h"
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
  Input(common::InputId id, InputType& input);

  /// @brief Destroy the Input object.
  ~Input();

  /// @brief Check for user input based on the current control mode.
  /// @param control_mode The control mode.
  /// @return The control action.
  Inputs::Event Check([[maybe_unused]] common::ControlMode control_mode) override; ///< This must be called repeatedly.

 private:

  /// @brief The input ID.
  common::InputId id_;

  /// @brief The input.
  InputType& input_;
};

} // namespace mtmotor_jig
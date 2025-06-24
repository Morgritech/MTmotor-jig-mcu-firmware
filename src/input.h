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

template<typename InputType>
struct InputTraits {
  typeid(InputType) type_id_; ///< The type of the input.
};

/// @brief The Input class.
class Input : public InputInterface {
 public:
 
  /// @brief Construct an Input object.
  template <typename InputType>
  Input(InputType test);

  /// @brief Destroy the Input object.
  ~Input();

};

} // namespace mtmotor_jig
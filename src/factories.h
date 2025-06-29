// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file factories.h
/// @brief Factory methods.

#pragma once

#include <Arduino.h>

#include "common_types.h"
#include "input_interface.h"
#include "input.h"

namespace mtmotor_jig::factories {

/// @brief Create an InputInterface object.
/// @tparam InputType The type of input.
/// @param id The input ID.
/// @param input The input.
/// @return The InputInterface object.
template <typename InputType>
InputInterface* CreateInput(common::InputId id, InputType& input) {
  return new Input<InputType>(id, input);
}

// Comment out the above and uncomment this if you want to use explicit instantiation in the .cpp file.  
//template <typename InputType>
//InputInterface* CreateInput(common::InputId id, InputType& input);

} // namespace mtmotor_jig::factories
// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file factories.cpp
/// @brief Factory methods.

#include "factories.h"

#include <Arduino.h>
#include <momentary_button.h>
#include <rotary_encoder.h>

#include "common_types.h"
#include "input_interface.h"
#include "input.h"

namespace mtmotor_jig::factories {

template <typename InputType>
InputInterface* CreateInput(common::InputId id, InputType& input) {
  return new Input<InputType>(id, input);
}

// Explicit instantiations.
template InputInterface* CreateInput<mt::MomentaryButton>(common::InputId, mt::MomentaryButton&);
template InputInterface* CreateInput<mt::RotaryEncoder>(common::InputId, mt::RotaryEncoder&);

} // namespace mtmotor_jig::factories
// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input.cpp
/// @brief Class for creating configurable inputs (buttons, serial, etc.).

#pragma once

#include "input.h"

#include <Arduino.h>
#include <ArduinoLog.h>
#include <momentary_button.h>
#include <rotary_encoder.h>

namespace mtmotor_jig {

template <typename InputType>
Input<InputType>::Input(Configuration& configuration, InputType& input)
    : configuration_(configuration), input_(input) {}

template <typename InputType>
Input<InputType>::~Input() {}

template <typename InputType>
void Input<InputType>::Begin() {
  // Implementation here
}

template <typename InputType>
Configuration::ControlAction Input<InputType>::Check(Configuration::ControlMode control_mode) {
  // Implementation here
  return Configuration::ControlAction::kIdle; // Example return
}

// Explicit instantiations
template class Input<mt::MomentaryButton>;
template class Input<mt::RotaryEncoder>;

} // namespace mtmotor_jig
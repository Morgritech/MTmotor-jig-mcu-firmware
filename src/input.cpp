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

#include "common_types.h"
#include "input_types.h"

namespace mtmotor_jig {

template <typename InputType>
Input<InputType>::Input(common::InputId id, InputType& input) : input_(input) {}

template <typename InputType>
Input<InputType>::~Input() {}

template <typename InputType>
inputs::Event Input<InputType>::Check() {
  // Implementation here
  return {id_, inputs::EventType::kIdle}; // Example return
}

// Explicit instantiations
template class Input<mt::MomentaryButton>;
template class Input<mt::RotaryEncoder>;

} // namespace mtmotor_jig
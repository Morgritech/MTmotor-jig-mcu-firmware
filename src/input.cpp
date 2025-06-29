// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input.cpp
/// @brief Class to create configurable inputs (encoders, buttons, serial, etc.).

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
Input<InputType>::Input(common::InputId id, InputType& input) : id_(id), input_(input) {}

template <typename InputType>
Input<InputType>::~Input() {}

template <>
inputs::Event Input<mt::RotaryEncoder>::Check() {
  using RotationDirection = mt::RotaryEncoder::RotationDirection;

  auto event = inputs::EventType::kIdle;

  if (RotationDirection rotation_direction = input_.DetectRotation();
      rotation_direction == RotationDirection::kPositive) {
    event = inputs::EventType::kPositiveRotation;
  }
  else if (rotation_direction == RotationDirection::kNegative) {
    event = inputs::EventType::kNegativeRotation;
  }

  return {id_, event};
}

template <>
inputs::Event Input<mt::MomentaryButton>::Check() {
  using PressType = mt::MomentaryButton::PressType;

  auto event = inputs::EventType::kIdle;

  if (PressType press_type = input_.DetectPressType();
      press_type == PressType::kShortPress) {
    event = inputs::EventType::kShortPress;
  }
  else if (press_type == PressType::kLongPress) {
    event = inputs::EventType::kLongPress;
  }

  return {id_, event};
}

// Explicit instantiations
template class Input<mt::MomentaryButton>;
template class Input<mt::RotaryEncoder>;

} // namespace mtmotor_jig
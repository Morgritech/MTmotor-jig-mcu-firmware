// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_types.h
/// @brief Common input types.

#pragma once

#include <Arduino.h>

#include "common_types.h"

namespace mtmotor_jig::inputs {

/// @brief Enum of event types.
enum class EventType : int8_t {
  kNegativeRotation = -1,
  kIdle = 0,
  kPositiveRotation = 1,    
  kShortPress,
  kLongPress,
};

/// @brief Struct of event data.
struct Event {
  common::InputId input_id;
  EventType event_type;
};

} // namespace mtmotor_jig::inputs
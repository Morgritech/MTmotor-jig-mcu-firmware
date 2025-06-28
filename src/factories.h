// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file factories.h
/// @brief Factory methods.

#pragma once

#include <Arduino.h>

#include "common_types.h"
#include "input_types.h"
#include "input_interface.h"
#include "input.h"

namespace mtmotor_jig::factories {

template <typename InputType>
InputInterface* CreateInput(common::InputId id, InputType& input);

} // namespace mtmotor_jig::factories
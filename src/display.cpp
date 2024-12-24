// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file display.cpp
/// @brief Class that handles visual output to the display.

#include "display.h"

#include <Arduino.h>
#include <LiquidCrystal.h>

namespace mtmotor_jig {

Display::Display() {}

Display::~Display() {}

void Display::Begin() const {}

void Display::Clear() const {}

void Display::DisplayFirmwareVersion() const {}

void Display::DisplayGeneralStatus() const {}

} // namespace mtmotor_jig

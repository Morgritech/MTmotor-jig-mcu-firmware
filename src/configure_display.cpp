// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file configure_display.cpp
/// @brief Class that sets up display configuration settings, including pin definitions, etc.

#include "configure_display.h"

namespace mtmotor_jig {

ConfigureDisplay& ConfigureDisplay::GetInstance() {
  static ConfigureDisplay instance;
  return instance;
}

ConfigureDisplay::ConfigureDisplay() {}

ConfigureDisplay::~ConfigureDisplay() {}

}  // namespace mtmotor_jig
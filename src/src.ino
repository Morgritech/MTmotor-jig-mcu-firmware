// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file src.ino
/// @brief Main sketch containing the setup() and loop() functions.

#include <ArduinoLog.h>

#include "control_system.h"

/// @brief The Control System instance.
mtmotor_jig::ControlSystem control_system{};

/// @brief The main application entry point for initialisation tasks.
void setup() {
  // Setup the control system.
  control_system.Begin();
  
  Log.noticeln(F("...Setup complete...\n"));
}

/// @brief The continuously running function for repetitive tasks.
void loop() {
  // Run the control system.
  control_system.CheckAndProcess();
}
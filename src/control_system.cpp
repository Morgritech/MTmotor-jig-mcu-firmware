// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file control_system.cpp
/// @brief Class that links sensor inputs (buttons, serial, etc.) to actuator outputs (display, steppers, etc.).

#include "control_system.h"

#include <Arduino.h>
#include <ArduinoLog.h>

#include "configuration.h"
#include "input_manager.h"
#include "motor_manager.h"
#include "display_manager.h"

namespace mtmotor_jig {

ControlSystem::ControlSystem() {}

ControlSystem::~ControlSystem() {}

void ControlSystem::Begin() {
  configuration_.BeginHardware();
  inputs_.Begin();
  motor_.Begin();
  display_.Begin();
  LogGeneralStatus(); // Log initial status of control system.
}

void ControlSystem::CheckAndProcess() {

  // Check for user input.
  control_action_ = inputs_.Check(control_mode_);

  // Process control actions.
  // TODO: Implement the control actions.

  display_.Draw(control_mode_);
  while (true);
}

void ControlSystem::LogGeneralStatus() const {}

} // namespace mtmotor_jig
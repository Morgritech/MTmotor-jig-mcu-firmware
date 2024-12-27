// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file control_system.cpp
/// @brief Class that links sensor inputs (buttons, serial, etc.) to actuator outputs (display, steppers, etc.).

#include "control_system.h"

#include <Arduino.h>
#include <ArduinoLog.h>
#include <momentary_button.h>
#include <stepper_driver.h>

#include "configuration.h"
#include "display_manager.h"

namespace mtmotor_jig {

ControlSystem::ControlSystem() {}

ControlSystem::~ControlSystem() {}

void ControlSystem::Begin() {
  configuration_.BeginHardware();
  display_.Begin();
}

void ControlSystem::CheckAndProcess() {
  display_.Draw(configuration_.kDefaultControlMode_);
  while (true);
}

void ControlSystem::LogGeneralStatus() const {}

} // namespace mtmotor_jig
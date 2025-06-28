// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file input_manager.cpp
/// @brief Class that handles user input (encoders, buttons, serial, etc.).

#include "input_manager.h"

#include <Arduino.h>
#include <ArduinoLog.h>

#include "common_types.h"
#include "configuration.h"
#include "input_types.h"
#include "input_interface.h"

namespace mtmotor_jig {

// Uncomment this if you want to use explicit instantiation.
//template <size_t Size>
//InputManager::InputManager(InputInterface* (&inputs)[Size]) : inputs_(inputs), inputs_size_(Size) {}

InputManager::~InputManager() {}

common::ControlAction InputManager::CheckAndProcess(common::ControlMode control_mode) {
  inputs::Event input_event;
  
  // Check for input events from all hardware inputs (encoder dial, buttons).
  for (auto i = 0; i < inputs_size_; i++) {
    input_event = inputs_[i]->Check();
    if (input_event.event_type != inputs::EventType::kIdle) {
      Log.noticeln(F("Input detected: ID=%d, Type=%d"), static_cast<int>(input_event.input_id),
                   static_cast<int>(input_event.event_type));
      break;
    }
  }

  common::ControlAction control_action = common::ControlAction::kIdle;

  // Process all hardware inputs and serial input (one character at a time).
  switch (input_event.event_type) {
    case inputs::EventType::kPositiveRotation:
      control_action = common::ControlAction::kSelectNext;
      break;

    case inputs::EventType::kNegativeRotation:
      control_action = common::ControlAction::kSelectPrevious;
      break;

    case inputs::EventType::kShortPress:
      if (input_event.input_id == common::InputId::kControllerButton) {
        control_action = common::ControlAction::kCycleSpeed;
      }
      else if (input_event.input_id == common::InputId::kEncoderButton) {
        
        switch (control_mode) {
          case common::ControlMode::kContinuousMenu:
            control_action = common::ControlAction::kToggleDirection;
            break;
          case common::ControlMode::kOscillateMenu:
            control_action = common::ControlAction::kCycleAngle;
            break;
        }
      }
      else if (input_event.input_id == common::InputId::kLimitSwitch) {
        control_action = common::ControlAction::kResetHome;
      }

      break;

    case inputs::EventType::kLongPress:
      if (input_event.input_id == common::InputId::kControllerButton
          || input_event.input_id == common::InputId::kEncoderButton) {
        control_action = common::ControlAction::kToggleMotion;
      }
      else if (input_event.input_id == common::InputId::kLimitSwitch) {
        control_action = common::ControlAction::kGoHome;
      }

      break;

    case inputs::EventType::kIdle:
      // Check for serial input if there are no hardware inputs.
      if (MTMOTOR_JIG_SERIAL.available() > 0) {
        char serial_input = MTMOTOR_JIG_SERIAL.read();
        control_action = static_cast<common::ControlAction>(serial_input);
        Log.noticeln(F("Serial input: %c"), serial_input);
      }
      else {
        control_action = common::ControlAction::kIdle;
      }
      
      break;
  }
  
  return control_action;
}

// Explicit instantiation. Uncomment if you want to use it. Change the size as needed.
//template InputManager::InputManager<4>(InputInterface* (&)[4]);

} // namespace mtmotor_jig
// Copyright (C) 2025 Morgritech
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
  tone(configuration_.kControllerBuzzerPin_, configuration_.kBuzzerStartupFrequency_Hz_,
       configuration_.kBuzzerStartupDuration_ms_); // Sound the buzzer at startup.
  inputs_.Begin();
  motor_.Begin();
  display_.Begin();
  LogGeneralStatus(); // Log initial status of control system.
}

void ControlSystem::CheckAndProcess() {

  // Check inputs.
  if (previous_control_mode_ == Configuration::ControlMode::kHoming) {
    control_action_ = Configuration::ControlAction::kResetHome;
    previous_control_mode_ = control_mode_;
  }
  else {
    control_action_ = inputs_.Check(control_mode_);
  }

  // Process inputs.
  switch (control_action_) {
    case Configuration::ControlAction::kSelectNext: {
      // Fall through to select previous which changes control mode.
      [[fallthrough]];
    }
    case Configuration::ControlAction::kSelectPrevious: {
      if (control_mode_ == Configuration::ControlMode::kContinuousMenu) {
        control_mode_ = Configuration::ControlMode::kOscillateMenu;
        Log.noticeln(F("Control mode: oscillate"));
      }
      else if (control_mode_ == Configuration::ControlMode::kOscillateMenu) {
        control_mode_ = Configuration::ControlMode::kContinuousMenu;
        Log.noticeln(F("Control mode: continuous"));
      }

      break;
    }
    case Configuration::ControlAction::kGoHome: {
      previous_control_mode_ = control_mode_;
      control_mode_ = Configuration::ControlMode::kHoming;
      Log.noticeln(F("Control mode: homing"));
      break;
    }
    case Configuration::ControlAction::kToggleLogReport: {
      // Toggle reporting/output of log messages over serial.
      configuration_.ToggleLogs();
      break;
    }
    case Configuration::ControlAction::kLogGeneralStatus: {
      // Log/report the general status of the control system.
      LogGeneralStatus();
      break;
    }
    case Configuration::ControlAction::kReportFirmwareVersion: {
      // Log/report the firmware version.
      configuration_.ReportFirmwareVersion();
      break;
    }
  }

  // Initiate outputs.
  motor_.Actuate(control_mode_, control_action_, status_);
  display_.Draw(control_mode_, control_action_, status_);

  // Transition through the initial control modes, and process any further control mode changes.
  switch (control_mode_) {
    case Configuration::ControlMode::kSplashScreen: {
      control_mode_ = Configuration::ControlMode::kHomeScreen;
      Log.noticeln(F("Control mode: home screen"));
      break;
    }
    case Configuration::ControlMode::kHomeScreen: {
      control_mode_ = Configuration::ControlMode::kContinuousMenu;
      Log.noticeln(F("Control mode: continuous"));
      break;
    }
    case Configuration::ControlMode::kHoming: {
      if (motor_.homing() == false) {
        control_mode_ = previous_control_mode_;
        previous_control_mode_ = Configuration::ControlMode::kHoming;
        Log.noticeln(F("Control mode: returned to previous mode"));
      }

      break;
    }
  }
}

void ControlSystem::LogGeneralStatus() const {
  Log.noticeln(F("General Status"));
  switch (control_mode_) {
    case Configuration::ControlMode::kSplashScreen: {
      Log.noticeln(F("Control mode: splash screen"));
      break;
    }
    case Configuration::ControlMode::kHomeScreen: {
      Log.noticeln(F("Control mode: home screen"));
      break;
    }
    case Configuration::ControlMode::kContinuousMenu: {
      Log.noticeln(F("Control mode: continuous"));
      break;
    }
    case Configuration::ControlMode::kOscillateMenu: {
      Log.noticeln(F("Control mode: oscillate"));
      break;
    }
    case Configuration::ControlMode::kHoming: {
      Log.noticeln(F("Control mode: homing"));
      break;
    }
  }

  motor_.LogGeneralStatus(control_mode_);
}

} // namespace mtmotor_jig
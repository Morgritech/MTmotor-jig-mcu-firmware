// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file control_system.cpp
/// @brief Class that links sensor inputs (encoders, buttons, serial, etc.) to actuator outputs (display, motors, etc.).

#include "control_system.h"

#include <Arduino.h>
#include <ArduinoLog.h>
#include <stepper_driver.h>

#include "common_types.h"


namespace mtmotor_jig {

ControlSystem::ControlSystem() {}

ControlSystem::~ControlSystem() {
  // Clean up dynamically allocated InputInterface objects.
  for (auto& input : inputs_) {
    delete input; // Deallocate/free memory.
    input = nullptr; // Prevent dangling pointers.
  }
}

void ControlSystem::Begin() {
  // Hardware.
  configuration_.BeginHardware();
  // Inputs.
  encoder_button_.set_long_press_option(configuration_.kLongPressOption_);
  controller_button_.set_long_press_option(configuration_.kLongPressOption_);
  limit_switch_.set_long_press_option(configuration_.kLongPressOption_);
  // Outputs - Stepper motor.
  stepper_driver_.set_pul_delay_us(configuration_.pul_delay_us_);
  stepper_driver_.set_dir_delay_us(configuration_.dir_delay_us_);
  stepper_driver_.set_ena_delay_us(configuration_.ena_delay_us_);
  stepper_driver_.SetSpeed(configuration_.kSpeeds_RPM_[configuration_.kDefaultSpeedIndex_],
                           mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
  stepper_driver_.SetAcceleration(configuration_.acceleration_microsteps_per_s_per_s_,
                                  mt::StepperDriver::AccelerationUnits::kMicrostepsPerSecondPerSecond);
  stepper_driver_.set_acceleration_algorithm(configuration_.kAccelerationAlgorithm_);
  stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kEnabled);
  // Outputs - Display
  dot_matrix_display_.begin(configuration_.kDisplayWidth_, configuration_.kDisplayHeight_);
  dot_matrix_display_.blink(); // Blink the display cursor.
  //dot_matrix_display_.cursor(); // Show a static display cursor.  
  // Outputs - Buzzer.
  if (configuration_.buzzer_enabled_) {
    tone(configuration_.kControllerBuzzerPin_, configuration_.buzzer_startup_frequency_Hz_,
        configuration_.buzzer_startup_duration_ms_); // Sound the buzzer at startup.
  }
  // Status.
  LogGeneralStatus(); // Log initial status of control system.
}

void ControlSystem::CheckAndProcess() {

  // Check inputs.
  if (previous_control_mode_ == common::ControlMode::kHoming) {
    control_action_ = common::ControlAction::kResetHome;
    previous_control_mode_ = control_mode_;
  }
  else {
    control_action_ = input_manager_.CheckAndProcess(control_mode_);
  }

  // Process inputs.
  switch (control_action_) {
    case common::ControlAction::kSelectNext: {
      // Fall through to select previous which changes control mode.
      [[fallthrough]];
    }
    case common::ControlAction::kSelectPrevious: {
      if (control_mode_ == common::ControlMode::kContinuousMenu) {
        control_mode_ = common::ControlMode::kOscillateMenu;
        Log.noticeln(F("Control mode: oscillate"));
      }
      else if (control_mode_ == common::ControlMode::kOscillateMenu) {
        control_mode_ = common::ControlMode::kContinuousMenu;
        Log.noticeln(F("Control mode: continuous"));
      }

      break;
    }
    case common::ControlAction::kGoHome: {
      previous_control_mode_ = control_mode_;
      control_mode_ = common::ControlMode::kHoming;
      Log.noticeln(F("Control mode: homing"));
      break;
    }
    case common::ControlAction::kToggleLogReport: {
      // Toggle reporting/output of log messages over serial.
      configuration_.ToggleLogs();
      break;
    }
    case common::ControlAction::kLogGeneralStatus: {
      // Log/report the general status of the control system.
      LogGeneralStatus();
      break;
    }
    case common::ControlAction::kReportFirmwareVersion: {
      // Log/report the firmware version.
      configuration_.ReportFirmwareVersion();
      break;
    }
  }

  // Initiate outputs.
  stepper_motor_.Actuate(control_mode_, control_action_, status_);
  display_.Draw(control_mode_, control_action_, status_);

  // Transition through the initial control modes, and process any further control mode changes.
  switch (control_mode_) {
    case common::ControlMode::kSplashScreen: {
      control_mode_ = common::ControlMode::kHomeScreen;
      Log.noticeln(F("Control mode: home screen"));
      break;
    }
    case common::ControlMode::kHomeScreen: {
      control_mode_ = common::ControlMode::kContinuousMenu;
      Log.noticeln(F("Control mode: continuous"));
      break;
    }
    case common::ControlMode::kHoming: {
      if (stepper_motor_.homing() == false) {
        control_mode_ = previous_control_mode_;
        previous_control_mode_ = common::ControlMode::kHoming;
        Log.noticeln(F("Control mode: returned to previous mode"));
      }

      break;
    }
  }
}

void ControlSystem::LogGeneralStatus() const {
  Log.noticeln(F("General Status"));
  switch (control_mode_) {
    case common::ControlMode::kSplashScreen: {
      Log.noticeln(F("Control mode: splash screen"));
      break;
    }
    case common::ControlMode::kHomeScreen: {
      Log.noticeln(F("Control mode: home screen"));
      break;
    }
    case common::ControlMode::kContinuousMenu: {
      Log.noticeln(F("Control mode: continuous"));
      break;
    }
    case common::ControlMode::kOscillateMenu: {
      Log.noticeln(F("Control mode: oscillate"));
      break;
    }
    case common::ControlMode::kHoming: {
      Log.noticeln(F("Control mode: homing"));
      break;
    }
  }

  stepper_motor_.LogGeneralStatus(control_mode_);
}

} // namespace mtmotor_jig
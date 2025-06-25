// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file control_system.h
/// @brief Class that links sensor inputs (buttons, serial, etc.) to actuator outputs (display, steppers, etc.).

#pragma once

#include <Arduino.h>
#include <momentary_button.h>
#include <rotary_encoder.h>
#include <stepper_driver.h>
#include <LiquidCrystal.h>

#include "configuration.h"
#include "input.h"
#include "input_manager.h"
#include "motor_manager.h"
#include "display_manager.h"

namespace mtmotor_jig {

/// @brief The Control System class.
class ControlSystem {
 public:

  /// @brief Construct a Control System object. 
  ControlSystem();

  /// @brief Destroy the Control System object.
  ~ControlSystem();

  /// @brief Initialise the hardware (Serial port, logging, pins, etc.).
  void Begin(); ///< This must be called only once.

  /// @brief Check inputs and trigger outputs/actions.
  void CheckAndProcess(); ///< This must be called repeatedly.

 private:

  /// @brief Log/report the general status of the control system.
  void LogGeneralStatus() const;

  /// @brief Configuration settings.
  Configuration& configuration_ = Configuration::GetInstance();

  // Buttons to control the motor.
  mt::RotaryEncoder encoder_dial_{configuration_.kEncoderContactAPin_,
                                  configuration_.kEncoderContactBPin_,
                                  configuration_.kEncoderDetents_,
                                  configuration_.kEncoderMaxRotationAngle_degrees_}; ///< Encoder dial to control mode selection.
  mt::MomentaryButton encoder_button_{configuration_.kEncoderButtonPin_,
                                       configuration_.kUnpressedPinState_,
                                       configuration_.kDebouncePeriod_ms_,
                                       configuration_.kShortPressPeriod_ms_,
                                       configuration_.kLongPressPeriod_ms_}; ///< Button to control motor direction or angle.
  mt::MomentaryButton controller_button_{configuration_.kControllerButtonPin_,
                                       configuration_.kUnpressedPinState_,
                                       configuration_.kDebouncePeriod_ms_,
                                       configuration_.kShortPressPeriod_ms_,
                                       configuration_.kLongPressPeriod_ms_}; ///< Button to control motor speed.
  mt::MomentaryButton limit_switch_{configuration_.kLimitSwitchPin_,
                                    configuration_.kUnpressedPinState_,
                                    configuration_.kDebouncePeriod_ms_,
                                    configuration_.kShortPressPeriod_ms_,
                                    configuration_.kLongPressPeriod_ms_}; ///< Limit switch to manipulate the motor with respect to a soft home position.  

  // Stepper motor driver.
  mt::StepperDriver stepper_driver_{configuration_.kMotorDriverPulPin_,
                                    configuration_.kMotorDriverDirPin_,
                                    configuration_.kMotorDriverEnaPin_,
                                    configuration_.kMicrostepMode_,
                                    configuration_.kFullStepAngle_degrees_,
                                    configuration_.kGearRatio_};

  /// @brief The LCD display.
  LiquidCrystal lcd_{configuration_.kLcdRsPin_, configuration_.kLcdEnaPin_, configuration_.kLcdD4Pin_,
                     configuration_.kLcdD5Pin_, configuration_.kLcdD6Pin_, configuration_.kLcdD7Pin_};

  Input<mt::RotaryEncoder> input1_{Configuration::InputId::kEncoderDial, encoder_dial_};


  // Sensors and actuators / inputs and outputs.
  InputManager input_manager{input1_}; ///< The User inputs (encoder, buttons, serial, etc.).
  MotorManager motor_{}; ///< The Motor drive system.
  DisplayManager display_{}; ///< The display (LCD).

  // Control flags and indicator variables.
  Configuration::ControlMode control_mode_ = configuration_.kDefaultControlMode_; ///< Variable to keep track of the control system mode.
  Configuration::ControlMode previous_control_mode_ = configuration_.kDefaultControlMode_; ///< Variable to keep track of the previously set control system mode.
  Configuration::ControlAction control_action_ = Configuration::ControlAction::kIdle; ///< Variable to keep track of the control actions from user inputs.
  String status_ = ""; ///< Variable to keep track of the status message to display.
};

} // namespace mtmotor_jig
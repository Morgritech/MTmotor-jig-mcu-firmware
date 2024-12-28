// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file motor_manager.cpp
/// @brief Class that handles motor control.

#include "motor_manager.h"

#include <Arduino.h>
#include <stepper_driver.h>

#include "configuration.h"

namespace mtmotor_jig {

MotorManager::MotorManager() {}

MotorManager::~MotorManager() {}

void MotorManager::Begin() {
  stepper_driver_.set_pul_delay_us(configuration_.kPulDelay_us_);
  stepper_driver_.set_dir_delay_us(configuration_.kDirDelay_us_);
  stepper_driver_.set_ena_delay_us(configuration_.kEnaDelay_us_);
  stepper_driver_.SetSpeed(configuration_.kSpeeds_RPM_[configuration_.kDefaultSpeedIndex_],
                           mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
  stepper_driver_.SetAcceleration(configuration_.kAcceleration_microsteps_per_s_per_s_,
                                  mt::StepperDriver::AccelerationUnits::kMicrostepsPerSecondPerSecond);
  stepper_driver_.set_acceleration_algorithm(configuration_.kAccelerationAlgorithm_);
  stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kDisabled); // Save power when idle.
}

void MotorManager::Actuate(Configuration::ControlMode control_mode, Configuration::ControlAction control_action) {
  // TODO: Implement the motor actuation.
}

}  // namespace mtmotor_jig
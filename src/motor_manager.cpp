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

void MotorManager::Actuate(Configuration::ControlMode control_mode, Configuration::ControlAction control_action,
                           String& status_output) {
  // Process control actions.
  switch(control_action) {
    case Configuration::ControlAction::kSelectNext: {
      // Fall through to select previous which resets the motor.
      [[fallthrough]];
    }
    case Configuration::ControlAction::kSelectPrevious: {
      motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
      break;
    }
    case Configuration::ControlAction::kToggleDirection: {
      // Start motor, or, change motor direction.
      if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kDisabled) {
        // Fall through to start motor.
        [[fallthrough]];
      }
      else {
        // change motor direction.
        if (motion_direction_ == mt::StepperDriver::MotionDirection::kPositive) {
          motion_direction_ = mt::StepperDriver::MotionDirection::kNegative;
          Log.noticeln(F("Motion direction: counter-clockwise (CCW)"));
        }
        else {
          motion_direction_ = mt::StepperDriver::MotionDirection::kPositive;
          Log.noticeln(F("Motion direction: clockwise (CW)"));
        }

        motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
        break;
      }
    }
    case Configuration::ControlAction::kCycleAngle: {
      // Start motor, or, cycle through sweep angles.
      if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kDisabled) {
        // Fall through to start motor.
        [[fallthrough]];
      }
      else {
        // Change sweep angles.
        if (sweep_angle_index_ == (configuration_.kSizeOfSweepAngles_ - 1)) {
          sweep_angle_index_ = 0;
        }
        else {
          sweep_angle_index_++;
        }

        Log.noticeln(F("Sweep angle (degrees): %F"), configuration_.kSweepAngles_degrees_[sweep_angle_index_]);
        motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
        break;
      }
    }
    case Configuration::ControlAction::kCycleSpeed: {
      //  Start motor, or, cycle through motor speed settings.
      if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kDisabled) {
        // Fall through to start motor.          
        [[fallthrough]];
      }
      else {
        // Change speed.
        if (speed_index_ == (configuration_.kSizeOfSpeeds_ - 1)) {
          speed_index_ = 0;
        }
        else {
          speed_index_++;
        }
        
        stepper_driver_.SetSpeed(configuration_.kSpeeds_RPM_[speed_index_],
                                 mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
        Log.noticeln(F("Speed (RPM): %F"), configuration_.kSpeeds_RPM_[speed_index_]);
        break;
      }
    }
    case Configuration::ControlAction::kToggleMotion: {
      // Toggle (start/stop) the motor.
      if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kDisabled) {
        // Allow movement.
        stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kEnabled); // Restore power to allow motion.
        // Not really needed since enabling power will achieve the same states in the move functions.
        //motion_type_ = mt::StepperDriver::MotionType::kRelative;
        //motion_direction_ = previous_motion_direction_;
        Log.noticeln(F("Motion status: started"));     
      }
      else {
        // Disallow movement.
        stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kDisabled); // Save power when idle.
        // Not really needed since disabling power will achieve the same states in the move functions.
        //motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
        //previous_motion_direction_ = motion_direction_;
        //motion_direction_ = mt::StepperDriver::MotionDirection::kNeutral;
        speed_index_ = configuration_.kDefaultSpeedIndex_;
        stepper_driver_.SetSpeed(configuration_.kSpeeds_RPM_[speed_index_],
                            mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
        Log.noticeln(F("Motion status: stopped"));       
      }
      
      break;
    }
    case Configuration::ControlAction::kResetHome: {
      // Stop motor.
      if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kEnabled) {
        stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kDisabled);
        Log.noticeln(F("Motion status: stopped"));       
      }

      // Reset the soft home position to the current position.
      stepper_driver_.ResetAngularPosition();
      break;
    }
    case Configuration::ControlAction::kGoHome: {
      // Start motor.
      if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kDisabled) {
        stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kEnabled);
      }

      homing_ = true;
      motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
      float angular_position_degrees_ = stepper_driver_.GetAngularPosition(mt::StepperDriver::AngleUnits::kDegrees);
      // Constrain the angular position to the range [0, 360).
      homing_angle_degrees_ = 0.0 - fmod(angular_position_degrees_, 360.0);
      Log.noticeln(F("Motion status: homing initiated"));
      Log.noticeln(F("Current angular position (degrees): %F"), angular_position_degrees_);
      Log.noticeln(F("Homing angular position (degrees): %F"), homing_angle_degrees_);
      break;
    }
  }

  // Actuate the motor based on the control mode.
  switch (control_mode) {
    case Configuration::ControlMode::kContinuousMenu: {      
      if (motion_status_ != mt::StepperDriver::MotionStatus::kConstantSpeed 
          || motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
        // Accelerate to constant speed.
        motion_status_ = stepper_driver_.MoveByAngle(static_cast<float>(motion_direction_) * 360.0,
                                                     mt::StepperDriver::AngleUnits::kDegrees, motion_type_);
        if (motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
          // Stop and reset issued by user changing direction, restart motion.
          motion_type_ = mt::StepperDriver::MotionType::kRelative;
        }
      }
      else {
        // Continue constant speed motion indefinitely.
        stepper_driver_.MoveByJogging(motion_direction_);
      }

      break;
    }
    case Configuration::ControlMode::kOscillateMenu: {
      mt::StepperDriver::MotionStatus motion_status = stepper_driver_.MoveByAngle(sweep_direction_ 
                                                      * configuration_.kSweepAngles_degrees_[sweep_angle_index_],
                                                      mt::StepperDriver::AngleUnits::kDegrees, motion_type_);
      if (motion_status == mt::StepperDriver::MotionStatus::kIdle) {
        // Motion completed OR stop and reset issued.
        if (motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
          // Stop and reset issued by user changing sweep angle, restart motion.
          motion_type_ = mt::StepperDriver::MotionType::kRelative;
        }
        else if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kEnabled) {
          // Change sweep direction.
          if (motion_direction_ == mt::StepperDriver::MotionDirection::kPositive) {
            motion_direction_ = mt::StepperDriver::MotionDirection::kNegative; 
          }
          else {
            motion_direction_ = mt::StepperDriver::MotionDirection::kPositive;
          }

          sweep_direction_ = static_cast<float>(motion_direction_);
        }
      }

      break;
    }
    case Configuration::ControlMode::kHoming: {
      mt::StepperDriver::MotionStatus motion_status = stepper_driver_.MoveByAngle(homing_angle_degrees_,
                                                                              mt::StepperDriver::AngleUnits::kDegrees,
                                                                              motion_type_);
      if (motion_status == mt::StepperDriver::MotionStatus::kIdle) {
        if (control_action == Configuration::ControlAction::kGoHome
            && motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
          // First iteration.
          motion_type_ = mt::StepperDriver::MotionType::kRelative;
          Log.noticeln(F("Motion status: homing started"));
        }
        else {
          // Homing completed OR stop and reset issued.
          motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
          homing_ = false;
          stepper_driver_.set_power_state(mt::StepperDriver::PowerState::kDisabled);
          Log.noticeln(F("Motion status: homing finished or stopped"));
        }
      }

      break;
    }
  }

  // Create the status message.
  if (control_action != Configuration::ControlAction::kIdle || control_mode == Configuration::ControlMode::kHomeScreen) {
    Log.noticeln(F("Creating display status"));

    status_output = F("..");
    
    if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kEnabled) {
      status_output += F("ON..");
    }
    else {
      status_output += F("OFF.");
    }

    status_output += String(configuration_.kSpeeds_RPM_[speed_index_], 0);
    status_output += F("RPM.");

    switch (control_mode) {
      case Configuration::ControlMode::kHomeScreen: {
        // Fall through to continuous menu.
        [[fallthrough]];
      }
      case Configuration::ControlMode::kContinuousMenu: {
        if (motion_direction_ == mt::StepperDriver::MotionDirection::kPositive) {
          status_output += F("CW......");
        }
        else {
          status_output += F("CCW.....");
        }

        break;
      }
      case Configuration::ControlMode::kOscillateMenu: {
        status_output += String(configuration_.kSweepAngles_degrees_[sweep_angle_index_], 0);
        status_output += F("deg..");
        break;
      }
      case Configuration::ControlMode::kHoming: {
        status_output += F("Homing..");
        break;
      }
    }
  }
}

void MotorManager::LogGeneralStatus(Configuration::ControlMode control_mode) const {
  Log.noticeln(F("General Motor Status"));
  if (motion_direction_ == mt::StepperDriver::MotionDirection::kPositive) {
    Log.noticeln(F("Motion direction: clockwise (CW)"));
  }
  else {
    Log.noticeln(F("Motion direction: counter-clockwise (CCW)"));
  }
  
  Log.noticeln(F("Sweep angle (degrees): %F"), configuration_.kSweepAngles_degrees_[sweep_angle_index_]);
  Log.noticeln(F("Speed (RPM): %F"), configuration_.kSpeeds_RPM_[speed_index_]);

  if (control_mode == Configuration::ControlMode::kHoming) {
    Log.noticeln(F("Motion status: homing"));
  }
  else if (stepper_driver_.power_state() == mt::StepperDriver::PowerState::kDisabled) {
    Log.noticeln(F("Motion status: stopped"));
  }
  else {
    Log.noticeln(F("Motion status: started"));
  }
}

bool MotorManager::homing() const {
  return homing_;
}

}  // namespace mtmotor_jig
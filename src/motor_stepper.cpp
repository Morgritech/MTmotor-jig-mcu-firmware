// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file motor_stepper.cpp
/// @brief Class that handles stepper motor control.

#include "motor_stepper.h"

#include <Arduino.h>
#include <stepper_driver.h>

#include "common_types.h"
#include "configuration.h"

namespace mtmotor_jig {

MotorStepper::MotorStepper(mt::StepperDriver& stepper_driver, Configuration& configuration)
    : stepper_driver_(stepper_driver), configuration_(configuration) {}

MotorStepper::~MotorStepper() {}

void MotorStepper::Actuate(common::ControlMode control_mode, common::ControlAction control_action,
                           String& status_output) {
  // Process control actions.
  switch(control_action) {
    case common::ControlAction::kSelectNext: {
      // Fall through to select previous which resets the motor.
      [[fallthrough]];
    }
    case common::ControlAction::kSelectPrevious: {
      motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
      break;
    }
    case common::ControlAction::kToggleDirection: {
      // Start motor, or, change motor direction.
      if (allow_motion_ == false) {
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
    case common::ControlAction::kCycleAngle: {
      // Start motor, or, cycle through sweep angles.
      if (allow_motion_ == false) {
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

        Log.noticeln(F("Sweep angle (degrees): %F"), configuration_.sweep_angles_degrees_[sweep_angle_index_]);
        motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
        break;
      }
    }
    case common::ControlAction::kCycleSpeed: {
      //  Start motor, or, cycle through motor speed settings.
      if (allow_motion_ == false) {
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
        
        stepper_driver_.SetSpeed(configuration_.speeds_RPM_[speed_index_],
                                 mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
        Log.noticeln(F("Speed (RPM): %F"), configuration_.speeds_RPM_[speed_index_]);
        break;
      }
    }
    case common::ControlAction::kToggleMotion: {
      // Toggle (start/stop) the motor.
      if (allow_motion_ == false) {
        // Allow movement.
        allow_motion_ = true;
        Log.noticeln(F("Motion status: started"));     
      }
      else {
        // Disallow movement.
        allow_motion_ = false;
        motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
        speed_index_ = configuration_.kDefaultSpeedIndex_;
        stepper_driver_.SetSpeed(configuration_.speeds_RPM_[speed_index_],
                            mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
        Log.noticeln(F("Motion status: stopped"));       
      }
      
      break;
    }
    case common::ControlAction::kResetHome: {
      // Stop motor.
      if (allow_motion_ == true) {
        allow_motion_ = false;
        Log.noticeln(F("Motion status: stopped"));       
      }

      // Reset the soft home position to the current position.
      stepper_driver_.ResetAngularPosition();
      break;
    }
    case common::ControlAction::kGoHome: {
      // Start motor.
      if (allow_motion_ == false) {
        allow_motion_ = true;
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

  // Create the status message.
  if (control_action != common::ControlAction::kIdle || control_mode == common::ControlMode::kHomeScreen) {
    Log.noticeln(F("Creating display status"));

    status_output = F("..");
    
    if (allow_motion_ == true) {
      status_output += F("ON..");
    }
    else {
      status_output += F("OFF.");
    }

    status_output += String(configuration_.speeds_RPM_[speed_index_], 0);
    status_output += F("RPM.");

    switch (control_mode) {
      case common::ControlMode::kHomeScreen: {
        // Fall through to continuous menu.
        [[fallthrough]];
      }
      case common::ControlMode::kContinuousMenu: {
        if (motion_direction_ == mt::StepperDriver::MotionDirection::kPositive) {
          status_output += F("CW......");
        }
        else {
          status_output += F("CCW.....");
        }

        break;
      }
      case common::ControlMode::kOscillateMenu: {
        status_output += String(configuration_.sweep_angles_degrees_[sweep_angle_index_], 0);
        status_output += F("deg..");
        break;
      }
      case common::ControlMode::kHoming: {
        status_output += F("Homing..");
        break;
      }
    }
  }

  // Actuate the motor based on the control mode.
  if (allow_motion_ == false) return; // Do not actuate the motor if movement is disallowed.

  switch (control_mode) {
    case common::ControlMode::kContinuousMenu: {      
      if (motion_status_ != mt::StepperDriver::MotionStatus::kConstantSpeed 
          || motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
        // Accelerate to constant speed.
        motion_status_ = stepper_driver_.MoveByAngle(static_cast<float>(motion_direction_) * 360.0,
                                                     mt::StepperDriver::AngleUnits::kDegrees, motion_type_);
        if (motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
          // Stop and reset issued by user, restart motion.
          motion_type_ = mt::StepperDriver::MotionType::kRelative;
        }
      }
      else {
        // Continue constant speed motion indefinitely.
        stepper_driver_.MoveByJogging(motion_direction_);
      }

      break;
    }
    case common::ControlMode::kOscillateMenu: {
      mt::StepperDriver::MotionStatus motion_status = stepper_driver_.MoveByAngle(sweep_direction_ 
                                                      * configuration_.sweep_angles_degrees_[sweep_angle_index_],
                                                      mt::StepperDriver::AngleUnits::kDegrees, motion_type_);
      if (motion_status == mt::StepperDriver::MotionStatus::kIdle) {
        // Motion completed OR stop and reset issued.
        if (motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
          // Stop and reset issued by user, restart motion.
          motion_type_ = mt::StepperDriver::MotionType::kRelative;
        }
        else {
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
    case common::ControlMode::kHoming: {
      mt::StepperDriver::MotionStatus motion_status = stepper_driver_.MoveByAngle(homing_angle_degrees_,
                                                                              mt::StepperDriver::AngleUnits::kDegrees,
                                                                              motion_type_);
      if (motion_status == mt::StepperDriver::MotionStatus::kIdle) {
        if (control_action == common::ControlAction::kGoHome
            && motion_type_ == mt::StepperDriver::MotionType::kStopAndReset) {
          // First iteration.
          motion_type_ = mt::StepperDriver::MotionType::kRelative;
          Log.noticeln(F("Motion status: homing started"));
        }
        else {
          // Homing completed OR stop and reset issued.
          motion_type_ = mt::StepperDriver::MotionType::kStopAndReset;
          homing_ = false;
          allow_motion_ = false;
          Log.noticeln(F("Motion status: homing finished or stopped"));
        }
      }

      break;
    }
  }
}

void MotorStepper::LogGeneralStatus(common::ControlMode control_mode) const {
  Log.noticeln(F("General Motor Status"));
  if (motion_direction_ == mt::StepperDriver::MotionDirection::kPositive) {
    Log.noticeln(F("Motion direction: clockwise (CW)"));
  }
  else {
    Log.noticeln(F("Motion direction: counter-clockwise (CCW)"));
  }
  
  Log.noticeln(F("Sweep angle (degrees): %F"), configuration_.sweep_angles_degrees_[sweep_angle_index_]);
  Log.noticeln(F("Speed (RPM): %F"), configuration_.speeds_RPM_[speed_index_]);

  if (control_mode == common::ControlMode::kHoming) {
    Log.noticeln(F("Motion status: homing"));
  }
  else if (allow_motion_ == false) {
    Log.noticeln(F("Motion status: stopped"));
  }
  else {
    Log.noticeln(F("Motion status: started"));
  }
}

bool MotorStepper::homing() const {
  return homing_;
}

}  // namespace mtmotor_jig
// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file common_types.h
/// @brief Common types.

#pragma once

#include <Arduino.h>

namespace mtmotor_jig::common {

/// @brief Enum of input ID's.
enum class InputId : uint8_t {
  kEncoderDial,
  kEncoderButton,
  kControllerButton,
  kLimitSwitch,
  kSerialInput,
};

/// @brief Enum of control system modes.
enum class ControlMode : uint8_t {
  kSplashScreen,
  kHomeScreen,
  kContinuousMenu,
  kOscillateMenu,
  kHoming,
};

/// @brief Enum of control actions.
enum class ControlAction : uint8_t {
  kSelectNext = 'n',
  kSelectPrevious = 'p',
  kToggleDirection = 'd',
  kCycleAngle = 'a',
  kCycleSpeed = 's',
  kToggleMotion = 'm',
  kResetHome = 'x',
  kGoHome = 'h',
  kToggleLogReport = 'r',
  kLogGeneralStatus = 'l',
  kReportFirmwareVersion = 'v',
  kIdle = '0',
};

} // namespace mtmotor_jig::common
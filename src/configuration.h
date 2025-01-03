// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file configuration.h
/// @brief Class that sets up common configuration settings, including serial port and pin definitions, etc.

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <Arduino.h>
#include <ArduinoLog.h>
#include <momentary_button.h>
#include <stepper_driver.h>

#include "version.h"

/// @brief Macro to define Serial port.
#ifndef MTMOTOR_JIG_SERIAL
#define MTMOTOR_JIG_SERIAL Serial // "Serial" for programming port, "SerialUSB" for native port (Due and Zero only).
#endif

namespace mtmotor_jig {

/// @brief The Configuration class using the singleton pattern i.e., only a single instance can exist.
class Configuration {
 public:

  /// @brief Enum of control system modes.
  enum class ControlMode {
    kSplashScreen,
    kHomeScreen,
    kContinuousMenu,
    kOscillateMenu,
  };

  /// @brief Enum of control actions.
  enum class ControlAction {
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

  /// @brief Static method to get the single instance.
  /// @return The Configuration instance. 
  static Configuration& GetInstance();

  /// @brief Delete the copy constructor to prevent copying of the single instance.
  Configuration(const Configuration&) = delete;

  /// @brief Delete the assignment operator to prevent copying of the single instance.
  Configuration& operator=(const Configuration&) = delete;

  /// @brief Initialise the hardware (Serial port, logging, pins, etc.).
  void BeginHardware() const; ///< This must be called only once.

  /// @brief Toggle log messages.
  void ToggleLogs();

  /// @brief Report the firmware version.
  void ReportFirmwareVersion();

  // GPIO pins.
  const uint8_t kEncoderButtonPin_ = 53; ///< Input pin for the encoder button to control motor direction or angle.
  const uint8_t kEncoderContactAPin_ = 24; ///< Input pin for the encoder contact A to control mode selection.
  const uint8_t kEncoderContactBPin_ = 23; ///< Input pin for the encoder contact B to control mode selection.
  const uint8_t kControllerButtonPin_ = 26; ///< Input pin for the controller button to control motor speed.
  const uint8_t kControllerBuzzerPin_ = 49; ///< Input pin for the controller buzzer.
  const uint8_t kControllerSdMosiPin_ = 22; ///< Input pin for the controller SD card MOSI (master out slave in) interface.
  const uint8_t kControllerSdMisoPin_ = 25; ///< Input pin for the controller SD card MISO (master in slave out) interface.
  const uint8_t kControllerSdCsPin_ = 28; ///< Input pin for the controller SD card CS (chip select) interface.
  const uint8_t kControllerSdCkPin_ = 29; ///< Input pin for the controller SD card CK (clock) interface.
  const uint8_t kLcdRsPin_ = 52; ///< Output pin for the LCD RS (register select) interface.
  const uint8_t kLcdEnaPin_ = 48; ///< Output pin for the LCD ENA (enable) interface.
  const uint8_t kLcdD4Pin_ = 47; ///< Output pin for the LCD D4 interface.
  const uint8_t kLcdD5Pin_ = 51; ///< Output pin for the LCD D5 interface.
  const uint8_t kLcdD6Pin_ = 46; ///< Output pin for the LCD D6 interface.
  const uint8_t kLcdD7Pin_ = 50; ///< Output pin for the LCD D7 interface.
  const uint8_t kMotorDriverPulPin_ = 2; ///< Output pin for the stepper driver PUL/STP/CLK (pulse/step) interface.
  const uint8_t kMotorDriverDirPin_ = 3; ///< Output pin for the stepper driver DIR/CW (direction) interface.
  const uint8_t kMotorDriverEnaPin_ = 4; ///< Output pin for the stepper driver ENA/EN (enable) interface.
  const uint8_t kLimitSwitchPin_ = 32; ///< Input pin for the the limit switch to simulate a soft home position.

  // Control system properties.
  const ControlMode kDefaultControlMode_ = ControlMode::kSplashScreen; ///< The default/initial control mode. 

  // Serial properties.
  const int kBaudRate_ = 9600; ///< The serial communication speed.

  // Encoder dial and button properties.
  const uint8_t kEncoderDetents_ = 20; ///< Encoder detents.
  const uint16_t kEncoderMaxRotationAngle_degrees_ = 360.0F; ///< Encoder maximum rotation angle (degrees).
  const mt::MomentaryButton::PinState kUnpressedPinState_ = mt::MomentaryButton::PinState::kHigh; ///< Button unpressed pin states.
  const uint16_t kDebouncePeriod_ms_ = 20; ///< Button debounce periods (ms).
  const uint16_t kShortPressPeriod_ms_ = 500; ///< Button short press periods (ms).
  const uint16_t kLongPressPeriod_ms_ = 1000; ///< Button long press periods (ms).
  const mt::MomentaryButton::LongPressOption kLongPressOption_ = mt::MomentaryButton::LongPressOption::kDetectWhileHolding; ///< Button long press options.

  // Stepper motor/drive system properties.
  const float kFullStepAngle_degrees_ = 1.8F; ///< The stepper motor full step angle (degrees).
  const float kGearRatio_ = 1.0F; ///< The system/stepper motor gear ratio.

  // Stepper driver properties.
  const uint16_t kMicrostepMode_ = 32; ///< Stepper driver microstep mode.
  const float kPulDelay_us_ = 1.0F; ///< Minimum delay (us) for the stepper driver PUL pin.
  const float kDirDelay_us_ = 5.0F; ///< Minimum delay (us) for the stepper driver Dir pin.
  const float kEnaDelay_us_ = 5.0F; ///< Minimum delay (us) for the stepper driver Ena pin.
  const mt::StepperDriver::MotionDirection kDefaultMotionDirection_ = mt::StepperDriver::MotionDirection::kPositive; ///< Initial/default motion direction (Clockwise (CW)).
  static const uint8_t kSizeOfSweepAngles_ = 4; ///< No. of sweep angles in the lookup table.
  const float kSweepAngles_degrees_[kSizeOfSweepAngles_] = {45.0F, 90.0F, 180.0F, 360.0F}; ///< Lookup table for sweep angles (degrees) during oscillation.
  const uint8_t kDefaultSweepAngleIndex_ = 0; ///< Index of initial/default sweep angle.
  static const uint8_t kSizeOfSpeeds_ = 4; ///< No. of speeds in the lookup table.
  const float kSpeeds_RPM_[kSizeOfSpeeds_] = {7.0F, 10.0F, 13.0F, 16.0F}; ///< Lookup table for rotation speeds (RPM).
  const uint8_t kDefaultSpeedIndex_ = 0; ///< Index of initial/default speed.
  const float kAcceleration_microsteps_per_s_per_s_ = 6000.0; //8000.0; ///< Acceleration (microsteps per second-squared).
  const mt::StepperDriver::AccelerationAlgorithm kAccelerationAlgorithm_ = mt::StepperDriver::AccelerationAlgorithm::kMorgridge24; ///< Acceleration algorithm.

  // Other properties.
  const uint16_t kStartupTime_ms_ = 1000; ///< Minimum startup/boot time in milliseconds (ms); based on the motor driver.

  // Display properties.
  const uint8_t kDisplayHeight_ = 4; ///< The Display height (Rows).
  const uint8_t kDisplayWidth_ = 20; ///< The Display width (Columns).
  static const uint8_t kSizeOfSplashScreenMenuItems_ = 4; ///< No. of splash screen menu items.
  /// @brief The splash screen menu items.
  const String kSplashScreenMenuItems_[kSizeOfSplashScreenMenuItems_] = {"____MTmotor-jig_____",   // 0
                                                                         "   Test motors &"    ,   // 1
                                                                         "   motor control"    ,   // 2
                                                                         "      devices"        }; // 3
  const uint16_t kSplashScreenDelay_ms_ = 1500; ///< Period of time in milliseconds (ms) to show the splash screen.
  static const uint8_t kSizeOfHomeScreenMenuItems_ = 3; ///< No. of home screen menu items.
  /// @brief The home screen menu items.
  const String kHomeScreenMenuItems_[kSizeOfHomeScreenMenuItems_] = {"__Select_Mode_______",   // 0
                                                                     "o Continuous"        ,   // 1
                                                                     "o Oscillate"          }; // 2
  const uint8_t kContinuousMenuCursorPositionY_ = 1; ///< The cursor position (y-axis) for the continuous menu.
  const uint8_t kOscillateMenuCursorPositionY_ = 2; ///< The cursor position (y-axis) for the oscillate menu.
  const uint8_t kStatusBarCursorPositionY_ = 3; ///< The cursor position (y-axis) for the status bar.
  const uint8_t kDefaultCursorPositionY_ = kContinuousMenuCursorPositionY_; ///< The default/initial cursor position (y-axis).

  // Buzzer properties.
  const uint16_t kBuzzerStartupFrequency_Hz_ = 4000; ///< The buzzer frequency (Hz) at startup.
  const uint16_t kBuzzerStartupDuration_ms_ = 100; ///< The buzzer duration (ms) at startup.

 private:

  /// @brief Private constructor so objects cannot be manually instantiated. 
  Configuration();

  /// @brief Private destructor so objects cannot be manually instantiated. 
  ~Configuration();

  // Debug helpers and logger properties (for debugging and system reporting).
  int log_level_ = LOG_LEVEL_SILENT; ///< The log level.
  //int log_level_ = LOG_LEVEL_VERBOSE; ///< The log level.

};

} // namespace mtmotor_jig

#endif // CONFIGURATION_H_
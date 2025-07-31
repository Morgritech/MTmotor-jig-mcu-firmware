// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file configuration.h
/// @brief Class that sets up common configuration settings, including serial port and pin definitions, etc.

#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include <momentary_button.h>
#include <stepper_driver.h>

#include "version.h"
#include "common_types.h"

/// @brief Macro to define Serial port.
#ifndef MTMOTOR_JIG_SERIAL
#define MTMOTOR_JIG_SERIAL Serial // "Serial" for programming port, "SerialUSB" for native port (Due and Zero only).
#endif

namespace mtmotor_jig {

/// @brief The Configuration class using the singleton pattern i.e., only a single instance can exist.
class Configuration {
 public:

  /// @brief Static method to get the single instance.
  /// @return The Configuration instance. 
  static Configuration& GetInstance();

  /// @brief Delete the copy constructor to prevent copying of the single instance.
  Configuration(const Configuration&) = delete;

  /// @brief Delete the assignment operator to prevent copying of the single instance.
  Configuration& operator=(const Configuration&) = delete;

  /// @brief Initialise the hardware (Serial port, logging, pins, etc.).
  void BeginHardware(); ///< This must be called only once.

  /// @brief Toggle log messages.
  void ToggleLogs();

  /// @brief Report the firmware version.
  void ReportFirmwareVersion();

  // GPIO pins.
  const uint8_t kEncoderButtonPin_ = 33; ///< Input pin for the encoder button to control motor direction or angle.
  const uint8_t kEncoderContactAPin_ = 27; ///< Input pin for the encoder contact A to control mode selection.
  const uint8_t kEncoderContactBPin_ = 26; ///< Input pin for the encoder contact B to control mode selection.
  const uint8_t kControllerButtonPin_ = 28; ///< Input pin for the controller button to control motor speed.
  const uint8_t kControllerBuzzerPin_ = 49; ///< Input pin for the controller buzzer.
  const uint8_t kSdMisoPin_ = 50; ///< Input/output pin for the SD card MISO/PICO (controller in, peripheral out) interface..
  const uint8_t kSdMosiPin_ = 51; ///< Input/output for the SD card MOSI/COPI (controller out, peripheral in) interface.
  const uint8_t kSdClkPin_ = 52; ///< Input/output pin for the SD card CLK (clock) interface. Pin 13 for Uno.
  const uint8_t kSdCsPin_ = 53; ///< Output pin for the SD card CS (chip select) interface. Pin 10 for Uno. This pin MUST be set as output.
  const uint8_t kLcdRsPin_ = 32; ///< Output pin for the LCD RS (register select) interface.
  const uint8_t kLcdEnaPin_ = 48; ///< Output pin for the LCD ENA (enable) interface.
  const uint8_t kLcdD4Pin_ = 47; ///< Output pin for the LCD D4 interface.
  const uint8_t kLcdD5Pin_ = 31; ///< Output pin for the LCD D5 interface.
  const uint8_t kLcdD6Pin_ = 46; ///< Output pin for the LCD D6 interface.
  const uint8_t kLcdD7Pin_ = 30; ///< Output pin for the LCD D7 interface.
  const uint8_t kMotorDriverPulPin_ = 2; ///< Output pin for the stepper driver PUL/STP/CLK (pulse/step) interface.
  const uint8_t kMotorDriverDirPin_ = 3; ///< Output pin for the stepper driver DIR/CW (direction) interface.
  const uint8_t kMotorDriverEnaPin_ = 4; ///< Output pin for the stepper driver ENA/EN (enable) interface.
  const uint8_t kLimitSwitchPin_ = 34; ///< Input pin for the the limit switch to simulate a soft home position.

  // Control system properties.
  const common::ControlMode kDefaultControlMode_ = common::ControlMode::kSplashScreen; ///< The default/initial control mode. 

  // Serial properties.
  int baud_rate_ = 9600; ///< The serial communication speed.

  // Encoder dial and button properties.
  const uint8_t kEncoderDetents_ = 20; ///< Encoder detents.
  const uint16_t kEncoderMaxRotationAngle_degrees_ = 360.0F; ///< Encoder maximum rotation angle (degrees).
  const mt::MomentaryButton::PinState kUnpressedPinState_ = mt::MomentaryButton::PinState::kHigh; ///< Button unpressed pin states.
  const uint16_t kDebouncePeriod_ms_ = 20; ///< Button debounce periods (ms).
  const uint16_t kShortPressPeriod_ms_ = 500; ///< Button short press periods (ms).
  const uint16_t kLongPressPeriod_ms_ = 1000; ///< Button long press periods (ms).
  const mt::MomentaryButton::LongPressOption kLongPressOption_ =
                                       mt::MomentaryButton::LongPressOption::kDetectWhileHolding; ///< Button long press options.

  // Stepper motor/drive system properties.
  float full_step_angle_degrees_ = 1.8F; ///< The stepper motor full step angle (degrees).
  float gear_ratio_ = 1.0F; ///< The system/stepper motor gear ratio.

  // Stepper driver properties.
  uint16_t microstep_mode_ = 32; ///< Stepper driver microstep mode.
  float pul_delay_us_ = 1.0F; ///< Minimum delay (us) for the stepper driver PUL pin.
  float dir_delay_us_ = 5.0F; ///< Minimum delay (us) for the stepper driver Dir pin.
  float ena_delay_us_ = 5.0F; ///< Minimum delay (us) for the stepper driver Ena pin.
  const mt::StepperDriver::MotionDirection kDefaultMotionDirection_ =
                                                   mt::StepperDriver::MotionDirection::kPositive; ///< Initial/default motion direction (Clockwise (CW)).
  inline static constexpr uint8_t kSizeOfSweepAngles_ = 4; ///< No. of sweep angles in the lookup table.
  float sweep_angles_degrees_[kSizeOfSweepAngles_] = {45.0F, 90.0F, 180.0F, 360.0F}; ///< Lookup table for sweep angles (degrees) during oscillation.
  const uint8_t kDefaultSweepAngleIndex_ = 0; ///< Index of initial/default sweep angle.
  inline static constexpr uint8_t kSizeOfSpeeds_ = 4; ///< No. of speeds in the lookup table.
  float speeds_RPM_[kSizeOfSpeeds_] = {7.0F, 10.0F, 13.0F, 16.0F}; ///< Lookup table for rotation speeds (RPM).
  const uint8_t kDefaultSpeedIndex_ = 0; ///< Index of initial/default speed.
  float acceleration_microsteps_per_s_per_s_ = 6000.0; //8000.0; ///< Acceleration (microsteps per second-squared).
  const mt::StepperDriver::AccelerationAlgorithm kAccelerationAlgorithm_ =
                                          mt::StepperDriver::AccelerationAlgorithm::kMorgridge24; ///< Acceleration algorithm.

  // Display properties.
  const uint8_t kDisplayHeight_ = 4; ///< The Display height (Rows).
  const uint8_t kDisplayWidth_ = 20; ///< The Display width (Columns).
  /// @brief The splash screen menu items.
  const String kSplashScreenMenuItems_[4] = {"____MTmotor-jig_____",   // 0
                                             "   Test motors &"    ,   // 1
                                             "   motor control"    ,   // 2
                                             "      devices"        }; // 3
  //                                          0                  19
  uint16_t splash_screen_delay_ms_ = 1500; ///< Period of time in milliseconds (ms) to show the splash screen.
  /// @brief The home screen menu items.
  const String kHomeScreenMenuItems_[3] = {"__Select_Mode_______",   // 0
                                           "o Continuous"        ,   // 1
                                           "o Oscillate"          }; // 2
  //                                        0                  19
  const uint8_t kContinuousMenuCursorPositionY_ = 1; ///< The cursor position (y-axis) for the continuous menu.
  const uint8_t kOscillateMenuCursorPositionY_ = 2; ///< The cursor position (y-axis) for the oscillate menu.
  const uint8_t kStatusBarCursorPositionY_ = 3; ///< The cursor position (y-axis) for the status bar.
  const uint8_t kDefaultCursorPositionY_ = kContinuousMenuCursorPositionY_; ///< The default/initial cursor position (y-axis).

  // Buzzer properties.
  bool buzzer_enabled_ = false; ///< Flag to control whether the buzzer is enabled at startup.
  uint16_t buzzer_startup_frequency_Hz_ = 4000; ///< The buzzer frequency (Hz) at startup.
  uint16_t buzzer_startup_duration_ms_ = 100; ///< The buzzer duration (ms) at startup.

  // Other properties.
  uint16_t startup_delay_ms_ = 1000; ///< Minimum startup/boot delay in milliseconds (ms); based on the motor driver.

  // SD card and configuration file properties.
  // NOTE: SD card must be formatted as FAT (FAT16 or FAT32).
  const char* kDefaultConfigFileName_ = "default.txt";  ///< The default configuration file name. SD library allows only short 8.3 names.

  // JSON configuration properties.
  // Nodes.
  const char* KSerialNode_ = "serial";
  const char* kInputsNode_ = "inputs";
  const char* kStepperNode_ = "stepper";
  const char* kDisplayNode_ = "display";
  const char* kBuzzerNode_ = "buzzer";
  const char* kOtherNode_ = "other";
  // Keys.
  const char* kBaudRateKey_ = "baudRate";
  const char* kLongPressOptionKey_ = "longPressOption";
  const char* kStepAngleKey_ = "stepAngle_deg";
  const char* kGearRatioKey_ = "gearRatio";
  const char* kMicrostepModeKey_ = "microstepMode";
  const char* kPulDelayKey_ = "pulDelay_us";
  const char* kDirDelayKey_ = "dirDelay_us";
  const char* kEnaDelayKey_ = "enaDelay_us";
  const char* kSweepAnglesKey_ = "sweepAngles_deg";
  const char* kSpeedsKey_ = "speeds_RPM";
  const char* kAccelerationKey_ = "acceleration_microsteps_per_s_per_s";
  const char* kAccelerationAlgorithmKey_ = "accelerationAlgorithm";
  const char* kSplashScreenDelayKey_ = "splashScreenDelay_ms";
  const char* kBuzzerEnabledKey_ = "enabled";
  const char* kBuzzerStartupFrequencyKey_ = "frequency_hz";
  const char* kBuzzerStartupDurationKey_ = "duration_ms";
  const char* kStartupDelayKey_ = "startupDelay_ms";
  // Valid JSON values.
  using LongPressOption = mt::MomentaryButton::LongPressOption;
  using AccelerationAlgorithm = mt::StepperDriver::AccelerationAlgorithm;
  inline static constexpr uint8_t kSizeOfLongPressOptions_ = 2; ///< No. of long press options.
  const char* kLongPressOptionsStrings_[kSizeOfLongPressOptions_] =
                                                {"detect while holding", "detect after release"}; ///< Long press options.
  const LongPressOption kLongPressOptionsTypes_[kSizeOfLongPressOptions_] =
                        {LongPressOption::kDetectWhileHolding, LongPressOption::kDetectAfterRelease};
  inline static constexpr uint8_t kSizeOfMicrostepModes_ = 6; ///< No. of microstep modes.
  const uint16_t kMicrostepModes_[kSizeOfMicrostepModes_] = {1, 2, 4, 8, 16, 32}; ///< Microstep modes.
  inline static constexpr uint8_t kSizeOfAccelerationAlgorithms_ = 3; ///< No. of acceleration algorithms
  const char* kAccelerationAlgorithmsStrings_[kSizeOfAccelerationAlgorithms_] =
                                                       {"morgridge24", "austin05", "eiderman04"}; ///< Acceleration algorithms.
  const AccelerationAlgorithm kAccelerationAlgorithmsTypes_[kSizeOfAccelerationAlgorithms_] =
           {AccelerationAlgorithm::kMorgridge24, AccelerationAlgorithm::kAustin05, AccelerationAlgorithm::kEiderman04};

 private:

  /// @brief Private constructor so objects cannot be manually instantiated. 
  Configuration();

  /// @brief Private destructor so objects cannot be manually instantiated. 
  ~Configuration();

  /// @brief Read configuration (JSON format) from a file (first available, if default.txt is not found) on the SD card.
  void ReadConfigFromFileOnSd();

  // Debug helpers and logger properties (for debugging and system reporting).
  int log_level_ = LOG_LEVEL_SILENT; ///< The log level.
};

} // namespace mtmotor_jig
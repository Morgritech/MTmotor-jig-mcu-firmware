// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file configuration.cpp
/// @brief Class that sets up common configuration settings, including serial port and pin definitions, etc.

#include "configuration.h"

#include <Arduino.h>
#include <ArduinoLog.h>
#include <SPI.h>
#include <SD.h>

//#define ARDUINOJSON_ENABLE_COMMENTS 1
#include <ArduinoJson.h>

namespace mtmotor_jig {

Configuration& Configuration::GetInstance() {
  static Configuration instance;
  return instance;
}

void mtmotor_jig::Configuration::BeginHardware() {
  // Initialise the serial port.
  MTMOTOR_JIG_SERIAL.begin(baud_rate_);

  // Initialise logging.
  Log.begin(log_level_, &MTMOTOR_JIG_SERIAL);

  // Initialise the input pins.
  pinMode(kEncoderButtonPin_, INPUT_PULLUP);
  pinMode(kEncoderContactAPin_, INPUT_PULLUP);
  pinMode(kEncoderContactBPin_, INPUT_PULLUP);
  pinMode(kControllerButtonPin_, INPUT_PULLUP);
  pinMode(kLimitSwitchPin_, INPUT_PULLUP);

  // Initialise the output pins.
  pinMode(kControllerBuzzerPin_, OUTPUT);
  pinMode(kMotorDriverPulPin_, OUTPUT);
  pinMode(kMotorDriverDirPin_, OUTPUT);
  pinMode(kMotorDriverEnaPin_, OUTPUT);
  pinMode(kSdCsPin_, OUTPUT);

  // Read the configuration the default config file
  // or the first available config file on the SD card.
  ReadConfigFromFileOnSd();

  // Delay for the startup time.
  delay(startup_delay_ms_);
}

void Configuration::ToggleLogs() {
  // Toggle log messages.
  if (log_level_ == LOG_LEVEL_SILENT) {
    log_level_ = LOG_LEVEL_VERBOSE;
  }
  else {
    Log.noticeln(F("Log messages disabled."));
    log_level_ = LOG_LEVEL_SILENT;
  }

  Log.begin(log_level_, &MTMOTOR_JIG_SERIAL);
  if (log_level_ == LOG_LEVEL_VERBOSE) Log.noticeln(F("Log messages enabled."));
}

void Configuration::ReportFirmwareVersion() {
  String version = kName;
  version += F("-");
  version += String(kMajor);
  version += F(".");
  version += String(kMinor);
  version += F(".");
  version += String(kPatch);
  if (strlen(kSuffix) > 0) version += F("-");
  version += kSuffix;
  MTMOTOR_JIG_SERIAL.println(version);
}

Configuration::Configuration() {}

Configuration::~Configuration() {}

void Configuration::ReadConfigFromFileOnSd() {

  auto hardcoded_settings_message = F("Using hardcoded config settings.");

  // Initialise SD card.

  if (!SD.begin(kSdCsPin_)) {
    Log.errorln(F("SD card initialisation failed!"));
    Log.noticeln(hardcoded_settings_message);
    return;
  }

  Log.noticeln(F("SD card initialised."));

  // Read config file from SD card.

  File config_file = SD.open(kDefaultConfigFileName_, FILE_READ);

  if (!config_file) {
    Log.errorln(F("Failed to open configuration file: %s"), kDefaultConfigFileName_);

    // Read the next available file.
    File root_directory = SD.open(F("/"));
    config_file = root_directory.openNextFile(FILE_READ);

    if (!config_file) {
      Log.errorln(F("No other valid files found."));
      Log.noticeln(hardcoded_settings_message);
      return;
    }
  }

  Log.noticeln(F("Configuration file opened: %s"), config_file.name());

  // Extract JSON from config file.

  JsonDocument config_json_doc;
  DeserializationError deserialisation_error = deserializeJson(config_json_doc, config_file);
  config_file.close();  

  if (deserialisation_error) {
    Log.errorln(F("JSON deserialisation/parse error: %s"), deserialisation_error.f_str());
    Log.noticeln(hardcoded_settings_message);
    return;
  }

  Log.noticeln(F("JSON parsed."));

  // Read JSON key-value pairs and check for errors. 

  // Flags and variables for error handling.
  const auto json_extraction_error_message = F("JSON extraction error: %s");
  const auto json_validity_error_message = F("JSON validity error: %s");
  bool json_error = false;
  const float numeric_error_value = 0.0;

  // Lambda functions for error handling.
  auto JsonNumericExtractionError =
  [&json_extraction_error_message, &json_error]
  (float check, float check_against, const char* message) -> bool {
    if (check == check_against) {
      json_error = true;
      Log.errorln(json_extraction_error_message, message);
    }

    return json_error;
  };

  auto JsonStringExtractionError =
  [&json_extraction_error_message, &json_error]
  (const char* check, const char* message) -> bool {
    if (check == nullptr) {
      json_error = true;
      Log.errorln(json_extraction_error_message, message);
    }

    return json_error;
  };

  auto JsonNumericValidityError =
  [&json_validity_error_message, &json_error]
  (uint16_t check, const uint16_t check_against[], uint8_t size_of_check_against, const char* message) {
    json_error = true; // Assume error until proven otherwise.
    for (auto i = 0; i < size_of_check_against; i++) {
      if (check == check_against[i]) {
        json_error = false; // Valid mode found.
        break;
      }
    }

    if (json_error == true) Log.errorln(json_validity_error_message, message);
  };

  auto JsonStringValidityError =
  [&json_validity_error_message, &json_error]
  (const char* check, const char* check_against[], uint8_t size_of_check_against, const char* message) -> uint8_t {
    json_error = true; // Assume error until proven otherwise.
    uint8_t valid_index;
    for (auto i = 0; i < size_of_check_against; i++) {
      if (strcmp(check, check_against[i]) == 0) {
        json_error = false; // Valid mode found.
        valid_index = i;
        break;
      }
    }

    if (json_error == true) Log.errorln(json_validity_error_message, message);

    return valid_index;
  };

  // Serial node key-value pairs.
  int baud_rate = config_json_doc[KSerialNode_][kBaudRateKey_];
  JsonNumericExtractionError(baud_rate, numeric_error_value, kBaudRateKey_);

  // Input node key-value pair.
  uint8_t valid_long_press_option_index;
  const char* long_press_option = config_json_doc[kInputsNode_][kLongPressOptionKey_];
  if (JsonStringExtractionError(long_press_option, kLongPressOptionKey_) == false)
    valid_long_press_option_index = JsonStringValidityError(long_press_option, kLongPressOptionsStrings_,
                                                            kSizeOfLongPressOptions_, kLongPressOptionKey_);

  // Stepper node key-value pairs.
  JsonObject stepper_node_obj = config_json_doc[kStepperNode_];

  float step_angle_deg = stepper_node_obj[kStepAngleKey_];
  JsonNumericExtractionError(step_angle_deg, numeric_error_value, kStepAngleKey_);

  float gear_ratio = stepper_node_obj[kGearRatioKey_];
  JsonNumericExtractionError(gear_ratio, numeric_error_value, kGearRatioKey_);

  uint16_t microstep_mode = stepper_node_obj[kMicrostepModeKey_];
  if (JsonNumericExtractionError(microstep_mode, numeric_error_value, kMicrostepModeKey_) == false)
    JsonNumericValidityError(microstep_mode, kMicrostepModes_, kSizeOfMicrostepModes_, kMicrostepModeKey_);

  float pul_delay_us = stepper_node_obj[kPulDelayKey_];
  JsonNumericExtractionError(pul_delay_us, numeric_error_value, kPulDelayKey_);

  float dir_delay_us = stepper_node_obj[kDirDelayKey_];
  JsonNumericExtractionError(dir_delay_us, numeric_error_value, kDirDelayKey_);

  float ena_delay_us = stepper_node_obj[kEnaDelayKey_];
  JsonNumericExtractionError(ena_delay_us, numeric_error_value, kEnaDelayKey_);

  JsonArray sweep_angles = stepper_node_obj[kSweepAnglesKey_];
  for (const auto& angle : sweep_angles)
    if (JsonNumericExtractionError(angle, numeric_error_value, kSweepAnglesKey_)) break;

  JsonArray speeds = stepper_node_obj[kSpeedsKey_];
  for (const auto& speed : speeds)
    if (JsonNumericExtractionError(speed, numeric_error_value, kSpeedsKey_)) break;
  
  float acceleration_error_value = -1.0f;
  float acceleration_microsteps_per_s_per_s = stepper_node_obj[kAccelerationKey_] | acceleration_error_value;
  JsonNumericExtractionError(acceleration_microsteps_per_s_per_s, acceleration_error_value, kAccelerationKey_);
  
  uint8_t valid_acceleration_algorithm_index;
  const char* acceleration_algorithm = stepper_node_obj[kAccelerationAlgorithmKey_];
  if (JsonStringExtractionError(acceleration_algorithm, kAccelerationAlgorithmKey_) == false)
    valid_acceleration_algorithm_index = JsonStringValidityError(acceleration_algorithm,
                                                                 kAccelerationAlgorithmsStrings_,
                                                                 kSizeOfAccelerationAlgorithms_,
                                                                 kAccelerationAlgorithmKey_);

  // Display node key-value pairs.
  uint16_t splash_screen_delay_ms = config_json_doc[kDisplayNode_][kSplashScreenDelayKey_];
  JsonNumericExtractionError(splash_screen_delay_ms, numeric_error_value, kSplashScreenDelayKey_);

  // Buzzer node key-value pairs.
  JsonObject buzzer_node_obj = config_json_doc[kBuzzerNode_];
  
  bool buzzer_enabled = buzzer_node_obj[kBuzzerEnabledKey_];
  
  uint16_t buzzer_startup_frequency_Hz = buzzer_node_obj[kBuzzerStartupFrequencyKey_];
  JsonNumericExtractionError(buzzer_startup_frequency_Hz, numeric_error_value, kBuzzerStartupFrequencyKey_);

  uint16_t buzzer_startup_duration_ms = buzzer_node_obj[kBuzzerStartupDurationKey_];
  JsonNumericExtractionError(buzzer_startup_duration_ms, numeric_error_value, kBuzzerStartupDurationKey_);

  // Other node key-value pairs.
  uint16_t startup_delay_ms = config_json_doc[kOtherNode_][kStartupDelayKey_];
  JsonNumericExtractionError(startup_delay_ms, numeric_error_value, kStartupDelayKey_);

  if (json_error) {
    Log.noticeln(hardcoded_settings_message);
    return;
  }

  // Assign JSON values to configuration settings.

  baud_rate_ = baud_rate;
  mt::MomentaryButton::LongPressOption kLongPressOption_ = kLongPressOptionsTypes_[valid_long_press_option_index];
  full_step_angle_degrees_ = step_angle_deg; 
  gear_ratio_ = gear_ratio;
  microstep_mode_ = microstep_mode;
  pul_delay_us_ = pul_delay_us;
  dir_delay_us_ = dir_delay_us;
  ena_delay_us_ = ena_delay_us;
  for (auto i = 0; i < kSizeOfSweepAngles_; i++) kSweepAngles_degrees_[i] = sweep_angles[i];
  for (auto i = 0; i < kSizeOfSpeeds_; i++) kSpeeds_RPM_[i] = speeds[i];
  acceleration_microsteps_per_s_per_s_ = acceleration_microsteps_per_s_per_s;
  mt::StepperDriver::AccelerationAlgorithm kAccelerationAlgorithm_ = 
                                                kAccelerationAlgorithmsTypes_[valid_acceleration_algorithm_index];
  splash_screen_delay_ms_ = splash_screen_delay_ms;
  buzzer_enabled_ = buzzer_enabled;
  buzzer_startup_frequency_Hz_ = buzzer_startup_frequency_Hz;
  buzzer_startup_duration_ms_ = buzzer_startup_duration_ms;
  startup_delay_ms_ = startup_delay_ms;
}

} // namespace mtmotor_jig
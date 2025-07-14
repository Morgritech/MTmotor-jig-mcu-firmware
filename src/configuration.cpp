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

#define ARDUINOJSON_ENABLE_COMMENTS 1
#include <ArduinoJson.h>

namespace mtmotor_jig {

Configuration& Configuration::GetInstance() {
  static Configuration instance;
  return instance;
}

void mtmotor_jig::Configuration::BeginHardware() {
  // Initialise the serial port.
  MTMOTOR_JIG_SERIAL.begin(kBaudRate_);

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
  delay(kStartupDelay_ms_);
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
  if (!SD.begin(kSdCsPin_)) {
    Log.errorln(F("SD card initialisation failed!"));
    return;
  }

  Log.noticeln(F("SD card initialised."));

  File config_file = SD.open(kDefaultConfigFileName_, FILE_READ);

  if (!config_file) {
    Log.errorln(F("Failed to open configuration file: %s"), kDefaultConfigFileName_);
    return;
    // TODO(JM):
    // Attempt to read the first available configuration file.
    // If ok, Log notice of the file that was opened.
    // Else log error and return.
  }

  Log.noticeln(F("Configuration file opened: %s"), kDefaultConfigFileName_);

  JsonDocument config_json_doc;
  DeserializationError deserialisation_error = deserializeJson(config_json_doc, config_file);
  config_file.close();  

  if (deserialisation_error) {
    Log.errorln(F("JSON deserialisation/parse error: %s"), deserialisation_error.f_str());
    return;
  }

  Log.noticeln(F("JSON parsed."));

  // Flags and variables for error handling.
  const auto json_extraction_error_message = F("JSON extraction error: ");
  const auto json_validity_error_message = F("JSON extraction error: ");
  bool json_error = false;
  const float numeric_error_value = 0.0;

  // Lambda functions for error handling.
  auto JsonNumericExtractionError =
  [&json_extraction_error_message, &json_error]
  (float check, float check_against, const __FlashStringHelper* message) -> bool {
    if (check == check_against) {
      json_error = true;
      Log.errorln(json_extraction_error_message, message);
    }

    return json_error;
  };

  auto JsonStringExtractionError =
  [&json_extraction_error_message, &json_error]
  (const char* check, const __FlashStringHelper* message) -> bool {
    if (check == nullptr) {
      json_error = true;
      Log.errorln(json_extraction_error_message, message);
    }

    return json_error;
  };

  auto JsonNumericValidityError =
  [&json_validity_error_message, &json_error]
  (uint16_t check, const uint16_t check_against[], uint8_t size_of_check_against, const __FlashStringHelper* message) {
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
  (const char* check, const char* check_against[], uint8_t size_of_check_against, const __FlashStringHelper* message) {
    json_error = true; // Assume error until proven otherwise.
    for (auto i = 0; i < size_of_check_against; i++) {
      if (strcmp(check, check_against[i]) == 0) {
        json_error = false; // Valid mode found.
        break;
      }
    }

    if (json_error == true) Log.errorln(json_validity_error_message, message);
  };  

  // Serial node key-value pairs.
  auto baud_rate_key = F("baudRate");
  int baud_rate = config_json_doc[KSerialNode_][baud_rate_key];
  JsonNumericExtractionError(baud_rate, numeric_error_value, baud_rate_key);

  // Input node key-value pair.
  auto long_press_option_key = F("longPressOption");
  const char* long_press_option = config_json_doc[kInputNode_][long_press_option_key]; // Further process required to change it to the correct type from the button library.
  if (JsonStringExtractionError(long_press_option, long_press_option_key) == false)
    JsonStringValidityError(long_press_option, kLongPressOptions_, kSizeOfLongPressOptions_, long_press_option_key);

  // Stepper node key-value pairs.
  JsonObject stepper_node_obj = config_json_doc[kStepperNode_];

  auto step_angle_key = F("stepAngle_deg");
  float step_angle_deg = stepper_node_obj[step_angle_key];
  JsonNumericExtractionError(step_angle_deg, numeric_error_value, step_angle_key);

  auto gear_ratio_key = F("gearRatio");
  float gear_ratio = stepper_node_obj[gear_ratio_key];
  JsonNumericExtractionError(gear_ratio, numeric_error_value, gear_ratio_key);

  auto microstep_mode_key = F("microstepMode");
  uint16_t microstep_mode = stepper_node_obj[microstep_mode_key];
  if (JsonNumericExtractionError(microstep_mode, numeric_error_value, microstep_mode_key) == false)
    JsonNumericValidityError(microstep_mode, kMicrostepModes_, kSizeOfMicrostepModes_, microstep_mode_key);

  auto pul_delay_key = F("pulDelay_us");
  float pul_delay_us = stepper_node_obj[pul_delay_key];
  JsonNumericExtractionError(pul_delay_us, numeric_error_value, pul_delay_key);

  auto dir_delay_key = F("dirDelay_us");
  float dir_delay_us = stepper_node_obj[dir_delay_key];
  JsonNumericExtractionError(dir_delay_us, numeric_error_value, dir_delay_key);

  auto ena_delay_key = F("enaDelay_us");
  float ena_delay_us = stepper_node_obj[ena_delay_key];
  JsonNumericExtractionError(ena_delay_us, numeric_error_value, ena_delay_key);

  auto sweep_angles_key = F("sweepAngles_deg");
  JsonArray sweep_angles = stepper_node_obj[sweep_angles_key];
  for (const auto& angle : sweep_angles)
    if (JsonNumericExtractionError(angle, numeric_error_value, sweep_angles_key)) break;

  auto speeds_key = F("speeds_RPM");
  JsonArray speeds = stepper_node_obj[speeds_key];
  for (const auto& speed : speeds)
    if (JsonNumericExtractionError(speed, numeric_error_value, speeds_key)) break;
  
  float acceleration_error_value = -1.0f;
  auto acceleration_key = F("acceleration_microsteps_per_s_per_s");
  float acceleration_microsteps_per_s_per_s = stepper_node_obj[acceleration_key] | acceleration_error_value;
  JsonNumericExtractionError(acceleration_microsteps_per_s_per_s, acceleration_error_value, acceleration_key);
  
  auto acceleration_algorithm_key = F("accelerationAlgorithm");
  const char* acceleration_algorithm = stepper_node_obj[acceleration_algorithm_key]; // Further process required to change it to the correct type from the stepper library.
  if (JsonStringExtractionError(acceleration_algorithm, acceleration_algorithm_key) == false)
    JsonStringValidityError(acceleration_algorithm, kAccelerationAlgorithms_, kSizeOfAccelerationAlgorithms_,
                            acceleration_algorithm_key);

  // Display node key-value pairs.
  auto splash_screen_delay_key = F("splashScreenDelay_ms");
  uint16_t splash_screen_delay_ms = config_json_doc[kDisplayNode_][splash_screen_delay_key];
  JsonNumericExtractionError(splash_screen_delay_ms, numeric_error_value, splash_screen_delay_key);

  // Buzzer node key-value pairs.
  JsonObject buzzer_node_obj = config_json_doc[kBuzzerNode_];
  
  bool buzzer_enabled = buzzer_node_obj[F("enabled")];
  
  auto buzzer_startup_frequency_key = F("frequency_hz");
  uint16_t buzzer_startup_frequency_hz = buzzer_node_obj[buzzer_startup_frequency_key];
  JsonNumericExtractionError(buzzer_startup_frequency_hz, numeric_error_value, buzzer_startup_frequency_key);

  auto buzzer_startup_duration_key = F("duration_ms");
  uint16_t buzzer_startup_duration_ms = buzzer_node_obj[buzzer_startup_duration_key];
  JsonNumericExtractionError(buzzer_startup_duration_ms, numeric_error_value, buzzer_startup_duration_key);

  // Other node key-value pairs.
  auto startup_time_key = F("startupTime_ms");
  uint16_t startup_time_ms = config_json_doc[kOtherNode_][startup_time_key];
  JsonNumericExtractionError(startup_time_ms, numeric_error_value, startup_time_key);
}

} // namespace mtmotor_jig
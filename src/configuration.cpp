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

  // Read the configuration the default config file 
  // or the first available config file on the SD card.
  ReadConfigFromFileOnSd();

  // Delay for the startup time.
  delay(kStartupTime_ms_);
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
  if (!SD.begin(kControllerSdCsPin_)) {
    Log.errorln(F("SD card initialisation failed!"));
    return;
  }

  Log.noticeln(F("SD card initialised."));

  char* serialised_json = nullptr; //""
  File config_file = SD.open(kDefaultConfigFileName_);

  if (config_file) {
    Log.noticeln(F("Configuration file opened: %s"), kDefaultConfigFileName_);
  }
  else {
    Log.errorln(F("Failed to open configuration file: %s"), kDefaultConfigFileName_);

    // TODO(JM): 
    // Attempt to read the first available configuration file.
    // If ok, Log notice of the file that was opened.
    // Else log error and return.
    return;
  }

  while (config_file.available()) {
    serialised_json += config_file.read();
  }

  // TODO(JM): 
  // deserialise the JSON string into a JSON object.
  // store the values in the relevant member variables.
  Serial.println(serialised_json);
}

} // namespace mtmotor_jig
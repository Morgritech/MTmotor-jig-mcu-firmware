// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file configuration.cpp
/// @brief Class that sets up common configuration settings, including serial port and pin definitions, etc.

#include "configuration.h"

#include <Arduino.h>
#include <ArduinoLog.h>

namespace mtmotor_jig {

Configuration& Configuration::GetInstance() {
  static Configuration instance;
  return instance;
}

void mtmotor_jig::Configuration::BeginHardware() const {
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

} // namespace mtmotor_jig
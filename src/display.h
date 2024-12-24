// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file display.h
/// @brief Class that handles visual output to the display.

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Arduino.h>

#include <LiquidCrystal.h>

#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Display class.
class Display {
 public:

  /// @brief Construct a Display object. 
  Display();

  /// @brief Destroy the Display object.
  ~Display();

  /// @brief Initialise the display.
  void Begin() const;

  /// @brief Clear the display.
  void Clear() const;

  /// @brief Display the firmware version.
  void DisplayFirmwareVersion() const;

  /// @brief Display the general status.
  void DisplayGeneralStatus() const;

 private:

  /// @brief Configuration settings.
  Configuration& configuration_ = Configuration::GetInstance();

};

} // namespace mtmotor_jig

#endif // DISPLAY_H_
// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file display_manager.h
/// @brief Class that handles visual output to the display.

#pragma once

#include <Arduino.h>
#include <LiquidCrystal.h>

#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Display class.
class DisplayManager {
 public:

  /// @brief Construct a Display Manager object. 
  DisplayManager();

  /// @brief Destroy the Display Manager object.
  ~DisplayManager();

  /// @brief Initialise the display.
  void Begin(); ///< This must be called only once.

  /// @brief Draw items to the display based on the control mode.
  /// @param control_mode The control mode.
  /// @param status The status message to display.
  void Draw(Configuration::ControlMode control_mode, Configuration::ControlAction control_action,
            const String& status);

 private:

  /// @brief Draw items to the display.
  /// @param screen_items The screen items to draw.
  /// @param size_of_screen_items The number of screen items.
  void DrawScreenItems(const String screen_items[], const uint8_t size_of_screen_items);

  /// @brief Configuration settings.
  Configuration& configuration_ = Configuration::GetInstance();

  /// @brief The LCD display.
  LiquidCrystal lcd_{configuration_.kLcdRsPin_, configuration_.kLcdEnaPin_, configuration_.kLcdD4Pin_,
                     configuration_.kLcdD5Pin_, configuration_.kLcdD6Pin_, configuration_.kLcdD7Pin_};

  /// @brief The current cursor position along the y-axis.
  uint8_t cursor_position_y_ = configuration_.kDefaultCursorPositionY_;
};

} // namespace mtmotor_jig
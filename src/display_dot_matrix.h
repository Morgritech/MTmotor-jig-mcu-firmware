// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file display_dot_matrix.h
/// @brief Class that handles visual output to the (dot-matrix) display.

#pragma once

#include <Arduino.h>
#include <LiquidCrystal.h>

#include "common_types.h"
#include "configuration.h"

namespace mtmotor_jig {

/// @brief The Display (Dot Matrix) class.
class DisplayDotMatrix {
 public:

  /// @brief Construct a Display (Dot Matrix) object.
  DisplayDotMatrix(LiquidCrystal& display, Configuration& configuration = Configuration::GetInstance());

  /// @brief Destroy the Display (Dot Matrix) object.
  ~DisplayDotMatrix();

  /// @brief Draw items to the display based on the control mode.
  /// @param control_mode The control mode.
  /// @param status The status message to display.
  void Draw(common::ControlMode control_mode, common::ControlAction control_action, const String& status);

 private:

  /// @brief Draw items to the display.
  /// @tparam Size The size of the screen items.
  /// @param screen_items The screen items to draw.
  template <size_t Size>
  void DrawScreenItems(const String (&screen_items)[Size]) const;

  /// @brief The LCD display.
  LiquidCrystal& display_;

  /// @brief Configuration settings.
  Configuration& configuration_;

  /// @brief The current cursor position along the y-axis.
  uint8_t cursor_position_y_ = configuration_.kDefaultCursorPositionY_;
};

} // namespace mtmotor_jig
// Copyright (C) 2025 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file display_manager.cpp
/// @brief Class that handles visual output to the display.

#include "display_manager.h"

#include <Arduino.h>
#include <LiquidCrystal.h>

#include "common_types.h"
#include "configuration.h"

namespace mtmotor_jig {

DisplayManager::DisplayManager() {}

DisplayManager::~DisplayManager() {}

void DisplayManager::Begin() {
  lcd_.begin(configuration_.kDisplayWidth_, configuration_.kDisplayHeight_);
  lcd_.blink(); // Blink the display cursor.
  //lcd_.cursor(); // Show a static display cursor.
}

void DisplayManager::Draw(common::ControlMode control_mode, common::ControlAction control_action,
                          const String& status) {
  switch (control_mode) {
    case common::ControlMode::kSplashScreen: {
      DrawScreenItems(configuration_.kSplashScreenMenuItems_);
      delay(configuration_.kSplashScreenDelay_ms_);
      break;
    }
    case common::ControlMode::kHomeScreen: {
      DrawScreenItems(configuration_.kHomeScreenMenuItems_);
      break;
    }
    case common::ControlMode::kContinuousMenu: {
      cursor_position_y_ = configuration_.kContinuousMenuCursorPositionY_;
      break;
    }
    case common::ControlMode::kOscillateMenu: {
      cursor_position_y_ = configuration_.kOscillateMenuCursorPositionY_;
      break;
    }
  }

  if (control_action != common::ControlAction::kIdle || control_mode == common::ControlMode::kHomeScreen) {
    lcd_.setCursor(0, configuration_.kStatusBarCursorPositionY_);
    lcd_.print(status);
    lcd_.setCursor(0, cursor_position_y_);
  }
}

template <size_t N>
void DisplayManager::DrawScreenItems(const String (&screen_items)[N]) {
  lcd_.clear(); //Clear the display and position the cursor in the upper-left corner (0, 0).

  uint8_t i = 0;
  for (auto screen_item : screen_items) {
    lcd_.setCursor(0, i);
    lcd_.print(screen_item);
    i++;
  }
}

} // namespace mtmotor_jig
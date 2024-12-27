// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file display_manager.cpp
/// @brief Class that handles visual output to the display.

#include "display_manager.h"

#include <Arduino.h>
#include <LiquidCrystal.h>

namespace mtmotor_jig {

DisplayManager::DisplayManager() {}

DisplayManager::~DisplayManager() {}

void DisplayManager::Begin() {
  lcd_.begin(configure_display_.kDisplayWidth_, configure_display_.kDisplayHeight_);
  lcd_.blink(); // Blink the display cursor.
}

void DisplayManager::Draw(Configuration::ControlMode control_mode, const String& status) {
  switch (control_mode) {
    case Configuration::ControlMode::kSplashScreen: {
      DrawScreenItems(configure_display_.kSplashScreenMenuItems_, configure_display_.kSizeOfSplashScreenMenuItems_);
      delay(configure_display_.kSplashScreenDelay_ms_);
      break;
    }
    case Configuration::ControlMode::kHomeScreen: {
      DrawScreenItems(configure_display_.kHomeScreenMenuItems_, configure_display_.kSizeOfHomeScreenMenuItems_);
      break;
    }
    case Configuration::ControlMode::kContinuousMenu: {
      cursor_position_y_ = configure_display_.kContinuousMenuCursorPositionY_;
      break;
    }
    case Configuration::ControlMode::kOscillateMenu: {
      cursor_position_y_ = configure_display_.kOscillateMenuCursorPositionY_;
      break;
    }
    case Configuration::ControlMode::kStatusBar: {
      lcd_.setCursor(0, configure_display_.kStatusBarCursorPositionY_);
      lcd_.print(status);
      break;
    }

    lcd_.setCursor(0, cursor_position_y_);
  }
}

void DisplayManager::DrawScreenItems(const String screen_items[], const uint8_t size_of_screen_items) {
  lcd_.clear(); //Clear the display and position the cursor in the upper-left corner (0, 0).

  for (uint8_t i = 0; i < size_of_screen_items; i++) {
    lcd_.setCursor(0, i);
    lcd_.print(screen_items[i]);
  }
}

} // namespace mtmotor_jig
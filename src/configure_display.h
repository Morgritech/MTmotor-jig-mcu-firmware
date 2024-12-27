// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file configure_display.h
/// @brief Class that sets up display configuration settings, including pin definitions, etc.

#ifndef CONFIGURE_DISPLAY_H_
#define CONFIGURE_DISPLAY_H_

#include <Arduino.h>

namespace mtmotor_jig {

/// @brief The Configure Display class using the singleton pattern i.e., only a single instance can exist.
class ConfigureDisplay {
 public:

  /// @brief Static method to get the single instance.
  /// @return The Configure Display instance. 
  static ConfigureDisplay& GetInstance();

  /// @brief Delete the copy constructor to prevent copying of the single instance.
  ConfigureDisplay(const ConfigureDisplay&) = delete;

  /// @brief Delete the assignment operator to prevent copying of the single instance.
  ConfigureDisplay& operator=(const ConfigureDisplay&) = delete;

  // Display screen properties.
  const uint8_t kDisplayHeight_ = 4; ///< The Display height (Rows).
  const uint8_t kDisplayWidth_ = 20; ///< The Display width (Columns).

  // Display screen items.
  static const uint8_t kSizeOfSplashScreenMenuItems_ = 4; ///< No. of splash screen menu items.
  /// @brief The splash screen menu items.
  const String kSplashScreenMenuItems_[kSizeOfSplashScreenMenuItems_] = {"____MTmotor-jig_____",   // 0
                                                                         "   Test motors &"    ,   // 1
                                                                         "   motor control"    ,   // 2
                                                                         "      devices"        }; // 3
  const uint16_t kSplashScreenDelay_ms_ = 1500; ///< Period of time in milliseconds (ms) to show the splash screen.
  static const uint8_t kSizeOfHomeScreenMenuItems_ = 3; ///< No. of home screen menu items.
  /// @brief The home screen menu items.
  const String kHomeScreenMenuItems_[kSizeOfHomeScreenMenuItems_] = {"__Select_Mode_______",   // 0
                                                                     "> Continuous"        ,   // 1
                                                                     "> Oscillate"          }; // 2
  const uint8_t kContinuousMenuCursorPositionY_ = 1; ///< The cursor position (y-axis) for the continuous menu.
  const uint8_t kOscillateMenuCursorPositionY_ = 2; ///< The cursor position (y-axis) for the oscillate menu.
  const uint8_t kStatusBarCursorPositionY_ = 3; ///< The cursor position (y-axis) for the status bar.

  const uint8_t kDefaultCursorPositionY_ = kContinuousMenuCursorPositionY_; ///< The default/initial cursor position (y-axis).

 private:

  /// @brief Private constructor so objects cannot be manually instantiated. 
  ConfigureDisplay();

  /// @brief Private destructor so objects cannot be manually instantiated. 
  ~ConfigureDisplay();
};

} // namespace mtmotor_jig

#endif // CONFIGURE_DISPLAY_H_
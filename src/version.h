// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file version.h
/// @brief Variables to define the firmware name and version.

#ifndef VERSION_H_
#define VERSION_H_

#include <Arduino.h>

namespace mtmotor_jig {

inline constexpr char* kName = "mtmotor-jig-mcu-firmware-";
inline constexpr uint16_t kMajor = 0;
inline constexpr uint16_t kMinor = 1;
inline constexpr uint16_t kPatch = 0;
inline constexpr char* kSuffix = "";

} // namespace mtmotor_jig

#endif // VERSION_H_
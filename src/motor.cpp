/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/device/motor.hpp"

namespace okapi {
inline namespace literals {
pros::Motor operator"" _m(const unsigned long long iport) {
  return pros::Motor(static_cast<uint8_t>(iport));
}

pros::Motor operator"" _rm(const unsigned long long iport) {
  return pros::Motor(static_cast<uint8_t>(iport));
}
}
}
/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/device/motor/motor.hpp"

namespace okapi {
void okapi::Motor::controllerSet(const double ivalue) {
  move_velocity(ivalue);
}

IntegratedEncoder okapi::Motor::getEncoder() const {
  return IntegratedEncoder(*this);
}

inline namespace literals {
okapi::Motor operator"" _m(const unsigned long long iport) {
  return okapi::Motor(static_cast<uint8_t>(iport));
}

okapi::Motor operator"" _rm(const unsigned long long iport) {
  return okapi::Motor(static_cast<uint8_t>(iport));
}
} // namespace literals
} // namespace okapi

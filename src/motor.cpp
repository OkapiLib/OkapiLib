/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/device/motor.hpp"

namespace okapi {
Motor::Motor(uint8_t port, const bool reverse, motor_encoder_units_e_t encoder_units,
             motor_gearset_e_t gearset)
  : motor(port, reverse, encoder_units, gearset) {
}

int32_t Motor::set_velocity(const int16_t ivelocity) const {
  // return motor.set_velocity(ivelocity);
}

inline namespace literals {
Motor operator"" _m(const unsigned long long iport) {
  return Motor(static_cast<uint8_t>(iport));
}

Motor operator"" _rm(const unsigned long long iport) {
  return Motor(static_cast<uint8_t>(iport));
}
} // namespace literals
} // namespace okapi

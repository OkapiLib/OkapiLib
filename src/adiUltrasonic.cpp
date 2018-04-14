/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/device/adiUltrasonic.hpp"

namespace okapi {
ADIUltrasonic::ADIUltrasonic(const uint8_t iportTop, const uint8_t iportBottom)
  : ultra(iportBottom, iportTop) {
}

ADIUltrasonic::~ADIUltrasonic() = default;

int32_t ADIUltrasonic::get() {
  return filter.filter(ultra.get_value());
}

double ADIUltrasonic::controllerGet() {
  return get();
}
} // namespace okapi

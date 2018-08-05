/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"

namespace okapi {
Potentiometer::Potentiometer(const std::uint8_t iport) : pot(iport) {
}

Potentiometer::~Potentiometer() = default;

double Potentiometer::get() const {
  return pot.get_value();
}

double Potentiometer::controllerGet() {
  return get();
}
} // namespace okapi

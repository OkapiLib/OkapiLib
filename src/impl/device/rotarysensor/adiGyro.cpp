/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/adiGyro.hpp"

namespace okapi {
ADIGyro::ADIGyro(const std::uint8_t iport, const double imultiplier) : gyro(iport, imultiplier) {
}

ADIGyro::~ADIGyro() = default;

double ADIGyro::get() const {
  return gyro.get_value();
}

std::int32_t ADIGyro::reset() {
  return gyro.reset();
}

double ADIGyro::controllerGet() {
  return get();
}
} // namespace okapi

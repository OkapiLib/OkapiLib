/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/rotation.hpp"

namespace okapi {
RotationSensor::RotationSensor(const std::uint8_t iport, const bool ireversed) {
  port = iport;
  pros::c::rotation_set_reversed(port, ireversed);
}

double RotationSensor::get() const {
  return pros::c::rotation_get_position(port);
}

double RotationSensor::getVelocity() const {
  return pros::c::rotation_get_velocity(port);
}

std::int32_t RotationSensor::reset() {
  return pros::c::rotation_reset(port);
}

double RotationSensor::controllerGet() {
  return get();
}
} // namespace okapi

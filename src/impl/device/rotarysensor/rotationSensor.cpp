/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"

namespace okapi {
RotationSensor::RotationSensor(const std::uint8_t iport, const bool ireversed)
  : port(iport), reversed(ireversed ? -1 : 1) {
}

double RotationSensor::get() const {
  const double out = pros::c::rotation_get_position(port);
  if (out == PROS_ERR_F) {
    return PROS_ERR_F;
  } else {
    // Convert from centidegrees to degrees
    return out * 0.01 * reversed;
  }
}

double RotationSensor::getVelocity() const {
  const double out = pros::c::rotation_get_velocity(port);
  if (out == PROS_ERR_F) {
    return PROS_ERR_F;
  } else {
    // Convert from centidegrees per second to degrees per second
    return out * 0.01 * reversed;
  }
}

std::int32_t RotationSensor::reset() {
  return pros::c::rotation_reset(port);
}

double RotationSensor::controllerGet() {
  return get();
}
} // namespace okapi

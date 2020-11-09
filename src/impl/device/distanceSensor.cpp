/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/distanceSensor.hpp"

namespace okapi {
DistanceSensor::DistanceSensor(std::uint8_t iport, std::unique_ptr<Filter> ifilter)
  : filter(std::move(ifilter)) {
  port = iport;
}

double DistanceSensor::get() {
  return filter->filter(pros::c::distance_get(port));
}

double DistanceSensor::controllerGet() {
  return get();
}

int32_t DistanceSensor::getConfidence() const {
  return pros::c::distance_get_confidence(port);
}

int32_t DistanceSensor::getObjectSize() const {
  return pros::c::distance_get_object_size(port);
}

double DistanceSensor::getObjectVelocity() const {
  return filter->filter(pros::c::distance_get_object_velocity(port));
}

} // namespace okapi

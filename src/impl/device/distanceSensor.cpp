/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/distanceSensor.hpp"

namespace okapi {
DistanceSensor::DistanceSensor(std::uint8_t iport, std::unique_ptr<Filter> ifilter)
  : d(iport), filter(std::move(ifilter)) {
}

DistanceSensor::~DistanceSensor() = default;

double DistanceSensor::get() {
  return filter->filter(d.get());
}

double DistanceSensor::controllerGet() {
  return get();
}

int32_t DistanceSensor::get_confidence() {
  return d.get_confidence();
}

int32_t DistanceSensor::get_object_size() {
  return d.get_object_size();
}

double DistanceSensor::get_object_velocity() {
  return filter->filter(d.get_object_velocity());
}

} // namespace okapi

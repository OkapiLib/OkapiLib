/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/adiUltrasonic.hpp"

namespace okapi {
ADIUltrasonic::ADIUltrasonic(const std::uint8_t iportPing,
                             const std::uint8_t iportEcho,
                             std::unique_ptr<Filter> ifilter)
  : ultra(pros::c::adi_ultrasonic_init(iportPing, iportEcho)), filter(std::move(ifilter)) {
}

ADIUltrasonic::~ADIUltrasonic() = default;

double ADIUltrasonic::get() {
  return filter->filter(pros::c::adi_ultrasonic_get(ultra));
}

double ADIUltrasonic::controllerGet() {
  return get();
}
} // namespace okapi

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
  : ADIUltrasonic(iportTop, iportBottom, std::make_unique<MedianFilter<5>>()) {
}

ADIUltrasonic::ADIUltrasonic(const std::uint8_t iportTop, const std::uint8_t iportBottom,
                             std::unique_ptr<Filter> ifilter)
  : ultra(iportBottom, iportTop), filter(std::move(ifilter)) {
}

ADIUltrasonic::~ADIUltrasonic() = default;

std::int32_t ADIUltrasonic::get() {
  return static_cast<std::int32_t>(filter->filter(ultra.get_value()));
}

double ADIUltrasonic::controllerGet() {
  return get();
}
} // namespace okapi

/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/device/rotarysensor/adiEncoder.hpp"

namespace okapi {
ADIEncoder::ADIEncoder(const std::uint8_t iportTop, const std::uint8_t iportBottom,
                       const bool ireversed)
  : enc(iportTop, iportBottom, ireversed) {
}

std::int32_t ADIEncoder::get() const {
  return enc.get_value();
}

std::int32_t ADIEncoder::reset() const {
  return enc.reset();
}

double ADIEncoder::controllerGet() {
  return get();
}
} // namespace okapi

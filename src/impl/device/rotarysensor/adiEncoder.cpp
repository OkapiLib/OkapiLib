/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"

namespace okapi {
ADIEncoder::ADIEncoder(const std::uint8_t iportTop,
                       const std::uint8_t iportBottom,
                       const bool ireversed)
  : ADIEncoder({INTERNAL_ADI_PORT, iportTop, iportBottom}, ireversed) {
}

ADIEncoder::ADIEncoder(std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> iports,
                       const bool ireversed)
  : enc(pros::c::ext_adi_encoder_init(std::get<0>(iports),
                                      std::get<1>(iports),
                                      std::get<2>(iports),
                                      ireversed)) {
}

double ADIEncoder::get() const {
  return static_cast<double>(pros::c::ext_adi_encoder_get(enc));
}

std::int32_t ADIEncoder::reset() {
  return pros::c::ext_adi_encoder_reset(enc);
}

double ADIEncoder::controllerGet() {
  return get();
}
} // namespace okapi

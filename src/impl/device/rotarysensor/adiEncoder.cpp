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
  : ADIEncoder(INTERNAL_ADI_PORT, iportTop, iportBottom, ireversed) {
}

ADIEncoder::ADIEncoder(const std::uint8_t ismartPort,
                       const std::uint8_t iportTop,
                       const std::uint8_t iportBottom,
                       const bool ireversed,
                       const std::shared_ptr<Logger> &logger) {
  std::int8_t transformedPortBottom = transformADIPort(iportBottom);
  if (transformedPortBottom == 1 || transformedPortBottom == 3 || transformedPortBottom == 5 ||
      transformedPortBottom == 7) {
    // Prevent the user from doing this and not noticing: auto enc = ADIEncoder(1, 'A', 'B');
    std::string msg =
      "ADIEncoder: The value for the bottom port (" + std::to_string(transformedPortBottom) +
      ") is outside the expected range (2, 4, 6, 8). Did you accidentally construct an "
      "ADIEncoder on a smart port without specifying whether it is reversed?";
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  enc = pros::c::ext_adi_encoder_init(ismartPort, iportTop, iportBottom, ireversed);
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

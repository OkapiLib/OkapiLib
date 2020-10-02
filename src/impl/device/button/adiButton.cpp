/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/button/adiButton.hpp"

namespace okapi {
ADIButton::ADIButton(const uint8_t iport, const bool iinverted)
  : ADIButton({INTERNAL_ADI_PORT, iport}, iinverted) {
}

ADIButton::ADIButton(std::pair<std::uint8_t, std::uint8_t> iports, const bool iinverted)
  : ButtonBase(iinverted), smartPort(std::get<0>(iports)), port(std::get<1>(iports)) {
  pros::c::ext_adi_port_set_config(smartPort, port, pros::E_ADI_DIGITAL_IN);
}

bool ADIButton::currentlyPressed() {
  const std::int32_t state = pros::c::ext_adi_digital_read(smartPort, port);
  const bool pressed = state != 0 && state != PROS_ERR;
  if (inverted) {
    return !pressed;
  } else {
    return pressed;
  }
}
} // namespace okapi

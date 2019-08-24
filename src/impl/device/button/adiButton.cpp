/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/button/adiButton.hpp"

namespace okapi {
ADIButton::ADIButton(const uint8_t iport, const bool iinverted)
  : ButtonBase(iinverted), port(iport) {
  pros::c::adi_port_set_config(port, pros::E_ADI_DIGITAL_IN);
}

bool ADIButton::currentlyPressed() {
  const std::int32_t state = pros::c::adi_digital_read(port);
  const bool pressed = state != 0 && state != PROS_ERR;
  return inverted ? !pressed : pressed;
}
} // namespace okapi

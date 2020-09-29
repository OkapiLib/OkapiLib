/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"

namespace okapi {
Potentiometer::Potentiometer(const std::uint8_t iport) : Potentiometer({INTERNAL_ADI_PORT, iport}) {
}

Potentiometer::Potentiometer(std::pair<std::uint8_t, std::uint8_t> iports)
  : smartPort(std::get<0>(iports)), port(std::get<1>(iports)) {
  pros::c::ext_adi_port_set_config(smartPort, port, pros::E_ADI_ANALOG_IN);
}

double Potentiometer::get() const {
  return pros::c::ext_adi_analog_read(smartPort, port);
}

double Potentiometer::controllerGet() {
  return get();
}
} // namespace okapi

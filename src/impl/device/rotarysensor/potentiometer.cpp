/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"

namespace okapi {
Potentiometer::Potentiometer(const std::uint8_t iport) : port(iport) {
  pros::c::adi_port_set_config(port, pros::E_ADI_ANALOG_IN);
}

Potentiometer::~Potentiometer() = default;

double Potentiometer::get() const {
  return pros::c::adi_analog_read(port);
}

double Potentiometer::controllerGet() {
  return get();
}
} // namespace okapi

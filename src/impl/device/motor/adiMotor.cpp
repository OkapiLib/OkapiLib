/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/motor/adiMotor.hpp"

namespace okapi {
ADIMotor::ADIMotor(const std::uint8_t iport, const bool ireverse)
  : port(iport), reversed(ireverse ? -1 : 1) {
}

void ADIMotor::moveVoltage(const std::int8_t ivoltage) const {
  pros::c::adi_motor_set(port, ivoltage * reversed);
}

void ADIMotor::controllerSet(const double ivalue) {
  pros::c::adi_motor_set(port, ivalue * reversed * 127);
}
} // namespace okapi

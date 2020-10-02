/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"

namespace okapi {
IntegratedEncoder::IntegratedEncoder(const okapi::Motor &imotor)
  : IntegratedEncoder(imotor.getPort(), imotor.isReversed()) {
}

IntegratedEncoder::IntegratedEncoder(const std::int8_t iport, const bool ireversed)
  : port(iport), reversed(ireversed ? -1 : 1) {
}

double IntegratedEncoder::get() const {
  return pros::c::motor_get_position(port) * reversed;
}

std::int32_t IntegratedEncoder::reset() {
  return pros::c::motor_tare_position(port);
}

double IntegratedEncoder::controllerGet() {
  return get();
}
} // namespace okapi

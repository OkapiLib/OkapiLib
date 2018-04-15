/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/device/rotarysensor/integratedEncoder.hpp"

namespace okapi {
IntegratedEncoder::IntegratedEncoder(pros::Motor imotor) : motor(imotor) {
}

std::int32_t IntegratedEncoder::get() const {
  return motor.get_position();
}

std::int32_t IntegratedEncoder::reset() const {
  return motor.tare_position();
}

double IntegratedEncoder::controllerGet() {
  return get();
}
} // namespace okapi

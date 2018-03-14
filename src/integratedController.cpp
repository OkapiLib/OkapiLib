/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/integratedController.hpp"

namespace okapi {
IntegratedControllerParams::IntegratedControllerParams(const AbstractMotor &imotor)
  : motor(imotor) {
}

IntegratedController::IntegratedController(const AbstractMotor &imotor) : motor(imotor) {
}

IntegratedController::IntegratedController(const IntegratedControllerParams &iparams)
  : motor(iparams.motor) {
}

IntegratedController::~IntegratedController() = default;

int32_t IntegratedController::moveAbsolute(const double position, const int32_t velocity) const {
  return motor.move_absolute(position, velocity);
}

int32_t IntegratedController::moveRelative(const double position, const int32_t velocity) const {
  return motor.move_relative(position, velocity);
}

int32_t IntegratedController::moveVelocity(const int16_t velocity) const {
  return motor.move_velocity(velocity);
}

int32_t IntegratedController::moveVoltage(const int16_t voltage) const {
  return motor.move_voltage(voltage);
}
} // namespace okapi

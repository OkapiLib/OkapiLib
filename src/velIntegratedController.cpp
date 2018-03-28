/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/velIntegratedController.hpp"

namespace okapi {
VelIntegratedControllerParams::VelIntegratedControllerParams(const AbstractMotor &imotor)
  : motor(imotor) {
}

VelIntegratedControllerParams::~VelIntegratedControllerParams() = default;

VelIntegratedController::VelIntegratedController(const AbstractMotor &imotor) : motor(imotor) {
}

VelIntegratedController::VelIntegratedController(const VelIntegratedControllerParams &iparams)
  : motor(iparams.motor) {
}

VelIntegratedController::~VelIntegratedController() = default;

void VelIntegratedController::setTarget(const double itarget) {
  motor.move_velocity(itarget);
  lastTarget = itarget;
}

double VelIntegratedController::getError() const {
  return lastTarget - motor.get_actual_velocity();
}

void VelIntegratedController::reset() {
  // Don't have to do anything
}
} // namespace okapi

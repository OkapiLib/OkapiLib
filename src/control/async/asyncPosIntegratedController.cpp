/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncPosIntegratedController.hpp"

namespace okapi {
AsyncPosIntegratedControllerArgs::AsyncPosIntegratedControllerArgs(const AbstractMotor &imotor)
  : motor(imotor) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(const AbstractMotor &imotor)
  : motor(imotor) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(
  const AsyncPosIntegratedControllerArgs &iparams)
  : motor(iparams.motor) {
}

void AsyncPosIntegratedController::setTarget(const double itarget) {
  motor.moveAbsolute(itarget + offset, 100);
  lastTarget = itarget;
}

double AsyncPosIntegratedController::getError() const {
  return lastTarget - motor.getPosition();
}

bool AsyncPosIntegratedController::isSettled() {
  return settledUtil.isSettled(getError());
}

void AsyncPosIntegratedController::reset() {
  // Save the current target as an offset for the new targets so the current target becomes a
  // "zero position"
  offset = lastTarget;
}
} // namespace okapi

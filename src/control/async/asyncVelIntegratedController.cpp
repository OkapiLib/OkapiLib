/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncVelIntegratedController.hpp"

namespace okapi {
AsyncVelIntegratedControllerArgs::AsyncVelIntegratedControllerArgs(
  std::shared_ptr<AbstractMotor> imotor)
  : motor(imotor) {
}

AsyncVelIntegratedController::AsyncVelIntegratedController(Motor imotor)
  : AsyncVelIntegratedController(std::make_shared<Motor>(imotor)) {
}

AsyncVelIntegratedController::AsyncVelIntegratedController(MotorGroup imotor)
  : AsyncVelIntegratedController(std::make_shared<MotorGroup>(imotor)) {
}

AsyncVelIntegratedController::AsyncVelIntegratedController(std::shared_ptr<AbstractMotor> imotor)
  : motor(imotor) {
}

AsyncVelIntegratedController::AsyncVelIntegratedController(
  const AsyncVelIntegratedControllerArgs &iparams)
  : motor(iparams.motor) {
}

void AsyncVelIntegratedController::setTarget(const double itarget) {
  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->moveVelocity(itarget);
  }

  lastTarget = itarget;
}

double AsyncVelIntegratedController::getError() const {
  return lastTarget - motor->getActualVelocity();
}

bool AsyncVelIntegratedController::isSettled() {
  return settledUtil.isSettled(getError());
}

void AsyncVelIntegratedController::reset() {
  hasFirstTarget = false;
}

void AsyncVelIntegratedController::flipDisable() {
  controllerIsDisabled = !controllerIsDisabled;
  resumeMovement();
}

void AsyncVelIntegratedController::flipDisable(const bool iisDisabled) {
  controllerIsDisabled = iisDisabled;
  resumeMovement();
}

bool AsyncVelIntegratedController::isDisabled() const {
  return controllerIsDisabled;
}

void AsyncVelIntegratedController::resumeMovement() {
  if (controllerIsDisabled) {
    motor->moveVoltage(0);
  } else {
    if (hasFirstTarget) {
      setTarget(lastTarget);
    }
  }
}
} // namespace okapi

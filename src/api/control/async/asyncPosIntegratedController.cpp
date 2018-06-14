/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"

namespace okapi {
AsyncPosIntegratedControllerArgs::AsyncPosIntegratedControllerArgs(
  std::shared_ptr<AbstractMotor> imotor)
  : motor(imotor) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(Motor imotor)
  : AsyncPosIntegratedController(std::make_shared<Motor>(imotor)) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(MotorGroup imotor)
  : AsyncPosIntegratedController(std::make_shared<MotorGroup>(imotor)) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(std::shared_ptr<AbstractMotor> imotor)
  : motor(imotor) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(
  const AsyncPosIntegratedControllerArgs &iparams)
  : motor(iparams.motor) {
}

void AsyncPosIntegratedController::setTarget(const double itarget) {
  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->moveAbsolute(itarget, 127);
  }

  lastTarget = itarget;
}

double AsyncPosIntegratedController::getError() const {
  return lastTarget - motor->getPosition();
}

bool AsyncPosIntegratedController::isSettled() {
  return settledUtil.isSettled(getError());
}

void AsyncPosIntegratedController::reset() {
  hasFirstTarget = false;
  settledUtil.reset();
}

void AsyncPosIntegratedController::flipDisable() {
  controllerIsDisabled = !controllerIsDisabled;
  resumeMovement();
}

void AsyncPosIntegratedController::flipDisable(const bool iisDisabled) {
  controllerIsDisabled = iisDisabled;
  resumeMovement();
}

bool AsyncPosIntegratedController::isDisabled() const {
  return controllerIsDisabled;
}

void AsyncPosIntegratedController::resumeMovement() {
  if (controllerIsDisabled) {
    motor->moveVoltage(0);
  } else {
    if (hasFirstTarget) {
      setTarget(lastTarget);
    }
  }
}
} // namespace okapi

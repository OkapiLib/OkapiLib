/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
AsyncVelIntegratedController::AsyncVelIntegratedController(
  const std::shared_ptr<AbstractMotor> &imotor,
  const TimeUtil &itimeUtil)
  : motor(imotor), settledUtil(itimeUtil.getSettledUtil()), rate(itimeUtil.getRate()) {
}

void AsyncVelIntegratedController::setTarget(const double itarget) {
  logger->info("AsyncVelIntegratedController: Set target to " + std::to_string(itarget));

  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->moveVelocity((std::int16_t)itarget);
  }

  lastTarget = itarget;
}

double AsyncVelIntegratedController::getTarget() {
  return lastTarget;
}

double AsyncVelIntegratedController::getError() const {
  return lastTarget - motor->getActualVelocity();
}

bool AsyncVelIntegratedController::isSettled() {
  return isDisabled() || settledUtil->isSettled(getError());
}

void AsyncVelIntegratedController::reset() {
  logger->info("AsyncVelIntegratedController: Reset");
  hasFirstTarget = false;
  settledUtil->reset();
}

void AsyncVelIntegratedController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void AsyncVelIntegratedController::flipDisable(const bool iisDisabled) {
  logger->info("AsyncVelIntegratedController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
  resumeMovement();
}

bool AsyncVelIntegratedController::isDisabled() const {
  return controllerIsDisabled;
}

void AsyncVelIntegratedController::resumeMovement() {
  if (isDisabled()) {
    motor->moveVelocity(0);
  } else {
    if (hasFirstTarget) {
      setTarget(lastTarget);
    }
  }
}

void AsyncVelIntegratedController::waitUntilSettled() {
  logger->info("AsyncVelIntegratedController: Waiting to settle");
  while (!isSettled()) {
    rate->delayUntil(motorUpdateRate);
  }
  logger->info("AsyncVelIntegratedController: Done waiting to settle");
}

void AsyncVelIntegratedController::controllerSet(double ivalue) {
  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->controllerSet(ivalue);
  }

  // Need to scale the controller output from [-1, 1] to the range of the motor based on its
  // internal gearset
  lastTarget = ivalue * toUnderlyingType(motor->getGearing());
}
} // namespace okapi

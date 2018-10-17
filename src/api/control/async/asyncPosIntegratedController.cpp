/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
AsyncPosIntegratedController::AsyncPosIntegratedController(std::shared_ptr<AbstractMotor> imotor,
                                                           const TimeUtil &itimeUtil)
  : AsyncPosIntegratedController(imotor, toUnderlyingType(imotor->getGearing()), itimeUtil) {
}

AsyncPosIntegratedController::AsyncPosIntegratedController(std::shared_ptr<AbstractMotor> imotor,
                                                           const std::int32_t imaxVelocity,
                                                           const TimeUtil &itimeUtil)
  : logger(Logger::instance()),
    motor(imotor),
    maxVelocity(imaxVelocity),
    settledUtil(itimeUtil.getSettledUtil()),
    rate(itimeUtil.getRate()) {
}

void AsyncPosIntegratedController::setTarget(const double itarget) {
  logger->info("AsyncPosIntegratedController: Set target to " + std::to_string(itarget));

  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->moveAbsolute(itarget, maxVelocity);
  }

  lastTarget = itarget;
}

double AsyncPosIntegratedController::getTarget() {
  return lastTarget;
}

double AsyncPosIntegratedController::getError() const {
  return lastTarget - motor->getPosition();
}

bool AsyncPosIntegratedController::isSettled() {
  return isDisabled() || settledUtil->isSettled(getError());
}

void AsyncPosIntegratedController::reset() {
  logger->info("AsyncPosIntegratedController: Reset");
  hasFirstTarget = false;
  settledUtil->reset();
}

void AsyncPosIntegratedController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void AsyncPosIntegratedController::flipDisable(const bool iisDisabled) {
  logger->info("AsyncPosIntegratedController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
  resumeMovement();
}

bool AsyncPosIntegratedController::isDisabled() const {
  return controllerIsDisabled;
}

void AsyncPosIntegratedController::resumeMovement() {
  if (isDisabled()) {
    motor->moveVelocity(0);
  } else {
    if (hasFirstTarget) {
      setTarget(lastTarget);
    }
  }
}

void AsyncPosIntegratedController::waitUntilSettled() {
  logger->info("AsyncPosIntegratedController: Waiting to settle");

  while (!isSettled()) {
    rate->delayUntil(motorUpdateRate);
  }

  logger->info("AsyncPosIntegratedController: Done waiting to settle");
}

void AsyncPosIntegratedController::controllerSet(double ivalue) {
  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->controllerSet(ivalue);
  }

  lastTarget = ivalue * toUnderlyingType(motor->getGearing());
}

void AsyncPosIntegratedController::setMaxVelocity(const std::int32_t imaxVelocity) {
  maxVelocity = imaxVelocity;
}
} // namespace okapi

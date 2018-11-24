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
AsyncPosIntegratedController::AsyncPosIntegratedController(
  const std::shared_ptr<AbstractMotor> &imotor,
  const AbstractMotor::GearsetRatioPair &ipair,
  const std::int32_t imaxVelocity,
  const TimeUtil &itimeUtil)
  : logger(Logger::instance()),
    motor(imotor),
    pair(ipair),
    maxVelocity(imaxVelocity),
    settledUtil(itimeUtil.getSettledUtil()),
    rate(itimeUtil.getRate()) {
  if (ipair.ratio == 0) {
    logger->error("AsyncPosIntegratedController: The gear ratio cannot be zero! Check if you are "
                  "using integer division.");
    throw std::invalid_argument("AsyncPosIntegratedController: The gear ratio cannot be zero! "
                                "Check if you are using integer division.");
  }

  motor->setGearing(ipair.internalGearset);
}

void AsyncPosIntegratedController::setTarget(const double itarget) {
  logger->info("AsyncPosIntegratedController: Set target to " + std::to_string(itarget));

  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->moveAbsolute(itarget * pair.ratio + offset, maxVelocity);
  }

  lastTarget = itarget;
}

double AsyncPosIntegratedController::getTarget() {
  return lastTarget;
}

double AsyncPosIntegratedController::getError() const {
  return (lastTarget * pair.ratio + offset) - motor->getPosition();
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
    stop();
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

  // Need to scale the controller output from [-1, 1] to the range of the motor based on its
  // internal gearset
  lastTarget = ivalue * toUnderlyingType(motor->getGearing());
}

void AsyncPosIntegratedController::setMaxVelocity(const std::int32_t imaxVelocity) {
  maxVelocity = imaxVelocity;
}

void AsyncPosIntegratedController::tarePosition() {
  offset = motor->getPosition();
}

void AsyncPosIntegratedController::stop() {
  motor->moveVelocity(0);
}
} // namespace okapi

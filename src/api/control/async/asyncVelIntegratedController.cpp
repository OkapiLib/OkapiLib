/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
AsyncVelIntegratedController::AsyncVelIntegratedController(
  const std::shared_ptr<AbstractMotor> &imotor,
  const AbstractMotor::GearsetRatioPair &ipair,
  const std::int32_t imaxVelocity,
  const TimeUtil &itimeUtil,
  const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    timeUtil(itimeUtil),
    motor(imotor),
    pair(ipair),
    maxVelocity(imaxVelocity),
    settledUtil(itimeUtil.getSettledUtil()) {
  if (ipair.ratio == 0) {
    std::string msg("AsyncVelIntegratedController: The gear ratio cannot be zero! Check if you are "
                    "using integer division.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }

  motor->setGearing(ipair.internalGearset);
}

void AsyncVelIntegratedController::setTarget(const double itarget) {
  double boundedTarget = itarget * pair.ratio;

  if (boundedTarget > maxVelocity) {
    boundedTarget = maxVelocity;
  }

  LOG_INFO("AsyncVelIntegratedController: Set target to " + std::to_string(boundedTarget));

  hasFirstTarget = true;

  if (!controllerIsDisabled) {
    motor->moveVelocity(static_cast<int16_t>(boundedTarget));
  }

  lastTarget = itarget;
}

double AsyncVelIntegratedController::getTarget() {
  return lastTarget;
}

double AsyncVelIntegratedController::getProcessValue() const {
  return motor->getActualVelocity();
}

double AsyncVelIntegratedController::getError() const {
  return lastTarget - getProcessValue() / pair.ratio;
}

bool AsyncVelIntegratedController::isSettled() {
  return isDisabled() || settledUtil->isSettled(getError());
}

void AsyncVelIntegratedController::reset() {
  LOG_INFO_S("AsyncVelIntegratedController: Reset");
  hasFirstTarget = false;
  settledUtil->reset();
}

void AsyncVelIntegratedController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void AsyncVelIntegratedController::flipDisable(const bool iisDisabled) {
  LOG_INFO("AsyncVelIntegratedController: flipDisable " + std::to_string(iisDisabled));
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
  LOG_INFO_S("AsyncVelIntegratedController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(motorUpdateRate);
  }

  LOG_INFO_S("AsyncVelIntegratedController: Done waiting to settle");
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

/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
ChassisControllerIntegrated::ChassisControllerIntegrated(
  const TimeUtil &itimeUtil,
  std::shared_ptr<ChassisModel> imodel,
  std::unique_ptr<AsyncPosIntegratedController> ileftController,
  std::unique_ptr<AsyncPosIntegratedController> irightController,
  AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales)
  : ChassisController(imodel, toUnderlyingType(igearset.internalGearset)),
    logger(Logger::instance()),
    rate(itimeUtil.getRate()),
    leftController(std::move(ileftController)),
    rightController(std::move(irightController)),
    lastTarget(0),
    scales(iscales),
    gearsetRatioPair(igearset) {
  if (igearset.ratio == 0) {
    logger->error("ChassisControllerIntegrated: The gear ratio cannot be zero! Check if you are "
                  "using integer division.");
    throw std::invalid_argument("ChassisControllerIntegrated: The gear ratio cannot be zero! Check "
                                "if you are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

void ChassisControllerIntegrated::moveDistance(const QLength itarget) {
  moveDistanceAsync(itarget);
  waitUntilSettled();
}

void ChassisControllerIntegrated::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / scales.straight) * meter);
}

void ChassisControllerIntegrated::moveDistanceAsync(const QLength itarget) {
  logger->info("ChassisControllerIntegrated: moving " + std::to_string(itarget.convert(meter)) +
               " meters");

  leftController->reset();
  rightController->reset();
  leftController->flipDisable(false);
  rightController->flipDisable(false);

  const double newTarget = itarget.convert(meter) * scales.straight * gearsetRatioPair.ratio;

  logger->info("ChassisControllerIntegrated: moving " + std::to_string(newTarget) +
               " motor degrees");

  const auto enc = model->getSensorVals();
  leftController->setTarget(newTarget + enc[0]);
  rightController->setTarget(newTarget + enc[1]);
}

void ChassisControllerIntegrated::moveDistanceAsync(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistanceAsync((itarget / scales.straight) * meter);
}

void ChassisControllerIntegrated::turnAngle(const QAngle idegTarget) {
  turnAngleAsync(idegTarget);
  waitUntilSettled();
}

void ChassisControllerIntegrated::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / scales.turn) * degree);
}

void ChassisControllerIntegrated::turnAngleAsync(const QAngle idegTarget) {
  logger->info("ChassisControllerIntegrated: turning " +
               std::to_string(idegTarget.convert(degree)) + " degrees");

  leftController->reset();
  rightController->reset();
  leftController->flipDisable(false);
  rightController->flipDisable(false);

  const double newTarget = idegTarget.convert(degree) * scales.turn * gearsetRatioPair.ratio;

  logger->info("ChassisControllerIntegrated: turning " + std::to_string(newTarget) +
               " motor degrees");

  const auto enc = model->getSensorVals();
  leftController->setTarget(newTarget + enc[0]);
  rightController->setTarget(-1 * newTarget + enc[1]);
}

void ChassisControllerIntegrated::turnAngleAsync(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngleAsync((idegTarget / scales.turn) * degree);
}

void ChassisControllerIntegrated::waitUntilSettled() {
  logger->info("ChassisControllerIntegrated: Waiting to settle");
  while (!(leftController->isSettled() && rightController->isSettled())) {
    rate->delayUntil(10_ms);
  }

  leftController->flipDisable(true);
  rightController->flipDisable(true);
  model->stop();

  logger->info("ChassisControllerIntegrated: Done waiting to settle");
}

void ChassisControllerIntegrated::stop() {
  leftController->flipDisable(true);
  rightController->flipDisable(true);

  ChassisController::stop();
}

void ChassisControllerIntegrated::setMaxVelocity(const double imaxVelocity) {
  leftController->setMaxVelocity(imaxVelocity);
  rightController->setMaxVelocity(imaxVelocity);
  ChassisController::setMaxVelocity(imaxVelocity);
}

ChassisScales ChassisControllerIntegrated::getChassisScales() const {
  return scales;
}

AbstractMotor::GearsetRatioPair ChassisControllerIntegrated::getGearsetRatioPair() const {
  return gearsetRatioPair;
}
} // namespace okapi

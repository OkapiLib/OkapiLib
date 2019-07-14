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
  std::shared_ptr<ChassisModel> ichassisModel,
  std::unique_ptr<AsyncPosIntegratedController> ileftController,
  std::unique_ptr<AsyncPosIntegratedController> irightController,
  const AbstractMotor::GearsetRatioPair &igearset,
  const ChassisScales &iscales,
  std::shared_ptr<Logger> ilogger)
  : logger(std::move(ilogger)),
    chassisModel(std::move(ichassisModel)),
    timeUtil(itimeUtil),
    leftController(std::move(ileftController)),
    rightController(std::move(irightController)),
    lastTarget(0),
    scales(iscales),
    gearsetRatioPair(igearset) {
  if (igearset.ratio == 0) {
    std::string msg("ChassisControllerIntegrated: The gear ratio cannot be zero! Check if you are "
                    "using integer division.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }

  chassisModel->setGearing(igearset.internalGearset);
  chassisModel->setEncoderUnits(AbstractMotor::encoderUnits::counts);
  leftController->setMaxVelocity(chassisModel->getMaxVelocity());
  rightController->setMaxVelocity(chassisModel->getMaxVelocity());
}

void ChassisControllerIntegrated::moveDistance(const QLength itarget) {
  moveDistanceAsync(itarget);
  waitUntilSettled();
}

void ChassisControllerIntegrated::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor ticks
  moveDistance((itarget / scales.straight) * meter);
}

void ChassisControllerIntegrated::moveDistanceAsync(const QLength itarget) {
  LOG_INFO("ChassisControllerIntegrated: moving " + std::to_string(itarget.convert(meter)) +
           " meters");

  leftController->reset();
  rightController->reset();
  leftController->flipDisable(false);
  rightController->flipDisable(false);

  const double newTarget = itarget.convert(meter) * scales.straight * gearsetRatioPair.ratio;

  LOG_INFO("ChassisControllerIntegrated: moving " + std::to_string(newTarget) + " motor ticks");

  const auto enc = chassisModel->getSensorVals();
  leftController->setTarget(newTarget + enc[0]);
  rightController->setTarget(newTarget + enc[1]);
}

void ChassisControllerIntegrated::moveDistanceAsync(const double itarget) {
  // Divide by straightScale so the final result turns back into motor ticks
  moveDistanceAsync((itarget / scales.straight) * meter);
}

void ChassisControllerIntegrated::turnAngle(const QAngle idegTarget) {
  turnAngleAsync(idegTarget);
  waitUntilSettled();
}

void ChassisControllerIntegrated::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor ticks
  turnAngle((idegTarget / scales.turn) * degree);
}

void ChassisControllerIntegrated::turnAngleAsync(const QAngle idegTarget) {
  LOG_INFO("ChassisControllerIntegrated: turning " + std::to_string(idegTarget.convert(degree)) +
           " degrees");

  leftController->reset();
  rightController->reset();
  leftController->flipDisable(false);
  rightController->flipDisable(false);

  const double newTarget =
    idegTarget.convert(degree) * scales.turn * gearsetRatioPair.ratio * boolToSign(normalTurns);

  LOG_INFO("ChassisControllerIntegrated: turning " + std::to_string(newTarget) + " motor ticks");

  const auto enc = chassisModel->getSensorVals();
  leftController->setTarget(newTarget + enc[0]);
  rightController->setTarget(-1 * newTarget + enc[1]);
}

void ChassisControllerIntegrated::turnAngleAsync(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor ticks
  turnAngleAsync((idegTarget / scales.turn) * degree);
}

void ChassisControllerIntegrated::setTurnsMirrored(const bool ishouldMirror) {
  normalTurns = !ishouldMirror;
}

void ChassisControllerIntegrated::waitUntilSettled() {
  LOG_INFO(std::string("ChassisControllerIntegrated: Waiting to settle"));

  auto rate = timeUtil.getRate();
  while (!(leftController->isSettled() && rightController->isSettled())) {
    rate->delayUntil(10_ms);
  }

  leftController->flipDisable(true);
  rightController->flipDisable(true);
  chassisModel->stop();

  LOG_INFO(std::string("ChassisControllerIntegrated: Done waiting to settle"));
}

void ChassisControllerIntegrated::stop() {
  LOG_INFO(std::string("ChassisControllerIntegrated: Stopping"));

  leftController->flipDisable(true);
  rightController->flipDisable(true);
  chassisModel->stop();
}

ChassisScales ChassisControllerIntegrated::getChassisScales() const {
  return scales;
}

AbstractMotor::GearsetRatioPair ChassisControllerIntegrated::getGearsetRatioPair() const {
  return gearsetRatioPair;
}

std::shared_ptr<ChassisModel> ChassisControllerIntegrated::getModel() {
  return chassisModel;
}

ChassisModel &ChassisControllerIntegrated::model() {
  return *chassisModel;
}
void ChassisControllerIntegrated::setMaxVelocity(double imaxVelocity) {
  leftController->setMaxVelocity(imaxVelocity);
  rightController->setMaxVelocity(imaxVelocity);
  chassisModel->setMaxVelocity(imaxVelocity);
}
double ChassisControllerIntegrated::getMaxVelocity() const {
  return chassisModel->getMaxVelocity();
}
} // namespace okapi

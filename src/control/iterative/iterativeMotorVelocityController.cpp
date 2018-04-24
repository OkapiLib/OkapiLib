/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/iterative/iterativeMotorVelocityController.hpp"

namespace okapi {
IterativeMotorVelocityControllerArgs::IterativeMotorVelocityControllerArgs(
  std::shared_ptr<AbstractMotor> imotor, std::shared_ptr<IterativeVelocityController> icontroller)
  : motor(imotor), controller(icontroller) {
}

IterativeMotorVelocityController::IterativeMotorVelocityController(
  Motor imotor, std::shared_ptr<IterativeVelocityController> icontroller)
  : IterativeMotorVelocityController(std::make_shared<Motor>(imotor), icontroller) {
}

IterativeMotorVelocityController::IterativeMotorVelocityController(
  MotorGroup imotor, std::shared_ptr<IterativeVelocityController> icontroller)
  : IterativeMotorVelocityController(std::make_shared<MotorGroup>(imotor), icontroller) {
}

IterativeMotorVelocityController::IterativeMotorVelocityController(
  std::shared_ptr<AbstractMotor> imotor, std::shared_ptr<IterativeVelocityController> icontroller)
  : motor(imotor), controller(icontroller) {
}

IterativeMotorVelocityController::IterativeMotorVelocityController(
  const IterativeMotorVelocityControllerArgs &iparams)
  : motor(iparams.motor), controller(iparams.controller) {
}

double IterativeMotorVelocityController::step(const double ireading) {
  motor->moveVelocity(static_cast<std::int16_t>(controller->step(ireading) * 127));
  return controller->getOutput();
}

void IterativeMotorVelocityController::setTarget(const double itarget) {
  controller->setTarget(itarget);
}

double IterativeMotorVelocityController::getOutput() const {
  return controller->getOutput();
}

double IterativeMotorVelocityController::getError() const {
  return controller->getError();
}

double IterativeMotorVelocityController::getDerivative() const {
  return controller->getDerivative();
}

bool IterativeMotorVelocityController::isSettled() {
  return controller->isSettled();
}

void IterativeMotorVelocityController::setSampleTime(const QTime isampleTime) {
  controller->setSampleTime(isampleTime);
}

void IterativeMotorVelocityController::setOutputLimits(double imax, double imin) {
  controller->setOutputLimits(imax, imin);
}

void IterativeMotorVelocityController::reset() {
  controller->reset();
}

void IterativeMotorVelocityController::flipDisable() {
  controller->flipDisable();
}

void IterativeMotorVelocityController::flipDisable(const bool iisDisabled) {
  controller->flipDisable(iisDisabled);
}

bool IterativeMotorVelocityController::isDisabled() const {
  return controller->isDisabled();
}

QTime IterativeMotorVelocityController::getSampleTime() const {
  return controller->getSampleTime();
}
} // namespace okapi

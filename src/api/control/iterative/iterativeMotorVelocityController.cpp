/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/iterative/iterativeMotorVelocityController.hpp"

namespace okapi {
IterativeMotorVelocityController::IterativeMotorVelocityController(
  std::shared_ptr<AbstractMotor> imotor,
  std::shared_ptr<IterativeVelocityController<double, double>> icontroller)
  : motor(imotor), controller(icontroller) {
}

double IterativeMotorVelocityController::step(const double ireading) {
  motor->controllerSet(controller->step(ireading));
  return controller->getOutput();
}

void IterativeMotorVelocityController::setTarget(const double itarget) {
  controller->setTarget(itarget);
}

void IterativeMotorVelocityController::controllerSet(double ivalue) {
  controller->controllerSet(ivalue);
}

double IterativeMotorVelocityController::getTarget() {
  return controller->getTarget();
}

double IterativeMotorVelocityController::getOutput() const {
  return controller->getOutput();
}

double IterativeMotorVelocityController::getMaxOutput() {
  return controller->getMaxOutput();
}

double IterativeMotorVelocityController::getMinOutput() {
  return controller->getMinOutput();
}

double IterativeMotorVelocityController::getError() const {
  return controller->getError();
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

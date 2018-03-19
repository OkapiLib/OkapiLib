/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/iterative/motorController.hpp"

namespace okapi {
MotorController::MotorController(const AbstractMotor &imotor, IterativeController &iptr)
  : motor(imotor), controller(iptr) {
}

double MotorController::step(const double ireading) {
  controller.step(ireading);
  motor.move_velocity(static_cast<int>(controller.getOutput()));
  return controller.getOutput();
}

void MotorController::setTarget(const double itarget) {
  controller.setTarget(itarget);
}

double MotorController::getOutput() const {
  return controller.getOutput();
}

double MotorController::getError() const {
  return controller.getError();
}

double MotorController::getDerivative() const {
  return controller.getDerivative();
}

void MotorController::setSampleTime(const int isampleTime) {
  controller.setSampleTime(isampleTime);
}

void MotorController::setOutputLimits(double imax, double imin) {
  controller.setOutputLimits(imax, imin);
}

void MotorController::reset() {
  controller.reset();
}

void MotorController::flipDisable() {
  controller.flipDisable();
}

uint32_t MotorController::getSampleTime() const {
  return controller.getSampleTime();
}
} // namespace okapi

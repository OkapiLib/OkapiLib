/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/iterative/iterativeMotorController.hpp"

namespace okapi {
IterativeMotorController::IterativeMotorController(const AbstractMotor &imotor,
                                                   IterativeVelocityController &icontroller)
  : motor(imotor), controller(icontroller) {
}

double IterativeMotorController::step(const double ireading) {
  controller.step(ireading);
  motor.move_velocity(static_cast<int>(controller.getOutput()));
  return controller.getOutput();
}

void IterativeMotorController::setTarget(const double itarget) {
  controller.setTarget(itarget);
}

double IterativeMotorController::getOutput() const {
  return controller.getOutput();
}

double IterativeMotorController::getError() const {
  return controller.getError();
}

double IterativeMotorController::getDerivative() const {
  return controller.getDerivative();
}

void IterativeMotorController::setSampleTime(const uint32_t isampleTime) {
  controller.setSampleTime(isampleTime);
}

void IterativeMotorController::setOutputLimits(double imax, double imin) {
  controller.setOutputLimits(imax, imin);
}

void IterativeMotorController::reset() {
  controller.reset();
}

void IterativeMotorController::flipDisable() {
  controller.flipDisable();
}

uint32_t IterativeMotorController::getSampleTime() const {
  return controller.getSampleTime();
}
} // namespace okapi

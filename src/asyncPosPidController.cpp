/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/async/asyncPosPidController.hpp"

namespace okapi {
AsyncPosPIDControllerParams::AsyncPosPIDControllerParams(const AbstractMotor &imotor,
                                                         const PosPIDControllerParams &iparams)
  : motor(imotor), params(iparams) {
}

AsyncPosPIDController::AsyncPosPIDController(const AbstractMotor &imotor,
                                             const RotarySensor &isensor,
                                             const PosPIDControllerParams &iparams)
  : motor(imotor), sensor(isensor), controller(iparams), prevTime(0), task(trampoline, this) {
}

AsyncPosPIDController::AsyncPosPIDController(const AbstractMotor &imotor,
                                             const RotarySensor &isensor, const double ikP,
                                             const double ikI, const double ikD,
                                             const double ikBias)
  : motor(imotor), sensor(isensor), controller(ikP, ikI, ikD, ikBias), prevTime(0), task(trampoline, this) {
}

AsyncPosPIDController::~AsyncPosPIDController() = default;

void AsyncPosPIDController::step() {
  while (true) {
    motor.move_velocity(controller.step(sensor.get()));
    task.delay_until(&prevTime, controller.getSampleTime());
  }
}

void AsyncPosPIDController::trampoline(void *context) {
  static_cast<AsyncPosPIDController *>(context)->step();
}

/**
 * Sets the target for the controller.
 */
void AsyncPosPIDController::setTarget(const double itarget) {
  controller.setTarget(itarget);
}

/**
 * Returns the last calculated output of the controller. Default is 0.
 */
double AsyncPosPIDController::getOutput() const {
  return controller.getOutput();
}

/**
 * Returns the last error of the controller.
 */
double AsyncPosPIDController::getError() const {
  return controller.getError();
}

/**
 * Set time between loops in ms. Default does nothing.
 *
 * @param isampleTime time between loops in ms
 */
void AsyncPosPIDController::setSampleTime(const uint32_t isampleTime) {
  controller.setSampleTime(isampleTime);
}

/**
 * Set controller output bounds. Default does nothing.
 *
 * @param imax max output
 * @param imin min output
 */
void AsyncPosPIDController::setOutputLimits(double imax, double imin) {
  controller.setOutputLimits(imax, imin);
}

/**
 * Resets the controller so it can start from 0 again properly. Keeps configuration from
 * before.
 */
void AsyncPosPIDController::reset() {
  controller.reset();
}

/**
 * Change whether the controll is off or on. Default does nothing.
 */
void AsyncPosPIDController::flipDisable() {
  controller.flipDisable();
}
} // namespace okapi

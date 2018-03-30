/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncPosPidController.hpp"

namespace okapi {
AsyncPosPIDControllerArgs::AsyncPosPIDControllerArgs(ControllerInput &iinput,
                                                     ControllerOutput &ioutput,
                                                     const PosPIDControllerArgs &iparams)
  : input(iinput), output(ioutput), params(iparams) {
}

AsyncPosPIDController::AsyncPosPIDController(ControllerInput &iinput, ControllerOutput &ioutput,
                                             const PosPIDControllerArgs &iparams)
  : input(iinput), output(ioutput), controller(iparams), task(trampoline, this) {
}

AsyncPosPIDController::AsyncPosPIDController(ControllerInput &iinput, ControllerOutput &ioutput,
                                             const double ikP, const double ikI, const double ikD,
                                             const double ikBias)
  : input(iinput), output(ioutput), controller(ikP, ikI, ikD, ikBias), task(trampoline, this) {
}

void AsyncPosPIDController::step() {
  while (true) {
    output.controllerSet(controller.step(input.controllerGet()));
    task.delay_until(&prevTime, controller.getSampleTime());
  }
}

void AsyncPosPIDController::trampoline(void *context) {
  static_cast<AsyncPosPIDController *>(context)->step();
}

void AsyncPosPIDController::setTarget(const double itarget) {
  controller.setTarget(itarget);
}

double AsyncPosPIDController::getOutput() const {
  return controller.getOutput();
}

double AsyncPosPIDController::getError() const {
  return controller.getError();
}

void AsyncPosPIDController::setSampleTime(const uint32_t isampleTime) {
  controller.setSampleTime(isampleTime);
}

void AsyncPosPIDController::setOutputLimits(double imax, double imin) {
  controller.setOutputLimits(imax, imin);
}

void AsyncPosPIDController::reset() {
  controller.reset();
}

void AsyncPosPIDController::flipDisable() {
  controller.flipDisable();
}
} // namespace okapi

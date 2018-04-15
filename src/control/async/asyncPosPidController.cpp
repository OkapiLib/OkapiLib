/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncPosPidController.hpp"

namespace okapi {
AsyncPosPIDControllerArgs::AsyncPosPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                                                     std::shared_ptr<ControllerOutput> ioutput,
                                                     const IterativePosPIDControllerArgs &iparams)
  : input(iinput), output(ioutput), params(iparams) {
}

AsyncPosPIDController::AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const IterativePosPIDControllerArgs &iparams)
  : input(iinput), output(ioutput), controller(iparams), task(trampoline, this) {
}

AsyncPosPIDController::AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const double ikP, const double ikI, const double ikD,
                                             const double ikBias)
  : input(iinput), output(ioutput), controller(ikP, ikI, ikD, ikBias), task(trampoline, this) {
}

void AsyncPosPIDController::step() {
  std::uint32_t prevTime = 0;

  while (true) {
    if (!controller.isDisabled()) {
      output->controllerSet(controller.step(input->controllerGet()));
    }
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

bool AsyncPosPIDController::isSettled() {
  return controller.isSettled();
}

void AsyncPosPIDController::setSampleTime(const std::uint32_t isampleTime) {
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

void AsyncPosPIDController::flipDisable(const bool iisDisabled) {
  controller.flipDisable(iisDisabled);
}

bool AsyncPosPIDController::isDisabled() const {
  return controller.isDisabled();
}
} // namespace okapi

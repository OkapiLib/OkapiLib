/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncWrapper.hpp"

namespace okapi {
AsyncWrapper::AsyncWrapper(std::shared_ptr<ControllerInput> iinput,
                           std::shared_ptr<ControllerOutput> ioutput,
                           std::unique_ptr<IterativeController> icontroller)
  : input(iinput), output(ioutput), controller(std::move(icontroller)), task(trampoline, this) {
}

void AsyncWrapper::loop() {
  std::uint32_t prevTime = 0;

  while (true) {
    if (!controller->isDisabled()) {
      output->controllerSet(127 * controller->step(input->controllerGet()));
    }

    task.delay_until(&prevTime, controller->getSampleTime().convert(millisecond));
  }
}

void AsyncWrapper::trampoline(void *context) {
  static_cast<AsyncWrapper *>(context)->loop();
}

void AsyncWrapper::setTarget(const double itarget) {
  controller->setTarget(itarget);
}

double AsyncWrapper::getOutput() const {
  return controller->getOutput();
}

double AsyncWrapper::getError() const {
  return controller->getError();
}

bool AsyncWrapper::isSettled() {
  return controller->isSettled();
}

void AsyncWrapper::setSampleTime(const QTime isampleTime) {
  controller->setSampleTime(isampleTime);
}

void AsyncWrapper::setOutputLimits(double imax, double imin) {
  controller->setOutputLimits(imax, imin);
}

void AsyncWrapper::reset() {
  controller->reset();
}

void AsyncWrapper::flipDisable() {
  controller->flipDisable();
}

void AsyncWrapper::flipDisable(const bool iisDisabled) {
  controller->flipDisable(iisDisabled);
}

bool AsyncWrapper::isDisabled() const {
  return controller->isDisabled();
}
} // namespace okapi

/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/coreProsAPI.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
AsyncWrapper::AsyncWrapper(std::shared_ptr<ControllerInput> iinput,
                           std::shared_ptr<ControllerOutput> ioutput,
                           std::unique_ptr<IterativeController> icontroller,
                           const Supplier<std::unique_ptr<AbstractRate>> &irateSupplier,
                           std::unique_ptr<SettledUtil> isettledUtil)
  : logger(Logger::instance()),
    input(iinput),
    output(ioutput),
    controller(std::move(icontroller)),
    loopRate(std::move(irateSupplier.get())),
    settledRate(std::move(irateSupplier.get())),
    settledUtil(std::move(isettledUtil)),
    task(trampoline, this) {
}

void AsyncWrapper::loop() {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  while (true) {
    if (!controller->isDisabled()) {
      output->controllerSet(controller->step(input->controllerGet()));
    }

    loopRate->delayUntil(controller->getSampleTime());
  }
#pragma clang diagnostic pop
}

void AsyncWrapper::trampoline(void *context) {
  static_cast<AsyncWrapper *>(context)->loop();
}

void AsyncWrapper::setTarget(const double itarget) {
  logger->info("AsyncWrapper: Set target to " + std::to_string(itarget));
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
  logger->info("AsyncWrapper: Reset");
  controller->reset();
}

void AsyncWrapper::flipDisable() {
  controller->flipDisable();
}

void AsyncWrapper::flipDisable(const bool iisDisabled) {
  logger->info("AsyncWrapper: flipDisable " + std::to_string(iisDisabled));
  controller->flipDisable(iisDisabled);
}

bool AsyncWrapper::isDisabled() const {
  return controller->isDisabled();
}

void AsyncWrapper::waitUntilSettled() {
  logger->info("AsyncWrapper: Waiting to settle");
  while (!settledUtil->isSettled(getError())) {
    loopRate->delayUntil(motorUpdateRate);
  }
  logger->info("AsyncWrapper: Done waiting to settle");
}
} // namespace okapi

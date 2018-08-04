/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCWRAPPER_HPP_
#define _OKAPI_ASYNCWRAPPER_HPP_

#include "okapi/api/control/async/asyncController.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/coreProsAPI.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/api/util/supplier.hpp"
#include <memory>

namespace okapi {
template <typename I, typename O> class AsyncWrapper : virtual public AsyncController<I, O> {
  public:
  /**
   * A wrapper class that transforms an IterativeController into an AsyncController by running it in
   * another task. The input controller will act like an AsyncController. The output of the
   * IterativeController will be scaled by the given scale (127 by default).
   *
   * @param iinput controller input, passed to the IterativeController
   * @param ioutput controller output, written to from the IterativeController
   * @param icontroller the controller to use
   * @param irateSupplier used for rates used in the main loop and in waitUntilSettled
   * @param isettledUtil used in waitUntilSettled
   * @param iscale the scale applied to the controller output
   */
  AsyncWrapper(std::shared_ptr<ControllerInput<I>> iinput,
               std::shared_ptr<ControllerOutput<O>> ioutput,
               std::unique_ptr<IterativeController<I, O>> icontroller,
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

  /**
   * Sets the target for the controller.
   */
  void setTarget(I itarget) override {
    logger->info("AsyncWrapper: Set target to " + std::to_string(itarget));
    controller->setTarget(itarget);
  }

  /**
   * Returns the last calculated output of the controller. Default is 0.
   */
  O getOutput() const {
    return controller->getOutput();
  }

  /**
   * Returns the last error of the controller.
   */
  O getError() const override {
    return controller->getError();
  }

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * If the controller is disabled, this method must return true.
   *
   * @return whether the controller is settled
   */
  bool isSettled() override {
    return controller->isSettled();
  }

  /**
   * Set time between loops. Default does nothing.
   *
   * @param isampleTime time between loops
   */
  void setSampleTime(QTime isampleTime) {
    controller->setSampleTime(isampleTime);
  }

  /**
   * Set controller output bounds. Default does nothing.
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(O imax, O imin) {
    controller->setOutputLimits(imax, imin);
  }

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  void reset() override {
    logger->info("AsyncWrapper: Reset");
    controller->reset();
  }

  /**
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  void flipDisable() override {
    controller->flipDisable();
  }

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  void flipDisable(bool iisDisabled) override {
    logger->info("AsyncWrapper: flipDisable " + std::to_string(iisDisabled));
    controller->flipDisable(iisDisabled);
  }

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  bool isDisabled() const override {
    return controller->isDisabled();
  }

  /**
   * Blocks the current task until the controller has settled. Determining what settling means is
   * implementation-dependent.
   */
  void waitUntilSettled() override {
    logger->info("AsyncWrapper: Waiting to settle");

    while (!settledUtil->isSettled(getError())) {
      loopRate->delayUntil(motorUpdateRate);
    }

    logger->info("AsyncWrapper: Done waiting to settle");
  }

  protected:
  Logger *logger;
  std::shared_ptr<ControllerInput<I>> input;
  std::shared_ptr<ControllerOutput<O>> output;
  std::unique_ptr<IterativeController<I, O>> controller;
  std::unique_ptr<AbstractRate> loopRate;
  std::unique_ptr<AbstractRate> settledRate;
  std::unique_ptr<SettledUtil> settledUtil;
  CrossplatformThread task;

  static void trampoline(void *context) {
    if (context) {
      static_cast<AsyncWrapper *>(context)->loop();
    }
  }

  void loop() {
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
};
} // namespace okapi

#endif

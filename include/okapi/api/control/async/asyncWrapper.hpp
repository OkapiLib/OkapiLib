/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/control/async/asyncController.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/coreProsAPI.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/api/util/supplier.hpp"
#include <atomic>
#include <memory>

namespace okapi {
template <typename Input, typename Output>
class AsyncWrapper : virtual public AsyncController<Input, Output> {
  public:
  /**
   * A wrapper class that transforms an IterativeController into an AsyncController by running it
   * in another task. The input controller will act like an AsyncController.
   *
   * @param iinput controller input, passed to the IterativeController
   * @param ioutput controller output, written to from the IterativeController
   * @param icontroller the controller to use
   * @param irateSupplier used for rates used in the main loop and in waitUntilSettled
   * @param isettledUtil used in waitUntilSettled
   * @param iscale the scale applied to the controller output
   */
  AsyncWrapper(const std::shared_ptr<ControllerInput<Input>> &iinput,
               const std::shared_ptr<ControllerOutput<Output>> &ioutput,
               std::unique_ptr<IterativeController<Input, Output>> icontroller,
               const Supplier<std::unique_ptr<AbstractRate>> &irateSupplier)
    : logger(Logger::instance()),
      rateSupplier(irateSupplier),
      input(iinput),
      output(ioutput),
      controller(std::move(icontroller)) {
  }

  AsyncWrapper(AsyncWrapper<Input, Output> &&other) noexcept
    : logger(other.logger),
      rateSupplier(std::move(other.rateSupplier)),
      input(std::move(other.input)),
      output(std::move(other.output)),
      controller(std::move(other.controller)),
      dtorCalled(other.dtorCalled.load(std::memory_order_acquire)),
      task(other.task) {
  }

  ~AsyncWrapper() override {
    dtorCalled.store(true, std::memory_order_release);
    delete task;
  }

  /**
   * Sets the target for the controller.
   */
  void setTarget(Input itarget) override {
    logger->info("AsyncWrapper: Set target to " + std::to_string(itarget));
    hasFirstTarget = true;
    controller->setTarget(itarget);
    lastTarget = itarget;
  }

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller.
   *
   * @param ivalue the controller's output
   */
  void controllerSet(Input ivalue) override {
    controller->controllerSet(ivalue);
  }

  /**
   * Gets the last set target, or the default target if none was set.
   *
   * @return the last target
   */
  Input getTarget() override {
    return controller->getTarget();
  }

  /**
   * Returns the last calculated output of the controller.
   */
  Output getOutput() const {
    return controller->getOutput();
  }

  /**
   * Returns the last error of the controller. Does not update when disabled.
   */
  Output getError() const override {
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
    return isDisabled() || controller->isSettled();
  }

  /**
   * Set time between loops.
   *
   * @param isampleTime time between loops
   */
  void setSampleTime(QTime isampleTime) {
    controller->setSampleTime(isampleTime);
  }

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(Output imax, Output imin) {
    controller->setOutputLimits(imax, imin);
  }

  /**
   * Get the upper output bound.
   *
   * @return  the upper output bound
   */
  Output getMaxOutput() {
    return controller->getMaxOutput();
  }

  /**
   * Get the lower output bound.
   *
   * @return the lower output bound
   */
  Output getMinOutput() {
    return controller->getMinOutput();
  }

  /**
   * Resets the controller's internal state so it is similar to when it was first initialized, while
   * keeping any user-configured information.
   */
  void reset() override {
    logger->info("AsyncWrapper: Reset");
    controller->reset();
    hasFirstTarget = false;
  }

  /**
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  void flipDisable() override {
    logger->info("AsyncWrapper: flipDisable " + std::to_string(!controller->isDisabled()));
    controller->flipDisable();
    resumeMovement();
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
    resumeMovement();
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

    auto rate = rateSupplier.get();
    while (!isSettled()) {
      rate->delayUntil(motorUpdateRate);
    }

    logger->info("AsyncWrapper: Done waiting to settle");
  }

  /**
   * Starts the internal thread. This should not be called by normal users. This method is called
   * by the AsyncControllerFactory when making a new instance of this class.
   */
  void startThread() {
    if (!task) {
      task = new CrossplatformThread(trampoline, this);
    }
  }

  protected:
  Logger *logger;
  Supplier<std::unique_ptr<AbstractRate>> rateSupplier;
  std::shared_ptr<ControllerInput<Input>> input;
  std::shared_ptr<ControllerOutput<Output>> output;
  std::unique_ptr<IterativeController<Input, Output>> controller;
  bool hasFirstTarget{false};
  Input lastTarget;
  std::atomic_bool dtorCalled{false};
  CrossplatformThread *task{nullptr};

  static void trampoline(void *context) {
    if (context) {
      static_cast<AsyncWrapper *>(context)->loop();
    }
  }

  void loop() {
    auto rate = rateSupplier.get();
    while (!dtorCalled.load(std::memory_order_acquire)) {
      if (!isDisabled()) {
        output->controllerSet(controller->step(input->controllerGet()));
      }

      rate->delayUntil(controller->getSampleTime());
    }
  }

  /**
   * Resumes moving after the controller is reset. Should not cause movement if the controller is
   * turned off, reset, and turned back on.
   */
  virtual void resumeMovement() {
    if (isDisabled()) {
      // This will grab the output *when disabled*
      output->controllerSet(controller->getOutput());
    } else {
      if (hasFirstTarget) {
        setTarget(lastTarget);
      }
    }
  }
};
} // namespace okapi

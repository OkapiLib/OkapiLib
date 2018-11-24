/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/control/async/asyncController.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/logging.hpp"
#include <memory>

namespace okapi {
template <typename Input, typename Output> class ControllerRunner {
  public:
  explicit ControllerRunner(std::unique_ptr<AbstractRate> irate)
    : logger(Logger::instance()), rate(std::move(irate)) {
  }

  /**
   * Runs the controller until it has settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @return the error when settled
   */
  virtual Output runUntilSettled(const Input itarget, AsyncController<Input, Output> &icontroller) {
    logger->info("ControllerRunner: runUntilSettled(AsyncController): Set target to " +
                 std::to_string(itarget));
    icontroller.setTarget(itarget);

    while (!icontroller.isSettled()) {
      rate->delay(10);
    }

    logger->info("ControllerRunner: runUntilSettled(AsyncController): Done waiting to settle");
    return icontroller.getError();
  }

  /**
   * Runs the controller until it has settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @param ioutput the output to write to
   * @return the error when settled
   */
  virtual Output runUntilSettled(const Input itarget,
                                 IterativeController<Input, Output> &icontroller,
                                 ControllerOutput<Output> &ioutput) {
    logger->info("ControllerRunner: runUntilSettled(IterativeController): Set target to " +
                 std::to_string(itarget));
    icontroller.setTarget(itarget);

    while (!icontroller.isSettled()) {
      ioutput.controllerSet(icontroller.getOutput());
      rate->delay(10);
    }

    logger->info("ControllerRunner: runUntilSettled(IterativeController): Done waiting to settle");
    return icontroller.getError();
  }

  /**
   * Runs the controller until it has reached its target, but not necessarily settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @return the error when settled
   */
  virtual Output runUntilAtTarget(const Input itarget,
                                  AsyncController<Input, Output> &icontroller) {
    logger->info("ControllerRunner: runUntilAtTarget(AsyncController): Set target to " +
                 std::to_string(itarget));
    icontroller.setTarget(itarget);

    double error = icontroller.getError();
    double lastError = error;
    while (error != 0 && std::copysign(1.0, error) == std::copysign(1.0, lastError)) {
      lastError = error;
      rate->delay(10);
      error = icontroller.getError();
    }

    logger->info("ControllerRunner: runUntilAtTarget(AsyncController): Done waiting to settle");
    return icontroller.getError();
  }

  /**
   * Runs the controller until it has reached its target, but not necessarily settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @param ioutput the output to write to
   * @return the error when settled
   */
  virtual Output runUntilAtTarget(const Input itarget,
                                  IterativeController<Input, Output> &icontroller,
                                  ControllerOutput<Output> &ioutput) {
    logger->info("ControllerRunner: runUntilAtTarget(IterativeController): Set target to " +
                 std::to_string(itarget));
    icontroller.setTarget(itarget);

    double error = icontroller.getError();
    double lastError = error;
    while (error != 0 && std::copysign(1.0, error) == std::copysign(1.0, lastError)) {
      ioutput.controllerSet(icontroller.getOutput());
      lastError = error;
      rate->delay(10);
      error = icontroller.getError();
    }

    logger->info("ControllerRunner: runUntilAtTarget(IterativeController): Done waiting to settle");
    return icontroller.getError();
  }

  protected:
  Logger *logger;
  std::unique_ptr<AbstractRate> rate;
};
} // namespace okapi

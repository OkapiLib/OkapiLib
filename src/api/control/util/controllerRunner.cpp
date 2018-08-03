/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/util/controllerRunner.hpp"
#include <cmath>

namespace okapi {
ControllerRunner::ControllerRunner(std::unique_ptr<AbstractRate> irate)
  : logger(Logger::instance()), rate(std::move(irate)) {
}

ControllerRunner::~ControllerRunner() = default;

double ControllerRunner::runUntilSettled(const double itarget, AsyncController &icontroller) {
  logger->info("ControllerRunner: runUntilSettled(AsyncController): Set target to " +
               std::to_string(itarget));
  icontroller.setTarget(itarget);

  while (!icontroller.isSettled()) {
    rate->delay(10);
  }

  logger->info("ControllerRunner: runUntilSettled(AsyncController): Done waiting to settle");
  return icontroller.getError();
}

double ControllerRunner::runUntilSettled(const double itarget, IterativeController &icontroller,
                                         ControllerOutput &ioutput) {
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

double ControllerRunner::runUntilAtTarget(const double itarget, AsyncController &icontroller) {
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

double ControllerRunner::runUntilAtTarget(const double itarget, IterativeController &icontroller,
                                          ControllerOutput &ioutput) {
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
} // namespace okapi

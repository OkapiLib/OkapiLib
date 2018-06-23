/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/util/controllerRunner.hpp"
#include "okapi/impl/util/rate.hpp"
#include <cmath>

namespace okapi {
ControllerRunner::ControllerRunner() : rate(std::make_unique<Rate>()) {
}

ControllerRunner::ControllerRunner(std::unique_ptr<AbstractRate> irate) : rate(std::move(irate)) {
}

ControllerRunner::~ControllerRunner() = default;

double ControllerRunner::runUntilSettled(const double itarget, AsyncController &icontroller) {
  icontroller.setTarget(itarget);

  while (!icontroller.isSettled()) {
    rate->delay(10);
  }

  return icontroller.getError();
}

double ControllerRunner::runUntilSettled(const double itarget, IterativeController &icontroller,
                                         ControllerOutput &ioutput) {
  icontroller.setTarget(itarget);

  while (!icontroller.isSettled()) {
    ioutput.controllerSet(icontroller.getOutput());
    rate->delay(10);
  }

  return icontroller.getError();
}

double ControllerRunner::runUntilAtTarget(const double itarget, AsyncController &icontroller) {
  icontroller.setTarget(itarget);

  double error = icontroller.getError();
  double lastError = error;
  while (error != 0 && std::copysign(1.0, error) == std::copysign(1.0, lastError)) {
    lastError = error;
    rate->delay(10);
    error = icontroller.getError();
  }

  return icontroller.getError();
}

double ControllerRunner::runUntilAtTarget(const double itarget, IterativeController &icontroller,
                                          ControllerOutput &ioutput) {
  icontroller.setTarget(itarget);

  double error = icontroller.getError();
  double lastError = error;
  while (error != 0 && std::copysign(1.0, error) == std::copysign(1.0, lastError)) {
    ioutput.controllerSet(icontroller.getOutput());
    lastError = error;
    rate->delay(10);
    error = icontroller.getError();
  }

  return icontroller.getError();
}
} // namespace okapi

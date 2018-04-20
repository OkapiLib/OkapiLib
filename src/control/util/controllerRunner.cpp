/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/util/controllerRunner.hpp"
#include <cmath>

namespace okapi {
ControllerRunner::ControllerRunner() = default;

ControllerRunner::~ControllerRunner() = default;

double ControllerRunner::runUntilSettled(const double itarget, AsyncController &icontroller) {
  icontroller.setTarget(itarget);

  while (!icontroller.isSettled()) {
    pros::c::task_delay(10);
  }

  return icontroller.getError();
}

double ControllerRunner::runUntilSettled(const double itarget, IterativeController &icontroller,
                                         ControllerOutput &ioutput) {
  icontroller.setTarget(itarget);

  while (!icontroller.isSettled()) {
    ioutput.controllerSet(icontroller.getOutput());
    pros::c::task_delay(10);
  }

  return icontroller.getError();
}

double ControllerRunner::runUntilAtTarget(const double itarget, AsyncController &icontroller) {
  icontroller.setTarget(itarget);

  double error = icontroller.getError();
  double lastError = error;
  while (error != 0 && std::copysign(1.0, error) == std::copysign(1.0, lastError)) {
    lastError = error;
    pros::c::task_delay(10);
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
    pros::c::task_delay(10);
    error = icontroller.getError();
  }

  return icontroller.getError();
}
}

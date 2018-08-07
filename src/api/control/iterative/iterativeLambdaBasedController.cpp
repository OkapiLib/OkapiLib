/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/iterative/iterativeLambdaBasedController.hpp"

namespace okapi {
IterativeLambdaBasedController::IterativeLambdaBasedController(
  std::function<double(double)> istepFunction, const TimeUtil &itimeUtil)
  : stepFunction(istepFunction),
    loopDtTimer(itimeUtil.getTimer()),
    settledUtil(itimeUtil.getSettledUtil()) {
}

void IterativeLambdaBasedController::setTarget(const double itarget) {
  target = itarget;
}

double IterativeLambdaBasedController::getOutput() const {
  return output;
}

double IterativeLambdaBasedController::getError() const {
  return error;
}

bool IterativeLambdaBasedController::isSettled() {
  return settledUtil->isSettled(error);
}

void IterativeLambdaBasedController::setSampleTime(const QTime isampleTime) {
  if (isampleTime > 0_ms) {
    sampleTime = isampleTime;
  }
}

void IterativeLambdaBasedController::setOutputLimits(double imax, double imin) {
  // Always use larger value as max
  if (imin > imax) {
    const double temp = imax;
    imax = imin;
    imin = temp;
  }

  outputMax = imax;
  outputMin = imin;

  output = std::clamp(output, outputMin, outputMax);
}

double IterativeLambdaBasedController::step(const double inewReading) {
  if (isOn) {
    loopDtTimer->placeHardMark();

    if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
      error = target - inewReading;
      output = stepFunction(error);

      loopDtTimer->clearHardMark(); // Important that we only clear if dt >= sampleTime

      settledUtil->isSettled(error);
    }
  } else {
    output = 0; // Controller is off so write 0
  }

  return output;
}

void IterativeLambdaBasedController::reset() {
  error = 0;
  output = 0;
}

void IterativeLambdaBasedController::flipDisable() {
  isOn = !isOn;
}

void IterativeLambdaBasedController::flipDisable(const bool iisDisabled) {
  isOn = !iisDisabled;
}

bool IterativeLambdaBasedController::isDisabled() const {
  return !isOn;
}

QTime IterativeLambdaBasedController::getSampleTime() const {
  return sampleTime;
}
} // namespace okapi

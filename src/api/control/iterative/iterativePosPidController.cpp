/**
 * Based on the Arduino PID controller: https://github.com/br3ttb/Arduino-PID-Library
 *
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>
#include <cmath>

namespace okapi {
IterativePosPIDController::IterativePosPIDController(const double ikP,
                                                     const double ikI,
                                                     const double ikD,
                                                     const double ikBias,
                                                     const TimeUtil &itimeUtil,
                                                     std::unique_ptr<Filter> iderivativeFilter)
  : logger(Logger::instance()),
    derivativeFilter(std::move(iderivativeFilter)),
    loopDtTimer(itimeUtil.getTimer()),
    settledUtil(itimeUtil.getSettledUtil()) {
  if (ikI != 0) {
    setIntegralLimits(1 / ikI, -1 / ikI);
  }
  setOutputLimits(-1, 1);
  setGains(ikP, ikI, ikD, ikBias);
}

IterativePosPIDController::IterativePosPIDController(const Gains &igains,
                                                     const TimeUtil &itimeUtil,
                                                     std::unique_ptr<Filter> iderivativeFilter)
  : IterativePosPIDController(igains.kP,
                              igains.kI,
                              igains.kD,
                              igains.kBias,
                              itimeUtil,
                              std::move(iderivativeFilter)) {
}

void IterativePosPIDController::setTarget(const double itarget) {
  logger->info("IterativePosPIDController: Set target to " + std::to_string(itarget));
  target = itarget;
}

void IterativePosPIDController::controllerSet(const double ivalue) {
  target = remapRange(ivalue, -1, 1, outputMin, outputMax);
}

double IterativePosPIDController::getTarget() {
  return target;
}

double IterativePosPIDController::getOutput() const {
  return isDisabled() ? 0 : output;
}

double IterativePosPIDController::getMaxOutput() {
  return outputMax;
}

double IterativePosPIDController::getMinOutput() {
  return outputMin;
}

double IterativePosPIDController::getError() const {
  return target - lastReading;
}

bool IterativePosPIDController::isSettled() {
  return isDisabled() ? true : settledUtil->isSettled(error);
}

void IterativePosPIDController::setSampleTime(const QTime isampleTime) {
  if (isampleTime > 0_ms) {
    const double ratio = isampleTime.convert(millisecond) / sampleTime.convert(millisecond);
    kI *= ratio;
    kD /= ratio;
    sampleTime = isampleTime;
  }
}

void IterativePosPIDController::setOutputLimits(double imax, double imin) {
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

void IterativePosPIDController::setIntegralLimits(double imax, double imin) {
  // Always use larger value as max
  if (imin > imax) {
    const double temp = imax;
    imax = imin;
    imin = temp;
  }

  integralMax = imax;
  integralMin = imin;

  integral = std::clamp(integral, integralMin, integralMax);
}

void IterativePosPIDController::setErrorSumLimits(const double imax, const double imin) {
  errorSumMax = imax;
  errorSumMin = imin;
}

double IterativePosPIDController::step(const double inewReading) {
  lastReading = inewReading;

  if (controllerIsDisabled) {
    return 0;
  } else {
    loopDtTimer->placeHardMark();

    if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
      error = getError();

      if ((std::abs(error) < target - errorSumMin && std::abs(error) > target - errorSumMax) ||
          (std::abs(error) > target + errorSumMin && std::abs(error) < target + errorSumMax)) {
        integral += kI * error; // Eliminate integral kick while realtime tuning
      }

      if (shouldResetOnCross && std::copysign(1.0, error) != std::copysign(1.0, lastError)) {
        integral = 0;
      }

      integral = std::clamp(integral, integralMin, integralMax);

      // Derivative over measurement to eliminate derivative kick on setpoint change
      derivative = derivativeFilter->filter(inewReading - lastReading);

      output = std::clamp(kP * error + integral - kD * derivative + kBias, outputMin, outputMax);

      lastError = error;
      loopDtTimer->clearHardMark(); // Important that we only clear if dt >= sampleTime

      settledUtil->isSettled(error);
    }
  }

  return output;
}

void IterativePosPIDController::setGains(const double ikP,
                                         const double ikI,
                                         const double ikD,
                                         const double ikBias) {
  const double sampleTimeSec = sampleTime.convert(second);
  kP = ikP;
  kI = ikI * sampleTimeSec;
  kD = ikD * sampleTimeSec;
  kBias = ikBias;
}

void IterativePosPIDController::reset() {
  logger->info("IterativePosPIDController: Reset");
  error = 0;
  lastError = 0;
  lastReading = 0;
  integral = 0;
  output = 0;
  settledUtil->reset();
}

void IterativePosPIDController::setIntegratorReset(bool iresetOnZero) {
  shouldResetOnCross = iresetOnZero;
}

void IterativePosPIDController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void IterativePosPIDController::flipDisable(const bool iisDisabled) {
  logger->info("IterativePosPIDController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
}

bool IterativePosPIDController::isDisabled() const {
  return controllerIsDisabled;
}

QTime IterativePosPIDController::getSampleTime() const {
  return sampleTime;
}
} // namespace okapi

/*
 * Based on the Arduino PID controller: https://github.com/br3ttb/Arduino-PID-Library
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
                                                     std::unique_ptr<Filter> iderivativeFilter,
                                                     std::shared_ptr<Logger> ilogger)
  : IterativePosPIDController({ikP, ikI, ikD, ikBias},
                              itimeUtil,
                              std::move(iderivativeFilter),
                              std::move(ilogger)) {
}

IterativePosPIDController::IterativePosPIDController(const Gains &igains,
                                                     const TimeUtil &itimeUtil,
                                                     std::unique_ptr<Filter> iderivativeFilter,
                                                     std::shared_ptr<Logger> ilogger)
  : logger(std::move(ilogger)),
    derivativeFilter(std::move(iderivativeFilter)),
    loopDtTimer(itimeUtil.getTimer()),
    settledUtil(itimeUtil.getSettledUtil()) {
  if (igains.kI != 0) {
    setIntegralLimits(1 / igains.kI, -1 / igains.kI);
  }
  setOutputLimits(1, -1);
  setGains(igains);
}

void IterativePosPIDController::setTarget(const double itarget) {
  LOG_INFO("IterativePosPIDController: Set target to " + std::to_string(itarget));
  target = itarget;
}

void IterativePosPIDController::controllerSet(const double ivalue) {
  target = remapRange(ivalue, -1, 1, controllerSetTargetMin, controllerSetTargetMax);
}

double IterativePosPIDController::getTarget() {
  return target;
}

double IterativePosPIDController::getTarget() const {
  return target;
}

double IterativePosPIDController::getProcessValue() const {
  return lastReading;
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
  return getTarget() - getProcessValue();
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

void IterativePosPIDController::setControllerSetTargetLimits(double itargetMax, double itargetMin) {
  // Always use larger value as max
  if (itargetMin > itargetMax) {
    const double temp = itargetMax;
    itargetMax = itargetMin;
    itargetMin = temp;
  }

  controllerSetTargetMax = itargetMax;
  controllerSetTargetMin = itargetMin;
}

double IterativePosPIDController::step(const double inewReading) {
  if (controllerIsDisabled) {
    return 0;
  } else {
    loopDtTimer->placeHardMark();

    if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
      // lastReading must only be updated here so its updates are time-gated by sampleTime
      const double readingDiff = inewReading - lastReading;
      lastReading = inewReading;

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
      derivative = derivativeFilter->filter(readingDiff);

      output = std::clamp(kP * error + integral - kD * derivative + kBias, outputMin, outputMax);

      lastError = error;
      loopDtTimer->clearHardMark(); // Important that we only clear if dt >= sampleTime

      settledUtil->isSettled(error);
    }
  }

  return output;
}

void IterativePosPIDController::reset() {
  LOG_INFO_S("IterativePosPIDController: Reset");

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
  LOG_INFO("IterativePosPIDController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
}

bool IterativePosPIDController::isDisabled() const {
  return controllerIsDisabled;
}

QTime IterativePosPIDController::getSampleTime() const {
  return sampleTime;
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

void IterativePosPIDController::setGains(const Gains &igains) {
  const double sampleTimeSec = sampleTime.convert(second);
  kP = igains.kP;
  kI = igains.kI * sampleTimeSec;
  kD = igains.kD / sampleTimeSec;
  kBias = igains.kBias;
}

IterativePosPIDController::Gains IterativePosPIDController::getGains() const {
  return {kP, kI / sampleTime.convert(second), kD * sampleTime.convert(second), kBias};
}

bool IterativePosPIDController::Gains::operator==(
  const IterativePosPIDController::Gains &rhs) const {
  return kP == rhs.kP && kI == rhs.kI && kD == rhs.kD && kBias == rhs.kBias;
}

bool IterativePosPIDController::Gains::operator!=(
  const IterativePosPIDController::Gains &rhs) const {
  return !(rhs == *this);
}
} // namespace okapi

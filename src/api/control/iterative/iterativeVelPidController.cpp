/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>
#include <cmath>

namespace okapi {
IterativeVelPIDController::IterativeVelPIDController(const double ikP,
                                                     const double ikD,
                                                     const double ikF,
                                                     const double ikSF,
                                                     std::unique_ptr<VelMath> ivelMath,
                                                     const TimeUtil &itimeUtil,
                                                     std::unique_ptr<Filter> iderivativeFilter)
  : logger(Logger::instance()),
    velMath(std::move(ivelMath)),
    derivativeFilter(std::move(iderivativeFilter)),
    loopDtTimer(itimeUtil.getTimer()),
    settledUtil(itimeUtil.getSettledUtil()) {
  setGains(ikP, ikD, ikF, ikSF);
}

void IterativeVelPIDController::setGains(const double ikP,
                                         const double ikD,
                                         const double ikF,
                                         const double ikSF) {
  kP = ikP;
  kD = ikD * sampleTime.convert(second);
  kF = ikF;
  kSF = ikSF;
}

void IterativeVelPIDController::setSampleTime(const QTime isampleTime) {
  if (isampleTime > 0_ms) {
    kD /= isampleTime.convert(millisecond) / sampleTime.convert(millisecond);
    sampleTime = isampleTime;
  }
}

void IterativeVelPIDController::setOutputLimits(double imax, double imin) {
  // Always use larger value as max
  if (imin > imax) {
    const double temp = imax;
    imax = imin;
    imin = temp;
  }

  outputMax = imax;
  outputMin = imin;

  outputSum = std::clamp(outputSum, outputMin, outputMax);
  output = std::clamp(output, outputMin, outputMax);
}

QAngularSpeed IterativeVelPIDController::stepVel(const double inewReading) {
  return velMath->step(inewReading);
}

double IterativeVelPIDController::step(const double inewReading) {
  if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
    stepVel(inewReading);
  }

  if (!controllerIsDisabled) {
    loopDtTimer->placeHardMark();

    if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
      error = getError();

      // Derivative over measurement to eliminate derivative kick on setpoint change
      derivative = derivativeFilter->filter(velMath->getAccel().getValue());

      outputSum += kP * error - kD * derivative;
      outputSum = std::clamp(outputSum, outputMin, outputMax);

      loopDtTimer->clearHardMark(); // Important that we only clear if dt >= sampleTime

      settledUtil->isSettled(error);
    }

    output =
      std::clamp(outputSum + kF * target + kSF * std::copysign(1.0, target), outputMin, outputMax);
    return output;
  }

  return 0; // Can't set output to zero because the entire loop in an integral
}

void IterativeVelPIDController::setTarget(const double itarget) {
  logger->info("IterativeVelPIDController: Set target to " + std::to_string(itarget));
  target = itarget;
}

void IterativeVelPIDController::controllerSet(const double ivalue) {
  target = remapRange(ivalue, -1, 1, outputMin, outputMax);
}

double IterativeVelPIDController::getTarget() {
  return target;
}

double IterativeVelPIDController::getOutput() const {
  return isDisabled() ? 0 : output;
}

double IterativeVelPIDController::getMaxOutput() {
  return outputMax;
}

double IterativeVelPIDController::getMinOutput() {
  return outputMin;
}

double IterativeVelPIDController::getError() const {
  return target - velMath->getVelocity().convert(rpm);
}

bool IterativeVelPIDController::isSettled() {
  return isDisabled() ? true : settledUtil->isSettled(error);
}

void IterativeVelPIDController::reset() {
  logger->info("IterativeVelPIDController: Reset");
  error = 0;
  outputSum = 0;
  output = 0;
  settledUtil->reset();
}

void IterativeVelPIDController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void IterativeVelPIDController::flipDisable(const bool iisDisabled) {
  logger->info("IterativeVelPIDController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
}

bool IterativeVelPIDController::isDisabled() const {
  return controllerIsDisabled;
}

void IterativeVelPIDController::setTicksPerRev(const double tpr) {
  velMath->setTicksPerRev(tpr);
}

QAngularSpeed IterativeVelPIDController::getVel() const {
  return velMath->getVelocity();
}

QTime IterativeVelPIDController::getSampleTime() const {
  return sampleTime;
}
} // namespace okapi

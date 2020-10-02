/*
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
                                                     std::unique_ptr<Filter> iderivativeFilter,
                                                     std::shared_ptr<Logger> ilogger)
  : IterativeVelPIDController({ikP, ikD, ikF, ikSF},
                              std::move(ivelMath),
                              itimeUtil,
                              std::move(iderivativeFilter),
                              std::move(ilogger)) {
}

IterativeVelPIDController::IterativeVelPIDController(const Gains &igains,
                                                     std::unique_ptr<VelMath> ivelMath,
                                                     const TimeUtil &itimeUtil,
                                                     std::unique_ptr<Filter> iderivativeFilter,
                                                     std::shared_ptr<Logger> ilogger)
  : logger(std::move(ilogger)),
    velMath(std::move(ivelMath)),
    derivativeFilter(std::move(iderivativeFilter)),
    loopDtTimer(itimeUtil.getTimer()),
    settledUtil(itimeUtil.getSettledUtil()) {
  setOutputLimits(1, -1);
  setGains(igains);
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

void IterativeVelPIDController::setControllerSetTargetLimits(double itargetMax, double itargetMin) {
  // Always use larger value as max
  if (itargetMin > itargetMax) {
    const double temp = itargetMax;
    itargetMax = itargetMin;
    itargetMin = temp;
  }

  controllerSetTargetMax = itargetMax;
  controllerSetTargetMin = itargetMin;
}

QAngularSpeed IterativeVelPIDController::stepVel(const double inewReading) {
  return velMath->step(inewReading);
}

double IterativeVelPIDController::step(const double inewReading) {
  if (!controllerIsDisabled) {
    loopDtTimer->placeHardMark();

    if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
      stepVel(inewReading);
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
  LOG_INFO("IterativeVelPIDController: Set target to " + std::to_string(itarget));
  target = itarget;
}

void IterativeVelPIDController::controllerSet(const double ivalue) {
  target = remapRange(ivalue, -1, 1, controllerSetTargetMin, controllerSetTargetMax);
}

double IterativeVelPIDController::getTarget() {
  return target;
}

double IterativeVelPIDController::getTarget() const {
  return target;
}

double IterativeVelPIDController::getProcessValue() const {
  return velMath->getVelocity().convert(rpm);
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
  return getTarget() - getProcessValue();
}

bool IterativeVelPIDController::isSettled() {
  return isDisabled() ? true : settledUtil->isSettled(error);
}

void IterativeVelPIDController::reset() {
  LOG_INFO_S("IterativeVelPIDController: Reset");

  error = 0;
  outputSum = 0;
  output = 0;
  settledUtil->reset();
}

void IterativeVelPIDController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void IterativeVelPIDController::flipDisable(const bool iisDisabled) {
  LOG_INFO("IterativeVelPIDController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
}

bool IterativeVelPIDController::isDisabled() const {
  return controllerIsDisabled;
}

void IterativeVelPIDController::setGains(const Gains &igains) {
  kP = igains.kP;
  kD = igains.kD / sampleTime.convert(second);
  kF = igains.kF;
  kSF = igains.kSF;
}

IterativeVelPIDController::Gains IterativeVelPIDController::getGains() const {
  return {kP, kD * sampleTime.convert(second), kF, kSF};
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

bool IterativeVelPIDController::Gains::operator==(
  const IterativeVelPIDController::Gains &rhs) const {
  return kP == rhs.kP && kD == rhs.kD && kF == rhs.kF && kSF == rhs.kSF;
}

bool IterativeVelPIDController::Gains::operator!=(
  const IterativeVelPIDController::Gains &rhs) const {
  return !(rhs == *this);
}
} // namespace okapi

/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/iterative/iterativeVelPidController.hpp"
#include "api.h"
#include <algorithm>
#include <cmath>

namespace okapi {

IterativeVelPIDControllerArgs::IterativeVelPIDControllerArgs(const double ikP, const double ikD)
  : kP(ikP), kD(ikD) {
}

IterativeVelPIDControllerArgs::IterativeVelPIDControllerArgs(const double ikP, const double ikD,
                                                             const VelMathArgs &iparams)
  : kP(ikP), kD(ikD), params(iparams) {
}

IterativeVelPIDController::IterativeVelPIDController(const double ikP, const double ikD) {
  setGains(ikP, ikD);
}

IterativeVelPIDController::IterativeVelPIDController(const double ikP, const double ikD,
                                                     const VelMathArgs &iparams)
  : velMath(iparams) {
  setGains(ikP, ikD);
}

IterativeVelPIDController::IterativeVelPIDController(const IterativeVelPIDControllerArgs &params)
  : velMath(params.params) {
  setGains(params.kP, params.kD);
}

void IterativeVelPIDController::setGains(const double ikP, const double ikD) {
  kP = ikP;
  kD = ikD * static_cast<double>(sampleTime) / 1000.0;
}

void IterativeVelPIDController::setSampleTime(const std::uint32_t isampleTime) {
  if (isampleTime > 0) {
    kD /= static_cast<double>(isampleTime) / static_cast<double>(sampleTime);
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

  output = std::clamp(output, outputMin, outputMax);
}

double IterativeVelPIDController::stepVel(const double inewReading) {
  return velMath.step(inewReading);
}

double IterativeVelPIDController::step(const double inewReading) {
  if (isOn) {
    const std::uint32_t now = pros::millis();
    if (now - lastTime >= sampleTime) {
      stepVel(inewReading);
      error = target - velMath.getVelocity();

      // Derivative over measurement to eliminate derivative kick on setpoint change
      derivative = velMath.getAccel();

      output += kP * error - kD * derivative;
      output = std::clamp(output, outputMin, outputMax);

      lastError = error;
      lastTime = now; // Important that we only assign lastTime if dt >= sampleTime
    }

    return output;
  }

  return 0;
}

void IterativeVelPIDController::setTarget(const double itarget) {
  target = itarget;
}

double IterativeVelPIDController::getOutput() const {
  return isOn ? output : 0;
}

double IterativeVelPIDController::getError() const {
  return error;
}

double IterativeVelPIDController::getDerivative() const {
  return derivative;
}

bool IterativeVelPIDController::isSettled() {
  return settledUtil.isSettled(error);
}

void IterativeVelPIDController::reset() {
  error = 0;
  lastError = 0;
  output = 0;
}

void IterativeVelPIDController::flipDisable() {
  isOn = !isOn;
}

void IterativeVelPIDController::flipDisable(const bool iisDisabled) {
  isOn = !iisDisabled;
}

bool IterativeVelPIDController::isDisabled() const {
  return !isOn;
}

void IterativeVelPIDController::setTicksPerRev(const double tpr) {
  velMath.setTicksPerRev(tpr);
}

double IterativeVelPIDController::getVel() const {
  return velMath.getVelocity();
}

std::uint32_t IterativeVelPIDController::getSampleTime() const {
  return sampleTime;
}
} // namespace okapi

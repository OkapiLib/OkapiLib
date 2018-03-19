/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/iterative/velPidController.hpp"
#include "api.h"
#include <cmath>

namespace okapi {

VelPIDControllerParams::VelPIDControllerParams(const double ikP, const double ikD)
  : kP(ikP), kD(ikD), params(360) {
}

VelPIDControllerParams::VelPIDControllerParams(const double ikP, const double ikD,
                                               const VelMathParams &iparams)
  : kP(ikP), kD(ikD), params(iparams) {
}

VelPIDControllerParams::~VelPIDControllerParams() = default;

VelPIDController::VelPIDController(const double ikP, const double ikD)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    derivative(0),
    target(0),
    output(0),
    outputMax(127),
    outputMin(-127),
    isOn(true),
    velMath(360) {
  setGains(ikP, ikD);
}

VelPIDController::VelPIDController(const double ikP, const double ikD, const VelMathParams &iparams)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    derivative(0),
    target(0),
    output(0),
    outputMax(127),
    outputMin(-127),
    isOn(true),
    velMath(iparams) {
  setGains(ikP, ikD);
}

VelPIDController::VelPIDController(const VelPIDControllerParams &params)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    derivative(0),
    target(0),
    output(0),
    outputMax(127),
    outputMin(-127),
    isOn(true),
    velMath(params.params) {
  setGains(params.kP, params.kD);
}

VelPIDController::~VelPIDController() = default;

void VelPIDController::setGains(const double ikP, const double ikD) {
  kP = ikP;
  kD = ikD * static_cast<double>(sampleTime) / 1000.0;
}

void VelPIDController::setSampleTime(const uint32_t isampleTime) {
  if (isampleTime > 0) {
    kD /= static_cast<double>(isampleTime) / static_cast<double>(sampleTime);
    sampleTime = isampleTime;
  }
}

void VelPIDController::setOutputLimits(double imax, double imin) {
  // Always use larger value as max
  if (imin > imax) {
    const double temp = imax;
    imax = imin;
    imin = temp;
  }

  outputMax = imax;
  outputMin = imin;

  // Fix output
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
}

double VelPIDController::stepVel(const double inewReading) {
  return velMath.step(inewReading);
}

double VelPIDController::step(const double inewReading) {
  if (isOn) {
    const uint32_t now = millis();
    if (now - lastTime >= sampleTime) {
      stepVel(inewReading);
      const double error = target - velMath.getOutput();

      // Derivative over measurement to eliminate derivative kick on setpoint change
      derivative = velMath.getDiff();

      output += kP * error - kD * derivative;

      if (output > outputMax)
        output = outputMax;
      else if (output < outputMin)
        output = outputMin;

      lastError = error;
      lastTime = now; // Important that we only assign lastTime if dt >= sampleTime
    }

    return output;
  }

  return 0;
}

void VelPIDController::setTarget(const double itarget) {
  target = itarget;
}

double VelPIDController::getOutput() const {
  return isOn ? output : 0;
}

double VelPIDController::getError() const {
  return error;
}

double VelPIDController::getDerivative() const {
  return derivative;
}

void VelPIDController::reset() {
  error = 0;
  lastError = 0;
  output = 0;
}

void VelPIDController::flipDisable() {
  isOn = !isOn;
}

void VelPIDController::setFilterGains(const double alpha, const double beta) {
  velMath.setGains(alpha, beta);
}

void VelPIDController::setTicksPerRev(const double tpr) {
  velMath.setTicksPerRev(tpr);
}

double VelPIDController::getVel() const {
  return velMath.getOutput();
}

uint32_t VelPIDController::getSampleTime() const {
  return sampleTime;
}
} // namespace okapi

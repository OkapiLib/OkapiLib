/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/pidController.hpp"
#include "api.h"
#include <cmath>

namespace okapi {
PIDController::PIDController(const double ikP, const double ikI, const double ikD,
                             const double ikBias)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    target(0),
    lastReading(0),
    integral(0),
    integralMax(1),
    integralMin(-1),
    output(0),
    outputMax(1),
    outputMin(-1),
    shouldResetOnCross(true),
    isOn(true) {
  setGains(ikP, ikI, ikD, ikBias);
}

PIDController::PIDController(const PIDControllerParams &params)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    target(0),
    lastReading(0),
    integral(0),
    integralMax(1),
    integralMin(-1),
    output(0),
    outputMax(1),
    outputMin(-1),
    shouldResetOnCross(true),
    isOn(true) {
  setGains(params.kP, params.kI, params.kD, params.kBias);
}

PIDController::~PIDController() = default;

void PIDController::setTarget(const double itarget) {
  target = itarget;
}

double PIDController::getOutput() const {
  return output;
}

double PIDController::getError() const {
  return error;
}

void PIDController::setSampleTime(const uint32_t isampleTime) {
  if (isampleTime > 0) {
    const double ratio = static_cast<double>(isampleTime) / static_cast<double>(sampleTime);
    kI *= ratio;
    kD /= ratio;
    sampleTime = isampleTime;
  }
}

void PIDController::setOutputLimits(double imax, double imin) {
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

  // Fix integral
  setIntegralLimits(imax, imin);
}

void PIDController::setIntegralLimits(double imax, double imin) {
  // Always use larger value as max
  if (imin > imax) {
    const double temp = imax;
    imax = imin;
    imin = temp;
  }

  integralMax = imax;
  integralMin = imin;

  // Fix integral
  if (integral > integralMax)
    integral = integralMax;
  else if (integral < integralMin)
    integral = integralMin;
}

double PIDController::step(const double inewReading) {
  if (isOn) {
    const uint32_t now = pros::millis();

    if (now - lastTime >= sampleTime) {
      error = target - inewReading;

      integral += kI * error; // Eliminate integral kick while realtime tuning

      if (shouldResetOnCross && copysign(1.0, (double)error) != copysign(1.0, (double)lastError))
        integral = 0;

      if (integral > integralMax)
        integral = integralMax;
      else if (integral < integralMin)
        integral = integralMin;

      const double derivative =
        inewReading -
        lastReading; // Derivative over measurement to eliminate derivative kick on setpoint change

      output = kP * error + integral - kD * derivative + kBias;

      if (output > outputMax)
        output = outputMax;
      else if (output < outputMin)
        output = outputMin;

      lastReading = inewReading;
      lastError = error;
      lastTime = now; // Important that we only assign lastTime if dt >= sampleTime
    }
  } else {
    output = 0; // Controller is off so write 0
  }

  return output;
}

void PIDController::setGains(const double ikP, const double ikI, const double ikD,
                             const double ikBias) {
  const double sampleTimeSec = static_cast<double>(sampleTime) / 1000.0;
  kP = ikP;
  kI = ikI * sampleTimeSec;
  kD = ikD * sampleTimeSec;
  kBias = ikBias;
}

void PIDController::reset() {
  error = 0;
  lastError = 0;
  lastReading = 0;
  integral = 0;
  output = 0;
}

void PIDController::setIntegratorReset(bool iresetOnZero) {
  shouldResetOnCross = iresetOnZero;
}

void PIDController::flipDisable() {
  isOn = !isOn;
}
} // namespace okapi

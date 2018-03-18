/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/iterative/posPidController.hpp"
#include "api.h"
#include <cmath>

namespace okapi {
PosPIDControllerParams::PosPIDControllerParams(const double ikP, const double ikI, const double ikD,
                                               const double ikBias)
  : kP(ikP), kI(ikI), kD(ikD), kBias(ikBias) {
}

PosPIDControllerParams::~PosPIDControllerParams() = default;

PosPIDController::PosPIDController(const double ikP, const double ikI, const double ikD,
                                   const double ikBias)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    target(0),
    lastReading(0),
    integral(0),
    integralMax(127),
    integralMin(-127),
    output(0),
    outputMax(127),
    outputMin(-127),
    shouldResetOnCross(true),
    isOn(true) {
  setGains(ikP, ikI, ikD, ikBias);
}

PosPIDController::PosPIDController(const PosPIDControllerParams &params)
  : lastTime(0),
    sampleTime(15),
    error(0),
    lastError(0),
    target(0),
    lastReading(0),
    integral(0),
    integralMax(127),
    integralMin(-127),
    output(0),
    outputMax(127),
    outputMin(-127),
    shouldResetOnCross(true),
    isOn(true) {
  setGains(params.kP, params.kI, params.kD, params.kBias);
}

PosPIDController::~PosPIDController() = default;

void PosPIDController::setTarget(const double itarget) {
  target = itarget;
}

double PosPIDController::getOutput() const {
  return output;
}

double PosPIDController::getError() const {
  return error;
}

void PosPIDController::setSampleTime(const uint32_t isampleTime) {
  if (isampleTime > 0) {
    const double ratio = static_cast<double>(isampleTime) / static_cast<double>(sampleTime);
    kI *= ratio;
    kD /= ratio;
    sampleTime = isampleTime;
  }
}

void PosPIDController::setOutputLimits(double imax, double imin) {
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

void PosPIDController::setIntegralLimits(double imax, double imin) {
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

double PosPIDController::step(const double inewReading) {
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

void PosPIDController::setGains(const double ikP, const double ikI, const double ikD,
                                const double ikBias) {
  const double sampleTimeSec = static_cast<double>(sampleTime) / 1000.0;
  kP = ikP;
  kI = ikI * sampleTimeSec;
  kD = ikD * sampleTimeSec;
  kBias = ikBias;
}

void PosPIDController::reset() {
  error = 0;
  lastError = 0;
  lastReading = 0;
  integral = 0;
  output = 0;
}

void PosPIDController::setIntegratorReset(bool iresetOnZero) {
  shouldResetOnCross = iresetOnZero;
}

void PosPIDController::flipDisable() {
  isOn = !isOn;
}

uint32_t PosPIDController::getSampleTime() const {
  return sampleTime;
}
} // namespace okapi

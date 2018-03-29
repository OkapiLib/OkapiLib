/**
 * Based on the Arduino PID controller: https://github.com/br3ttb/Arduino-PID-Library
 *
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/iterative/posPidController.hpp"
#include "api.h"
#include <algorithm>
#include <cmath>

namespace okapi {
PosPIDControllerParams::PosPIDControllerParams(const double ikP, const double ikI, const double ikD,
                                               const double ikBias)
  : kP(ikP), kI(ikI), kD(ikD), kBias(ikBias) {
}

PosPIDControllerParams::~PosPIDControllerParams() = default;

PosPIDController::PosPIDController(const double ikP, const double ikI, const double ikD,
                                   const double ikBias) {
  if (ikI != 0) {
    setIntegralLimits(-1 / ikI, 1 / ikI);
  }
  setOutputLimits(-1, 1);
  setGains(ikP, ikI, ikD, ikBias);
}

PosPIDController::PosPIDController(const PosPIDControllerParams &params) {
  if (params.kI != 0) {
    setIntegralLimits(-1 / params.kI, 1 / params.kI);
  }
  setOutputLimits(-1, 1);
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

double PosPIDController::getDerivative() const {
  return derivative;
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

  output = std::clamp(output, outputMin, outputMax);

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

  integral = std::clamp(integral, integralMin, integralMax);
}

double PosPIDController::step(const double inewReading) {
  if (isOn) {
    const uint32_t now = millis();

    if (now - lastTime >= sampleTime) {
      error = (target - inewReading) / errorScale;

      if (fabs(error) > errorSumMin && fabs(error) < errorSumMax) {
        integral += kI * error; // Eliminate integral kick while realtime tuning
      }

      if (shouldResetOnCross && copysign(1.0, (double)error) != copysign(1.0, (double)lastError)) {
        integral = 0;
      }

      integral = std::clamp(integral, integralMin, integralMax);

      derivative =
        inewReading -
        lastReading; // Derivative over measurement to eliminate derivative kick on setpoint change

      output = std::clamp(kP * error + integral - kD * derivative + kBias, outputMin, outputMax);

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

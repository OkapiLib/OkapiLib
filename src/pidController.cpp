/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/control/pidController.hpp"
#include "api.h"
#include <cmath>

namespace okapi {
  void PIDController::setSampleTime(const int isampleTime) {
    if (isampleTime > 0) {
      const float ratio = static_cast<float>(isampleTime) / static_cast<float>(sampleTime);
      kI *= ratio;
      kD /= ratio;
      sampleTime = isampleTime;
    }
  }

  void PIDController::setOutputLimits(float imax, float imin) {
    //Always use larger value as max
    if (imin > imax) {
      const float temp = imax;
      imax = imin;
      imin = temp;
    }

    outputMax = imax;
    outputMin = imin;

    //Fix output
    if (output > outputMax)
      output = outputMax;
    else if (output < outputMin)
      output = outputMin;

    //Fix integral
    setIntegralLimits(imax, imin);
  }

  void PIDController::setIntegralLimits(float imax, float imin) {
    //Always use larger value as max
    if (imin > imax) {
      const float temp = imax;
      imax = imin;
      imin = temp;
    }

    integralMax = imax;
    integralMin = imin;

    //Fix integral
    if (integral > integralMax)
      integral = integralMax;
    else if (integral < integralMin)
      integral = integralMin;
  }

  float PIDController::step(const float inewReading) {
    if (isOn) {
      const long now = pros::millis();

      if (now - lastTime >= sampleTime) {
        error = target - inewReading;

        integral += kI * error; //Eliminate integral kick while realtime tuning

        if (shouldResetOnCross && copysign(1.0, (float)error) != copysign(1.0, (float)lastError))
          integral = 0;

        if (integral > integralMax)
          integral = integralMax;
        else if (integral < integralMin)
          integral = integralMin;

        const float derivative = inewReading - lastReading; //Derivative over measurement to eliminate derivative kick on setpoint change

        output = kP * error + integral - kD * derivative + kBias;

        if (output > outputMax)
          output = outputMax;
        else if (output < outputMin)
          output = outputMin;

        lastReading = inewReading;
        lastError = error;
        lastTime = now; //Important that we only assign lastTime if dt >= sampleTime
      }
    } else {
      output = 0; //Controller is off so write 0
    }

    return output;
  }

  void PIDController::setGains(const float ikP, const float ikI, const float ikD, const float ikBias) {
    const float sampleTimeSec = static_cast<float>(sampleTime) / 1000.0;
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
}

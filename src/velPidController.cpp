/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <cmath>
#include "okapi/control/velPidController.hpp"
#include "api.h"

namespace okapi {
  void VelPIDController::setGains(const float ikP, const float ikD) {
    kP = ikP;
    kD = ikD * static_cast<float>(sampleTime) / 1000.0;
  }

  void VelPIDController::setSampleTime(const int isampleTime) {
    if (isampleTime > 0) {
      kD /= static_cast<float>(isampleTime) / static_cast<float>(sampleTime);
      sampleTime = isampleTime;
    }
  }

  void VelPIDController::setOutputLimits(float imax, float imin) {
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
  }

  float VelPIDController::stepVel(const float inewReading) {
    return velMath.step(inewReading);
  }

  float VelPIDController::step(const float inewReading) {
    if (isOn) {
      const long now = pros::millis();
      if (now - lastTime >= sampleTime) {
        stepVel(inewReading);
        const float error = target - velMath.getOutput();

        const float derivative = velMath.getDiff(); //Derivative over measurement to eliminate derivative kick on setpoint change

        output += kP * error - kD * derivative;

        if (output > outputMax)
          output = outputMax;
        else if (output < outputMin)
          output = outputMin;

        lastError = error;
        lastTime = now; //Important that we only assign lastTime if dt >= sampleTime
      }

      return output;
    }

    return 0;
  }

  void VelPIDController::reset() {
    error = 0;
    lastError = 0;
    output = 0;
  }
}

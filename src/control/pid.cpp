#include <cmath>
#include <API.h>
#include "control/pid.h"
#include "PAL/PAL.h"

namespace okapi {
  void Pid::setSampleTime(const int isampleTime) {
    if (isampleTime > 0) {
      const float ratio = static_cast<float>(isampleTime) / static_cast<float>(sampleTime);
      kI *= ratio;
      kD /= ratio;
      sampleTime = isampleTime;
    }
  }

  void Pid::setOutputLimits(float imax, float imin) {
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

  void Pid::setIntegralLimits(float imax, float imin) {
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

  float Pid::step(const float inewReading) {
    using namespace std; //Needed to get copysign to compile

    if (isOn) {
      const long now = PAL::millis();

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

  void Pid::setGains(const float ikP, const float ikI, const float ikD, const float ikBias) {
    const float sampleTimeSec = static_cast<float>(sampleTime) / 1000.0;
    kP = ikP;
    kI = ikI * sampleTimeSec;
    kD = ikD * sampleTimeSec;
    kBias = ikBias;
  }

  void Pid::reset() {
    error = 0;
    lastError = 0;
    lastReading = 0;
    integral = 0;
    output = 0;
  }
}

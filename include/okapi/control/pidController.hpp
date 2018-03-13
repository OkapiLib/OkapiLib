/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_PID_HPP_
#define _OKAPI_PID_HPP_

#include "okapi/control/controlObject.hpp"

namespace okapi {
  class PIDControllerParams : public ControlObjectParams {
  public:
    PIDControllerParams(const float ikP, const float ikI, const float ikD, const float ikBias = 0):
      kP(ikP),
      kI(ikI),
      kD(ikD),
      kBias(ikBias) {}

    const float kP, kI, kD, kBias;
  };

  class PIDController : public ControlObject {
  public:
    /**
     * PID controller.
     * 
     * @param ikP proportional gain
     * @param ikI integral gain
     * @param ikD derivative gain
     * @param ikBias controller bias (constant offset added to the output)
     */
    PIDController(const float ikP, const float ikI, const float ikD, const float ikBias = 0);

    /**
     * PID controller.
     * 
     * @param params params (see PIDControllerParams docs)
     */
    PIDController(const PIDControllerParams& params);

    virtual ~PIDController();

    /**
     * Do one iteration of the controller.
     * 
     * @param inewReading new measurement
     * @return controller output
     */
    virtual float step(const float inewReading) override;

    /**
     * Sets the target for the controller.
     */
    void setTarget(const float itarget) override;

    /**
     * Returns the last calculated output of the controller.
     */
    float getOutput() const override;

    /**
     * Returns the last error of the controller.
     */
    float getError() const override;

    /**
     * Set controller gains.
     * 
     * @param ikP proportional gain
     * @param ikI integral gain
     * @param ikD derivative gain
     * @param ikBias bias (constant offset added to the output)
     */
    void setGains(const float ikP, const float ikI, const float ikD, const float ikBias = 0);

    /**
     * Set time between loops in ms.
     * 
     * @param isampleTime time between loops in ms
     */
    void setSampleTime(const int isampleTime) override;

    /**
     * Set controller output bounds.
     * 
     * @param imax max output
     * @param imin min output
     */
    void setOutputLimits(float imax, float imin) override;

    /**
     * Set integrator bounds.
     * 
     * @param imax max integrator value
     * @param imin min integrator value
     */
    void setIntegralLimits(float imax, float imin);

    /**
     * Resets the controller so it can start from 0 again properly. Keeps gains and limits from
     * before.
     */
    void reset() override;

    /**
     * Set whether the integrator should be reset when error is 0 or changes sign.
     * 
     * @param iresetOnZero true to reset
     */
    void setIntegratorReset(bool iresetOnZero);

    /**
     * Change whether the controll is off or on.
     */
    void flipDisable() override;

  protected:
    float kP, kI, kD, kBias;
    long lastTime, sampleTime;
    float error, lastError;
    float target, lastReading;
    float integral, integralMax, integralMin;
    float output, outputMax, outputMin;
    bool shouldResetOnCross, isOn;
  };
}

#endif

#ifndef OKAPI_PID
#define OKAPI_PID

#include "control/controlObject.h"

namespace okapi {
  class PidParams : public ControlObjectParams {
  public:
    PidParams(const float ikP, const float ikI, const float ikD, const float ikBias = 0):
      kP(ikP),
      kI(ikI),
      kD(ikD),
      kBias(ikBias) {}

    float kP, kI, kD, kBias;
  };

  class Pid : public ControlObject {
  public:
    /**
     * PID controller
     * @param ikP    Proportional gain
     * @param ikI    Integral gain
     * @param ikD    Derivative gain
     * @param ikBias Controller bias (added to final output)
     */
    Pid(const float ikP, const float ikI, const float ikD, const float ikBias = 0):
      lastTime(0),
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

    /**
     * PID controller
     * @param params Params (see PidParams docs)
     */
    Pid(const PidParams& params):
      lastTime(0),
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

    virtual ~Pid() = default;

    /**
     * Do one iteration of the controller
     * @param  inewReading New measurement
     * @return            Controller output
     */
    virtual float step(const float inewReading) override;
    
    void setTarget(const float itarget) override { target = itarget; }

    float getOutput() const override { return output; }

    float getError() const override { return error; }

    /**
     * Set controller gains
     * @param ikP    Proportional gain
     * @param ikI    Integral gain
     * @param ikD    Derivative gain
     * @param ikBias Controller bias
     */
    void setGains(const float ikP, const float ikI, const float ikD, const float ikBias = 0);

    /**
     * Set time between loops in ms
     * @param isampleTime Time between loops in ms
     */
    void setSampleTime(const int isampleTime) override;

    /**
     * Set controller output bounds
     * @param imax Max output
     * @param imin Min output
     */
    void setOutputLimits(float imax, float imin) override;

    /**
     * Set integrator bounds
     * @param imax Max integrator value
     * @param imin Min integrator value
     */
    void setIntegralLimits(float imax, float imin);

    /**
     * Resets the controller so it can start from 0 again properly. Keeps gains
     * and limits from before
     */
    void reset() override;

    /**
     * Set whether the integrator should be reset when error is 0 or changes sign
     * @param iresetOnZero True to reset
     */
    void setIntegratorReset(bool iresetOnZero) { shouldResetOnCross = iresetOnZero; }

    void flipDisable() override { isOn = !isOn; }
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

#endif /* end of include guard: OKAPI_PID */

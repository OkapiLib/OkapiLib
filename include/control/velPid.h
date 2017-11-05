#ifndef OKAPI_VELPID
#define OKAPI_VELPID

#include "control/velMath.h"
#include "control/controlObject.h"

namespace okapi {
  class VelPidParams : public ControlObjectParams {
  public:
    VelPidParams(const float ikP, const float ikD):
      kP(ikP),
      kD(ikD) {}

    float kP, kD;
  };

  class VelPid : public ControlObject {
  public:
    /**
     * Velocity PID controller
     * @param ikP    Proportional gain
     * @param ikD    Derivative gain
     */
    VelPid(const float ikP, const float ikD):
      lastTime(0),
      sampleTime(15),
      error(0),
      lastError(0),
      target(0),
      output(0),
      outputMax(127),
      outputMin(-127),
      isOn(true),
      velMath(360) {
        setGains(ikP, ikD);
      }

    /**
     * Velocity PID controller
     * @param params Params (see VelPidParams docs)
     */
    VelPid(const VelPidParams& params):
      lastTime(0),
      sampleTime(15),
      error(0),
      lastError(0),
      target(0),
      output(0),
      outputMax(127),
      outputMin(-127),
      isOn(true),
      velMath(360) {
        setGains(params.kP, params.kD);
      }

    virtual ~VelPid() = default;

    /**
     * Do one iteration of velocity calculation
     * @param  inewReading New measurement
     * @return             Filtered velocity
     */
    virtual float stepVel(const float inewReading);

    /**
     * Do one iteration of the controller
     * @param  inewReading New measurement
     * @return            Controller output
     */
    virtual float step(const float inewReading) override;

    void setTarget(const float itarget) override { target = itarget; }
    
    float getOutput() const override { return isOn ? output : 0; }

    float getError() const override { return error; }
    
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
    
    void reset() override;

    void flipDisable() override { isOn = !isOn; }
    
    /**
     * Set controller gains
     * @param ikP    Proportional gain
     * @param ikD    Derivative gain
     * @param ikBias Controller bias
     */
    void setGains(const float ikP, const float ikD);

    /**
     * Set the gains for the double moving average filter. Defaults are 0.19 and 0.0526, respectively
     * @param alpha Alpha gain
     * @param beta  Beta gain
     */
    void setFilterGains(const float alpha, const float beta) { velMath.setGains(alpha, beta); }

    /**
     * Set the number of measurements per revolution. Default is 360
     * @param tpr Number of measured units per revolution
     */
    void setTicksPerRev(const float tpr) { velMath.setTicksPerRev(tpr); }

    float getVel() const { return velMath.getOutput(); }

  private:
    float kP, kD;
    long lastTime, sampleTime;
    float error, lastError;
    float target;
    float output, outputMax, outputMin;
    bool isOn;
    VelMath velMath;
  };
}

#endif /* end of include guard: OKAPI_VELPID */

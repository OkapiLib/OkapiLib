#ifndef OKAPI_VELPID
#define OKAPI_VELPID

#include "filter/demaFilter.h"

namespace okapi {
  class VelPidParams {
  public:
    VelPidParams(const float ikP, const float ikD):
      kP(ikP),
      kD(ikD) {}

    float kP, kD;
  };

  class VelPid {
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
      shouldResetOnCross(true),
      vel(0),
      lastVel(0),
      lastPos(0),
      ticksPerRev(360),
      filter(0.19, 0.0526) {
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
      shouldResetOnCross(true),
      vel(0),
      lastVel(0),
      lastPos(0),
      ticksPerRev(360),
      filter(0.19, 0.0526) {
        setGains(params.kP, params.kD);
      }

    virtual ~VelPid() = default;

    /**
     * Do one iteration of velocity calculation
     * @param  inewReading New measurement
     * @return             Filtered velocity
     */
    virtual float loopVel(const float inewReading);

    /**
     * Do one iteration of the controller
     * @param  inewReading New measurement
     * @return            Controller output
     */
    virtual float loop(const float inewReading);

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
    void setFilterGains(const float alpha, const float beta) { filter.setGains(alpha, beta); }

    /**
     * Set time between loops in ms
     * @param isampleTime Time between loops in ms
     */
    void setSampleTime(const int isampleTime);

    /**
     * Set the number of measurements per revolution. Default is 360
     * @param tpr Number of measured units per revolution
     */
    void setTicksPerRev(const float tpr) { ticksPerRev = tpr; }

    /**
     * Set controller output bounds
     * @param imax Max output
     * @param imin Min output
     */
    void setOutputLimits(float imax, float imin);

    void reset();

    void setTarget(const float itarget) { target = itarget; }

    float getOutput() const { return output; }

  private:
    float kP, kD;
    long lastTime, sampleTime;
    float error, lastError;
    float target;
    float output, outputMax, outputMin;
    bool shouldResetOnCross;
    float vel, lastVel, lastPos, ticksPerRev;
    DemaFilter filter;
  };
}

#endif /* end of include guard: OKAPI_VELPID */
